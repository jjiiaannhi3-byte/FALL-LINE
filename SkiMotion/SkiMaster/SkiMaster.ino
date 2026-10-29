#include "MotionCommon.h"

constexpr int SDA_PIN=8,SCL_PIN=9;
constexpr uint8_t IMU_ADDRESS=0x68;
constexpr int MOTOR_IN1=4,MOTOR_IN2=5,MOTOR_SLEEP=6;
constexpr uint8_t MOTOR_DUTY=160;
// Demo settings only; NOT validated ski coaching criteria.
constexpr float ROLL_LIMIT=20,PITCH_LIMIT=25,HYSTERESIS=4;
// Enable only after selecting useful limits for the actual mounting positions.
constexpr bool ENABLE_LIMB_ALERTS=false;
constexpr float LIMB_ROLL_LIMIT[4]={45,45,30,30};
constexpr float LIMB_PITCH_LIMIT[4]={45,45,30,30};
constexpr uint32_t HOLD_MS=800,GAP_MS=2500,PULSE_MS=150;
struct Received {Packet packet;uint8_t mac[6];uint32_t arrival;};
struct NodeState {
  Packet packet={};uint8_t mac[6]={};uint32_t arrival=0,missed=0;bool seen=false;
};
MotionSensor imu;
QueueHandle_t rxQueue;
NodeState nodes[4];
bool armed=false,motorRunning=false,exceeded=false;
uint32_t motorStarted=0,badSince=0,lastReminder=0,calSequence=0;

void motorOff() {
  digitalWrite(MOTOR_SLEEP,LOW);ledcWrite(MOTOR_IN1,0);
  digitalWrite(MOTOR_IN2,LOW);motorRunning=false;
}
void pulse() {
  digitalWrite(MOTOR_IN2,LOW);ledcWrite(MOTOR_IN1,MOTOR_DUTY);
  digitalWrite(MOTOR_SLEEP,HIGH);motorStarted=millis();motorRunning=true;
}
void receive(const esp_now_recv_info_t *info,const uint8_t *data,int length) {
  if(length!=sizeof(Packet)) return;
  Received r={};memcpy(&r.packet,data,sizeof(Packet));
  const Packet &p=r.packet;
  if(!headerOK(p)||p.type!=DATA||p.node<1||p.node>4||p.valid>1) return;
  if(!isfinite(p.roll)||!isfinite(p.pitch)||fabsf(p.roll)>180||fabsf(p.pitch)>180) return;
  memcpy(r.mac,info->src_addr,6);r.arrival=millis();xQueueSend(rxQueue,&r,0);
}
bool allHealthy() {
  if(!imu.ready) return false;
  for(auto &n:nodes)
    if(!n.seen||!n.packet.valid||uint32_t(millis()-n.arrival)>STALE_MS) return false;
  return true;
}
void recalibrate() {
  armed=false;motorOff();exceeded=false;
  Packet p={};p.group=GROUP_ID;p.version=VERSION;p.type=CALIBRATE;
  p.sequence=++calSequence;
  auto err=esp_now_send(BROADCAST,(uint8_t*)&p,sizeof(p));
  Serial.printf("CAL broadcast enqueue: %s (no delivery guarantee)\n",esp_err_to_name(err));
  imu.calibrate();xQueueReset(rxQueue);
  for(auto &n:nodes) n.seen=false;
  Serial.println("DISARMED; wait for four OK nodes, then a to arm");
}
void setup() {
  pinMode(MOTOR_SLEEP,OUTPUT);digitalWrite(MOTOR_SLEEP,LOW);
  pinMode(MOTOR_IN2,OUTPUT);digitalWrite(MOTOR_IN2,LOW);
  pinMode(MOTOR_IN1,OUTPUT);digitalWrite(MOTOR_IN1,LOW);
  Serial.begin(115200);delay(800);
  if(!ledcAttach(MOTOR_IN1,20000,8)) halt("PWM init failed");
  motorOff();rxQueue=xQueueCreate(24,sizeof(Received));if(!rxQueue) halt("Queue allocation failed");
  if(!imu.begin(SDA_PIN,SCL_PIN,0x70,IMU_ADDRESS)) halt("MPU6500 init failed: check wiring/model");
  if(!startRadio(receive)) halt("ESP-NOW init failed");
  imu.calibrate();
  Serial.println("c=calibrate all, a=arm, s=stop, t=150ms motor test");
}
void loop() {
  uint32_t now=millis();
  if(motorRunning&&uint32_t(now-motorStarted)>=PULSE_MS) motorOff();
  while(Serial.available()) {
    char c=Serial.read();
    if(c=='c') recalibrate();
    if(c=='s') {armed=false;motorOff();exceeded=false;Serial.println("DISARMED");}
    if(c=='a') {
      armed=allHealthy();exceeded=false;
      Serial.println(armed?"ARMED":"Cannot arm: chest + four valid nodes required");
    }
    if(c=='t'&&!motorRunning) pulse();
  }
  static uint32_t lastSample=0,lastPrint=0;
  if(uint32_t(micros()-lastSample)>=SAMPLE_US) {lastSample=micros();imu.update();}
  Received r;
  while(xQueueReceive(rxQueue,&r,0)==pdTRUE) {
    NodeState &n=nodes[r.packet.node-1];
    if(n.seen&&memcmp(n.mac,r.mac,6)!=0&&uint32_t(millis()-n.arrival)<2000) continue;
    if(n.seen&&memcmp(n.mac,r.mac,6)==0&&n.packet.boot==r.packet.boot) {
      int32_t step=int32_t(r.packet.sequence-n.packet.sequence);
      if(step<=0) continue;
      n.missed+=uint32_t(step-1);
    }
    n.packet=r.packet;memcpy(n.mac,r.mac,6);n.arrival=r.arrival;n.seen=true;
  }
  now=millis();
  if(armed&&!allHealthy()) {
    armed=false;motorOff();exceeded=false;
    Serial.println("DISARMED: sensor fault/timeout; repair and press a");
  }
  Packet chest={};imu.fill(chest);
  if(armed) {
    bool bad=fabsf(chest.roll)>ROLL_LIMIT||fabsf(chest.pitch)>PITCH_LIMIT;
    bool good=fabsf(chest.roll)<ROLL_LIMIT-HYSTERESIS&&fabsf(chest.pitch)<PITCH_LIMIT-HYSTERESIS;
    if(ENABLE_LIMB_ALERTS) {
      for(int i=0;i<4;i++) {
        const Packet &p=nodes[i].packet;
        bad=bad||fabsf(p.roll)>LIMB_ROLL_LIMIT[i]||fabsf(p.pitch)>LIMB_PITCH_LIMIT[i];
        good=good&&fabsf(p.roll)<LIMB_ROLL_LIMIT[i]-HYSTERESIS&&fabsf(p.pitch)<LIMB_PITCH_LIMIT[i]-HYSTERESIS;
      }
    }
    if(bad&&!exceeded) {exceeded=true;badSince=now;}
    if(good) exceeded=false;
    if(exceeded&&uint32_t(now-badSince)>=HOLD_MS&&uint32_t(now-lastReminder)>=GAP_MS&&!motorRunning) {
      pulse();lastReminder=now;
    }
  }
  if(uint32_t(now-lastPrint)>=250) {
    lastPrint=now;
    Serial.printf("armed=%d chest(valid=%d,r=%.1f,p=%.1f)",armed,imu.ready,chest.roll,chest.pitch);
    for(int i=0;i<4;i++) {
      NodeState &n=nodes[i];bool ok=n.seen&&n.packet.valid&&uint32_t(now-n.arrival)<=STALE_MS;
      Serial.printf(" N%d[%s,r=%.1f,p=%.1f,age=%lu,miss=%lu]",i+1,ok?"OK":"OFF",
        n.packet.roll,n.packet.pitch,n.seen?(unsigned long)(now-n.arrival):99999UL,(unsigned long)n.missed);
    }
    Serial.println();
  }
  delay(1);
}

#include "MotionCommon.h"

// Change for each board: 1 left arm, 2 right arm, 3 left leg, 4 right leg.
constexpr uint8_t NODE_ID=1;
constexpr int SDA_PIN=4,SCL_PIN=5;
constexpr uint8_t IMU_ADDRESS=0x68;
static_assert(NODE_ID>=1&&NODE_ID<=4,"NODE_ID must be 1..4");
MotionSensor imu;
QueueHandle_t calQueue;
uint32_t bootId,sequence=0,enqueueErrors=0;

void receive(const esp_now_recv_info_t *,const uint8_t *data,int length) {
  if(length!=sizeof(Packet)) return;
  Packet p;memcpy(&p,data,sizeof(p));
  if(headerOK(p)&&p.type==CALIBRATE&&p.node==0) {
    uint8_t req=1;xQueueOverwrite(calQueue,&req);
  }
}
void setup() {
  Serial.begin(115200);delay(800);
  calQueue=xQueueCreate(1,sizeof(uint8_t));if(!calQueue) halt("Queue allocation failed");
  if(!imu.begin(SDA_PIN,SCL_PIN,0x68,IMU_ADDRESS)) halt("MPU6050 init failed: check wiring/model");
  if(!startRadio(receive)) halt("ESP-NOW init failed");
  bootId=esp_random();Serial.printf("NODE %u; c=calibrate\n",NODE_ID);imu.calibrate();
}
void loop() {
  uint8_t req=0;bool cal=xQueueReceive(calQueue,&req,0)==pdTRUE;
  while(Serial.available()) if(Serial.read()=='c') cal=true;
  if(cal) imu.calibrate();
  static uint32_t lastSample=0,lastSend=0,lastLog=0;
  if(uint32_t(micros()-lastSample)>=SAMPLE_US) {lastSample=micros();imu.update();}
  if(uint32_t(millis()-lastSend)>=uint32_t(40+NODE_ID)) {
    lastSend=millis();Packet p={};p.group=GROUP_ID;p.version=VERSION;
    p.type=DATA;p.node=NODE_ID;p.boot=bootId;p.sequence=sequence++;imu.fill(p);
    if(esp_now_send(BROADCAST,(uint8_t*)&p,sizeof(p))!=ESP_OK) enqueueErrors++;
  }
  if(uint32_t(millis()-lastLog)>=1000) {
    lastLog=millis();Packet p={};imu.fill(p);
    Serial.printf("node=%u valid=%u r=%.1f p=%.1f enqueueErrors=%lu\n",
      NODE_ID,p.valid,p.roll,p.pitch,(unsigned long)enqueueErrors);
  }
  delay(1);
}

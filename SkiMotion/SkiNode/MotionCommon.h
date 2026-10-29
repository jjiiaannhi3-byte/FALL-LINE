#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <math.h>

// Target: Arduino-ESP32 3.0.7. Keep this file identical in both sketches.
constexpr uint8_t RADIO_CHANNEL=6, VERSION=1, DATA=1, CALIBRATE=2;
constexpr uint32_t GROUP_ID=0x534B4931, SAMPLE_US=10000, STALE_MS=350;
constexpr uint8_t BROADCAST[6]={255,255,255,255,255,255};
struct __attribute__((packed)) Packet {
  uint32_t group;
  uint8_t version,type,node,valid;
  uint32_t boot,sequence,sampleMs;
  float roll,pitch,ax,ay,az,gx,gy,gz;
};
static_assert(sizeof(Packet)==52,"Packet layout changed");
inline bool headerOK(const Packet &p) {return p.group==GROUP_ID && p.version==VERSION;}
inline float wrap(float x) {
  while(x>180) x-=360;
  while(x< -180) x+=360;
  return x;
}

class MotionSensor {
  uint8_t address=0x68;
  float bx=0,by=0,bz=0,zeroR=0,zeroP=0;
  uint32_t previous=0;
  bool wr(uint8_t reg,uint8_t value) {
    Wire.beginTransmission(address);Wire.write(reg);Wire.write(value);
    return Wire.endTransmission()==0;
  }
  bool rd(uint8_t reg,uint8_t *out,uint8_t n) {
    Wire.beginTransmission(address);Wire.write(reg);
    if(Wire.endTransmission(false)!=0) return false;
    if(Wire.requestFrom(address,n)!=n) return false;
    for(int i=0;i<n;i++) out[i]=Wire.read();
    return true;
  }
  bool raw(float &x,float &y,float &z,float &u,float &v,float &w) {
    uint8_t d[14];if(!rd(0x3B,d,14)) return false;
    auto s=[&](int i){return (int16_t)((uint16_t(d[i])<<8)|d[i+1]);};
    x=s(0)/8192.0f;y=s(2)/8192.0f;z=s(4)/8192.0f;
    u=s(8)/65.5f;v=s(10)/65.5f;w=s(12)/65.5f;
    return true;
  }
 public:
  bool ready=false;
  float roll=0,pitch=0,ax=0,ay=0,az=1,gx=0,gy=0,gz=0;
  bool begin(int sda,int scl,uint8_t whoExpected,uint8_t addr=0x68) {
    address=addr;Wire.begin(sda,scl);Wire.setClock(400000);Wire.setTimeOut(20);
    uint8_t who=0;
    if(!rd(0x75,&who,1)||who!=whoExpected) {
      Serial.printf("WHO_AM_I=0x%02X expected=0x%02X\n",who,whoExpected);return false;
    }
    if(!wr(0x6B,0x80)) return false;
    delay(100);
    if(!wr(0x6B,1)||!wr(0x6C,0)||!wr(0x1A,3)||!wr(0x19,9)||
       !wr(0x1B,8)||!wr(0x1C,8)) return false;
    if(whoExpected==0x70&&!wr(0x1D,3)) return false;
    delay(100);return true;
  }
  bool calibrate() {
    ready=false;
    Serial.println("CAL: hold neutral pose STILL for 3 seconds");
    float sx=0,sy=0,sz=0,sa=0,sb=0,sc=0;
    for(int i=0;i<300;i++) {
      float x,y,z,u,v,w;
      if(!raw(x,y,z,u,v,w)) {Serial.println("CAL failed: I2C");return false;}
      float g=sqrtf(x*x+y*y+z*z);
      if(g<0.85f||g>1.15f||fabsf(u)>8||fabsf(v)>8||fabsf(w)>8) {
        Serial.println("CAL failed: motion; retry c");return false;
      }
      sx+=u;sy+=v;sz+=w;sa+=x;sb+=y;sc+=z;delay(10);
    }
    bx=sx/300;by=sy/300;bz=sz/300;
    roll=atan2f(sb,sc)*RAD_TO_DEG;
    pitch=atan2f(-sa,sqrtf(sb*sb+sc*sc))*RAD_TO_DEG;
    zeroR=roll;zeroP=pitch;
    if(fabsf(pitch)>75) {Serial.println("CAL failed: remount IMU, X axis too vertical");return false;}
    previous=micros();ready=true;Serial.println("CAL OK");return true;
  }
  bool update() {
    if(!ready) return false;
    if(!raw(ax,ay,az,gx,gy,gz)) {ready=false;return false;}
    gx-=bx;gy-=by;gz-=bz;
    uint32_t now=micros();float dt=(now-previous)*1e-6f;previous=now;
    if(dt<=0||dt>0.1f) {ready=false;return false;}
    float r=roll*DEG_TO_RAD,p=pitch*DEG_TO_RAD;
    if(fabsf(cosf(p))<0.17f) {ready=false;return false;}
    float rp=roll+(gx+sinf(r)*tanf(p)*gy+cosf(r)*tanf(p)*gz)*dt;
    float pp=pitch+(cosf(r)*gy-sinf(r)*gz)*dt;
    float g=sqrtf(ax*ax+ay*ay+az*az);
    if(g>0.85f&&g<1.15f) {
      float ar=atan2f(ay,az)*RAD_TO_DEG;
      float ap=atan2f(-ax,sqrtf(ay*ay+az*az))*RAD_TO_DEG;
      float k=dt/(0.5f+dt);
      rp+=k*wrap(ar-rp);pp+=k*wrap(ap-pp);
    }
    roll=wrap(rp);pitch=wrap(pp);return true;
  }
  void fill(Packet &p) const {
    p.valid=ready;p.sampleMs=millis();p.roll=wrap(roll-zeroR);p.pitch=wrap(pitch-zeroP);
    p.ax=ax;p.ay=ay;p.az=az;p.gx=gx;p.gy=gy;p.gz=gz;
  }
};

inline bool startRadio(esp_now_recv_cb_t cb) {
  WiFi.mode(WIFI_STA);WiFi.setSleep(false);
  if(esp_wifi_set_channel(RADIO_CHANNEL,WIFI_SECOND_CHAN_NONE)!=ESP_OK) return false;
  if(esp_now_init()!=ESP_OK||esp_now_register_recv_cb(cb)!=ESP_OK) return false;
  esp_now_peer_info_t peer={};memcpy(peer.peer_addr,BROADCAST,6);
  peer.channel=RADIO_CHANNEL;peer.ifidx=WIFI_IF_STA;peer.encrypt=false;
  if(esp_now_add_peer(&peer)!=ESP_OK) return false;
  Serial.print("MAC: ");Serial.println(WiFi.macAddress());return true;
}
inline void halt(const char *s) {Serial.println(s);while(true) delay(1000);}

#include<Wire.h>
#include"GY80.h"
#include <SoftwareSerial.h>
#include <math.h>
#include <stdint.h>
#include <string.h>
#define G_scale 0.007629
#define Qa 0.01
#define Qg 0.01
#define Qm 0.01
#define MA 8
#define ceiling 1200

bool at = 0;

SoftwareSerial BT(8, 9); // Rx, Tx
SoftwareSerial BT2(5, 6); // Rx, Tx

GY80 sensor = GY80(); //create GY80 instance

float dT = 0.034;
float pitch,roll,yaw0=0;
float angle[2]={0};  //pitch,roll
float gb = 0;

void setup() {

  
  Serial.begin(9600);
  BT.begin(38400);
  BT2.begin(38400);

  Serial.println("BT is ready!");
  sensor.begin();       //initialize sensors
  GY80_raw raw = sensor.read_raw();
  
  for(int i=0;i<100;++i){
    raw = sensor.read_raw();
    gb += raw.a_z;
  }
  gb = 255 - gb/100;
 
//  Serial.println("init done");
}

float Kalman(int8_t n, int16_t Acc, int16_t Accz, int16_t Gyro) {
  static float bias[2];
  static float K[2],Kb[2];
  static float M0[2], M1[2];
  static float dT = 0.034;
  float g0 = Accz + gb;
 
  float Y = atan2(Acc, g0) ;
  angle[n] += (Gyro- bias[n]) * G_scale  * dT;
  
  M0[n] = 1 + (Qa + dT) * dT;
  M1[n] = -dT;
  K[n] = M0[n] / (M0[n] + Qa);
  Kb[n] = M1[n] / (M1[n] + Qa);
  angle[n] += K[n] * (Y - angle[n]);
  bias[n] += Kb[n] * (Y - angle[n]);
  M1[n] += M0[n] * Kb[n];
  M0[n] *= 1 - K[n];  
  return angle[n] * 180 / 3.1415926F;
}
/*
inline void BTsend(float ang){
  char buf[32];
  sprintf(buf,"%ld,",(long int)(ang*1000));
  BT.write(buf);  
}
*/
int step = 0;
float dist = 0;

int step_counter(int16_t ay,int16_t az , int16_t gx){
  static int state = 0;
  static float v=0,d=0;
  static unsigned long long int t = 0;
  static unsigned long long int tl = 0;
  static int zero_cnt = 0;
  static bool flag = 0;

  if(state == 0) flag = 0;

  if(state!=0) ++zero_cnt;

  if(zero_cnt > 30) {
    state = 0;
    zero_cnt = 0;
  }
  
  if(gx >80 && state == 0){ 
    tl = t;
    state = 1;
    zero_cnt = 0;
      
  }
  
  if(gx <-80 && state == 1 && t-tl<30) {
    tl = t;
    state = 2;
    zero_cnt = 0;

  }
  
  if(gx >20 && state == 2 && t-tl<30){ 
    tl = t;
    state = 3;
    zero_cnt = 0;
  }
  
  if(  abs(gx) < 10 && state == 3 && abs(pitch) < 5){ 
    state = 0;
    flag = 1;
    dist+=abs(d*25);
  }
        
  if(state==2 || state==3){
    v=v+dT*(ay*cos(pitch))/255*9.81 ;
    d=d+dT*v;
  }  
  ++t;  
  return flag;
}

void loop(){
  // receive another GY801's sensor data
  char cmd[32];
  memset(cmd,0,32);

  char v;
  bool col = 0;
  int idx = 0;
  
  while(BT2.available()){
    v = BT2.read();
    if(col==1) {
      cmd[idx++] = v;
    }
    if(v=='|') {
      col = 1;
      idx=0;
      continue;
    }
    if(v=='\n' && col ==1){
      delay(15);
      col=0;
      break;
    }
    //  delay(1);
  }

  static float ax,ay,az;
  // static float gx,gy,gz;
  static float mx,my,mz;
  static int16_t iter = 0;

  static unsigned long lt = 0;  
  dT = (millis() - lt) / 1000.0  ;
  lt = millis();
  
  GY80_raw raw = sensor.read_raw();

  if(abs(raw.a_x) <ceiling) ax = raw.a_x;
  if(abs(raw.a_y) <ceiling) ay = -raw.a_y;
  if(abs(raw.a_z) <ceiling) az = raw.a_z;
  if(abs(raw.m_x) <182) mx = -raw.m_x;
  if(abs(raw.m_y) <182) my = -raw.m_y;
  if(abs(raw.m_z) <182) mz = raw.m_z;

  static int acc_array[3][MA]={0};
  float acc_avg[3]={0};


  // average and filtering
  acc_array[0][iter] = ax;
  acc_array[1][iter] = ay;
  acc_array[2][iter] = az;

  iter = (iter+1)%MA;
  
  for(int i=0;i<3;++i){  
    for(int j=0;j<MA;++j){
      acc_avg[i] += acc_array[i][j];
    }
    acc_avg[i]/=MA;
  }

  pitch = Kalman(0, acc_avg[1], acc_avg[2], raw.g_x ) ;

  roll = Kalman(1, acc_avg[0], acc_avg[2], raw.g_y );
  
  float Ny = my/cos(angle[0]);
  float Nx = mx/cos(angle[1]);
  float yaw = atan2(Ny,Nx)*180/3.1415926F;

  step += step_counter(ay,az,raw.g_x/100);

  char buf[64];
  memset(buf,0,64);

  sprintf(buf,"%ld,%ld,%ld,%d,%ld,",(long int)(pitch*1000),(long int)(roll*1000),(long int)(yaw*1000),step,(long int)(dist*1000));
  
  BT.write(buf);
  BT.write(cmd);
  
  BT.flush();
  delay(43);
}



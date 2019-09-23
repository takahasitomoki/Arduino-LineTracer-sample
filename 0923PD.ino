#include "ESC2nd.h"
#include <MsTimer2.h>

//タイマー割り込み周期
#define TIM_FREQ 0.005

//センサー生値
int value[8];
int value_MAX[8] = {0,0,0,0,0,0,0,0};
int value_MIN[8] = {5000,5000,5000,5000,5000,5000,5000,5000};

//キャリブレーション後の値
int value_ave[8];

//int weight[4] = {9,4,2,1};
int weight[8] = {7,3,2,1,1,2,4,8};

int line;
int line_diff = 5;
float P_GAIN = 0.25;
//float R_para = 1.6;
float R_para = 1.0;
//float D_GAIN = 15.5;
float D_GAIN = 20.5;
float duty_r;
float duty_l;
float prev_line;
Motor motorR('r');
Motor motorL('l');

//センサーの値取得の構造体のインスタンス生成
Sensor sensor(DEVICE_ADDR1);

//タイマー割り込み
void timerFire(){
  //dutyを計算(PD制御)
  duty_r = (45 + line * P_GAIN * R_para)*R_para + (prev_line - line) * TIM_FREQ * D_GAIN;
  duty_l = 26 + -1 * line * P_GAIN - (prev_line - line) * TIM_FREQ * D_GAIN;

  //モーターに出力
  motorR.drive(duty_r);
  motorL.drive(duty_l);

  //現在の重み付け平均を保存
  prev_line = line;
}

void setup(){
  //シリアル出力準備
  Serial.begin(9600);
  delay(1000);
  //タイマー割り込みスタート
  MsTimer2::set(5,timerFire);
  MsTimer2::start();
}

void loop(){
  //センサーの値を取得
    sensor.read(value);
    
  for(int i=0;i<8;i++){
    //センサの値の最大値と最小値を更新
   if(value_MAX[i] < value[i] ){
    value_MAX[i] = value[i];
   }
   if(value_MIN[i] > value[i] ){
    value_MIN[i] = value[i];
   }
   //キャリブレーション後の値を代入
   value_ave[i] = (float)(value[i] - value_MIN[i]) / (float)(value_MAX[i] - value_MIN[i]) * 100.0;
  }

  //重み付け平均を計算
  line = line_diff + -value_ave[0]*weight[0] - value_ave[1]*weight[1] - value_ave[2]*weight[2] - value_ave[3]*weight[3]+value_ave[4]*weight[4]+value_ave[5]*weight[5]+value_ave[6]*weight[6]+value_ave[7]*weight[7];

   //シリアルモニタに出力
   Serial.print(line);
//   Serial.print(value_ave[0]);
   Serial.print(" \n");
}

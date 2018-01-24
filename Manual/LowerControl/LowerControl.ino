 #include <DueTimer.h>
#include <Wire.h>
#include <DuePWM.h>

#define rpmmax 300
#define Time 0.2
#define MaxPwm 255
#define GearRatio 0.87
#define desiredpwm(x) (x*255.0)/320.0  
#define PWM_FREQ1 2500
DuePWM pwm(PWM_FREQ1,3000);

int RPM[3]={0};
int flag=1;
int zeroflag[3];

struct gain {
  float kd;
  float kp;
  float ki;
  float required;
  float maxControl;
  float minControl;
  float error;
  float previousError;
  float derivativeError;
  float integralError;
};

struct gain Motor[3];
struct gain *pMotorgain[3]={&Motor[0],&Motor[1],&Motor[2]};

//Struct for motors
struct motor{
  int pin1;
  int pin2;
  int pwmpin;
};
typedef struct motor MOTOR;

MOTOR MotorA={50,52,8};
MOTOR MotorB={46,48,7};
MOTOR MotorC={36,38,6};

MOTOR *motors[3]={&MotorA,&MotorB,&MotorC};


//struct for encoder and lower control systemX

int Dirflag=0;
//****************************************************************

struct encoder{
  int channelA;
  int channelB;
  long int Count;
  long int previousCount;
  int Rpm; 
  int Ppr;
};

typedef struct encoder Encoder;

//Encoder encoderA,encoderB,encoderC;
Encoder encoder1={22,24,0,0,0,200};
Encoder encoder2={28,26,0,0,0,200};
Encoder encoder3={34,32,0,0,0,200};
Encoder *encoderA=&encoder1;
Encoder *encoderB=&encoder2;
Encoder *encoderC=&encoder3;
//encoderC={};
float outsee[3]={0};
float Output[3];

void setup(){
  //initEncoders();
  Serial.begin(9600);
  //Serial3.begin(115200);
  Timer1.attachInterrupt(Timerhandler);
  Timer1.start(200000);
  Wire.begin(8);
  Wire.onReceive(getData);
  
  initMotor(motors[0]);
  initMotor(motors[1]);
  initMotor(motors[2]);
  pwm.setFreq1(PWM_FREQ1);
  pwm.pinFreq1( 6 );  // Pin 6 freq set to "pwm_freq1" on clock A
  pwm.pinFreq1( 7 );  // Pin 7 freq set to "pwm_freq2" on clock B
  pwm.pinFreq1( 8 );  // Pin 8 freq set to "pwm_freq2" on clock B
    
//  PIDinit(0.038,0.015,0,0,0,255,pMotorgain[0]);
//  
//  PIDinit(0.038,0.015,0,0,0,255,pMotorgain[1]);
//  
//  PIDinit(0.038,0.015,0,0,0,255,pMotorgain[2]);
//  
  Serial.println("================================");
 // delay(1000);
  //analogWrite(6,100);
  
}



void loop(){
}

void Timerhandler(){ 
outsee[0]=drivewheel((float)pMotorgain[0]->required/4,MaxPwm,motors[0], zeroflag[0]);
outsee[1]=drivewheel((float)pMotorgain[1]->required/6.5 ,MaxPwm,motors[1],zeroflag[1]);
outsee[2]=drivewheel((float)pMotorgain[2]->required/6.5,MaxPwm,motors[2],zeroflag[2]);   
}
//  if(flag==1){
//   Serial.print('a');
//  encoderA->Rpm=((encoderA->Count-encoderA->previousCount)*60.0)/(Time*GearRatio*encoderA->Ppr);
//  encoderB->Rpm=((encoderB->Count-encoderB->previousCount)*60.0)/(Time*GearRatio*encoderB->Ppr);//Why do we need previous count....
//  encoderC->Rpm=((encoderC->Count-encoderC->previousCount)*60.0)/(Time*GearRatio*encoderC->Ppr);
//  encoderA->previousCount=encoderA->Count;
//  encoderB->previousCount=encoderB->Count;
//  encoderC->previousCount=encoderC->Count;
//  
//  float temp[3];
//  
//  temp[0]=PID(encoderA->Rpm,pMotorgain[0]);
//  temp[1]=PID(encoderB->Rpm,pMotorgain[1]);
//  temp[2]=PID(encoderC->Rpm,pMotorgain[2]);
//  //Serial.println("A: "+String(temp[0])+"B: "+String(temp[1])+"C: "+String(temp[2]));
//  Output[0]+=temp[0];
//  Output[1]+=temp[1];
//  Output[2]+=temp[2];
//
//  //outsee[0]=drivewheel(128,MaxPwm,motors[0]);
//  //outsee[1]=drivewheel(128,MaxPwm,motors[1]);
//  //outsee[2]=drivewheel(128,MaxPwm,motors[2]);
//  
//  outsee[0]=drivewheel(Output[0],MaxPwm,motors[0],zeroflag[0]);
//  outsee[1]=drivewheel(Output[1],MaxPwm,motors[1],zeroflag[1]);
//  outsee[2]=drivewheel(Output[2],MaxPwm,motors[2],zeroflag[2]);
//  }
//  else
//  {
//    for(int i=0;i<3;i++){
//    digitalWrite(motors[i]->pin1,HIGH);
//    digitalWrite(motors[i]->pin2,HIGH);
//    }
//  }
////Serial.println("Rpm A:"+ String(encoderA->Rpm)+" Rpm B:"+ String(encoderB->Rpm)+" Rpm C:"+ String(encoderC->Rpm) + "Out: "+String(Output[0])+" "+String(Output[1])+" "+String(Output[2])+"outsee:"+String(outsee[0]));//+"Rpm a:"+ String(encoderA->Rpm));
////
////}

//void Timerhandler(){}
//void initEncoders(){
//  pinMode(encoderA->channelA, INPUT);
//  pinMode(encoderA->channelB, INPUT);
//  
//  pinMode(encoderB->channelA, INPUT);
//  pinMode(encoderB->channelB, INPUT);
//  
//  pinMode(encoderC->channelA, INPUT);
//  pinMode(encoderC->channelB, INPUT);
//  
//  attachInterrupt(digitalPinToInterrupt(encoderA->channelA),TicksA,RISING);
//  attachInterrupt(digitalPinToInterrupt(encoderB->channelA),TicksB,RISING);
//  attachInterrupt(digitalPinToInterrupt(encoderC->channelA),TicksC,RISING);
//}

//
//void TicksA(){
//  if(digitalRead(encoderA->channelB))
//  encoderA->Count+=1;
//  else
//  encoderA->Count-=1;
//}
//
//void TicksB(){
//  if(digitalRead(encoderB->channelB))
//  encoderB->Count+=1;
//  else
//  encoderB->Count-=1;
//}
//
//void TicksC(){
//  if(digitalRead(encoderC->channelB))
//  encoderC->Count+=1;
//  else
//  encoderC->Count-=1;
//}
int x;
void getData(int howMany){
  while(Wire.available())
  {
    for(int y =0;y<3;y++)
     {
        x=1;
        RPM[y]=0;
        zeroflag[y]=0;
        for(int r=0;r<3;r++)
        {
          RPM[y]+=(int)Wire.read()*x;
          x*=10;
        } 
        RPM[y]-=400;
        if(RPM[y]==0)
        zeroflag[y]=1;
        }
    pMotorgain[0]->required=RPM[0];
    pMotorgain[1]->required=RPM[1];
    pMotorgain[2]->required=RPM[2];
    Serial.println(pMotorgain[0]->required);
    Serial.println(pMotorgain[1]->required);
    Serial.println(pMotorgain[2]->required); 
   }
}


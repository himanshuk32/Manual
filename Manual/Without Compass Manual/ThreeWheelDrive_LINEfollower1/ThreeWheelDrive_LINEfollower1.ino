#include <Wire.h>
#include <SPI.h>
#include <LIS3MDL.h>

#define SerialPS2 Serial3

LIS3MDL mag;
//3472  -3399   3697
int bias[]={1534,-2943,2964};//{3472,-3399,3697}
long int magn[3];
float compassHeadingOffset=0;
float compassHeading;
float prevcompassHeading;
//int baud=300;
/* Structs/Variables for driving */
      
typedef struct wheels {
  float trans_rpm; //RPM for translational Velocity
  float ang_rpm; //RPM due to angular velocity
  int angle; //Position of tyre from x-axis
  float rpmmax; //Max RPM of wheel
  int pinpwm;//Pin PWM of motor to be connected to
  int pina;//a = 1, b=0 positive
  int pinb;
  int rpm;
} wheel; 

const int anglea = 90;
const int angleb = 225;
const int anglec = 315;
const int rpmmax = 400;
const int pinpwma = 3;   //4; //Channel1
const int pinpwmb = 4;  //Channel2
const int pinpwmc = 2;   //2;  //Channel1
const int pinaa = 33;   //29;
const int pinab = 37;     //25;     //ALL WHEELS ARE TRYING TO ROTATE BOT CLOCKWISE WHEN A HIGH AND B LOW
const int pinba = 25;    //37;
const int pinbb = 29;    //33;
const int pinca = 41;    //41;
const int pincb = 45;
const float HeadTheta=54.2;



/*The following code is to set up PID constants*/

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
float ModuloFloat(float x,int y);

/*Constants/Struct for Line Align*/

struct AlignSensor{
int n;                               // for n=7 the value =(4+2+3)*10/3=30;  for n=5 the value=(0+1+2+3)*10/4=15; for n=4 the value=15
//int a[6];//={2,3,4,5,6,7,8};
byte ReceivedValue[6];
float val;
float sum;
float outputAS;
int tempAS;
int count;
float Previousvalue;};

const int a[5]={49,47,43,39,37};
const int lsscalPin=53;
int rpmrotate=80;
/* PS2 interfacing */
float distAnalogleft,distAnalogright;

#include <PS2X_lib.h>  //for v1.6

/******************************************************************
 * set pins connected to PS2 controller:
 *   - 1e column: original 
 *   - 2e colmun: Stef?
 * replace pin numbers by the ones you use
 ******************************************************************/
#define PS2_DAT        50  //14    
#define PS2_CMD        51  //15
#define PS2_SEL        31  //16
#define PS2_CLK        52  //17

/******************************************************************
 * select modes of PS2 controller:
 *   - pressures = analog reading of push-butttons 
 *   - rumble    = motor rumbling
 * uncomment 1 of the lines for each mode selection
 ******************************************************************/
//#define pressures   true
#define pressures   false
//#define rumble      true
#define rumble      false

PS2X ps2x; // create PS2 Controller Class

double xSquare_leftdistance=0,ySquare_leftdistance=0,xSquare_rightdistance=0,ySquare_rightdistance=0,xCircle_leftdistance=0,yCircle_leftdistance=0,xCircle_rightdistance=0,yCircle_rightdistance=0;
double LeftAnalogTheta=0,RightAnalogTheta=0;//w1=0,w2=0;
float LeftAnalogDistance=0,RightAnalogDistance=0;//r1=0,r2=0; //r1-Leftanalogdist w1-Leftanalogtheta

//right now, the library does NOT support hot pluggable controllers, meaning 
//you must always either restart your Arduino after you connect the controller, 
//or call config_gamepad(pins) again after connecting the controller.

int error = 0;
byte type = 0;
byte vibrate = 0;


/* Variables etc.*/

struct gain IMUgain,Compassgain,Aligngain, AligngainNormal;
struct gain *pIMUgain = &IMUgain, *pCompassgain = &Compassgain, *pAligngain = &Aligngain,*pAligngainNormal=&AligngainNormal;

struct AlignSensor AlignmentSensor;
struct AlignSensor *pAlignmentSensor=&AlignmentSensor;

const float rad = 1;
const float pi = 3.14159;

int Stopflag = 1, Linearflag = 0, rotateFlag=1; //rotateFlag 1 IMU, 0 Analog

wheel wheela = {0, 0, anglea, rpmmax, pinpwma, pinaa, pinab,0}, wheelb = {0, 0, angleb, rpmmax, pinpwmb, pinba, pinbb,0}, wheelc = {0, 0, anglec, rpmmax, pinpwmc, pinca, pincb,0};
wheel *pwheel[3]={&wheela,&wheelb,&wheelc};

int lineFollowFlag=0,upcontinuousFlag=0,downcontinuousFlag=0;

float omegaControl;
float omega; 
float rpm;
int semiAutonomousFlag = 0;
float compassOffset;
int Rpm;

void setup() {                                                                                           
  Serial.begin(9600);
  Wire.begin();
  SerialPS2.begin(9600);
  initDriving(pwheel);    // 0.95    0.1
  initLineAlign();
  PIDinit(0.8 ,0,0.00,0,-40,40,pAligngain);
  PIDinit(0.75,0.1,0,0,-40,40,pAligngainNormal);
 // PIDinit(20,0,0,0,-255,255, pCompassgain); //p=40
  initPS2();
  //CompassInit();
  //GetCompassHeading(1);
}
float PWMfactor=2;
void loop(){
      getPS2value();    
      scalePS2value();
      PS2executePressed();      
      //GetCompassHeading(2);
      //Serial.println(compassHeading);;
      //Serial.println("omegaControl  "+String(omegaControl)); 
      LeftAnalogTheta=((int)(LeftAnalogTheta+180)%360); 
      //Serial.println("compassHeading  "+String(compassHeading));
     
      if((LeftAnalogTheta < 20 && LeftAnalogTheta >340) || (LeftAnalogTheta < 200 && LeftAnalogTheta >160)) 
        PWMfactor=2; // Forward/Backward  
        
      else if((LeftAnalogTheta < 110 && LeftAnalogTheta > 70) || (LeftAnalogTheta < 290 && LeftAnalogTheta > 250)) 
        PWMfactor=1; //Left/Right
      else
        PWMfactor=2;

      if(rotateFlag){
        if(RightAnalogTheta >= 270 || RightAnalogTheta <= 90)
          omegaControl = -RightAnalogDistance;//Rotate Left
        else if(RightAnalogTheta >= 90 && RightAnalogTheta <= 270)
          omegaControl = RightAnalogDistance;//Rotate Rightu
      }
      else{
        omegaControl = PID(compassHeading, pCompassgain);
        omegaControl/=255;
      }
      if(Stopflag==0){  
         if(semiAutonomousFlag){
           lineAlign();
         }
         else{ 
         calcRPM(omegaControl*rpmrotate,LeftAnalogTheta,LeftAnalogDistance*rpm,pwheel);          
         }     
      }
      else if(Stopflag==1){
        for(int k =0;k<3;k++){
          pwheel[k]->rpm=0;
        }
      }
      startMotion(pwheel);
      //Serial.println("compassHeading"+String(compassHeading));
      //Serial.println("prevcompassHeading"+String(prevcompassHeading));
      //Serial.println("pgain->required"+String(pCompassgain->required));
      
} 

 

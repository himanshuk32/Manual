 /*
 * TO DO:
 * Check if IMU giving readings- Timer interrupt to calc loop counter value, if same then rese
 * Limit ang_velocity
 * Different PID rotation!
 */
 
// 0 //Compass

 
//#include <TimerOne.h>
#include <Wire.h>
#include <LSM303.h>
#include <SPI.h>
#include <DueTimer.h>
#include <DuePWM.h>

//#define rpmmax 300
#define Time 0.2
#define MaxPwm 255
#define GearRatio 0.87
#define desiredpwm(x) (x*255.0)/320.0  
#define PWM_FREQ1 2500
DuePWM pwm(PWM_FREQ1,3000);

const int omegaMode=1;
LSM303 compass;

void TicksA();
//The following code is for setting up IMU constants
//**********************************************************************
#define Gyro_Gain_X 0.07 //X axis Gyro gain
#define Gyro_Gain_Y 0.07 //Y axis Gyro gain
#define Gyro_Gain_Z 0.07 //Z axis Gyro gain
#define Gyro_Scaled_X(x) ((x)*ToRad(Gyro_Gain_X)) //Return the scaled ADC raw data of the gyro in radians for second
#define Gyro_Scaled_Y(x) ((x)*ToRad(Gyro_Gain_Y)) //Return the scaled ADC raw data of the gyro in radians for second
#define Gyro_Scaled_Z(x) ((x)*ToRad(Gyro_Gain_Z)) //Return the scaled ADC raw data of the gyro in radians for second
int SENSOR_SIGN[9] = {1,1,1,-1,-1,-1,1,1,1}; //Correct directions x,y,z - gyro, accelerometer, magnetometer

// accelerometer: 8 g sensitivity
// 3.9 mg/digit; 1 g = 256
#define GRAVITY 256  //this equivalent to 1G in the raw data coming from the accelerometer

#define ToRad(x) ((x)*0.01745329252)  // *pi/180
#define ToDeg(x) ((x)*57.2957795131)  // *180/pi

#define M_X_MIN -2292
#define M_Y_MIN -2374
#define M_Z_MIN -2605
#define M_X_MAX +2672
#define M_Y_MAX +2674
#define M_Z_MAX +2255 

#define Kp_ROLLPITCH 0.02
#define Ki_ROLLPITCH 0.00002
#define Kp_YAW 1.2
#define Ki_YAW 0.00002

/*For debugging purposes*/
//OUTPUTMODE=1 will print the corrected data,
//OUTPUTMODE=0 will print uncorrected data of the gyros (with drift)
#define OUTPUTMODE 1

#define PRINT_DCM 0     //Will print the whole direction cosine matrix
#define PRINT_ANALOGS 0 //Will print the analog raw data
#define PRINT_EULER 1   //Will print the Euler angles Roll, Pitch and Yaw
//min: { -2308,  -2293,  -1676}    max: { +27        52,  +2620,  +1846}

long timer=0;   //general purpuse timer
long timer_old;
long timer24=0; //Second timer used to print values
int AN[6]; //array that stores the gyro and accelerometer data
int AN_OFFSET[6]={0,0,0,0,0,0}; //Array that stores the Offset of the sensors

int gyro_x;
int gyro_y;
int gyro_z;
int accel_x;
int accel_y;
int accel_z;
int magnetom_x;
int magnetom_y;
int magnetom_z;
float c_magnetom_x;
float c_magnetom_y;
float c_magnetom_z;
float MAG_Heading;

float Accel_Vector[3]= {0,0,0}; //Store the acceleration in a vector
float Gyro_Vector[3]= {0,0,0};//Store the gyros turn rate in a vector
float Omega_Vector[3]= {0,0,0}; //Corrected Gyro_Vector data
float Omega_P[3]= {0,0,0};//Omega Proportional correction
float Omega_I[3]= {0,0,0};//Omega Integrator
float Omega[3]= {0,0,0};

// Euler angles
float roll;
float pitch;
float yaw;

float errorRollPitch[3]= {0,0,0};
float errorYaw[3]= {0,0,0};

unsigned int counter=0;
byte gyro_sat=0;
float G_Dt=0.02;    // Integratio n time (DCM algorithm)  We will run the integration loop at 50Hz if possible
float CompassHeadingOffset=0;
float CompassHeading;


  
//*********************************************************************










//The following code is to set up driving constants/structs

//******************************************************
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
const int rpmmax = 300;
const int pinpwma = 12; //Channel1
const int pinpwmb = 7;  //Channel2
const int pinpwmc = 4;  //Channel1
const int pinaa = 13;
const int pinab = 11;     //ALL WHEELS ARE TRYING TO ROTATE BOT CLOCKWISE WHEN A HIGH AND B LOW
const int pinba = 10;
const int pinbb = 6;
const int pinca = 5;
const int pincb = 2;
const float HeadTheta=54.2;
//*****************************************************************



//The following code is to set up PID constants
//***************************************************


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
//****************************************************************




//Constants for manual driving 
//*******************************************************************
  
  
  
//  const int slave=41;
//  SPISettings settingA(500000,LSBFIRST,SPI_MODE3);
//  volatile char data_array[21];
//  //SPISettings settingA(500000,LSBFIRST,SPI_MODE3);
//  char PS2_POLLBUTTON[9];
//  enum {released,pressed};
//  enum {select,leftStick, rightStick,start, up, right, down, left}; //3rd byte from ps2 controller
//  enum {leftFront2, rightFront2, leftFront1, rightFront1, triangle_up, circle_right, cross_down, square_left}; // 4th byte from ps2 controlle uint8_t x,y,a,b,c,d;
//  enum {sel_Button,lStick,rStick,start_Button,up_Button,right_Button,down_Button,left_Button,l2_Button,r2_Button,l1_Button,r1_Button,triangle_Button,circle_Button,cross_Button,square_Button,crossandl2,crossandr2};//Make array of buttons
//  uint8_t x,y,rv,lh,lv,rh;
//  float distAnalogleft,distAnalogright;
//  double thetaAnalogleft,thetaAnalogright;
//  double xSquare_leftdistance,ySquare_leftdistance,xCircle_leftdistance,yCircle_leftdistance,xSquare_rightdistance,ySquare_rightdistance,xCircle_rightdistance,yCircle_rightdistance;;
//  bool CurrButtonState[18]={released};

float distAnalogleft,distAnalogright;
//  bool CurrButtonState[18]={released};
  #include <PS2X_lib.h>  //for v1.6

/******************************************************************
 * set pins connected to PS2 controller:
 *   - 1e column: original 
 *   - 2e colmun: Stef?
 * replace pin numbers by the ones you use
 ******************************************************************/
#define PS2_DAT        13  //14    
#define PS2_CMD        11  //15
#define PS2_SEL        41  //16
#define PS2_CLK        12  //17

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

//****************************************************************************


//Constants for Y Positional Encoder
//*******************************************************************************
typedef struct encoder{
  int channelA;
  int channelB;
  long int count; 
  long int previousCount;
  int rpm; 
  int ppr;
  void (*Tickfunction)(void);
  float diameter;
} encoder;


//************************************************************************************


struct gain IMUgain,Compassgain, Adjustgain;
struct gain *pIMUgain = &IMUgain, *pCompassgain = &Compassgain, *pAdjustgain = &Adjustgain;
const float rad = 1;
const float pi = 3.14159;
int Stopflag = 1, Linearflag = 0;
wheel wheela = {0, 0, anglea, rpmmax, pinpwma, pinaa, pinab,0}, wheelb = {0, 0, angleb, rpmmax, pinpwmb, pinba, pinbb,0}, wheelc = {0, 0, anglec, rpmmax, pinpwmc, pinca, pincb,0};
wheel *pwheel[3]={&wheela,&wheelb,&wheelc};
encoder encoderA={1,1,0,0,0,1024,&TicksA,5.6};
encoder * pencoderA = &encoderA;     

float OmegaControl;
void setup() {                                                                                           
  Serial.begin(9600);
  Wire.begin();
  initDriving(pwheel);
//  CompassInit();
//  IMUinit();                //Initialise IMU
//  SetIMUOffset();              //Take initial readings for offset  
//  PS2init();
//  if(omegaMode==0)
//  {
//    PIDinit(2.5,0.5,0 ,0,-255,255, pCompassgain);
//  }
//  if(omegaMode==1)
//  {
//    PIDinit(2.5,0.5,0 ,0,-255,255, pIMUgain);
//    timer=millis();           //save ccurrent time in timer ffor gyro integration
//    delay(20);
//    int counter=0;
//  }
//  PIDinit(0,0,0 ,0,-255,255, pAdjustgain);
//  initPosEncoder(pencoderA);  
    OmegaControl = 0;
    initPS2();
    if(error == 1) //skip loop if no controller found
    Serial.print("No controller found"); 
//    pwm.setFreq1(PWM_FREQ1);
//    pwm.pinFreq1( 6 );  // Pin 6 freq set to "pwm_freq1" on clock A
//    pwm.pinFreq1( 7 );  // Pin 7 freq set to "pwm_freq2" on clock B
//    pwm.pinFreq1( 8 );  // Pin 8 freq set to "pwm_freq2" on clock B
}
int yawdeg;
void loop(){
      if(type == 2)
      getPS2value();
      else
      getVibratevalue();
      
      scalePS2value();
      PS2executePressed();
      
      LeftAnalogTheta=((int)(LeftAnalogTheta+180)%360); 
//      if(omegaMode==0){
//      OmegaControl=CompassHeadControl(HeadTheta,pCompassgain);
//      }
//      if(omegaMode==1)
//      {
//      OmegaControl = HeadControl(pIMUgain->required,pIMUgain);
//      }
      Serial.println("Control"+String(OmegaControl)+"Yaw" +String(yawdeg)+"Required"+String(pIMUgain->required)); 
      if(Stopflag==0){
        Serial.print(" Started ");
//        if(Linearflag)
//         adjustVertical(10,pAdjustgain,pencoderA);
//        else
         calcRPM(0,LeftAnalogTheta,LeftAnalogDistance*rpmmax,pwheel);
      }
      else if(Stopflag==1){
        Serial.println(" Stopped ");
        for(int k =0;k<3;k++){
          pwheel[k]->rpm=0;
        }
      }
      startMotion(pwheel);
} 



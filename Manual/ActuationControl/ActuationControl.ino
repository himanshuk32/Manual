#include <Wire.h>
int current=0,desired=0,flag=0;
int ta1[]={5,6,7,8};
int ta2[]={1,2,3,4};
int ta1count=0, ta2count=0;
enum direction {forward, backward};
enum direction motordir;
int desiredpos, currentpos;
const int StrikerPin = 9;
const int liftRackPinA = 11;
const int liftRackPinB = 12;
const int ClutchWallPin = 13;
const int channelB = 4;
const int pulse = 2;
const int channelA = 3;
const int motorA=5,motorB=7,motorPWM=6;
const int rackSpeed=80;
const int rackSlowSpeed=40;
volatile bool dataReceived = false;
volatile char data;
int wallFlag =0 ;
int liftFlag=0;
int LSB=0,MSB=0;
bool CheckSum=0;
int Sum=0;
int PreviousSum=0;
void setup() {
  Wire.begin(8);
  Serial.begin(9600);
  Wire.onReceive(getData);
  initRackMovement();
  initRackLift();
  initWallClutch();
  initStriker();
}


void loop() {
      //if(dataReceived){
      //executeI2Creceived(data);
      //dataReceived=false;
      //}
      //moveRack();
      
//    if(Serial.available()){
////     desired=atoi(Serial.readString().c_str());
////     desiredpos=desired*2-1;
//        char dir = Serial.read();
//        if(dir=='a'){
//          desired=ta1[ta1count%4];
//           ta1count++;
//        }
//        if(dir=='b'){
//          desired=ta2[ta2count%4];
//          ta2count++;
//         }
//      desiredpos=desired*2-1;
//      }
  
}


void initStriker()
{
 pinMode(StrikerPin,OUTPUT);//for first pneumatic 
}
void strike()
{
  digitalWrite(StrikerPin,HIGH);
  delay(1000);
  digitalWrite(StrikerPin,LOW);
}
void initWallClutch()
{
  pinMode(ClutchWallPin,OUTPUT);//for second pneumatic
}
void clutchWall()
{
  digitalWrite(ClutchWallPin,HIGH);
}
void unclutchWall()
{
  digitalWrite(ClutchWallPin,LOW);
}
void initRackLift()
{
  pinMode(liftRackPinA,OUTPUT);//for third pneumatic
  pinMode(liftRackPinB,OUTPUT);
}
void liftRack()
{
  digitalWrite(liftRackPinA,HIGH);
  delay(500);
  digitalWrite(liftRackPinA,LOW);
}
void releaseRack()
{
  digitalWrite(liftRackPinB,HIGH);
  delay(500);
  digitalWrite(liftRackPinB,LOW);
}

void initRackMovement(){
  pinMode(channelA,INPUT);   //INPUT PULSE
  pinMode(pulse,INPUT);
  pinMode(channelB,INPUT);
  pinMode(motorA,OUTPUT);  //MOTOR DIRECTION
  pinMode(motorB,OUTPUT);
  pinMode(motorPWM,OUTPUT); 
  analogWrite(motorPWM,rackSpeed);  
  currentpos=0;
  attachInterrupt(digitalPinToInterrupt(pulse),changePosition,CHANGE);
  attachInterrupt(digitalPinToInterrupt(channelA),checkDirection,CHANGE);
  attachInterrupt(digitalPinToInterrupt(channelB),checkDirection,CHANGE);
}

void moveRack(){
  if(abs(currentpos-desiredpos)==1)
  {
    analogWrite(motorPWM,rackSlowSpeed);
  }
  else
  {
    analogWrite(motorPWM,rackSpeed);
  }   
  if(currentpos>desiredpos){
    digitalWrite(motorA,HIGH);
    digitalWrite(motorB,LOW);
   }
  else if(currentpos<desiredpos){
    digitalWrite(motorA,LOW);
    digitalWrite(motorB,HIGH);
  }
  if(currentpos==desiredpos){
    digitalWrite(motorA,HIGH);
    digitalWrite(motorB,HIGH);
  } 
  current = (currentpos+1)/2;
  desired = (desiredpos+1)/2; 
  Serial.println("POS:"+String(current)+"des:"+String(desired)+"des");
}


void executeI2Creceived(volatile char data){
  if(data=='l')
  {
    //Move Linear left
    Serial.println('l');
    desiredpos-=2;
  }
    
  if(data=='r')
  {
    //Move linear right
    Serial.println('r');
    desiredpos+=2;
  }
  
  if(data=='s')
  {
    //Strike
    Serial.println('s');
    strike();
  }
  
  if(data=='w')
  {
    //Toggle wall clutch
    Serial.println('w');
    wallFlag^=1;
    if(wallFlag)
    clutchWall();
    else
    unclutchWall();  
  }
  
  if(data=='x')
  {
    //toggle rack lift
    Serial.println('x');
    liftFlag^=1;
    if(liftFlag)
    liftRack();
    else
    releaseRack();
  }
}


void getData(int howMany){
  //dataReceived = true;
  data = Wire.read();
  Serial.println(data);
}


void changePosition(){
    if(motordir == forward){
      currentpos+=1;
      } 
     else if(motordir == backward)
     {
      currentpos-=1; 
     }
}

void checkDirection(){
  MSB = digitalRead(channelB);
  LSB = digitalRead(channelA);
  Sum = (MSB<<1)|(LSB);
  CheckSum =(PreviousSum<<2)|Sum;
  if(CheckSum==0b1011 ||CheckSum==0b1101 ||CheckSum==0b0100 ||CheckSum==0b0010)
  motordir=forward; 
  if(CheckSum==0b0111 ||CheckSum==0b1110 ||CheckSum==0b0001 ||CheckSum==0b1000)
  motordir=backward;
  PreviousSum=Sum;
}


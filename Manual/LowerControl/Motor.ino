void initMotor(MOTOR *motor){
  pinMode(motor->pin1,OUTPUT);
  pinMode(motor->pin2,OUTPUT);
  pinMode(motor->pwmpin,OUTPUT);
}
float drivewheel(float output,int maxval,MOTOR *motor, int flag){
  if(output>maxval){
    output=maxval;
  }
  if(output<-maxval){
    output=-maxval;
  }
  if(output>0)
  {
    digitalWrite(motor->pin1,HIGH);
    digitalWrite(motor->pin2,LOW);
  }
  else if(output<0)
  {
    output=-1*output;
    digitalWrite(motor->pin1,LOW);
    digitalWrite(motor->pin2,HIGH);
  }
  else{
//    digitalWrite(motor->pin1,HIGH);
//    digitalWrite(motor->pin2,HIGH);  
  }
  if(flag== 1)
  {
    digitalWrite(motor->pin1,HIGH);
    digitalWrite(motor->pin2,HIGH);  
  }
  //pwm.pinDuty( motor->pwmpin, output );  // 50% duty cycle on Pin 6
    
  analogWrite(motor->pwmpin,output);
  return output;
}



float HeadControl(float head,gain* IMUgain){
  GetIMUReading();
  yawdeg=(int(ToDeg(yaw)+360))%360;    
  float OmegaControl = PID(yawdeg,IMUgain);
  return OmegaControl;
}

float CompassHeadControl(float head,gain* Compassgain){
  GetCompassHeading();    
  float CompassControl = PID(CompassHeading,Compassgain);
  return CompassControl;
}

 

void PIDinit(float kp, float kd, float ki,float required,float minControl, float maxControl, struct gain * gain) {
    gain->kp = kp;
    gain->kd = kd;
    gain->ki = ki;
    gain->required=required;
    gain->minControl = minControl;
    gain->maxControl = maxControl;
    gain->error=0;
    gain->previousError=1;
    gain->derivativeError=0;
    gain->integralError=0;
  }

float PID(float current, struct gain * gain) {
    gain->error = gain->required - current; //WHen car left, right sensors off, take right control positive
    gain->integralError += gain->error;
    gain->derivativeError = gain->previousError - gain->error; ///*********************
    gain->previousError = gain->error;

    float control = gain->kp * gain->error + gain->ki * gain->integralError + gain->kd * gain->derivativeError;

//    if (control > gain->maxControl)
//      control = gain->maxControl;
//    if (control < gain->minControl)
//      control = gain->minControl;
    return control;
}



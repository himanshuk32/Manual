void TransmitRPM(wheel ** whee)
{
  Wire.beginTransmission(8);
        for(int y =0;y<3;y++)
        {
         int temp = ((int)whee[y]->rpm+400);
         Serial.println(temp);
        for(int r=0;r<3;r++)
        {
          Wire.write(temp%10);
          temp/=10;
        } 
        }
  Wire.endTransmission();
 }

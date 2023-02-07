fn I2Cscan()
{
// scan for i2c devices
  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for(address = 1; address < 127; address++ ) 
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");

      nDevices++;
    }
    else if (error==4) 
    {
      Serial.print("Unknow error at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
    
}

    // for mahony
    // Kp = 30.0;
     //Ki = 0.0;


    
     newTime = micros();
     deltaT = newTime - oldTime;
     oldTime = newTime;
     deltaT = fabs(deltaT * 0.001 * 0.001);

 

 fn no_filter(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float* q) {
     let q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];  // variable for readability
     q[0] += 0.5 * (-q1 * gx - q2 * gy - q3 * gz) * deltaT;
     q[1] += 0.5 * (q0 * gx + q2 * gz - q3 * gy) * deltaT;
     q[2] += 0.5 * (q0 * gy - q1 * gz + q3 * gx) * deltaT;
     q[3] += 0.5 * (q0 * gz + q1 * gy - q2 * gx) * deltaT;
     let norm = fisr(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
     q[0] *= norm;
     q[1] *= norm;
     q[2] *= norm;
     q[3] *= norm;
 }


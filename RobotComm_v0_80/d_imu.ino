// gyro
#include <Wire.h> // for I2C
#include <L3G.h> // for gyro

#define GYRO_GAIN_YAW 0.07 // full scale sensitivity is 2000 dps = 70 mdps/digit
#define GYRO_LOOP_PERIOD 20 // this determines how long between readings and also used for 
                        // integrating the degress/sec data to degrees

L3G gyro;

double gyroYaw, gyroZBaseline, totalYaw = 0, gyroBaselinePoints = 0, gyroBaselineMaxLength = 32;

boolean Gyro_Init()
{
  if (!gyro.init())
  {
     SERIAL_PORT.println("Gyro not present");
     return false;
  }
  gyro.writeReg(L3G_CTRL_REG1, 0x0F); // normal power mode, all axes enabled, 100 Hz
  // Low byte, bit 3 = power state, bits 0,1,2 = Y,X,Z axis enables
  gyro.writeReg(L3G_CTRL_REG4, 0x20); // 2000 dps full scale
  // High byte: (00: 250 dps; 01: 500 dps; 10: 2000 dps; 11: 2000 dps)
  
  // Now turn on the L3G's gyro and places it in normal mode:
  gyro.enableDefault(); 
  
  // gather some baseline data:
  updateGyroBaseline(gyroBaselineMaxLength);
  //SERIAL_PORT.println("Gyro initialized, baseline data acquired."); 
  return true;
}

double readGyro()
{
  gyro.read();
  gyroYaw = (gyro.g.z - gyroZBaseline) * GYRO_GAIN_YAW;
  return gyroYaw;

}

double updateYaw(unsigned long deltaT)
{
  double integrationTime = ((double) deltaT) / 1000.; // convert long ms to double seconds
  gyro.read();
  totalYaw += ((gyro.g.z - gyroZBaseline) * GYRO_GAIN_YAW) * integrationTime;
  return totalYaw;
}

// for a single baseline datapoint, this routine takes around 1 to 2 msec
// for more than one, it takes about 22 msec per point, due to the 20 msec delay time used
// to space out the data for meaningful variations
void updateGyroBaseline(int numPoints)
{
   double gyroZCumulative = 0;
   if (numPoints < 1 || numPoints > gyroBaselineMaxLength) numPoints = gyroBaselineMaxLength;
   gyroBaselinePoints -= numPoints;
   if (gyroBaselinePoints < 0) gyroBaselinePoints = 0; // make an entirely new baseline
   for(int i=0; i < numPoints; i++)    // We take some readings...
   {
    gyro.read();
    gyroZCumulative += gyro.g.z;
    if (i + 1 < numPoints) delay(20); // don't delay on the last pass through the loop
   }
   gyroZBaseline = (gyroZCumulative + (gyroZBaseline * gyroBaselinePoints))
                      / ((double)(numPoints + gyroBaselinePoints));
   gyroBaselinePoints = numPoints + gyroBaselinePoints;
   
   //  // dont let the baseline get too old or long
   if (gyroBaselinePoints > gyroBaselineMaxLength) gyroBaselinePoints = gyroBaselineMaxLength;
   if (numPoints > 1)  // don't print the results of the loop updates
   {
     SERIAL_PORT.print("Gyro baseline yaw = ");
     SERIAL_PORT.print(gyroZBaseline * GYRO_GAIN_YAW); 
     SERIAL_PORT.println(" degrees (positive is to the right)");
   }
}

void resetYaw()
{
  totalYaw = 0;
}

int gyroTurnDelay(int degrees)
{
      SERIAL_PORT.print("in gyro turn delay for degrees = ");
      SERIAL_PORT.println(degrees);
      double cumulativeYaw = 0, timeOut = 10, totalT;
      long previousTime = millis();
      // delay until we reach the specified number of degrees
      // or reach the timeOut value (in seconds)
      while (abs(cumulativeYaw) < degrees && totalT < timeOut ) 
      {
        delay(20);
        gyro.read();
        long currentTime = millis();
        double deltaT = ((double) (currentTime - previousTime)) / 1000.; // convert long ms to double seconds
        previousTime = currentTime; 
        cumulativeYaw += (gyro.g.z - gyroZBaseline) * GYRO_GAIN_YAW * deltaT;
        totalT += deltaT;
        //SERIAL_PORT.print("cumulativeYaw, deltaYaw, deltaT = ");
        //SERIAL_PORT.print(cumulativeYaw);
        //SERIAL_PORT.print(", ");
        //SERIAL_PORT.print((gyro.g.z - gyroZBaseline) * GYRO_GAIN_YAW * deltaT);
        //SERIAL_PORT.print(", ");
        //SERIAL_PORT.println(deltaT);
      }
      coast();
      SERIAL_PORT.print("When coast command issued, yaw was = ");
      SERIAL_PORT.print(cumulativeYaw);
      // give it a moment to stop, monitor yaw during this time
      for (int i=0; i < 10; i++)
      {
        delay(20);
        gyro.read();
        long currentTime = millis();
        double deltaT = ((double) (currentTime - previousTime)) / 1000.; // convert long ms to double seconds
        previousTime = currentTime; 
        cumulativeYaw += (gyro.g.z - gyroZBaseline) * GYRO_GAIN_YAW * deltaT;
        totalT += deltaT;
        //SERIAL_PORT.print("cumulativeYaw, deltaYaw, deltaT = ");
        //SERIAL_PORT.print(cumulativeYaw);
        //SERIAL_PORT.print(", ");
        //SERIAL_PORT.print((gyro.g.z - gyroZBaseline) * GYRO_GAIN_YAW * deltaT);
        //SERIAL_PORT.print(", ");
        //SERIAL_PORT.println(deltaT);
      }
    
      totalYaw += cumulativeYaw;  // total degrees
      SERIAL_PORT.print("Yaw this turn, totalYaw = ");
      SERIAL_PORT.print(cumulativeYaw);
      SERIAL_PORT.print(", ");
      SERIAL_PORT.println(totalYaw);
      SERIAL_PORT.print("Yaw baseline = ");
      SERIAL_PORT.print(gyroZBaseline * GYRO_GAIN_YAW);
     
      return (int) (totalT * 1000.); // return total delay
}


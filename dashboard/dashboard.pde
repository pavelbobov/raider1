RawSensorData sensors = new RawSensorData();

void setup()
{
  size(640, 480);
  frameRate(30);
}

void draw()
{
  background(0,0,64);
  text(sensors.toString(), 20, 20);
  translate(width/2, height/2);
  scale(4.0);
  fill(255, 0, 0);
  arc(0, 0, 19, 72, radians(110), radians(430), CHORD);
}

class RawSensorData
{
  public GPS gps = new GPS();
  public WindVane wind = new WindVane();   
  public MPU mpu = new MPU(); 
  public Motor rudder = new Motor();
  public Motor sail = new Motor();
  
  public String toString()
  {
    return gps.toString() + "," + wind.toString() + "," + mpu.toString() + "," + rudder.toString() + "," + sail.toString();
  }
}

class GPS
{
  //$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A
  public String NMEA = "";
  
  public String toString() 
  {
    return NMEA;
  }
}

//Wind vane data
class WindVane
{
  //Wind direction 0-15
  public byte direction; 
  //Wind speed 0-511 with 0.1 m/sec units
  public int speed;
  
  public String toString()
  {
    return str(direction) + "," + str(speed);
  }
}

//Motion processing unit data
class MPU 
{
  //Raw accelerometer data
  public float ax, ay, az; 
  //Raw gyroscope data 
  public float gx, gy, gz;
  //Raw magnetometer data
  public float mx, my, mz;
  //Temperature in degrees C
  public float temperature;
  
  public String toString()
  {
    return str(ax) + "," + str(ay) + "," + str(az) + "," + str(gx) + "," + str(gy) + "," + str(gz) + "," + str(mx) + "," + str(my) + "," + str(mz) + "," + str(temperature); 
  }
}

//Rudder and sail control motors
class Motor
{
  //Motor speed 0-1023 
  public int speed;
  //Turn angle 0-1023
  public int angle;
  //Current sensing 0-1023
  public int current;
  
  public String toString()
  {
    return str(speed) + "," + str(angle) + "," + str(current);
  }
}
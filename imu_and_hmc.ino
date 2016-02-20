#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050.h>
#include <Servo.h>

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelgyro;

#define address 0x1E

int16_t ax, ay, az;
int16_t gx, gy, gz;
int x,y,z; //triple axis data
double Roll;
double Pitch;
double Yaw;

int servoPin = 10;
#define MAX_FDIR 1900
#define MIN 1500
#define MAX_RDIR 1100
Servo servo;

#define LED_PIN 13
bool blinkState = false;

void setup()
{ 
  // Initializing of IMU 
  Wire.begin();
  Serial.begin(9600);
  Serial.println("Initializing I2C devices...");
  accelgyro.initialize();

  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
   delay(500);
  pinMode(LED_PIN, OUTPUT);
  
  // Initializing of Magnetometer
  Serial.begin(9600);
  Wire.begin();
  digitalWrite(22,HIGH);
  //Put the HMC5883 IC into the correct operating mode
  Wire.beginTransmission(address); //open communication with HMC5883
  Wire.write(0x02); //select mode register
  Wire.write(0x00); //continuous measurement mode
  Wire.endTransmission();
   servo.attach(servoPin);
   servo.writeMicroseconds(1500);// send" stop" signal
   delay(1000);
    
  servo.attach(servoPin);
}

void loop() 
{
  static int signal = 1000;
  
  // read raw accel/gyro measurements from device
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  Roll = atan2(ax, az) * 180/PI;
  Pitch = atan2(ax, sqrt(ay*ay + az*az)) * 180/PI;
  Yaw=atan2(ay,ax)* 180/PI;
  
  //Tell the HMC5883L where to begin reading data
  Wire.beginTransmission(address);
  Wire.write(0x03); //select register 3, X MSB register
  Wire.endTransmission();
  
 
 //Read data from each axis, 2 registers per axis
  Wire.requestFrom(address, 6);
  if(6<=Wire.available())
  {
    x = Wire.read()<<8; //X msb
    x |= Wire.read(); //X lsb
  
    z = Wire.read()<<8; //Z msb
    z |= Wire.read(); //Z lsb
    y = Wire.read()<<8; //Y msb
    y |= Wire.read(); //Y lsb

  
    if(Roll>70) 
  {
    static uint32_t timer = millis();
    static bool reverseDir = true;
    
    if(millis() - timer >= 1000) 
    {
      timer = millis();
      //Serial.println(signal);
      
      if(signal > MAX_FDIR) 
      {
        reverseDir = false;
      } 
      else if (signal < MAX_RDIR) 
      {
        reverseDir = true;
      }
    
    if(reverseDir == true) 
    {
      signal+=100 ;
    }
    else 
    {
      signal -=100; 
    }
   }
   
  }
  else
  {
    servo.writeMicroseconds(1500);
  }
  servo.writeMicroseconds(signal);
  }
  Serial.print("a/g:\t");
  Serial.print("Roll"); Serial.print("\t");
  Serial.print(Roll); Serial.print("\t");
  Serial.print("Pitch"); Serial.print("\t");
  Serial.print(Pitch); Serial.print("\t");
  Serial.print("Yaw"); Serial.print("\t");
  Serial.println(Yaw);Serial.print("\t");
  Serial.print("gx");Serial.print("\t");
  Serial.print(gx);Serial.print("\t");
  Serial.print("gy");Serial.print("\t");
  Serial.print(gy);Serial.print("\t");
  Serial.print("gz");Serial.print("\t");
  Serial.print(gz);Serial.print("\t");
  Serial.print("signal");Serial.print("\t");
  Serial.print(signal); 
  delay(100);
  
  //Print out values of each axis
  Serial.print("x: ");
  Serial.print(x);
  Serial.print("  y: ");
  Serial.print(y);
  Serial.print("  z: ");
  Serial.println(z);
  delay(100);
  

  // blink LED to indicate activity
  blinkState = !blinkState;
  digitalWrite(LED_PIN, blinkState);
}

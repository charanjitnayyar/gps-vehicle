#include <TinyGPS.h>
#include <Wire.h>

#define address 0x1E //0011110b, I2C 7bit address of HMC5883
TinyGPS gps;

bool feedgps();

#define lmotora 3
#define lmotorb 4
#define rmotora 5
#define rmotorb 6

#define IRemitter 2
#define IRpin A0

int ambientIR;
int groundIR;
int IRvalue[10];
int tolerance = 5;

bool newData = false;

// Destination Coordinates

//float finaldestlat=15.39243;
//float finaldestlon=73.87515;
float finaldestlat;
float finaldestlon;


void setup() 
{
  Serial2.begin(9600);          // GPS Communication
  Serial.begin(9600);
  Serial1.begin(9600);          //Bluetooth Commn.
  delay(100);                   // Power up delay

  //
  pinMode(lmotora,OUTPUT);
  pinMode(lmotorb,OUTPUT);
  pinMode(rmotora,OUTPUT);
  pinMode(rmotorb,OUTPUT);

  pinMode(IRemitter, INPUT);
  digitalWrite(IRemitter, LOW);     //IR emitter setup as OFF

  Wire.begin();
  Wire.beginTransmission(address); //open communication with HMC5883
  Wire.write(0x01); //select CRB
  Wire.write(0x00); //Gain configuration gain=1370; +-0.88 gauss. Earth's magnetic field 0.25-0.65 gauss
  Wire.endTransmission();
  
  Wire.beginTransmission(address); //open communication with HMC5883
  Wire.write(0x02); //select mode register
  Wire.write(0x00); //continuous measurement mode
  Wire.endTransmission();

  Serial1.println("Enter the Latitude (15.39243 as 15392.43): ");
  while(Serial1.available()==0);
  finaldestlat=Serial1.parseFloat()/1000;

  Serial1.println("Enter the Longitude (73.87515 as 73875.15): ");
  while(Serial1.available()==0);
  finaldestlon=Serial1.parseFloat()/1000;  
}

void stopmotor()
{ 
analogWrite(lmotora,0);
analogWrite(lmotorb,0);

analogWrite(rmotorb,0);
analogWrite(rmotora,0);

}
void forward() 
{
analogWrite(lmotora,255);
analogWrite(lmotorb,0);

analogWrite(rmotorb,255);
analogWrite(rmotora,0);
delay(10000);
 
}

void left(float angle) 
{
 analogWrite(lmotora,150);
analogWrite(lmotorb,0);

analogWrite(rmotorb,255);
analogWrite(rmotora,0);
delay(angle*1000/24);
 
}

void right(float angle) 
{
 analogWrite(lmotora,255);
analogWrite(lmotorb,0);

analogWrite(rmotorb,150);
analogWrite(rmotora,0);
delay(angle*1000/24);
}

float findroute();

void loop() 
{
  
  bool newData = false;
  unsigned long chars;
  unsigned short sentences, failed;

  // For one second we parse GPS data and report some key values
  for (unsigned long start = millis(); millis() - start < 1000;)
  {
    while (Serial2.available())
    {
      char c = Serial2.read();
      
      if (gps.encode(c)) // Did a new valid sentence come in?
        newData = true;
    }
  }
  
  float flat, flon;  
    unsigned long age;
    gps.f_get_position(&flat, &flon, &age);
    flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6;
    flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6;


    float distance = sqrt(sq(finaldestlon-flon)+sq(finaldestlat-flat));     //Distance form the destination

    if (distance<0.000100)// 4m buffer
    {
    //forward();
    Serial1.print("motor Stop : Destination");    // Destination reached
    stopmotor();
    delay(5000);// to stop the motor for 5 seconds after reaching destination
    exit(0);
    }

    else
    {
      
      float tilt_angle=findroute();

      if(tilt_angle>0)
      {right(tilt_angle);Serial1.print(" Right ");}

      else
      {left((tilt_angle));Serial1.print(" Left ");}    //negative changed to positive tilt_angle
    }

    Serial1.println(" Forward");
    forward();
}    
  

 
bool feedgps()        
{
  while (Serial2.available())
  {
    if (gps.encode(Serial2.read()))
    { newData=true;
      return true;
    }
  }
  return false;
}


float findroute()
{ 
  bool newData = false;
  unsigned long chars;
  unsigned short sentences, failed;

  // For one second we parse GPS data and report some key values
  for (unsigned long start = millis(); millis() - start < 1000;)
  {
    while (Serial2.available())
    {
      char c = Serial2.read();
      // Serial.write(c); // uncomment this line if you want to see the GPS data flowing
      if (gps.encode(c)) // Did a new valid sentence come in?
        newData = true;
    }
  }
  float flat, flon;
  if (1)
  {
    
    unsigned long age;
    gps.f_get_position(&flat, &flon, &age);
    Serial1.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);
    Serial1.print("  ");
    Serial1.print(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);
    
  }

  int x,y,z; //triple axis data
  //Tell the HMC5883L where to begin reading data
 Wire.beginTransmission(address);
 Wire.write(0x03); //select register 3, X MSB register
 Wire.endTransmission();
 
 //Read data from each axis, 2 registers per axis
  Wire.requestFrom(address, 6);
  
  if(Wire.available()!=0)
  {
    x = Wire.read()<<8; //X msb   // x=Wire.read()*256;  //x=x+Wire.read();
    x |= Wire.read(); //X lsb
    z = Wire.read()<<8; //Z msb
    z |= Wire.read(); //Z lsb
    y = Wire.read()<<8; //Y msb
    y |= Wire.read(); //Y lsb
  }
  float heading = atan2(x,y);
   heading = heading * 180/M_PI;

    // Correct for when signs are reversed.
 if(heading < 0)
 {    heading = -heading;  }

  // Check for wrap due to addition of declination.
 else           
 {    heading = 360-heading;  }

  
  Serial1.print("   heading: ");
  Serial1.print(heading);
   


  float goto_angle = atan2((finaldestlon-flon),(finaldestlat-flat));
  goto_angle = goto_angle * 180/M_PI;
 
  // Correct for when signs are reversed.
  if(goto_angle < 0){    goto_angle = 360+goto_angle;  }

  // Check for wrap due to addition of declination.
    
  Serial1.print("  goto_angle: ");
  Serial1.print(goto_angle);

  float angle_tilt=goto_angle-heading;
  Serial1.print("  angle_tilt: ");
  Serial1.print(angle_tilt);

  return angle_tilt;
  
}

boolean readIR() 
{
  int sum = 0;
  for(int i=0;i<10;i++) {
    digitalWrite(IRemitter,LOW);           // turning the IR LEDs off to read the IR coming from the ambient
    delay(1);                                             // minimum delay necessary to read values
    ambientIR = analogRead(IRpin);  // storing IR coming from the ambient
    digitalWrite(IRemitter,HIGH);          // turning the IR LEDs on to read the IR coming from the obstacle
    delay(1);                                             // minimum delay necessary to read values
    groundIR = analogRead(IRpin);  // storing IR coming from the obstacle
    IRvalue[i] = ambientIR-groundIR;   // calculating changes in IR values and storing it for future average
    sum += IRvalue[i];
  }
  if(sum >= tolerance)              //the aggregrate difference between ambient and ground
  return true;                      //ground is present
  else
  return false;
 
}


//the program can be optimised, this is only the beta and is optimised only at an esponential value
//created by Nicola di Gruttola Giardino and Lorenzo Bianco on March 2018

#include <MPU6050.h>

#include <I2Cdev.h>
#include <BMP180.h>

#include <Wire.h>
#include <SD.h>
#include <dht.h>

dht DHT;
float praltitude;
#define sd 4
#define tmp 7

MPU6050 accelgyro;
int sec=0, mins=0, hour=0;
int16_t ax, ay, az;
int16_t gx, gy, gz;

//HMC5883L Digital Compass
const int hmc5883Address = 0x1E; //0011110b, I2C 7bit address for compass
const byte hmc5883ModeRegister = 0x02;
const byte hmcContinuousMode = 0x00;
const byte hmcDataOutputXMSBAddress = 0x03;

//The BMP180 Digital Barometer
BMP180 barometer;
// Store the current sea level pressure at your location in Pascals.
float seaLevelPressure = 101325;
int x,y,z,i; //triple axis data from HMC5883L.
bool led=0;

void setup() {
  Wire.begin();
  Serial.begin(9600);
  pinMode(sd, OUTPUT);
  pinMode(LED_BUILTIN,OUTPUT);
  digitalWrite(LED_BUILTIN,LOW);
  File dataFile = SD.open("newfile.txt", FILE_WRITE);
   Serial.println("Initializing I2C devices...");
   dataFile.print("Initializing I2C devices...");
    // verify connection
    Serial.println("Testing device connections...");
    dataFile.print("Testing device connections..."); 
    accelgyro.initialize();
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
    accelgyro.setI2CBypassEnabled(true);  //This sets the bypass so the HMC5883L gets a look in.  //This sets the bypass so the HMC5883L gets a look in.
    
    //Initialise the Digital Compass
    Wire.beginTransmission(hmc5883Address);  //Begin communication with compass
    Wire.write(hmc5883ModeRegister);  //select the mode register
    Wire.write(hmcContinuousMode); //continuous measurement mode
    Wire.endTransmission();
    //Initialise the BMP180 Barometer (and Temperature Sensor)
    barometer = BMP180();
    // We check to see if we can connect to the BMP180 sensor.
    if(barometer.EnsureConnected())
    {
      Serial.print("Connected to BMP180.");
      dataFile.print("Connected to BMP180.");
      digitalWrite(LED_BUILTIN,HIGH);
       // When we have connected, we reset the device to ensure a clean start.
      barometer.SoftReset();
      // Now we initialize the sensor and pull the calibration data.
      barometer.Initialize();
    }else
    { 
      Serial.print("No barometer found");
      dataFile.print("No barometer found");
      digitalWrite(LED_BUILTIN,LOW);
    }

  if (SD.begin(sd)) {
    Serial.println("Card Initialization Successful!");
    dataFile.print("Card Initialization Successful!");
    digitalWrite(LED_BUILTIN,HIGH);
  }else{
    Serial.println("Card Initialization Failed!");
    dataFile.print("Card Initialization Failed!");
    digitalWrite(LED_BUILTIN,LOW);
    }
  //Clock.setDOW(MONDAY);     // Set Day-of-Week
  //Clock.setTime(16, 34, 0);     // Set the time (24hr format)
  //Clock.setDate(23, 4, 2018);
  dataFile.close();
  delay(5000);
}

void loop() {
  File dataFile = SD.open("newfile.txt", FILE_WRITE);
  File file = SD.open("mpu6050.txt", FILE_WRITE);
  Serial.print(" ");
  dataFile.print(" ");
  int chk = DHT.read11(tmp);
  Serial.print("\nTemperature = ");
  dataFile.print("\nTemperature = ");
  Serial.print(DHT.temperature);
  dataFile.print(DHT.temperature);
  Serial.print("\nHumidity = ");
  dataFile.print("\nHumidity = ");
  Serial.println(DHT.humidity);
  dataFile.println(DHT.humidity);
  
    //Accessing the HMC5883L Digital Compass  
    //Tell the HMC5883L where to begin reading the data
    Wire.beginTransmission(hmc5883Address);
    Wire.write(hmcDataOutputXMSBAddress);  //Select register 3, X MSB register
    Wire.endTransmission();

    //Read data from each axis of the Digital Compass
    Wire.requestFrom(hmc5883Address,6);
    if(6<=Wire.available())
    {
      x = Wire.read()<<8; //X msb
      x |= Wire.read();   //X lsb
      z = Wire.read()<<8; //Z msb
      z |= Wire.read();   //Z lsb
      y = Wire.read()<<8; //Y msb
      y |= Wire.read();   //Y lsb    
    }

    int angle = atan2(-y,x)/M_PI*180;
    if (angle < 0)
    {
      angle = angle + 360;
    }
    
    Serial.print("Dir(deg):\t");
    Serial.print(angle); Serial.print("\t");
    dataFile.print("Dir(deg):\t");
    dataFile.print(angle); dataFile.print("\t");
  if(barometer.IsConnected)
    {
      long currentPressure = barometer.GetPressure();

      // Print out the Pressure.
      Serial.print("\nBMP180 P:\t");
      Serial.print(currentPressure);Serial.print("Pa");Serial.print("\t");
      dataFile.print("\nBMP180 P:  ");
      dataFile.print(currentPressure);dataFile.print("Pa");dataFile.print("\n");
      // Retrieve the current altitude (in meters). Current Sea Level Pressure is required for this.
      float altitude = barometer.GetAltitude(seaLevelPressure);
      //altitude;
      // Print out the Altitude.
      Serial.print("Alt:\t");
      Serial.print(altitude);Serial.print(" m a.s.l.");Serial.print("\t\n");
      dataFile.print("Alt:   ");
      dataFile.print(altitude);dataFile.print(" m a.s.l.");dataFile.print("\n");
      float speed1=(altitude-praltitude)/20;
      Serial.print("Speed:\t");Serial.println(speed1);
      dataFile.print("Speed:\t");dataFile.println(speed1);
      praltitude=altitude;
    }
  
  for(i=0;i<20;i++){
    File file = SD.open("mpu6050.txt", FILE_WRITE);
  // read raw accel/gyro measurements from device
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // these methods (and a few others) are also available
    //accelgyro.getAcceleration(&ax, &ay, &az);
    //accelgyro.getRotation(&gx, &gy, &gz);
        
    // display tab-separated accel/gyro x/y/z values
    Serial.print("\na/g:\t");
    Serial.print(ax); Serial.print("\t");
    Serial.print(ay); Serial.print("\t");
    Serial.print(az); Serial.print("\t");
    Serial.print(gx); Serial.print("\t");
    Serial.print(gy); Serial.print("\t");
    Serial.print(gz); Serial.print("\t");
    file.print("\na/g:\t");
    file.print(ax); file.print("\t");
    file.print(ay); file.print("\t");
    file.print(az); file.print("\t");
    file.print(gx); file.print("\t");
    file.print(gy); file.print("\t");
    file.print(gz); file.print("\t");
    Serial.print(hour);
        file.print(hour);
        dataFile.print(hour);
        Serial.print(":");
        file.print(":");
        dataFile.print(":");
        Serial.print(mins);
        file.print(mins);
        dataFile.print(mins);
        Serial.print(":");
        file.print(":");
        dataFile.print(":");
        Serial.print(sec);
        file.print(sec);
        dataFile.print(sec);
        Serial.print("\t");
        file.print("\t");
        dataFile.print("\t");
        sec+=1;
      if(sec==60){
        mins++;
        sec=0;
        }
        if (mins==60) {
            hour++;
            mins=0;
        } 
        file.close();
        delay (1000);
  }
   
    dataFile.close();
  
  
}

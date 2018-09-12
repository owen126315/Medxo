#include <CurieBLE.h>
#include "CurieIMU.h"
#include "CurieTimerOne.h"
#include <Wire.h>
#include <DFRobot_QMC5883.h>

DFRobot_QMC5883 compass;

int minX = 0;
int maxX = 0;
int minY = 0;
int maxY = 0;
int offX = 0;
int offY = 0;

float gx, gy, gz;
float ax, ay, az;
int P0, P1, P2, P3;
Vector mag;
BLEService SensorService("19B10010-E8F2-537E-4F6C-D104768A1214"); // BLE Battery Service

BLECharCharacteristic switchChar("19B10001-E8F2-537E-4F6C-D104768A1214", BLEWrite);    

BLECharacteristic IMU_Value("19B10013-E8F2-537E-4F6C-D104768A1214", BLENotify, 18);
BLECharacteristic pressure_Value("19B10013-E8F2-537E-4F6C-D104768A1215", BLENotify, 11);

BLEUnsignedCharCharacteristic batteryLevelChar("2A19", BLENotify);

// get notifications if this characteristic changes

//int oldBatteryLevel = 0;  // last battery level reading from analog input
long previousMillis = 0;  // last time the battery level was checked, in ms

void setup() {
  Serial.begin(9600);         // initialize serial communication

  CurieIMU.begin();
  CurieIMU.setGyroRange(250);
  CurieIMU.setAccelerometerRange(2);
   
  while (!compass.begin())   // Initialize Initialize QMC5883
  {
    Serial.println("Could not find a valid QMC5883 sensor, check wiring!");
    delay(500);
  }
  
  if(compass.isHMC())
  {
    Serial.println("Initialize HMC5883");
    compass.setRange(HMC5883L_RANGE_1_3GA);
    compass.setMeasurementMode(HMC5883L_CONTINOUS);
    compass.setDataRate(HMC5883L_DATARATE_15HZ);
    compass.setSamples(HMC5883L_SAMPLES_8);
  }
  else if(compass.isQMC())
  {
    Serial.println("Initialize QMC5883");
    compass.setRange(QMC5883_RANGE_2GA);
    compass.setMeasurementMode(QMC5883_CONTINOUS); 
    compass.setDataRate(QMC5883_DATARATE_50HZ);
    compass.setSamples(QMC5883_SAMPLES_8);
  }
  
  BLE.begin();                // begin initialization
  BLE.setLocalName("LeftInsole");
  BLE.setAdvertisedService(SensorService);  // add the service UUID
 
  SensorService.addCharacteristic(switchChar);//test
  
  SensorService.addCharacteristic(IMU_Value);
  SensorService.addCharacteristic(pressure_Value);
  
  BLE.addService(SensorService);   // Add the BLE Battery service
  //SensorService.addCharacteristic(batteryLevelChar); // add the battery level characteristic
  //batteryLevelChar.setValue(oldBatteryLevel);   // initial value for this characteristic

  
  BLE.setEventHandler(BLEConnected, blePeripheralConnectHandler);
  BLE.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler);
  switchChar.setEventHandler(BLEWritten, switchCharacteristicWritten);
  switchChar.setValue(0);
  
  BLE.advertise();          // start advertising
  Serial.println("Bluetooth device active, waiting for connections...");
}

void loop() 
{
  // listen for BLE peripherals to connect:
  BLEDevice central = BLE.central();

  // if a central is connected to peripheral:
  if (central) 
  {  
    while (central.connected()) 
    {
      
      update_IMU_value();
      update_Pressure_value(); 
      long currentMillis = millis();            // as long as the central is still connected:
      if (currentMillis - previousMillis >= 200) // if 200ms have passed, check the Pressure and IMU values
      {
        previousMillis = currentMillis;
        send_IMU_BLE();
        send_Pressure_BLE();
        //updateBatteryLevel();
      }
      display_all_values();
      
    }
  }
}



void blePeripheralConnectHandler(BLEDevice central) 
{
  // central connected event handler
  Serial.print("Connected event, central: ");
  Serial.println(central.address());
}

void blePeripheralDisconnectHandler(BLEDevice central) 
{
  // central disconnected event handler
  Serial.print("Disconnected event, central: ");
  Serial.println(central.address());
}

void switchCharacteristicWritten(BLEDevice central, BLECharacteristic characteristic) 
{
  // central wrote new value to characteristic, update LED
  Serial.print("Characteristic event, written: ");

  if (switchChar.value()) 
  {
    Serial.println("Viberator On");
    CurieTimerOne.pwmStart(2, 15.0, 2500000);
  } 
  else 
  {
    Serial.println("Viberator Off");
    CurieTimerOne.pwmStop();
  }
}

void update_IMU_value()
{
  CurieIMU.readGyroScaled(gx, gy, gz);
  CurieIMU.readAccelerometerScaled(ax, ay, az);
  mag = compass.readRaw();
    
}

void update_Pressure_value()
{
  P0 = analogRead(A0);
  P1 = analogRead(A1);
  P2 = analogRead(A2);
  P3 = analogRead(A3);
}


void send_IMU_BLE()
{
  unsigned char data[17];
  data[0] = ((int)(gx*1000) >> 8) & 0xFF;
  data[1] = (int)(gx*1000) & 0xFF;
  data[2] = ((int)(gy*1000) >> 8) & 0xFF;
  data[3] = (int)(gy*1000) & 0xFF;
  data[4] = ((int)(gz*1000) >> 8) & 0xFF;
  data[5] = (int)(gz*1000) & 0xFF;
  data[6] = ((int)(ax*1000) >> 8) & 0xFF;
  data[7] = (int)(ax*1000) & 0xFF;
  data[8] = ((int)(ay*1000) >> 8) & 0xFF;
  data[9] = (int)(ay*1000) & 0xFF;
  data[10] = ((int)(az*1000) >> 8) & 0xFF;
  data[11] = (int)(az*1000) & 0xFF;
  data[12] = ((int)(mag.XAxis/1000)+253 >> 8) & 0xFF;
  data[13] = ((int)(mag.XAxis/1000)+253) & 0xFF;
  data[14] = ((int)(mag.YAxis/1000)+253 >> 8) & 0xFF;
  data[15] = ((int)(mag.YAxis/1000)+253) & 0xFF; 
  data[16] = ((int)(mag.ZAxis/1000)+253 >> 8) & 0xFF;
  data[17] = ((int)(mag.ZAxis/1000)+253) & 0xFF;
  IMU_Value.setValue(data,18);
}

void send_Pressure_BLE()
{
  unsigned char data[11];
  data[0] = (P0 >> 8) & 0xFF;
  data[1] = P0 & 0xFF;
  data[2] = ':';
  data[3] = (P1 >> 8) & 0xFF;
  data[4] = P1 & 0xFF;
  data[5] = ':';
  data[6] = (P2 >> 8) & 0xFF;
  data[7] = P2 & 0xFF;
  data[8] = ':';
  data[9] = (P3 >> 8) & 0xFF;
  data[10] = P3 & 0xFF;
  pressure_Value.setValue(data,11);
}


void display_all_values()
{
  //G values
  Serial.print("G:");
  Serial.print(gx);
  Serial.print("\t");
  Serial.print(gy);
  Serial.print("\t");
  Serial.print(gz);
  Serial.print("\t");
  //Acceration values
  Serial.print("A:");
  Serial.print(ax);
  Serial.print("\t");
  Serial.print(ay);
  Serial.print("\t");
  Serial.print(az);
  Serial.print("\t");
  //B field values
  Serial.print("B:");
  Serial.print((int)(mag.XAxis/1000)+253);
  Serial.print("\t");
  Serial.print((int)(mag.YAxis/1000)+253);
  Serial.print("\t");
  Serial.print((int)(mag.ZAxis/1000)+253);
  Serial.print("\t");
  
  //Pressure values
  Serial.print("P:");
  Serial.print(P0);
  Serial.print("\t");
  Serial.print(P1);
  Serial.print("\t");
  Serial.print(P2);
  Serial.print("\t");
  Serial.println(P3);  
  
}

/*
void updateBatteryLevel() 
{
  int battery = analogRead(A0);
  int batteryLevel = map(battery, 0, 1023, 0, 100);
      
     // if the battery level has changed
    //Serial.print("Battery Level % is now: "); // print it
    //Serial.println(batteryLevel);
    batteryLevelChar.setValue(batteryLevel);  // and update the battery level characteristic
    oldBatteryLevel = batteryLevel;           // save the level for next comparison
}
*/

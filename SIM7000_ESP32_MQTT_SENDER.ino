/*
  SIM7000 ESP32 MQTT Cellular MQTT Sender

  This example connects to an MQTT sever via cellular LTE CATM-1 connection
  It sends data in a jSon format
  Time provided by cellular network.
  Includes support for a BME280 Environmental sensor
  Supports Botletics SIM7000 or other SIM7000 module
  Code by Bill Poulsen
  www.iot-techshop.com
  Purchase Botelics SIM7000 using this link to help support our work: https://amzn.to/2NRmyWH
  
  This example code is in the public domain.
*/



String myString;
String myDateTime;
float myTemp;
float myHumid;
float myPress;
float tmpOffset = 0;//temp offset cal factor if needed

//MQTT
//#define MQTT_USERNAME    "MQTT_USERNAME" //only if required by mqtt server
//#define MQTT_PASSWORD    "MQTT_PASSWORD" //only if required by mqtt server
//Set MQTT Server Ino at Line 227
// Set topic names to publish and subscribe to
#define MQTT_TOPIC       "sensors/bill/"

int sendInt;

char tempBuff[16];
char rBuff[80];
char msg[200];
String devID = "";
String myMac = "";
char battBuff[5];

//SENSORS
#include <Wire.h> //I2C
#include <BME280I2C.h> //
BME280I2C bme;    // Default : addr 0x76 forced mode, standby time = 1000 ms

// SIMCOM Library
#include "Adafruit_FONA.h" // https://github.com/botletics/SIM7000-LTE-Shield/tree/master/Code

#define SIMCOM_7000 // SIM7000A/C/E/G  St for Simcom SIM7000 Modems

// I/O For CUSPIX V1 with ESP32 Change I/O pins for your configuration
#define FONA_PWRKEY 13
#define FONA_RST 14 //SIMCOM Hard Reset High = Normal/LOW = Reset
#define FONA_TX 25 // ESP32 hardware serial RX2 
#define FONA_RX 27 // ESP32 hardware serial TX2 
#define SEN_PWR_CTL 15 //Controls 3.3V Power to All Sensors  High = ON/Low = OFF
#define LTE_PWR 26 //Controls 3.3V Power to SIMCOM Module  High = ON/Low = OFF
#define LED_RED 2 //On Board RED LED   

#include <WiFi.h> //Although we are not using wifi,this library allows us to easily retrieve the ESP32 ID
// For ESP32 hardware serial
#include <HardwareSerial.h>
HardwareSerial fonaSS(1);

Adafruit_FONA_LTE fona = Adafruit_FONA_LTE();

uint8_t type;
char imei[16] = {0}; // MUST use a 16 character buffer for IMEI!
float temp(NAN), hum(NAN), pres(NAN);
uint8_t month, day, hour, minute;


void setup() {
  devID = getEspID();
  //Setup Sensor Power:My custom pcb controls sensor power,you may not need
  digitalWrite(SEN_PWR_CTL, HIGH);
  pinMode(SEN_PWR_CTL, OUTPUT);
  Wire.begin();
  //Setup LTE Power
  digitalWrite(LTE_PWR, HIGH);
  pinMode(LTE_PWR, OUTPUT);
  delay(3000);
  //Setup SIMCOM Reset
  pinMode(FONA_RST, OUTPUT);
  digitalWrite(FONA_RST, HIGH); // Default state

  //Set SIMCOM Power Key as output
  pinMode(FONA_PWRKEY, OUTPUT);

  //Set Onboard LED I/O to Output
  pinMode(LED_RED, OUTPUT);

  // Turn on the module by pulsing PWRKEY low for a little bit
  // This amount of time depends on the specific module that's used
  powerOn(); // See function definition at the very end of the sketch
  delay(3000);
  Serial.begin(115200);
  Serial.println(F("ESP32 Basic Test"));
  Serial.println(F("Initializing....(May take several seconds)"));
  Serial.println(F("Configuring to 115200 baud"));
  fonaSS.println("AT+IPR=115200"); // Set baud rate
  delay(100); // Short pause to let the command run
  fonaSS.begin(115200, SERIAL_8N1, FONA_TX, FONA_RX);

  if (! fona.begin(fonaSS)) {
    Serial.println(F("Couldn't find FONA"));
    Serial.println(F("Rebooting in 5 secs!!"));
    delay(5000);
    ESP.restart();
    while (1); // Don't proceed if it couldn't find the device
  }

  type = fona.type();
  Serial.println(F("FONA is OK"));
  Serial.print(F("Found "));
  switch (type) {
    case SIM7000A:
      Serial.println(F("SIM7000A (American)")); break;
    case SIM7000E:
      Serial.println(F("SIM7000E (European)")); break;
    case SIM7000G:
      Serial.println(F("SIM7000G (Global)")); break;
    default:
      Serial.println(F("???")); break;
  }

  // Print module IMEI number(CellModem)
  uint8_t imeiLen = fona.getIMEI(imei);
  if (imeiLen > 0) {
    Serial.print("Module IMEI: "); Serial.println(imei);
  }

  // Set modem to full functionality
  fona.setFunctionality(1); // AT+CFUN=1

  // Configure a GPRS APN, username, and password.
  fona.setNetworkSettings(F("hologram")); // For Hologram SIM card

  //LTE Only, CAT-M, RTC Enabled
  fona.setPreferredMode(38); // Use LTE only, not 2G
  fona.setPreferredLTEMode(1); // Use LTE CAT-M only, not NB-IoT
  fona.enableRTC(true);

  // Set the network status LED blinking pattern while connected to a network (see AT+SLEDS command)
  //fona.setNetLED(true, 2, 64, 1000); // on/off, mode, timer_on, timer_off
  fona.setNetLED(false); // Disable network status LED

  fona.enableGPS(false);//Disables or Enables Cell Module GPS
  delay(5000);

  sendMQTT(32000, 5);//Starts Sending MQTT Data (max sends,interval in secs)
}


void loop() {


}

// Power on the module
void powerOn() {
  digitalWrite(FONA_PWRKEY, LOW);
  delay(100); // For SIM7000
  digitalWrite(FONA_PWRKEY, HIGH);
}


void bme280Test(int rCnts) {
  while (!bme.begin())
  {
    Serial.println("Could not find BME280 sensor!");
    delay(1000);
  }

  // bme.chipID(); // Deprecated. See chipModel().
  switch (bme.chipModel())
  {
    case BME280::ChipModel_BME280:
      Serial.println("Found BME280 sensor! Success.");
      break;
    case BME280::ChipModel_BMP280:
      Serial.println("Found BMP280 sensor! No Humidity available.");
      break;
    default:
      Serial.println("Found UNKNOWN sensor! Error!");
  }
  int x = 0;
  Serial.println("START");
  do {
    // wait for sensors to stabilize
    printBME280Data();
    delay(500);


    x = x + 1;
  } while (x < rCnts);

  Serial.println("END");

}

void printBME280Data() {

  BME280::TempUnit tempUnit(BME280::TempUnit_Fahrenheit);
  BME280::PresUnit presUnit(BME280::PresUnit_Pa);

  // bme.read(pres, temp, hum, tempUnit, presUnit);
  Serial.print("Temp:"); Serial.print(temp); Serial.print("*F Humid:"); Serial.print(hum); Serial.print("% RH Pres:"); Serial.print(pres); Serial.println(" Pa");
  myTemp = temp + tmpOffset;
  myHumid = hum;
  myPress = pres;
  Serial.println(myTemp);
  Serial.println(myHumid);
  Serial.println(myPress);

  delay(100);
}


void sendMQTT(int msgCnt, int sDelay) {
  for (int mc = 0; mc < msgCnt + 1; mc++) {
    mqttMsg();
    fona.sendCheckReply("AT+CREG?", "OK", 1000);
    const char* cmd1 = "\"AT+CNACT=1,\"hologram\"\"";
    
    //Set MQTT Server URL and PORT innext line
    const char* cmd2 = "AT+SMCONF=\"URL\",\"iot.YYY.com\",1883"; // MQTT Server and Port#
   
    const char* cmd3 = "\"AT+CNACT=1,\"hologram\"\"";
    const char* cmd4 = "\"AT+CNACT=1,\"hologram\"\"";

    fona.sendCheckReply(cmd1, "OK", 1000);
    fona.sendCheckReply(cmd2, "OK", 1000);

    fona.MQTT_connect(true);

    fona.MQTT_publish(MQTT_TOPIC, msg, strlen(msg), 1, 0);


    fona.MQTT_connect(false);

delay(sDelay * 1000);

  }

}


String getEspID() {
  myMac = WiFi.macAddress();
  //Creates device ID from ES32 MAC address
  devID = myMac.substring(0, 2) +  myMac.substring(3, 5) +  myMac.substring(6, 8) +  myMac.substring(9, 11) +  myMac.substring(12, 14) +  myMac.substring(15, 17);
  return myMac.substring(0, 2) +  myMac.substring(3, 5) +  myMac.substring(6, 8) +  myMac.substring(9, 11) +  myMac.substring(12, 14) +  myMac.substring(15, 17);

}

//retrieves cell network time and formats
String getCellTime() {
  char tbuffer[23];
  fona.getTime(tbuffer, 23);  // make sure replybuffer is at least 23 bytes!
  myDateTime = tbuffer;
  myDateTime = "20" + myDateTime.substring(1, 3) + myDateTime.substring(4, 6) + myDateTime.substring(7, 9) + myDateTime.substring(10, 12) + myDateTime.substring(13, 15) + myDateTime.substring(16, 18);
  return myDateTime;
}


void mqttMsg() {

  uint16_t vBat;
  myDateTime = getCellTime();
  bme280Test(1);// comment out if no BME280 Sensor attached
  fona.getBattVoltage(&vBat);

  //Formats data to jSon type string
  myString = "{\"ID\":\"" + devID + "\",\"tim\":\"" + myDateTime + "\",\"" + "Tmp" + "\":\"" + myTemp + "\",\"" + "Hum" + "\":\"" + myHumid + "\",\"" + "mvBat" + "\":\"" + vBat + "\"}";

  //Converts formatted string to char array
  myString.toCharArray(msg, myString.length() + 1);

}

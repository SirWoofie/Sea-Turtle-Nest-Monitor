//////////////////////////////////////////////////////////////////////////////////////////////////
/*
  Pin Assignments

  Photoresistor: A0
  Mic1-3, A1-A3
  DHT11: D12
  Water Level: D4
  I2C Bus: SCL, SDA: A4, A5
  VL53L0X: I2C
  KX-132: I2C
  SoftwareSerial: RX: D10, TX: D11

*/
//////////////////////////////////////////////////////////////////////////////////////////////////

#include <Adafruit_VL53L0X.h> //Time-of-flight sensor library
#include <SparkFun_KX13X.h> //Accelerometer library
#include <Notecard.h> //Blues Notecard library

#include <Adafruit_Sensor.h> //Overall Adafruit Sensor Library, needed specifically for the DHT11
#include <dht11.h> //Temp. & Humid. library
#include <DHT_U.h>

 #include <SoftwareSerial.h> //Used only because the Arduino Uno's single serial port is used by the IDE.
 #include <ArduinoJson.h> //Allows us to create, send, parse, and receive Json objects.

#include <string.h>

//Global variables for KX132
SparkFun_KX132 kxAccel;
outputData myData; // Struct for the accelerometer's data

//Global variable for VL53L0X
//Moved to a local context to save memory

//Global variables for DHT11
#define DHTPIN 12
uint8_t dhtFreq = 0;


//Photoresistor define
#define photoPin A0

//Microphone defines
#define mic1 A3
#define mic2 A2
#define mic3 A1

//Global variable for Software Serial
SoftwareSerial softSerial(10, 11); //RX, TX

//Setup functions
void serialSetup(uint8_t &errors) {
  Wire.begin();

  softSerial.begin(38400);

  Serial.begin(115200);
    // errSerial = true;
    if (!Serial) {
      errors |= 1;
    }
  
  // wait until serial port opens for native USB devices
  while (! Serial) {
    delay(1);
  }
}

void loxSetup(uint8_t &errors) {
  Adafruit_VL53L0X lox = Adafruit_VL53L0X();
  if (!lox.begin()) {
    errors |= 2;
  }
}

void kxSetup(uint8_t &errors) {
  if (!kxAccel.begin())
  {
    errors |= 4;
  }
  // Give some time for the accelerometer to reset.
  // It needs two, but give it five for good measure.
  delay(5);

  // Many settings for KX13X can only be
  // applied when the accelerometer is powered down.
  // However there are many that can be changed "on-the-fly"
  // check datasheet for more info, or the comments in the
  // "...regs.h" file which specify which can be changed when.
  kxAccel.enableAccel(false);

  kxAccel.setRange(SFE_KX132_RANGE2G); // 2g Range

  kxAccel.enableDataEngine(); // Enables the bit that indicates data is ready.
  // kxAccel.setOutputDataRate(); // Default is 50Hz
  kxAccel.enableAccel();
}

//Loop functions
void photoLoop() {
  int photoValue;
  photoValue = analogRead(photoPin);
  if (photoValue > 500) {
    Serial.println("[Light] " + String(photoValue) + ", Waiting until Nighttime");
  } else {
    Serial.println("[Light] " + String(photoValue) + ", Active");
  }
  while (photoValue > 500) {
    photoValue = analogRead(photoPin);
    delay(1000);
  }
}

void loxLoop(JsonDocument &doc) {
  Adafruit_VL53L0X lox = Adafruit_VL53L0X();
  if (!lox.begin()) {}
  delay(5);

  VL53L0X_RangingMeasurementData_t measure;
  lox.rangingTest(&measure, false);
  doc["tof"]["data"][0] = measure.RangeMilliMeter;
}

void kxLoop(JsonDocument &doc) {
  if (kxAccel.dataReady()) {
    kxAccel.getAccelData(&myData);
  }
  doc["acc"]["data"][0] = myData.xData;
  doc["acc"]["data"][1] = myData.yData;
  doc["acc"]["data"][2] = myData.zData;
}

void dhtTempLoop(JsonDocument &doc) {
  DHT_Unified dht(DHTPIN, DHT11);
  dht.begin();
  sensors_event_t dhtEvent; 
  dht.temperature().getEvent(&dhtEvent); //Retrieve temperature event
  if (!isnan(dhtEvent.temperature)) {
    doc["tmp"]["data"][0] = dhtEvent.temperature;
  } else {
    doc["tmp"]["data"][0] = (char*)0;;
  }

  //We need to access both temperature and humidity from this single event. Otherwise, the humidity will always be null.
  dht.humidity().getEvent(&dhtEvent); //Retrieve humidity event
  if (!isnan(dhtEvent.relative_humidity)) {
    doc["hum"]["data"][0] = dhtEvent.relative_humidity;
  } else {
    doc["hum"]["data"][0] = (char*)0;;
  }
}

void dhtHumLoop(JsonDocument &doc) {
  DHT_Unified dht(DHTPIN, DHT11);
  dht.begin();
  sensors_event_t dhtEvent;
  dht.humidity().getEvent(&dhtEvent); //Retrieve humidity event
  if (!isnan(dhtEvent.relative_humidity)) {
    doc["hum"]["data"][0] = dhtEvent.relative_humidity;
  } else {
    doc["hum"]["data"][0] = (char*)0;;
  }
}

void micLoop(JsonDocument &doc) {
  //Send microphone values to Json
  doc["mic"]["data"][0] = analogRead(mic1);
  doc["mic"]["data"][1] = analogRead(mic2);
  doc["mic"]["data"][2] = analogRead(mic3);
}

void waterLoop(JsonDocument &doc, int whichPin) {
  //If water is detected, send TRUE to Json
  //whichPin is either 4, 5, or 6. So subtract 3 to get the index of the sensor's data in the Json document - 0, 1, or 2.
  if (digitalRead(whichPin)) {
    doc["wtr"]["data"][whichPin-3] = true;
  } else {
    doc["wtr"]["data"][whichPin-3] = false;
  }
}

void setup() {
  //Error Checking
  uint8_t errors = 0;
  digitalWrite(13, LOW);

  //I2C and Serial Initialization
  serialSetup(errors);

  //Device setup and testing
  //VL53L0X
  loxSetup(errors);
  //KX132
  kxSetup(errors);
  //DHT11
  DHT_Unified dht(DHTPIN, DHT11);
  dht.begin();
  sensor_t dht11;
  //Photoresistor
  pinMode(photoPin, INPUT);
  //Setup complete

////////////////////////////////////////////////////////////////////////////////
//Check for errors after initialization
////////////////////////////////////////////////////////////////////////////////
  // if (errors) {
  //   Serial.print(F("/////////////////////////////////\n[Error] Errors Detected\n/////////////////////////////////\n"));
  //   //Breakdown of the errors:
  //   delay(1000);
  //   if (errors & 1) {
  //     pinMode(13, OUTPUT);
  //     digitalWrite(13, HIGH);
  //   }
  //   if (errors & 2) {
  //     Serial.println(F("[Error] VL53L0X Not Detected"));
  //   }
  //   if (errors & 4) {
  //     Serial.println(F("[Error] KX132 Not Detected"));
  //   }
  //   delay(1000);
  // }
  // digitalWrite(13, LOW);
}

void loop() {
  // put your main code here, to run repeatedly:
  //Photoresistor - should run first instead of freezing the loop halfway through.
  // photoLoop();

  //Json Document does not need to be defined globally.
  JsonDocument doc;

  //VL53L0X
  loxLoop(doc);

  //KX132
  kxLoop(doc);

  //DHT11 - can only be read at 1Hz
  // if (dhtFreq == 5) {
  //   dhtTempLoop(doc);
  //   doc["hum"]["data"][0] = (char*)0;
  //   dhtFreq++;
  // } else if (dhtFreq >= 10) {
  //   dhtHumLoop(doc);
  //   doc["tmp"]["data"][0] = (char*)0;
  //   dhtFreq = 0;
  // } else {
  //   doc["hum"]["data"][0] = (char*)0;
  //   doc["tmp"]["data"][0] = (char*)0;
  //   dhtFreq++;
  // }
  //DHT11 - If values from sensor are null, transmit the null value anyway. We need to transmit null so the array isn't incomplete on the Python receiving end.
  dhtTempLoop(doc);
  //dhtHumLoop(doc);


  //Microphones
  micLoop(doc);

  //Water Level Detector
  waterLoop(doc, 4);
  waterLoop(doc, 5);
  waterLoop(doc, 6);

  //Print a minified Json Document to Software Serial
  doc.shrinkToFit();
  serializeJson(doc, softSerial);
  softSerial.println();

  //Delay before looping and add space between loops
  delay(200);
}

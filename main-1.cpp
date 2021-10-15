#include <Adafruit_SleepyDog.h>
#include <SoftwareSerial.h>
#include "Adafruit_FONA.h"
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_FONA.h"
#include <ComponentObject.h>
#include <RangeSensor.h>
#include <SparkFun_VL53L1X.h>
#include <vl53l1x_class.h>
#include <vl53l1_error_codes.h>
#include <Wire.h>
#include "SparkFun_VL53L1X.h" 
#include <Adafruit_AHTX0.h>

/*************************** FONA Pins ***********************************/

// Default pins for Feather 32u4 FONA
#define FONA_RX  2
#define FONA_TX  3
#define FONA_RST 4
SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);

Adafruit_FONA fona = Adafruit_FONA(FONA_RST);

Adafruit_AHTX0 aht;

SFEVL53L1X distanceSensor;


/************************* WiFi Access Point *********************************/
#define FONA_APN       "m2mglobal"
#define FONA_USERNAME  ""
#define FONA_PASSWORD  ""

/************************* Adafruit.io Setup *********************************/

#define AIO_SERVER      "23.239.21.195"
#define AIO_SERVERPORT  1883
#define AIO_USERNAME    ""
#define AIO_KEY         ""

/************ Global State (you don't need to change this!) ******************/

// Setup the FONA MQTT class by passing in the FONA class and MQTT server and login details.
Adafruit_MQTT_FONA mqtt(&fona, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

// You don't need to change anything below this line!
#define halt(s) { Serial.println(F( s )); while(1);  }

// FONAconnect is a helper function that sets up the FONA and connects to
// the GPRS network. See the fonahelper.cpp tab above for the source!
boolean FONAconnect(const __FlashStringHelper *apn, const __FlashStringHelper *username, const __FlashStringHelper *password);

/****************************** Feeds ***************************************/

// Setup a feed called 'photocell' for publishing.
// Notice MQTT paths for AIO follow the form: <username>/feeds/<feedname>
Adafruit_MQTT_Publish photocell = Adafruit_MQTT_Publish(&mqtt, "sensors/temperature");
Adafruit_MQTT_Publish humidityPub = Adafruit_MQTT_Publish(&mqtt, "sensors/humidity");
Adafruit_MQTT_Publish depthPub = Adafruit_MQTT_Publish(&mqtt, "sensors/depth");

/*************************** Sketch Code ************************************/

// How many transmission failures in a row we're willing to be ok with before reset
uint8_t txfailures = 0;
#define MAXTXFAILURES 3

void setup() {
Wire.begin();
  while (!Serial);

  // Watchdog is optional!
  //Watchdog.enable(8000);


  
  Serial.begin(115200);


    Serial.println("VL53L1X Qwiic Test");
    if (! aht.begin()) {
    Serial.println("Could not find AHT? Check wiring");
    while (1) delay(10);
  }
  Serial.println("AHT10 or AHT20 found");

  if (distanceSensor.begin() != 0) //Begin returns 0 on a good init
  {
    Serial.println("Sensor failed to begin. Please check wiring. Freezing...");
    while (1)
      ;
  }
  Serial.println("Sensor online!");



   if (! aht.begin()) {
    Serial.println("Could not find AHT? Check wiring");
    while (1) delay(10);
  }
  Serial.println("AHT10 or AHT20 found");

  Serial.println(F("Adafruit FONA MQTT demo"));

  //mqtt.subscribe(&onoffbutton);

  Watchdog.reset();
  delay(5000);  // wait a few seconds to stabilize connection
  Watchdog.reset();
  
  // Initialise the FONA module
  while (! FONAconnect(F(FONA_APN), F(FONA_USERNAME), F(FONA_PASSWORD))) {
    Serial.println("Retrying FONA");
  }

  Serial.println(F("Connected to Cellular!"));

  Watchdog.reset();
  delay(5000);  // wait a few seconds to stabilize connection
  Watchdog.reset();
}

//uint32_t x=0;


void loop() {
    delay(42000);


  distanceSensor.startRanging(); //Write configuration bytes to initiate measurement
  while (!distanceSensor.checkForDataReady())
  {
    delay(1);
  }
  int distance = distanceSensor.getDistance(); //Get the result of the measurement from the sensor
  distanceSensor.clearInterrupt();
  distanceSensor.stopRanging();

  Serial.print("Distance(mm): ");
  Serial.print(distance);

  float distanceInches = distance * 0.0393701;
  float distanceFeet = distanceInches / 12.0;
  float snowDepth = 96.0 - distanceFeet;

  Serial.print("\tDistance(ft): ");
  Serial.println(distanceFeet, 2);



    
sensors_event_t humidity, temp;
aht.getEvent(&humidity, &temp);// populate temp and humidity objects with fresh data

  float tempF = temp.temperature * (9.0/5.0) + 32.0;
  float hum = humidity.relative_humidity;

  String tempString = String(tempF, 2);
  
  Serial.print("Temperature: "); Serial.print(tempF); Serial.println(" degrees C");
  Serial.print("Humidity: "); Serial.print(humidity.relative_humidity); Serial.println("% rH");


  
  // Make sure to reset watchdog every loop iteration!
  Watchdog.reset();

  // Ensure the connection to the MQTT server is alive (this will make the first
  // connection and automatically reconnect when disconnected).  See the MQTT_connect
  // function definition further below.
  MQTT_connect();

  Watchdog.reset();
  // Now we can publish stuff!
  Serial.print(F("\nSending photocell val "));
  Serial.print(tempF);
  Serial.print("...");
  if (! photocell.publish(tempF)) {
    Serial.println(F("Failed"));
    txfailures++;
  } else {
    Serial.println(F("OK!"));
    txfailures = 0;
  }

    Serial.print(F("\nSending humidityPub val "));
  Serial.print(hum);
  Serial.print("...");
  if (! humidityPub.publish(hum)) {
    Serial.println(F("Failed"));
    txfailures++;
  } else {
    Serial.println(F("OK!"));
    txfailures = 0;
  }

      Serial.print(F("\nSending depthPub val "));
  Serial.print(snowDepth);
  Serial.print("...");
  if (! depthPub.publish(snowDepth)) {
    Serial.println(F("Failed"));
    txfailures++;
  } else {
    Serial.println(F("OK!"));
    txfailures = 0;
  }


  Watchdog.reset();  

}

void sensorReading() {
  
}


// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care if connecting.
void MQTT_connect() {
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }

  Serial.print("Connecting to MQTT... ");

  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
    Serial.println(mqtt.connectErrorString(ret));
    Serial.println("Retrying MQTT connection in 5 seconds...");
    mqtt.disconnect();
    delay(5000);  // wait 5 seconds
  }
  Serial.println("MQTT Connected!");
}
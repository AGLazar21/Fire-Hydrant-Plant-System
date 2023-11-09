/* 
 * Project myProject
 * Author: Your Name
 * Date: 
 * For comprehensive documentation and examples, please visit:
 * https://docs.particle.io/firmware/best-practices/firmware-template/
 */


#include "Particle.h"
#include "Air_quality_Sensor.h"
#include <Adafruit_MQTT.h>
#include "Adafruit_MQTT/Adafruit_MQTT_SPARK.h"
#include "Adafruit_MQTT/Adafruit_MQTT.h"
#include "Credentials.h"
#include <math.h>
#include "Adafruit_SSD1306.h"
#include "Adafruit_BME280.h"
SYSTEM_MODE(AUTOMATIC);
TCPClient TheClient;
Adafruit_MQTT_SPARK mqtt(&TheClient,AIO_SERVER,AIO_SERVERPORT,AIO_USERNAME,AIO_KEY);
Adafruit_MQTT_Subscribe waterPlantFeed = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/waterPlantFeed"); 
Adafruit_MQTT_Publish airQFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/airQualityFeed");
Adafruit_MQTT_Publish humFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/humFeed");
Adafruit_MQTT_Publish moistFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/mositFeed");
Adafruit_MQTT_Publish tempFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/tempFeed");
const int OLED_RESET = -1, PUMP= D16;
const char hexAddress = 0x76;
Adafruit_SSD1306 display(OLED_RESET);
AirQualitySensor airQualSens(A1);
Adafruit_BME280 bme;
int status,tempC,pressPA,humidRH,tempF,inHG, soilMoist = A2,subValue;
const int DUSTSENS = A0, ONEHOUR=600000;
unsigned int startTime1, airValue, duration, lowPulseOcc, currentQuality = -1;
float ratio = 0, concentration = 0;
void bmeReading(int interval);
void dustSensor(int interval);
void MQTT_connect();
void airSens(int interval);
void waterPump(int pump,int timeON);
void readSoil(int pin, int interval, int threshold);

void setup() {
  WiFi.on();
  WiFi.connect();
  Serial.begin(9600);
  waitFor(Serial.isConnected,10000);
  pinMode(soilMoist,INPUT);
  status = bme.begin(hexAddress);
  if(status == false){
    Serial.printf("BME280 at address 0x%02X failed to start", hexAddress);
  }
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.display();
  delay(2000); 
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  pinMode(PUMP,OUTPUT);
  mqtt.subscribe(&waterPlantFeed);
}

void loop() {
  MQTT_connect();
  bmeReading(10000);
  airSens(10000);
  readSoil(soilMoist,ONEHOUR,2700);
  //dustSensor(60000);
  Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(100))) {
    if (subscription == &waterPlantFeed) {
      subValue = atof((char *)waterPlantFeed.lastread);
      digitalWrite(PUMP,subValue);
      Serial.printf("water plant:%i\n", subValue);
  }
}
}

void bmeReading(int interval){
  static unsigned int currentTime, lastTime;
  currentTime =  millis();

    if((currentTime-lastTime)>interval){
      lastTime = millis();
      display.clearDisplay();
      tempC= bme.readTemperature();
      pressPA = bme.readPressure();
      humidRH = bme.readHumidity();

      tempF = map(tempC,0,38,32,100);
      inHG = map(pressPA,0,135456,0,40);
      display.setRotation(3);
      display.setCursor(0,0);
      display.printf("Temp F: %i",tempF);
      display.display();
      display.setCursor(0,20);
      display.printf("Press: %i",inHG);
      display.display();display.setCursor(0,40);
      display.printf("Hum: %i",humidRH);
      display.display();
      humFeed.publish(humidRH);
      tempFeed.publish(tempF);
    }
}

void dustSensor(int interval){  //Reads air particles in room and prints/publishs the data 
static unsigned int currentTime, lastTime;
int startTime = millis();

currentTime = millis();
if((currentTime - lastTime)>interval){
  lastTime = millis();
  duration = pulseIn(DUSTSENS,LOW);
  lowPulseOcc = lowPulseOcc+duration;

  if((millis()-startTime)>30000){
    ratio = lowPulseOcc/(300000.0);
    concentration = 1.1*pow(ratio,3)-3.8*pow(ratio,3)+520*ratio+0.62;
    Serial.printf("Low Pulse Occ = %i, Ratio = %f, Concentration = %f\n",lowPulseOcc,ratio,concentration);
    //dustFeed.publish(concentration);
    lowPulseOcc = 0;
    startTime = millis();
  }
  return;
}
}


void airSens(int interval){  //reads air qualuity and gives back a reading in case form asell as quantitative value 
  static unsigned int currentTime, lastTime;
  currentTime =  millis();

  if((currentTime-lastTime)>interval){
    lastTime = millis();
    airValue = airQualSens.getValue();
    currentQuality = airQualSens.slope();
    switch(currentQuality){
      case 0:
        Serial.printf("HIGH POLLUTION, YOURE GOING TO DIE!\n");
        airQFeed.publish("HIGH POLLUTION, YOURE GOING TO DIE!\n");
        break;
      case 1:
        Serial.printf("High pollution!\n");
        airQFeed.publish("High pollution!\n");
        break;
      case 2:
        Serial.printf("Low polution.\n");
        airQFeed.publish("Low polution.\n");
        break;
      case 3:
        Serial.printf("Fresh Air.\n");
        airQFeed.publish("Fresh Air.\n");
        break;
  }
  Serial.printf("Quant Value= %i\n",airValue);
}
}


void readSoil(int pin, int interval, int threshold){ // checks moisture of soil and waters if neccesary 
  static unsigned int currentTime, lastTime;
  int moistReading;

  currentTime =millis();
  moistReading = analogRead(pin);
  if((currentTime-lastTime)>interval){
    lastTime = millis();
    moistFeed.publish(moistReading);
    if(moistReading > threshold){
      //waterPump(PUMP,500);
      Serial.printf("Soil dry, watering now\n");
    }
    else{
      Serial.printf("Soil good");
    }
}
}

void waterPump(int pump,int timeON){
  digitalWrite(pump,HIGH);
  delay(timeON);
  digitalWrite(pump,LOW);
}


void MQTT_connect() {
  int8_t ret;
 
  // Return if already connected.
  if (mqtt.connected()) {
    return;
  }
 
  Serial.print("Connecting to MQTT... ");
 
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
       Serial.printf("Error Code %s\n",mqtt.connectErrorString(ret));
       Serial.printf("Retrying MQTT connection in 5 seconds...\n");
       mqtt.disconnect();
       delay(5000);  // wait 5 seconds and try again
  }
  Serial.printf("MQTT Connected!\n");
}
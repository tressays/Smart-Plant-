/* 
 * Project SmartPlant_Midterm
 * Author: CCrow
 * Date: 6-NOV-2023
 * For comprehensive documentation and examples, please visit:
 * https://docs.particle.io/firmware/best-practices/firmware-template/
 */

#include "Particle.h"
#include "Grove_Air_quality_Sensor.h"
#include "Adafruit_SSD1306.h"
#include "Adafruit_BME280.h"
#include "IoTClassroom_CNM.h"
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_SPARK.h"
#include "Credentials.h"
//#define OLED_RESET D4
#include "math.h"
bool On_off;                              //Subscribe//Publish
TCPClient TheClient; 
Adafruit_MQTT_SPARK mqtt(&TheClient,AIO_SERVER,AIO_SERVERPORT,AIO_USERNAME,AIO_KEY);
Adafruit_MQTT_Subscribe subFeed = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/12oz N Foyer");  
void MQTT_connect();
bool MQTT_ping();
const int MOISTUREPIN = A1;               //Moisture Sensor
unsigned int _timerStart;
unsigned long currentTime;
unsigned long lastMinute;
int halfHour = 1800000;
//unsigned long millis;
int moistureValue;
int quality;                            //Air quality sensor
const int Optoc = D5;                   //Optoc
int bStatus;                            //BME_280
int hexAdd = 0x76;
int Temp;
int Press;
int Altit;
int Humid;
Adafruit_BME280 BME;
IoTTimer Optoctimer;
//int month = 12;                       //OLED
//int day = 18;
//int year = 1981;
//const char honor = 0xA4;
AirQualitySensor AQS (A0);
String DateTime, TimeOnly;               //Mositure Sensor
//Adafruit_SSD1306 display(OLED_RESET);   //OLED
int DustPin = D6;                       //Dust Particle Sensor                      
float ratio = 0;
float concentration = 0;
unsigned long duration;
unsigned long starttime;
unsigned long sampletime_ms = 30000;
unsigned long lowpulseoccupancy = 0;

SYSTEM_MODE(AUTOMATIC);
//SYSTEM_THREAD(ENABLED);
SerialLogHandler logHandler(LOG_LEVEL_INFO);

void setup() {
Serial.begin (9600);
 WiFi.on();
  WiFi.connect();
  while(WiFi.connecting())
  AQS.init();                           //Air quality sensor
Time.zone(-7);                          //command to connect to particle cloud
  Particle.syncTime();  
mqtt.subscribe(&subFeed);               //Subscribe and publish  
  pinMode(MOISTUREPIN, INPUT);            //Moisture Sensor
  pinMode(Optoc, OUTPUT);                   //Optoc
  bStatus = BME.begin(hexAdd);
  //display.begin(SSD1306_SWITCHCAPVCC, 0x3c);  // initialize with the I2C addr 0x3D (for the 128x64)
  //display.setTextSize(2);
  //display.setTextColor(WHITE);
  //display.setCursor(0,0);
  //display.setTextColor(WHITE);
  //delay(2000);
  //display.clearDisplay();
 pinMode(DustPin, OUTPUT);             //Dust Particle Sensor  
  starttime = millis();
  }


void loop() {
    MQTT_connect();                     //Sunscribe and publish
    MQTT_connect();
    Adafruit_MQTT_Subscribe *subscription;
    while ((subscription = mqtt.readSubscription(100))) {//resdSubscription
    if (subscription == &subFeed){
      On_off = atof((char *)subFeed.lastread); 
      digitalWrite(D5, On_off);       
    }
    moistureValue =analogRead(MOISTUREPIN);         //Moisture pin, analog read from mo
    DateTime = Time.timeStr();                      //mositure sensor tells optoc when to digitalWrite(HIGH) turn on/off
    TimeOnly = DateTime.substring(11,19);
    Serial.printf("%s\n,Moisture Value %i\n", TimeOnly.c_str(), moistureValue);
        delay(500); 
    currentTime = millis();
  if(currentTime - lastMinute >10000);  

  if(moistureValue>= 2720 ){   
      digitalWrite(Optoc, HIGH); 
      Optoctimer.startTimer(500);
      Serial.printf("%i\n", moistureValue);
  }
  if(Optoctimer.isTimerReady()){
      digitalWrite(Optoc, LOW);
  }
     duration = pulseIn(DustPin, LOW);               //Dust Particle Sensor
    lowpulseoccupancy = lowpulseoccupancy+duration;

     int quality=AQS.slope();       //Air quality sensor
        if (quality ==0){
      Serial.printf("high pollution! Force Signal Active\n");
      }
    else if (quality==1){
      Serial.printf("High Pollution!\n");
      }
    else if (quality==2){
      Serial.printf("Low pollution\n");
      }
    else if (quality==3){
      Serial.printf("Base\n");
      }
    //display.setTextSize(1);      //OLED
    //display.setTextColor(WHITE);
    //display.setCursor(0,0);
    //display.clearDisplay();
    //display.printf("Se%cior Celestino born %i/%i/%i", honor, month, day, year);
    //display.display();

    Temp = BME.readTemperature();           //BME
    Serial.printf("%i\n", Temp);
    delay(1000);
    Press = BME.readPressure();
    Serial.printf("%i\n", Press);
    delay(1000);
    Humid = BME.readHumidity();
    Serial.printf("%i\n", Humid);
    delay(1000);

    if ((millis()-starttime) > sampletime_ms)       //Dust Particle Sensor
    {
    ratio = lowpulseoccupancy/(sampletime_ms*10.0); 
        concentration = 1.1*pow(ratio,3)-3.8*pow(ratio,2)+520*ratio+0.62; 
        Serial.print(lowpulseoccupancy); 
        Serial.print(",");
        Serial.print(ratio);
        Serial.print(",");
        Serial.println(concentration);
        lowpulseoccupancy = 0;
        starttime = millis();
    
    Serial.printf("%i\n", DustPin);
    }

}
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

bool MQTT_ping() {
  static unsigned int last;
  bool pingStatus;

  if ((millis()-last)>120000) {
      Serial.printf("Pinging MQTT \n");
      pingStatus = mqtt.ping();
      if(!pingStatus) {
        Serial.printf("Disconnecting \n");
        mqtt.disconnect();
      }
      last = millis();
  }
  return pingStatus;
}


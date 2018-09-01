#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <DHT.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <ArduinoOTA.h>

#define LVL_PWR 17
#define LVL_DATA 16
#define DHT_PWR 4
#define DHT_DATA 18
#define DHTTYPE DHT11 
#define TOKEN_TB "S3WqxtQ17v1ETcqMCgWB"
#define ANALOG_PIN_0 34

#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  600      /* Time ESP32 will go to sleep (in seconds) */

DHT dht(DHT_DATA, DHTTYPE);
static int taskCore = 0;
int Temp=0;
int Nivel=1;
char bufferT[28]="";

const char* ssid     = "SSID";
const char* password = "PASS";

static int Wconectado = 0;
const char* mqtt_server = "190.2.22.61";
RTC_DATA_ATTR int bootCount = 0;
WiFiClient espClient;
PubSubClient client(espClient);

void WiFiEvent(WiFiEvent_t event)
{
    Serial.printf("[WiFi-event] event: %d\n", event);

    switch (event)
    {
    case SYSTEM_EVENT_STA_GOT_IP:
      Serial.println("WiFi connected");
      Serial.println("IP address: ");
      Serial.println(WiFi.localIP());
      Wconectado=1;
      ArduinoOTA
            .onStart([]() {
            String type;
            if (ArduinoOTA.getCommand() == U_FLASH)
              type = "sketch";
            else // U_SPIFFS
              type = "filesystem";

            // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
              Serial.println("Start updating " + type);
            })
          .onEnd([]() {
              Serial.println("\nEnd");
            })
          .onProgress([](unsigned int progress, unsigned int total) {
              Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
            })
          .onError([](ota_error_t error) {
              Serial.printf("Error[%u]: ", error);
              if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
              else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
              else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
              else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
              else if (error == OTA_END_ERROR) Serial.println("End Failed");
          });

    ArduinoOTA.begin();
    Serial.println("Ready");
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      Serial.println("WiFi lost connection.  Attempting to reconnect...");
      //WiFi.reconnect();
      Wconectado=0;
      break;
    case SYSTEM_EVENT_STA_START:
      Serial.println("ESP32 station start");
      break;
    case SYSTEM_EVENT_STA_CONNECTED:
      Serial.println("ESP32 station connected to AP");
      break;
    default:      
      Serial.println("Unhandled WiFi Event raised.");
      break;
    }
}

void Wifi_init(){ 
  WiFi.disconnect(true);
  delay(1000);
  WiFi.onEvent(WiFiEvent);
  WiFi.begin(ssid, password);
}

void print_wakeup_reason(){
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch(wakeup_reason)
  {
    case 1  : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case 2  : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case 3  : Serial.println("Wakeup caused by timer"); break;
    case 4  : Serial.println("Wakeup caused by touchpad"); break;
    case 5  : Serial.println("Wakeup caused by ULP program"); break;
    default : Serial.println("Wakeup was not caused by deep sleep"); break;
  }
}

void send_mqtt(float t, float h,float hic, int n, float v){
  if (Wconectado == 1){
    if(!client.connected()) {
      Serial.print("Conectando ThingsBoard node ...");
      if ( client.connect("Esp32COSensor", TOKEN_TB, NULL) ) {
        Serial.println( "[DONE]" );
      } else {
        Serial.print( "[FAILED] [ rc = " );
        Serial.print( client.state() );
      }
    }  
    if (client.connected()) {
        
      String temperatura = String(t);
      String humedad = String(h);
      String stermica = String(hic);
      String nivel = String(n);
      String vbat = String(v);
      
      String payload = "{";
      payload += "\"temp\":";
      payload += temperatura;
      payload += ",";
      payload += "\"hum\":";
      payload += humedad;
      payload += ",";
      payload += "\"sterm\":";
      payload += stermica;
      payload += ",";
      payload += "\"nivel\":";
      payload += nivel;
      payload += ",";
      payload += "\"vbat\":";
      payload += vbat;
      payload += "}";

      // Send payload
      char attributes[200];
      payload.toCharArray( attributes, 200 );
      int rsus=client.publish( "v1/devices/me/telemetry", attributes );
      Serial.print( "Publish : ");
      Serial.println(rsus);
      //client.publish(TEMP_TOPIC, msg);
      Serial.println( attributes );
    }
  }
}

float get_vcc(){
  int steps=analogRead(ANALOG_PIN_0);
  Serial.print("Voltage Steps = ");
  Serial.println(steps);
  float VBAT = 3.90f * float(steps) / 4096.0f;  // LiPo battery
  return VBAT;
}


void setup() { 
  Serial.begin(115200);  
  pinMode(LVL_DATA,INPUT_PULLUP);
  pinMode(LVL_PWR,OUTPUT);             
  pinMode(DHT_PWR, OUTPUT);
  pinMode(DHT_DATA,INPUT_PULLUP);
  digitalWrite(DHT_PWR, HIGH);
  digitalWrite(LVL_PWR, HIGH);
  analogReadResolution(12); //12 bits
  analogSetAttenuation(ADC_11db);  //For all pins
  analogSetPinAttenuation(ANALOG_PIN_0, ADC_11db); //0db attenu
  delay(400);
  dht.begin();
  client.setServer(mqtt_server, 1883);
  print_wakeup_reason();
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  Serial.println("Setup ESP32 to sleep for every " + String(TIME_TO_SLEEP) +" Seconds");
  
  // Port defaults to 3232
  ArduinoOTA.setPort(3232);

  // Hostname defaults to esp3232-[MAC]
  ArduinoOTA.setHostname("myesp32");

  // No authentication by default
  ArduinoOTA.setPassword("quelPP77");

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  

}


void loop() {
 if (Wconectado == 0){
   Serial.println("Error No conectado wifi Wifi_init.");
   Wifi_init();
 }else{
  ArduinoOTA.handle();
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  if (isnan(h) || isnan(t)) {
    Serial.println("Error obteniendo los datos del sensor DHT11");
    delay(600);
    return;
  }
  float hic = dht.computeHeatIndex(t, h, false);
  float v =get_vcc();
  
  Serial.print("Humedad :");
  Serial.print(h);
  Serial.print(" Temperatura :");
  Serial.print(t);
  Serial.print(" Sensacion Termica :");
  Serial.println(hic);
  Nivel=!digitalRead(LVL_DATA);
  Serial.print("Nivel = ");
  Serial.print(Nivel);
  Serial.print("Battery Voltage = ");
  Serial.print(v);
  Serial.println(" V");
  
  String taskMessage = "Corriendo en core ";
  taskMessage = taskMessage + xPortGetCoreID();
  Serial.println(taskMessage);
  send_mqtt(t,h,hic,Nivel,v);
  digitalWrite(DHT_PWR, LOW);
  digitalWrite(LVL_PWR, LOW);
  Serial.println("Duerme.");
  esp_deep_sleep_start();
 }
 delay(6000); 
}
 


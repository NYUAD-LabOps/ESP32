// ESP32 WiFi <-> 3x UART Bridge
// by AlphaLima
// www.LK8000.com

// Disclaimer: Don't use  for life support systems
// or any other situations where system failure may affect
// user or environmental safety.

#include "config.h"
#include <esp_wifi.h>
#include <WiFi.h>




#ifdef BLUETOOTH
#include <BluetoothSerial.h>
BluetoothSerial SerialBT; 
#endif

#ifdef OTA_HANDLER  
#include <ArduinoOTA.h> 

#endif // OTA_HANDLER

HardwareSerial Serial_one(1);
HardwareSerial Serial_two(2);
HardwareSerial* COM[NUM_COM] = {&Serial, &Serial_one , &Serial_two};

#define MAX_NMEA_CLIENTS 4
#define PACKET_SIZE 50
#ifdef PROTOCOL_TCP
#include <WiFiClient.h>
WiFiServer server_0(SERIAL1_TCP_PORT);
// WiFiServer server_1(SERIAL1_TCP_PORT);
// WiFiServer server_2(SERIAL2_TCP_PORT);
// WiFiServer *server[NUM_COM]={&server_0,&server_1,&server_2};
WiFiClient TCPClient;
#endif


uint8_t buf1[bufferSize];
uint16_t i1 = 0;

uint8_t buf2[bufferSize];
uint16_t i2 = 0;

uint8_t BTbuf[bufferSize];
uint16_t iBT = 0;


void setup() {

  delay(500);
  // Serial.begin(115200);
  // Serial.println("test");
  COM[0]->begin(UART_BAUD0, SERIAL_PARAM0, SERIAL0_RXPIN, SERIAL0_TXPIN);
  COM[1]->begin(UART_BAUD1, SERIAL_PARAM1, SERIAL1_RXPIN, SERIAL1_TXPIN);
  // COM[2]->begin(UART_BAUD2, SERIAL_PARAM2, SERIAL2_RXPIN, SERIAL2_TXPIN);
  
  if(debug) COM[DEBUG_COM]->println("\n\nLK8000 WiFi serial bridge V1.00");
  #ifdef MODE_AP 
   if(debug) COM[DEBUG_COM]->println("Open ESP Access Point mode");
  //AP mode (phone connects directly to ESP) (no router)
  WiFi.mode(WIFI_AP);
   
  WiFi.softAP(ssid, pw); // configure ssid and password for softAP
  delay(2000); // VERY IMPORTANT
  WiFi.softAPConfig(ip, ip, netmask); // configure ip address for softAP

  #endif


  #ifdef MODE_STA
   if(debug) COM[DEBUG_COM]->println("Open ESP Station mode");
  // STATION mode (ESP connects to router and gets an IP)
  // Assuming phone is also connected to that router
  // from RoboRemo you must connect to the IP of the ESP
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pw);
  if(debug) COM[DEBUG_COM]->print("try to Connect to Wireless network: ");
  if(debug) COM[DEBUG_COM]->println(ssid);
  while (WiFi.status() != WL_CONNECTED) {   
    delay(500);
    if(debug) COM[DEBUG_COM]->print(".");
  }
  if(debug) COM[DEBUG_COM]->println("\nWiFi connected");
  
  #endif
#ifdef BLUETOOTH
  if(debug) COM[DEBUG_COM]->println("Open Bluetooth Server");  
  SerialBT.begin(ssid); //Bluetooth device name
 #endif
#ifdef OTA_HANDLER  
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
  // if DNSServer is started with "*" for domain name, it will reply with
  // provided IP to all DNS request

  ArduinoOTA.begin();
#endif // OTA_HANDLER    

  #ifdef PROTOCOL_TCP
  // COM[0]->println("Starting TCP Server 1");  
  // if(debug) COM[DEBUG_COM]->println("Starting TCP Server 1");  
  // server[0]->begin(); // start TCP server 
  // server[0]->setNoDelay(true);
  COM[1]->println("Starting TCP Server 2");
  if(debug) COM[DEBUG_COM]->println("Starting TCP Server 2");  
  server_0.begin(); // start TCP server 
  server_0.setNoDelay(true);
  // COM[2]->println("Starting TCP Server 3");
  // if(debug) COM[DEBUG_COM]->println("Starting TCP Server 3");  
  // server[2]->begin(); // start TCP server   
  // server[2]->setNoDelay(true);
  #endif

  esp_err_t esp_wifi_set_max_tx_power(50);  //lower WiFi Power
}


void loop() 
{  
#ifdef OTA_HANDLER  
  ArduinoOTA.handle();
#endif // OTA_HANDLER
  
#ifdef BLUETOOTH
  // receive from Bluetooth:
  if(SerialBT.hasClient()) 
  {
    while(SerialBT.available())
    {
      BTbuf[iBT] = SerialBT.read(); // read char from client (LK8000 app)
      if(iBT <bufferSize-1) iBT++;
    }          
    for(int num= 0; num < NUM_COM ; num++)
      COM[num]->write(BTbuf,iBT); // now send to UART(num):          
    iBT = 0;
  }  
#endif  
#ifdef PROTOCOL_TCP

    if (server_0.hasClient())
    {
        if (!TCPClient || !TCPClient.connected()){
          if(TCPClient) TCPClient.stop();
          TCPClient = server_0.available();
          if(debug) COM[DEBUG_COM]->print("New client"); 
        }
    }
  
#endif
 

           
        if(TCPClient.available()) 
        {
          i1 = 0;
          while(TCPClient.available() && i1 < PACKET_SIZE)
          {
            buf1[i1] = TCPClient.read(); // read char from client (LK8000 app)
            i1++;
          } 
          // Serial.println("\nTest.");
          // Serial.write(buf1[num], i1[num]);
          // COM[0]->write(buf1, i1);
          COM[1]->write(buf1, PACKET_SIZE); // now send to UART(num):
          
        }
      
  
      if(COM[1]->available())
      {
        i2 = 0;
        while(COM[1]->available() && i2 < PACKET_SIZE)
        {     
          buf2[i2] = COM[1]->read(); // read char from UART(num)
          // COM[0]->write(buf2, PACKET_SIZE);
          ///Debug
          // COM[0]->write(buf2[i2]);
          i2++;
        }
          // COM[0]->write(buf2, PACKET_SIZE);
          if(TCPClient)                     
            TCPClient.write(buf2, PACKET_SIZE);
        
#ifdef BLUETOOTH        
        // now send to Bluetooth:
        if(SerialBT.hasClient())      
          SerialBT.write(buf2[num], i2[num]);               
#endif  
        
      }
    }    
  



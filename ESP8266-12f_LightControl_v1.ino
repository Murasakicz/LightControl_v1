#include <ESP8266WiFi.h>
#include "FS.h"
#include <WiFiClient.h>
#include <TimeLib.h>
#include <NtpClientLib.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ESP8266mDNS.h>
#include <Ticker.h>
#include <ArduinoOTA.h>
#include <ArduinoJson.h>
#include "FSWebServerLib.h"
#include "MQTT.h"
#include <Hash.h>

MQTT MQTTServer;

#define BUTTON_0 12 // Pin with 0
#define BUTTON_1 13 // Pin with 0
#define BUTTON_2 14 // Pin with 0
#define BUTTON_3 5 // Pin with 0

enum ChanelType { UP , RIGHT, DOWN, LEFT};

uint32_t secondTimer = 0;
uint8_t movmentTimeout = 1000;
array<int,4> buttonsLast = {0,0,0,0};
array<int,4> buttonsActual = {0,0,0,0};
ChanelType activeChanel = UP; 

void callback(char* topic, uint8_t* payload, unsigned int length) {
  
}

void setup() {
    // WiFi is started inside library
    SPIFFS.begin(); // Not really needed, checked inside library and started if needed
    ESPHTTPServer.begin(&SPIFFS);
    /* add setup code here */

    Serial.begin(115200);
    pinMode(BUTTON_0, INPUT_PULLUP);
    pinMode(BUTTON_1, INPUT_PULLUP);
    pinMode(BUTTON_2, INPUT_PULLUP);
    pinMode(BUTTON_3, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(BUTTON_0), buttonPress, CHANGE);
    attachInterrupt(digitalPinToInterrupt(BUTTON_1), buttonPress, CHANGE);
    attachInterrupt(digitalPinToInterrupt(BUTTON_2), buttonPress, CHANGE);
    attachInterrupt(digitalPinToInterrupt(BUTTON_3), buttonPress, CHANGE);
    
    //const char* test =  "/Led1/value"; 

    MQTTServer.setMQTTCallback(callback);
    MQTTServer.begin(&SPIFFS, ESPHTTPServer.getDeviceName());
}

void loop() {
    noInterrupts();
    /* add main program code here */
    MQTTServer.loop();  //run the loop() method as often as possible - this keeps the MQTT services running
    //server.handleClient();
    ESPHTTPServer.mqttConnectionStatus = MQTTServer.state();

    // DO NOT REMOVE. Attend OTA update from Arduino IDE
    ESPHTTPServer.handle();	
    interrupts();        
}

void sendAction(char payload[12]){
  switch (activeChanel){
    case UP:
      MQTTServer.publish(MQTTServer.constructChanelString(0).c_str(), payload);
    break;
    case RIGHT:
      MQTTServer.publish(MQTTServer.constructChanelString(1).c_str(), payload);
    break;
    case DOWN:
      MQTTServer.publish(MQTTServer.constructChanelString(2).c_str(), payload);
    break;
    case LEFT:
      MQTTServer.publish(MQTTServer.constructChanelString(3).c_str(), payload);
    break;
  }  
}

void right(){
//  MQTTServer.publish(MQTTServer.constructChanelString().c_str(), "Right");
}

void left(){
//  MQTTServer.publish(MQTTServer.constructChanelString().c_str(), "Left");
}

void motion(){
  if (buttonsActual == array<int,4>{1,1,1,1}){
    Serial.println("test 001");
    sendAction("switch");
  } else if (buttonsActual == array<int,4>{1,1,1,0}){
    Serial.println("test 001");
  } else if (buttonsActual == array<int,4>{0,1,1,1}){
    Serial.println("test 002");
  } else if ((buttonsActual != array<int,4>{0,0,0,0}) 
              && (buttonsActual != array<int,4>{1,1,0,0})
              && (buttonsActual != array<int,4>{1,0,1,0})
              && (buttonsActual != array<int,4>{1,1,1,0})
              && (buttonsActual != array<int,4>{1,1,1,1})
              && (buttonsActual != array<int,4>{1,0,1,1})
              && (buttonsActual != array<int,4>{1,0,0,1})
              && (buttonsActual != array<int,4>{0,1,1,1})
              && (buttonsActual != array<int,4>{0,0,1,1})){
    if (millis() < (secondTimer+movmentTimeout)){
      //Serial.println("test 003");
      if (buttonsActual == array<int,4>{1,0,0,0}){
        if (buttonsLast == array<int,4>{0,0,0,1}){  //stepUp movment
              Serial.println("stepUp");
              sendAction("stepUp");
              right();
        } 
        if (buttonsLast == array<int,4>{0,1,0,0}){  //stepDown movment
              Serial.println("stepDown");
              sendAction("stepDown");
              left();
          }           
        if (buttonsLast == array<int,4>{0,0,1,0}){  //down movment
              Serial.println("down");
              activeChanel = DOWN;
          } 
      }

      if (buttonsActual == array<int,4>{0,1,0,0}){
        if (buttonsLast == array<int,4>{1,0,0,0}){  //stepUp movment
              Serial.println("stepUp");
              sendAction("stepUp");
        } 
        if (buttonsLast == array<int,4>{0,0,1,0}){  //stepDown movment
              Serial.println("stepDown");
              sendAction("stepDown");
          }          
        if (buttonsLast == array<int,4>{0,0,0,1}){  //left movment
              Serial.println("left");
              activeChanel = LEFT;
          } 
      }  

      if (buttonsActual == array<int,4>{0,0,1,0}){
        if (buttonsLast == array<int,4>{0,1,0,0}){  //stepUp movment
              Serial.println("stepUp");
              sendAction("stepUp");
        } 
        if (buttonsLast == array<int,4>{0,0,0,1}){  //stepDown movment
              Serial.println("stepDown");
              sendAction("stepDown");
          }          
        if (buttonsLast == array<int,4>{1,0,0,0}){  //up movment
              Serial.println("up");
              activeChanel = UP;
          } 
      }  

      if (buttonsActual == array<int,4>{0,0,0,1}){
        if (buttonsLast == array<int,4>{0,0,1,0}){  //stepUp movment
              Serial.println("stepUp");
              sendAction("stepUp");
        } 
        if (buttonsLast == array<int,4>{1,0,0,0}){  //stepDown movment
              Serial.println("stepDown");
              sendAction("stepDown");
          }          
        if (buttonsLast == array<int,4>{0,1,0,0}){  //right movment
              Serial.println("right");
              activeChanel = RIGHT;
          } 
      }  
    }
    memcpy( &buttonsLast, &buttonsActual, sizeof(int)*4 );
  }
}

void buttonPress(){
  int button0State = digitalRead(BUTTON_0);
  int button1State = digitalRead(BUTTON_1);
  int button2State = digitalRead(BUTTON_2);
  int button3State = digitalRead(BUTTON_3);
  
  if ( button0State != buttonsActual[0]){
    buttonsActual[0] = button0State;
    
    }
  if ( button1State != buttonsActual[1]){
    buttonsActual[1] = button1State;
    }
  if ( button2State != buttonsActual[2]){
    buttonsActual[2] = button2State;
    }
  if ( button3State != buttonsActual[3]){
    buttonsActual[3] = button3State;
    }
  motion();
  secondTimer = millis();
}

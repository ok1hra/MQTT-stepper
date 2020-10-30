/*
MQTT stepper - compile for Arduino Leonardo
-----------------------------------------------------------
https://remoteqth.com/civ-stepper.php

 ___               _        ___ _____ _  _
| _ \___ _ __  ___| |_ ___ / _ \_   _| || |  __ ___ _ __
|   / -_) '  \/ _ \  _/ -_) (_) || | | __ |_/ _/ _ \ '  \
|_|_\___|_|_|_\___/\__\___|\__\_\|_| |_||_(_)__\___/_|_|_|


  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 2 of the License, or
  (at your option) any later version.
  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.

  Credit Trinamic_TMC2130.h driver
  @author   Moritz Walter
  https://github.com/makertum/Trinamic_TMC2130

Features:
---------
- control stepper Nema 17 by received command from MQTT protocol
- support endstop for adjusting after start up
- without endstop available save lat position to retain message (not recomended)
- configure in sourcecode
- TX inhibit output during stepper operate
- reverse on/off

Changelog:
----------
2020-10 first initial version

USB debug (if enable)
--------------------
screen /dev/ttyUSB.mm 115200

MQTT monitor
------------
mosquitto_sub -v -h 54.38.157.134 -t 'OK1HRA/sm/#'

MQTT topic
------------
mosquitto_pub -h 54.38.157.134 -t OK1HRA/sm/0/sTarget -m '10' // storage last stepper stop memory, do not use
mosquitto_pub -h 54.38.157.134 -t OK1HRA/sm/0/Target -m '100'  // set new target 0-SteppersCounterLimit
mosquitto_pub -h 54.38.157.134 -t OK1HRA/sm/0/Target -m '+'   // increment or decrement with "-"
mosquitto_pub -h 54.38.157.134 -t OK1HRA/sm/0/Target -m 'H'   // homing to endstop and return to last target position
mosquitto_pub -h 54.38.157.134 -t OK1HRA/sm/0/Target -m '?'   // return raw temperature

Retain message can be use for preset
mosquitto_pub -h 54.38.157.134 -t OK1HRA/sm/0/sSteppersCounterLimit -m '1000'
mosquitto_pub -h 54.38.157.134 -t OK1HRA/sm/0/sCurrentRun -m '20'
mosquitto_pub -h 54.38.157.134 -t OK1HRA/sm/0/sReverse -m '1'
mosquitto_pub -h 54.38.157.134 -t OK1HRA/sm/0/sMicroSteps -m '16' // off, because low memory space

TIP:
----
- s[Topic] can be retain message for preset value after start up
- sTarget use for storage last position and load after start up (using do not recommend)
- for reset step counter, send sTarget topic with message 0

How to setup
------------
- configure YOUR_CALL, unique NET_ID, StepsByTurn, MicroSteps, Endstop, StepperEnable and mqttBroker IP in code sourcecode
- compile for Arduino Leonardo and upload via ICSP programmer
- control via selected mqtt broker


// Configure setting //////////////////////////////////////////*/
  String YOUR_CALL = "OK1HRA";        // master topic identification
  // byte NET_ID = 0x01;                 // NetID [hex] MUST BE UNIQUE IN NETWORK
  const int StepsByTurn = 200;        // dependancy to using stepper motor
  int MicroSteps = 16;                // [1,2,4,8,16,32,64,128,256] number of microstep
  bool Endstop = true;                // enable endstop reset
  bool StepperEnable = false;         // ENABLE switch current run/standby
                                      // DISABLE switch stepper enable/disable
  byte mqttBroker[4]={54,38,157,134}; // MQTT broker IP address
//////////////////////////////////////////////////////////////

int CurrentRun = 25;                     // [0-31] current using during running stepper motor
// C
byte NET_ID = 0x00;                 // NetID [hex] MUST BE UNIQUE IN NETWORK
bool Reverse = true;
long SteppersCounterLimit=100*MicroSteps;

// L
// byte NET_ID = 0x01;                 // NetID [hex] MUST BE UNIQUE IN NETWORK
// bool Reverse = false;
// long SteppersCounterLimit=99200;    // 200*MicroSteps*31turn

// #define SERIAL_debug             // Disable save 2% of program storage space
const char* REV = "20201030";
#define EthModule             // enable Ethernet module if installed
#define __USE_DHCP__          // enable DHCP
#define HW_BCD_SW             // Enable hardware NET-ID bcd switch on bottom side
#define MQTT             // enable Ethernet module if installed
const int BcdPin[4] = {9, 11, 5, A1};  // BCD encoder PINs
long BcdRefresh[2] = {0,500};
bool Homing = false;
bool HomingReturn = false;

#include "Trinamic_TMC2130.h"
const int StepPin = 8;
const int CS_PIN = 6;
const int DIR_PIN = 4;
const int EN_PIN = 12;
// bool CurrentStatus = false;
Trinamic_TMC2130 myStepper(CS_PIN);
const int Stepper_us = 200; // one step in us (SPEED)

const int EndStopPin = 7;
const int TxInhibitPin = 13;
const int TempPin = A1;
long EndstopDebounceTimer[2] = {0,1000};
bool NeedPublishZero = false;

// Serial
const int BAUDRATE0 = 115200; // USB CLI/Debug
bool WizardEnable = false;
int WizardStep = 0;

// Steppers
int CurrentStandby = 0;             // [0-31] current using during stepper motor standby
long EnableTimeout[2] = {0,1000};  // Time, after them current switch to standby value
unsigned int Multiplier = 1;
int uSeconds;
volatile long SteppersCounter = 0;
volatile long SteppersCounterTmp = 0;
volatile long SteppersCounterTarget = 0;
volatile int InDeCrement;

#if defined(EthModule)
  bool EthLinkStatus = 0;
  long EthLinkStatusTimer[2]={1500,1000};
  // bool EnableEthernet = 1;
  bool EnableDHCP     = 1;
  #include <Ethernet.h>
  // #include <EthernetUdp.h>
  // #include <Ethernet2.h>
  // #include <EthernetUdp2.h>
  //  #include <util.h>
  // #include <Dhcp.h>
  // #include <EthernetServer.h>
  #include <SPI.h>
  byte LastMac = 0x00 + NET_ID;

  byte mac[] = {0xDE, 0xAD, 0xBE, 0xdF, 0x22, LastMac};
  // byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, LastMac};
  // IPAddress ip(192, 168, 1, 222);         // IP
  // IPAddress gateway(192, 168, 1, 200);    // GATE
  // IPAddress subnet(255, 255, 255, 0);     // MASK
  // IPAddress myDns(8, 8, 8, 8);            // DNS (google pub)
  // EthernetServer server(80);              // Web server PORT
  // String HTTP_req;

  // MQTT
  #include <PubSubClient.h>
  // #include "PubSubClient.h" // lokalni verze s upravou #define MQTT_MAX_PACKET_SIZE 128
  int MQTT_PORT           = 1883;       // MQTT broker PORT
  bool MQTT_LOGIN         = 0;          // enable MQTT broker login
  char MQTT_USER          = 'login';    // MQTT broker user login
  char MQTT_PASS          = 'passwd';   // MQTT broker password
  // const byte mqttBroker[4]={192, 168, 1, 200}; // MQTT broker IP address
  // const byte mqttBroker[4]={78, 111, 124, 210}; // MQTT broker IP address
  IPAddress MqttServer(mqttBroker[0], mqttBroker[1], mqttBroker[2], mqttBroker[3]);       // MQTT broker IP address
  EthernetClient ethClient;
  PubSubClient mqttClient(ethClient);
  long lastMqttReconnectAttempt = 0;
  char mqttTX[128];
  char mqttPath[128];
  bool StartUpTarget = false;
#endif

//------------------------------------------------------------------------------------

void setup(){
  pinMode(StepPin, OUTPUT);
    digitalWrite(StepPin, LOW); // no step yet
  pinMode(CS_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
    digitalWrite(DIR_PIN, LOW);
  pinMode(EN_PIN, OUTPUT);
  pinMode(EndStopPin, INPUT_PULLUP);
  pinMode(TxInhibitPin, OUTPUT);
    digitalWrite(TxInhibitPin, LOW);
  pinMode(TempPin, INPUT);

  Serial.begin(BAUDRATE0, SERIAL_8N2);

  #if defined(EthModule)
    LastMac = 0x00 + NET_ID;
    mac[5] = LastMac;
    mqttClient.setServer(MqttServer, MQTT_PORT);
    mqttClient.setCallback(MqttRx);
    EthernetCheck();
  #endif

  // stepper 1
  myStepper.init();
  myStepper.set_mres(MicroSteps); // ({1,2,4,8,16,32,64,128,256}) number of microsteps
  myStepper.set_IHOLD_IRUN(CurrentRun,CurrentRun,5); // ([0-31],[0-31],[0-5]) sets all currents to maximum
  myStepper.set_I_scale_analog(1); // ({0,1}) 0: I_REF internal, 1: sets I_REF to AIN
  myStepper.set_tbl(1); // ([0-3]) set comparator blank time to 16, 24, 36 or 54 clocks, 1 or 2 is recommended
  myStepper.set_toff(8); // ([0-15]) 0: driver disable, 1: use only with TBL>2, 2-15: off time setting during slow decay phase
  myStepper.set_en_pwm_mode(1); // StealtChop mode 1-ON 0-OFF
  uStepTouSeconds();

  // if(Endstop==true){
  //   SteppersCounter = SteppersCounterLimit;
  //   SteppersCounterTarget = 0;
  // }
if(digitalRead(EndStopPin)==LOW){
  NeedPublishZero=true;
}
  // if(MicroSteps>1){
  //   StepperEnable = true;
  // }
  if(StepperEnable==true){
    digitalWrite(EN_PIN, LOW);    // LOW = enable
  }else{
    digitalWrite(EN_PIN, HIGH);    // HIGH = disable
  }
  InterruptON(1); // endstop
}
//------------------------------------------------------------------------------------

void loop(){
  EthernetCheck();
  Mqtt();
  OnTheFlyStepperControl();
  StepperWatchdog();
}

//------------------------------------------------------------------------------------
// SUBROUTINES ----------------------------------------------------------------------

void OnTheFlyStepperControl(){
  if(SteppersCounterTarget<=SteppersCounterLimit){
    if(SteppersCounterTarget!=SteppersCounter && StartUpTarget==true){
      #if defined(SERIAL_debug)
        MqttPubString("status", "RUN", false);
        Serial.print(F("RUN "));
      #endif

      if(StepperEnable==true){
        myStepper.set_IHOLD_IRUN(CurrentRun,CurrentRun,5); // ([0-31],[0-31],[0-5]) sets all currents to maximum
      }

      digitalWrite(EN_PIN, LOW);    // LOW = enable
      digitalWrite(TxInhibitPin, HIGH);

      if(SteppersCounterTarget<SteppersCounter){
        digitalWrite(DIR_PIN, !Reverse);
        InDeCrement = -1;
        #if defined(SERIAL_debug)
          Serial.print(F("> "));
        #endif
      }else if(SteppersCounterTarget>SteppersCounter){
        digitalWrite(DIR_PIN, Reverse);
        InDeCrement = 1;
        #if defined(SERIAL_debug)
          Serial.print(F("< "));
        #endif
      }
      if(MicroSteps==1){
        delay(100);
      }
      while(SteppersCounterTarget!=SteppersCounter){
        // InterruptON(0);
        digitalWrite(StepPin, HIGH);
        delayMicroseconds(uSeconds);
        digitalWrite(StepPin, LOW);
        delayMicroseconds(uSeconds);
        SteppersCounter=SteppersCounter+InDeCrement;
        #if defined(SERIAL_debug)
          Serial.print(F("."));
        #endif
      }
      EnableTimeout[0] = millis();
      #if defined(SERIAL_debug)
        Serial.println(F(" STOP"));
        MqttPubString("status", "STOP", false);
      #endif
      if(Homing==false && HomingReturn==false){
        MqttPubString("sTarget", String(SteppersCounter/MicroSteps), true);
      }
      if(HomingReturn == true){
        SteppersCounterTarget = SteppersCounterTmp;
        HomingReturn= false;
      }
      MqttPubString("TempRAW", String(analogRead(TempPin)), false);
    }
  }else{
    #if defined(SERIAL_debug)
        Serial.println(F("Target OVER limit!"));
    #endif
    SteppersCounterTarget=SteppersCounterLimit;
  }
}

//------------------------------------------------------------------------------------
void uStepSet(int us){
  // Microstep long in uSeconds
  switch (us) {
      case 1: MicroSteps = 1 ; break;
      case 2: MicroSteps = 2 ; break;
      case 4: MicroSteps = 4 ; break;
      case 8: MicroSteps = 8 ; break;
      case 16: MicroSteps = 16 ; break;
      case 32: MicroSteps = 32 ; break;
      case 64: MicroSteps = 64 ; break;
      case 128: MicroSteps = 128 ; break;
      case 256: MicroSteps = 256 ; break;
  }
}
//------------------------------------------------------------------------------------
void uStepTouSeconds(){
  // Microstep long in uSeconds
  switch (MicroSteps) {
      case 1: uSeconds = 2000 ; break;
      case 2: uSeconds = 700 ; break;
      case 4: uSeconds = 600 ; break;
      case 8: uSeconds = 300 ; break;
      case 16: uSeconds = 200 ; break;
      case 32: uSeconds = 60 ; break;
      case 64: uSeconds = 16 ; break;
      case 128: uSeconds = 11 ; break;
      case 256: uSeconds = 11 ; break;
  }
}
//------------------------------------------------------------------------------------

void InterruptON(int ON){
  if(ON==1 && Endstop==true){
    attachInterrupt(digitalPinToInterrupt(EndStopPin), EndstopNow, FALLING);
  }else{
    detachInterrupt(digitalPinToInterrupt(EndStopPin));
  }
}

//------------------------------------------------------------------------------------
void EndstopNow(){
  if(millis()-EndstopDebounceTimer[0]>EndstopDebounceTimer[1]){
    if(SteppersCounterTarget<SteppersCounter){
      SteppersCounter = 0-InDeCrement;
      Homing = false;
      HomingReturn = true;
    }
    EndstopDebounceTimer[0]=millis();
    #if defined(SERIAL_debug)
      Serial.println(F("* INTERRUPT NOW *"));
    #endif
  }
}

//------------------------------------------------------------------------------------
void StepperWatchdog(){
    if(millis()-EnableTimeout[0]>EnableTimeout[1]){
      if(StepperEnable==true){
        myStepper.set_IHOLD_IRUN(CurrentRun,CurrentRun,5); // ([0-31],[0-31],[0-5]) sets all currents to maximum
      }else{
        digitalWrite(EN_PIN, HIGH);    // HIGH = disable
      }
      digitalWrite(TxInhibitPin, LOW);
    }
}

//-------------------------------------------------------------------------------------------------------

void EthernetCheck(){
  #if defined(EthModule)
    if(millis()-EthLinkStatusTimer[0]>EthLinkStatusTimer[1]){
      if ((Ethernet.linkStatus() == Unknown || Ethernet.linkStatus() == LinkOFF) && EthLinkStatus==1) {
        EthLinkStatus=0;
        #if defined(SERIAL_debug)
          Serial.println(F("Ethernet DISCONNECTED"));
        #endif
      }else if (Ethernet.linkStatus() == LinkON && EthLinkStatus==0) {
        EthLinkStatus=1;
        #if defined(SERIAL_debug)
          Serial.println(F("Ethernet CONNECTED"));
        #endif
        if(EnableDHCP==1){
            Ethernet.begin(mac);
            IPAddress CheckIP = Ethernet.localIP();
            if( CheckIP[0]==0 && CheckIP[1]==0 && CheckIP[2]==0 && CheckIP[3]==0 ){
                #if defined(SERIAL_debug)
                  Serial.println(F("DHCP FAIL - please restart"));
                #endif
              while(1) {
                // infinite loop
              }
            }
        }else{
          // Ethernet.begin(mac, ip, myDns, gateway, subnet);     // Fixed IP
        }
        // if(Ethernet.linkStatus()==LinkON && EthLinkStatus==1){
        //   Mqtt();
        // }
        #if defined(SERIAL_debug)
          Serial.print(F("IP address: "));
          Serial.println(Ethernet.localIP());
          Serial.print(F("NET-ID: "));
          Serial.println(NET_ID, HEX);
        #endif
        // server.begin();                     // Web
        // UdpCommand.begin(UdpCommandPort);   // UDP
        // TxUDP(ThisDevice, RemoteDevice, 'b', 'r', 'o');
        // NeedRxSettings=1;
      }
      EthLinkStatusTimer[0]=millis();
    }
  #endif
}
//------------------------------------------------------------------------------------
void MqttRx(char *topic, byte *payload, unsigned int length) {
  // InterruptON(0,0,0,0); // keyb, enc, gps, Interlock
  String CheckTopicBase; // = String(YOUR_CALL) + "/OI3/" + String(NET_ID, HEX) + "/";
  CheckTopicBase.reserve(50);
  byte* p = (byte*)malloc(length);
  memcpy(p,payload,length);

  // String StringTMP = String(topic);
  // for (int i = 0; i < length; i++) {
  //   StringTMP = StringTMP + p[i];
  // }
  // Debugging(">"+StringTMP);
  // Debugging(">"+String(p[0]));

  #if defined(SERIAL_debug)
    Serial.print(F("MQTT RX "));
  #endif

  // SteppersCounterTarget AFTER START ONLY
  CheckTopicBase = String(YOUR_CALL) + "/sm/" + String(NET_ID, HEX) + "/sTarget";
  if ( CheckTopicBase.equals( String(topic) ) && StartUpTarget==false){
    SteppersCounterTarget = 0;
    unsigned long exp = 1;
    for (int i = length-1; i >=0 ; i--) {
      // Numbers only
      if(p[i]>=48 && p[i]<=58){
        SteppersCounterTarget = SteppersCounterTarget + (p[i]-48)*exp;
        exp = exp*10;
      }
    }
    SteppersCounterTarget=SteppersCounterTarget*MicroSteps;
    SteppersCounter = SteppersCounterTarget;
    #if defined(SERIAL_debug)
      Serial.print(F("sTarget "));
      Serial.print(SteppersCounterTarget);
    #endif
    StartUpTarget=true;
  }

  // SteppersCounterTarget
  CheckTopicBase = String(YOUR_CALL) + "/sm/" + String(NET_ID, HEX) + "/Target";
  if ( CheckTopicBase.equals( String(topic) )){
    unsigned long exp = 1;
    Serial.println(p[0], DEC);
    // +
    if(p[0]==43){
      SteppersCounterTarget = SteppersCounter+MicroSteps;
    // -
    }else if(p[0]==45){
      SteppersCounterTarget = SteppersCounter-MicroSteps;
    // Homing
    }else if(p[0]==72){
      SteppersCounterTmp = SteppersCounter;
      SteppersCounter = SteppersCounterLimit*2;
      SteppersCounterTarget = 0;
      Homing = true;
    }else{
      SteppersCounterTarget = 0;
      for (int i = length-1; i >=0 ; i--) {
        // Numbers only
        if(p[i]>=48 && p[i]<=58){
          SteppersCounterTarget = SteppersCounterTarget + (p[i]-48)*exp;
          exp = exp*10;
        }else{
          MqttListSettings();
          break;
        }
      }
      SteppersCounterTarget=SteppersCounterTarget*MicroSteps;
    }
    #if defined(SERIAL_debug)
      Serial.print(F("Target "));
      Serial.print(SteppersCounterTarget);
    #endif
  }


  // SteppersCounterLimit
  CheckTopicBase = String(YOUR_CALL) + "/sm/" + String(NET_ID, HEX) + "/sSteppersCounterLimit";
  if ( CheckTopicBase.equals( String(topic) )){
    SteppersCounterLimit = 0;
    unsigned long exp = 1;
    for (int i = length-1; i >=0 ; i--) {
      // Numbers only
      if(p[i]>=48 && p[i]<=58){
        SteppersCounterLimit = SteppersCounterLimit + (p[i]-48)*exp;
        exp = exp*10;
      }
    }
    SteppersCounterLimit = SteppersCounterLimit * MicroSteps;
    MqttListSettings();
    #if defined(SERIAL_debug)
      Serial.print(F("sSteppersCounterLimit "));
      Serial.print(SteppersCounterLimit);
    #endif
  }

  // CurrentRun
  CheckTopicBase = String(YOUR_CALL) + "/sm/" + String(NET_ID, HEX) + "/sCurrentRun";
  if ( CheckTopicBase.equals( String(topic) )){
    CurrentRun = 0;
    unsigned long exp = 1;
    for (int i = length-1; i >=0 ; i--) {
      // Numbers only
      if(p[i]>=48 && p[i]<=58){
        CurrentRun = CurrentRun + (p[i]-48)*exp;
        exp = exp*10;
      }
    }
    if(CurrentRun>31){
      CurrentRun=31;
    }
    MqttListSettings();
    #if defined(SERIAL_debug)
      Serial.print(F("sCurrentRun "));
      Serial.print(CurrentRun);
    #endif
  }

  // Reverse
  CheckTopicBase = String(YOUR_CALL) + "/sm/" + String(NET_ID, HEX) + "/sReverse";
  if ( CheckTopicBase.equals( String(topic) )){
    Reverse = 0;
    unsigned long exp = 1;
    // 0/1 only
    if(p[0]>=48 && p[0]<=49){
      Reverse = p[0]-48;
    }
    MqttListSettings();
    #if defined(SERIAL_debug)
    Serial.print(F("sReverse "));
    Serial.print(Reverse);
    #endif
  }

  // #if !defined(SERIAL_debug)
  //   // MicroSteps
  //   CheckTopicBase = String(YOUR_CALL) + "/sm/" + String(NET_ID, HEX) + "/sMicroSteps";
  //   if ( CheckTopicBase.equals( String(topic) )){
  //     int intBuf = 0;
  //     unsigned long exp = 1;
  //     for (int i = length-1; i >=0 ; i--) {
  //       // Numbers only
  //       if(p[i]>=48 && p[i]<=58){
  //         intBuf = intBuf + (p[i]-48)*exp;
  //         exp = exp*10;
  //       }
  //     }
  //     uStepSet(intBuf);
  //     myStepper.set_mres(MicroSteps); // ({1,2,4,8,16,32,64,128,256}) number of microsteps
  //     MqttListSettings();
  //     // #if defined(SERIAL_debug)
  //       Serial.print(F("sMicroSteps "));
  //       Serial.print(MicroSteps);
  //     // #endif
  //   }
  // #endif

  #if defined(SERIAL_debug)
    Serial.println();
  #endif

  // InterruptON(1,1,1,1); // keyb, enc, gps, Interlock
}
//-------------------------------------------------------------------------------------------------------
bool mqttReconnect() {
  if (mqttClient.connect(mac)) {
    // InterruptON(0,0,0,0); // keyb, enc, gps, Interlock
    IPAddress IPlocalAddr = Ethernet.localIP();                           // get
    String IPlocalAddrString = String(IPlocalAddr[0]) + "." + String(IPlocalAddr[1]) + "." + String(IPlocalAddr[2]) + "." + String(IPlocalAddr[3]);   // to string
    IPlocalAddrString.reserve(15);
    MqttPubString("IP", IPlocalAddrString, true);
    MqttPubString("REV", REV, false);
    MqttListSettings();
    if(NeedPublishZero==true){
      MqttPubString("sTarget", "0", true);
      NeedPublishZero=false;
    }


    #if defined(SERIAL_debug)
      Serial.print(F("MQTT connected "));
      Serial.println(MqttServer);
    // resubscribe
      Serial.print(F("Subscribe "));
      Serial.print(String(YOUR_CALL) + "/sm/" + String(NET_ID, HEX) + "/");
    #endif

      // sTarget
      String topic = String(YOUR_CALL) + "/sm/" + String(NET_ID, HEX) + "/sTarget";
      topic.reserve(30);
      const char *cstr0 = topic.c_str();
      if(mqttClient.subscribe(cstr0)==true){
        #if defined(SERIAL_debug)
          Serial.println(F("sTarget"));
        #endif
      }

      // target
      topic = String(YOUR_CALL) + "/sm/" + String(NET_ID, HEX) + "/Target";
      topic.reserve(30);
      const char *cstr1 = topic.c_str();
      if(mqttClient.subscribe(cstr1)==true){
        #if defined(SERIAL_debug)
          Serial.println(F("Target"));
        #endif
      }

      // set Reverse
      topic = String(YOUR_CALL) + "/sm/" + String(NET_ID, HEX) + "/sReverse";
      topic.reserve(30);
      const char *cstr2 = topic.c_str();
      if(mqttClient.subscribe(cstr2)==true){
        #if defined(SERIAL_debug)
          Serial.println(F("sReverse"));
        #endif
      }

      // SteppersCounterLimit
      topic = String(YOUR_CALL) + "/sm/" + String(NET_ID, HEX) + "/sSteppersCounterLimit";
      topic.reserve(30);
      const char *cstr3 = topic.c_str();
      if(mqttClient.subscribe(cstr3)==true){
        #if defined(SERIAL_debug)
          Serial.println(F("sSteppersCounterLimit"));
        #endif
      }

      // CurrentRun
      topic = String(YOUR_CALL) + "/sm/" + String(NET_ID, HEX) + "/sCurrentRun";
      topic.reserve(30);
      const char *cstr4 = topic.c_str();
      if(mqttClient.subscribe(cstr4)==true){
        #if defined(SERIAL_debug)
          Serial.println(F("sCurrentRun"));
        #endif
      }

      // MicroSteps
      topic = String(YOUR_CALL) + "/sm/" + String(NET_ID, HEX) + "/sMicroSteps";
      topic.reserve(30);
      const char *cstr5 = topic.c_str();
      if(mqttClient.subscribe(cstr5)==true){
        #if defined(SERIAL_debug)
          Serial.println(F("sMicroSteps"));
        #endif
      }

    // InterruptON(1,1,1,1); // keyb, enc, gps, Interlock
  }
  return mqttClient.connected();
}

//-------------------------------------------------------------------------------------------------------
void Mqtt(){
  #if defined(EthModule)
    if (EthLinkStatus==1){
      // InterruptON(0,0,0,0); // keyb, enc, gps, Interlock
      if (!mqttClient.connected()) {
        long now = millis();
        if (now - lastMqttReconnectAttempt > 5000) {
          lastMqttReconnectAttempt = now;
          // Attempt to reconnect
          if (mqttReconnect()) {
            lastMqttReconnectAttempt = 0;
          }
        }
      } else {
        // Client connected
        mqttClient.loop();
      }
      // InterruptON(1,1,1,1); // keyb, enc, gps, Interlock
    }
  #endif
}
//------------------------------------------------------------------------------------
void MqttListSettings(){
  MqttPubString("Reverse", String(Reverse), false);
  MqttPubString("SteppersCounterLimit", String(SteppersCounterLimit/MicroSteps), false);
  MqttPubString("CurrentRun", String(CurrentRun), false);
  MqttPubString("MicroSteps", String(MicroSteps), false);
  MqttPubString("TempRAW", String(analogRead(TempPin)), false);
  // MqttPubString("TempC", getTemp(), false);
}

//------------------------------------------------------------------------------------
String getTemp(){
  const int analogPin = TempPin; // replace 0 with analog pin
  const float invBeta = 1.00 / 4250.00;   // replace "Beta" with beta of thermistor
  const  float adcMax = 1023.00;
  const float invT0 = 1.00 / 298.15;   // room temp in Kelvin
  int adcVal, i, numSamples = 5;
  float  K, C;

  adcVal = analogRead(analogPin);
  // for (i = 0; i < numSamples; i++)
  //  {
  //    adcVal = adcVal + analogRead(analogPin);
  //    delay(100);
  //  }
  // adcVal = adcVal/5;
  K = 1.00 / (invT0 + invBeta*(log ( adcMax / (float) adcVal - 1.00)));
  C = K - 273.15;                      // convert to Celsius
  return String(C);
}

//------------------------------------------------------------------------------------
void MqttPubString(String TOPIC, String DATA, bool RETAIN){
  TOPIC.reserve(128);
  DATA.reserve(128);
  char charbuf[50];
   memcpy( charbuf, mac, 6);
   charbuf[6] = 0;
  // InterruptON(0,0,0); // keyb, enc, gps
  if(EthLinkStatus==1 && mqttClient.connected()==true){
    // InterruptON(0,0,0,0); // keyb, enc, gps, Interlock
    if (mqttClient.connect(charbuf)) {
      TOPIC = String(YOUR_CALL) + "/sm/" + String(NET_ID, HEX) + "/" + TOPIC;
      TOPIC.toCharArray( mqttPath, 128 );
      DATA.toCharArray( mqttTX, 128 );
      mqttClient.publish(mqttPath, mqttTX, RETAIN);
    }
  }
  // InterruptON(1,1,1); // keyb, enc, gps
}

//------------------------------------------------------------------------------------

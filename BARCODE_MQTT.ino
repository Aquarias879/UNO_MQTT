#include <usbhid.h>
#include <usbhub.h>
#include <hiduniversal.h>
#include <hidboot.h>
#include <SPI.h>
#include <ArduinoUniqueID.h>
#include <UrlEncode.h>
#include "WiFiEsp.h"
#include <PubSubClient.h>
#ifndef HAVE_HWSERIAL1
#include "SoftwareSerial.h"
SoftwareSerial Serial1(2, 3); // RX, TX
#endif

#define LED 19
long lastSendTime = 0;        // last send time to gateway
int interval = 5*1000;          // interval between sends to gateway
int rsttime = 30*10000;
int Door = 0;  //門的顏色與狀態

//開鎖狀態
boolean lock1 = true;  
boolean lock2 = true;  
boolean lock3 = true;

char ssid[] = "JWISDOM";
char pass[] = "062340161";
int status = WL_IDLE_STATUS;

const char* mqttServer = "broker.emqx.io";
const int mqttPort = 1883;
const char* ClientID = " ";
const char* Read_topic = "state/read";
const char* control_topic = "door/control";
const char* barcode_topic = "barcode/trans";
const int LEDin[3] = {7, 6, 5};
String ArduinoID = "";
String uniqueID = "";
String IDnumber = "";
String incoming = "";                 // payload of packet
String Comparison = "";
String colorlock = "RYG";
String color;
String number = "";
String message= "";
String Status = "UnLock";
int Count = 0;

const uint8_t numKeys[10] PROGMEM = {'!', '@', '#', '$', '%', '^', '&', '*', '(', ')'};  // define symbols  
const uint8_t symKeysUp[14] PROGMEM = {'_', '+', '{', '}', '|',':', '4', '`' , '<', '>', '?','(',')'}; // define symbols  
const uint8_t symKeysLo[12] PROGMEM = {'-', '=', '[', ']', '\\', ';', '\'', '`', ',', '.', '/'}; // define symbols  
const uint8_t padKeys[4] PROGMEM = {'/', '*', '-', '+'}; // define symbols  
bool shift = true;

WiFiEspClient espClient;
PubSubClient client(espClient);

class MyParser : public HIDReportParser{
  public:
    MyParser();
    void Parse(USBHID *hid, bool is_rpt_id, uint8_t len, uint8_t *buf);
  protected:
    uint8_t KeyToAscii(bool upper, uint8_t mod, uint8_t key);
    virtual void OnKeyScanned(bool upper, uint8_t mod, uint8_t key);
    virtual void OnScanFinished(); 
};

MyParser::MyParser() {}

void MyParser::Parse(USBHID *hid, bool is_rpt_id, uint8_t len, uint8_t *buf) {
  // 如果錯誤或為空，則返回
  //if (buf[2] == 1 || buf[2] == 0) return;
  if (buf[0] == 2)
      shift = true;
  //for (uint8_t i = 7; i >= 2; i--) {
  for (uint8_t i = 2; i <= 7; i++) {
    // 如果為空，跳過
    if (buf[i] == 0) continue;

    // 如果輸入信號發出，掃描完成
    if (buf[i] == UHS_HID_BOOT_KEY_ENTER) {
      OnScanFinished();
    }

    // 如果沒有，正常繼續
    else {
      // 如果位置不在2中，則為大寫字
      OnKeyScanned(shift, buf, buf[i]);
      shift = false;
    }
    return;
  }
}

uint8_t MyParser::KeyToAscii(bool upper, uint8_t mod, uint8_t key) {
  // Letters(字母)
  if (VALUE_WITHIN(key, 0x04, 0x1d)) {
    if (upper) return (key - 4 + 'A');
    else return (key - 4 + 'a');
  }
  // Numbers(數字)
  else if (VALUE_WITHIN(key, 0x1e, 0x29)) {
    if (shift)
      return ((uint8_t)pgm_read_byte(&numKeys[key - 0x29]));
    else
      return ((key == UHS_HID_BOOT_KEY_ZERO) ? '0' : key - 0x1e + '1');
  }
  // keypad
  else if (VALUE_WITHIN(key, 0x59, 0x61)) {
    return (key - 0x59 + '1');}
  else if (VALUE_WITHIN(key, 0x2d, 0x38))
    return ((upper) ? (uint8_t)pgm_read_byte(&symKeysUp[key - 0x2d]) : (uint8_t)pgm_read_byte(&symKeysLo[key - 0x2d]));
  else if (VALUE_WITHIN(key, 0x54, 0x58))
    return (uint8_t)pgm_read_byte(&padKeys[key - 0x54]);
  else {
    switch (key) {
      case UHS_HID_BOOT_KEY_SPACE: return (0x20);
      case UHS_HID_BOOT_KEY_ENTER: return ('\r'); // Carriage return (0x0D)
    }
  }
  return 0;
}
bool check_state = false;

void MyParser::OnKeyScanned(bool upper, uint8_t mod, uint8_t key) {
  uint8_t ascii = KeyToAscii(upper, mod, key);
  number += char(ascii);
  return number;
}

//掃描完成
void MyParser::OnScanFinished() {
  String barcode1 = ArduinoID + "*" +number;
  String barcode = urlEncode(barcode1);
  delay(500);
  client.publish(barcode_topic,String(barcode).c_str());
  Serial.println(barcode);
  Count = 0;
  number = "";
  incoming ="";
  IDnumber = "";
}

USB          Usb;
USBHub       Hub(&Usb);
HIDUniversal Hid(&Usb);
MyParser     Parser;

void callback(char *topic, byte *payload, unsigned int length) {
  for (int i = 0; i < length; i++) {
    //Serial.print((char)payload[i]);
    message += (char)payload[i];
  }
  //Serial.print(message);
  if (message[6] == 'R') {  // Use single quotes for character comparison
    Serial.println(message[6]);
    digitalWrite(A2, LOW);  // Turn on the LED on pin 13
    delay(100);
    //digitalWrite(13, LOW);
    message = "";
  }
  if (message[6] == 'Y') {  // Use single quotes for character comparison
    Serial.println(message[6]);
    digitalWrite(A2, HIGH);  // Turn on the LED on pin 13
    delay(100);
    //digitalWrite(A2, LOW);
    message = "";
  }
  if (message[6] == 'G') {  // Use single quotes for character comparison
    Serial.println(message[6]);
    digitalWrite(A3, HIGH);  // Turn on the LED on pin 13
    delay(100);
    digitalWrite(A3, LOW);
    message = "";
  }}
 
void setup() {
    Serial.begin(9600);
    Serial1.begin(9600);
    Serial.println("Start");
    
    if (Usb.Init() == -1) {
      Serial.println("OSC did not start.");
    }
    delay( 200 );
    Hid.SetReportParser(0, &Parser);

    //腳位設定
    pinMode(A1,OUTPUT); 
    pinMode(A2,OUTPUT);
    pinMode(A3,OUTPUT);
    pinMode(13,OUTPUT);
    pinMode(LED,OUTPUT);
    
    pinMode(LEDin[0],INPUT); 
    pinMode(LEDin[1],INPUT);
    pinMode(LEDin[2],INPUT);
    WiFi.init(&Serial1);

    if (WiFi.status() == WL_NO_SHIELD) {
        Serial.println("WiFi shield not present");
        while (true);
    }

    while (status != WL_CONNECTED) {
        Serial.print("Attempting to connect to WPA SSID: ");
        Serial.println(ssid);
        status = WiFi.begin(ssid, pass);
    }
    //Serial.println("You're connected to the network");
    printWifiStatus();
    client.setServer(mqttServer, mqttPort);
    client.setCallback(callback);
    while (!client.connected()) {
    Serial.println("Connecting to MQTT broker...");
    if (client.connect(ClientID)) {
        Serial.println("Connected to MQTT broker");
        client.subscribe(control_topic);
    } else {
        Serial.print("Failed to connect to MQTT broker. Retrying...");
        delay(5000);
    }}

    for (size_t i = 0; i < 8; i++)
      {
        if (UniqueID8[i] < 0x10)
        {
          uniqueID = 0 + String(UniqueID8[i], HEX);
        }
        else
        {
          uniqueID = String(UniqueID8[i], HEX);
        }
        ArduinoID += uniqueID;
    }
    client.subscribe(control_topic);
}

void(* resetFunc) (void) = 0; //製造重啟命令 

void loop() {
  digitalWrite(LED,HIGH);
  client.loop();
  Usb.Task();
  if(digitalRead(A1) != HIGH && lock1 != true)
  {
    Door = 1;
    lock1 = true;
  }
  //紅門開
  if(digitalRead(A1) == HIGH && lock1 == true)
  {
    Door = 2;
    lock1 = false;
  }

  //黃門關
  if(digitalRead(A2) != HIGH && lock2 != true)
  {    
    Door = 3;
    lock2 = true;
  }
  //黃門開
  if(digitalRead(A2) == HIGH && lock2 == true)
  {
    Door = 4;
    lock2 = false;
  }

  //綠門關
  if(digitalRead(A3) != HIGH && lock3 != true)
  { 
    Door = 5;  
    lock3= true;
  }
  //綠門開
  if(digitalRead(A3) == HIGH && lock3 == true)
  {
    Door = 6;
    lock3 = false;
  }
  if (Door == 0) {
    if (millis() - lastSendTime > interval) {
      if (millis() >= rsttime) {
        resetFunc(); //重啟程序開始
      } else {
        client.loop();
        Serial.println(millis());
      }
      lastSendTime = millis(); // timestamp the message
    }
} 
  else {
    if (check_state == false){
      if(Door == 1 || Door == 2)
      {
        String message = ArduinoID + "*R*" + String(Door);
        delay(1500);
        //sendMessage(message);
        client.publish(Read_topic, String(message).c_str());
        //check_state = true;
        Serial.println(message);
        Door = 0;          
      }
        
      if(Door == 3 || Door == 4)
      {
        String message = ArduinoID + "*Y*" + String(Door);
        delay(1500);
        //sendMessage(message);
        Serial.println(message);
        client.publish(Read_topic, String(message).c_str());
        //check_state = true;
        Door = 0;          
      }

      if(Door == 5 || Door == 6)
      {
        String message = ArduinoID + "*G*" + String(Door);
        delay(1500);
        //sendMessage(message);
        Serial.println(message);
        client.publish(Read_topic, String(message).c_str());
        //check_state = true;     
        Door = 0;    
      }
    }
  }
  digitalWrite(LED,LOW);
  if (!client.connected()) {
    resetFunc();
  }
}
    
void printWifiStatus() {
    Serial.print("SSID: ");
    Serial.println(WiFi.SSID());
    IPAddress ip = WiFi.localIP();
    Serial.print("IP Address: ");
    Serial.println(ip);
    long rssi = WiFi.RSSI();
    Serial.print("Signal strength (RSSI):");
    Serial.print(rssi);
    Serial.println(" dBm");
}



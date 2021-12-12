#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif



BluetoothSerial SerialBT;


#include <WiFi.h>

#include <ArduinoOTA.h>
#include <Update.h>
#include <ArduinoJson.h>

#include <EEPROM.h>

#include <ModbusMaster.h>
#include "REG_CONFIG.h"
#include <HardwareSerial.h>
#include "HardwareSerial_NB_BC95.h"

#define trigWDTPin    32
#define ledHeartPIN   0

#include <TaskScheduler.h>

#define _TASK_TIMECRITICAL
HardwareSerial_NB_BC95 AISnb;
HardwareSerial modbus(2);

boolean isValid = true;
uint16_t data[64];
uint8_t j, result;
/**********************************************  WIFI Client 注意编译时要设置此值 *********************************
   wifi client
*/
const char* ssid = "greenio"; //replace "xxxxxx" with your WIFI's ssid
const char* password = "green7650"; //replace "xxxxxx" with your WIFI's password

//WiFi&OTA 参数
String HOSTNAME = "CRAWaterTreatment01";
#define PASSWORD "7650" //the password for OTA upgrade, can set it in any char you want




String deviceToken = "8966031940014308369";
String serverIP = "147.50.151.130"; // Your Server IP;
String serverPort = "19956"; // Your Server Port;

String json = "";

ModbusMaster node;
void t1CallgetMeter();
void t2CallsendViaNBIOT();
void t3CallHeartbeat();
//void t4Restart();
//TASK
Task t1(30000, TASK_FOREVER, &t1CallgetMeter);
Task t2(60000, TASK_FOREVER, &t2CallsendViaNBIOT);
//Task t3(5000, TASK_FOREVER, &t3CallHeartbeat);
//Task t4(3600000, TASK_FOREVER, &t4Restart);

Scheduler runner;
String _config = "";
unsigned long _epoch = 0;
String _IP = "";
String dataJson = "";
boolean validEpoc = false;

StaticJsonDocument<400> doc;

struct Device
{


  String DO;
  String TempDO;
  String pH1;
  String pH2;
  String TempH;
  String TemppH;
  String TSS1;
  String TSS2;
  String Cond;
  String TempCond;
  String COD;
  String BOD;

};
Device device;
signal meta ;


//********************************************************************//
//*********************** HeartBeat Function **************************//
//********************************************************************//
void HeartBeat() {
  //   Sink current to drain charge from watchdog circuit
  //  pinMode(trigWDTPin, OUTPUT);
  //  digitalWrite(trigWDTPin, LOW);
  //
  //  // Led monitor for Heartbeat
  //  digitalWrite(ledHeartPIN, LOW);
  //  delay(300);
  //  digitalWrite(ledHeartPIN, HIGH);
  //
  //  // Return to high-Z
  //  pinMode(trigWDTPin, INPUT);

  Serial.println("Heartbeat sent");
  //    SerialBT.println("Heartbeat sent");
}


String getMacAddress() {
  uint8_t baseMac[6];
  // Get MAC address for WiFi station
  esp_read_mac(baseMac, ESP_MAC_WIFI_STA);
  char baseMacChr[18] = {0};
  sprintf(baseMacChr, "%02X%02X%02X%02X%02X%02X", baseMac[0], baseMac[1], baseMac[2], baseMac[3], baseMac[4], baseMac[5]);
  return String(baseMacChr);
}

char  char_to_byte(char c)
{
  if ((c >= '0') && (c <= '9'))
  {
    return (c - 0x30);
  }
  if ((c >= 'A') && (c <= 'F'))
  {
    return (c - 55);
  }
}


void _init() {

  AISnb.setupDevice(serverPort);
  HeartBeat();

  do {
    UDPSend udp = AISnb.sendUDPmsgStr(serverIP, serverPort, _config);
    dataJson = "";
    deviceToken = AISnb.getNCCID();
    Serial.print("nccid:");
    Serial.println(deviceToken);
    _config = "{\"_type\":\"retrattr\",\"Tn\":\"";
    _config.concat(deviceToken);
    _config.concat("\",\"keys\":[\"epoch\",\"ip\"]}");
    Serial.println(_config);
    HeartBeat();
    UDPReceive resp = AISnb.waitResponse();
    AISnb.receive_UDP(resp);
    Serial.print("waitData:");
    Serial.println(resp.data);
    SerialBT.println(resp.data);

    for (int x = 0; x < resp.data.length(); x += 2)
    {
      char c =  char_to_byte(resp.data[x]) << 4 | char_to_byte(resp.data[x + 1]);

      dataJson += c;
    }
    Serial.println(dataJson);
    SerialBT.println(dataJson);
    DeserializationError error = deserializeJson(doc, dataJson);

    // Test if parsing succeeds.
    if (error) {
      Serial.print(F("deserializeJson() failed: "));
      Serial.println(error.f_str());
      validEpoc = true;
      delay(4000);
    } else {
      validEpoc = false;
      unsigned long epoch = doc["epoch"];
      _epoch = epoch;
      String ip = doc["ip"];
      _IP = ip;
      Serial.println(dataJson);
      Serial.print("epoch:");  Serial.println(_epoch);
      _writeEEPROM(_IP);
      Serial.println(_IP);

    }
    delay(5000);
    HeartBeat();
  } while (validEpoc);


}


void writeString(char add, String data)
{
  EEPROM.begin(512);
  int _size = data.length();
  int i;
  for (i = 0; i < _size; i++)
  {
    EEPROM.write(add + i, data[i]);
  }
  EEPROM.write(add + _size, '\0'); //Add termination null character for String Data
  EEPROM.commit();
}


String read_String(char add)
{
  int i;
  char data[100]; //Max 100 Bytes
  int len = 0;
  unsigned char k;
  k = EEPROM.read(add);
  while (k != '\0' && len < 500) //Read until null character
  {
    k = EEPROM.read(add + len);
    data[len] = k;
    len++;
  }
  data[len] = '\0';
  Serial.print("Debug:");
  Serial.println(String(data));
  return String(data);
}


void setupOTA()
{
  //Port defaults to 8266
  //ArduinoOTA.setPort(8266);

  //Hostname defaults to esp8266-[ChipID]
  ArduinoOTA.setHostname(HOSTNAME.c_str());

  //No authentication by default
  ArduinoOTA.setPassword(PASSWORD);

  //Password can be set with it's md5 value as well
  //MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  //ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA.onStart([]()
  {
    Serial.println("Start Updating....");

    Serial.printf("Start Updating....Type:%s\n", (ArduinoOTA.getCommand() == U_FLASH) ? "sketch" : "filesystem");
  });

  ArduinoOTA.onEnd([]()
  {

    Serial.println("Update Complete!");
    SerialBT.println("Update Complete!");
    ESP.restart();
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total)
  {
    String pro = String(progress / (total / 100)) + "%";
    int progressbar = (progress / (total / 100));
    //int progressbar = (progress / 5) % 100;
    //int pro = progress / (total / 100);


    SerialBT.printf("Progress: %u%%\n", (progress / (total / 100)));
    Serial.printf("Progress: %u%%\n", (progress / (total / 100)));
  });

  ArduinoOTA.onError([](ota_error_t error)
  {
    Serial.printf("Error[%u]: ", error);
    String info = "Error Info:";
    switch (error)
    {
      case OTA_AUTH_ERROR:
        info += "Auth Failed";
        Serial.println("Auth Failed");
        break;

      case OTA_BEGIN_ERROR:
        info += "Begin Failed";
        Serial.println("Begin Failed");
        break;

      case OTA_CONNECT_ERROR:
        info += "Connect Failed";
        Serial.println("Connect Failed");
        break;

      case OTA_RECEIVE_ERROR:
        info += "Receive Failed";
        Serial.println("Receive Failed");
        break;

      case OTA_END_ERROR:
        info += "End Failed";
        Serial.println("End Failed");
        break;
    }


    Serial.println(info);
    ESP.restart();
  });

  ArduinoOTA.begin();
}

void setupWIFI()
{

  Serial.println("Connecting...");
  Serial.println(String(ssid));
  SerialBT.println("Connecting...");
  SerialBT.println(String(ssid));

  //连接WiFi，删除旧的配置，关闭WIFI，准备重新配置
  //  WiFi.disconnect(true);
  //  delay(1000);

  WiFi.mode(WIFI_STA);
  //WiFi.onEvent(WiFiEvent);
  WiFi.setAutoConnect(true);
  WiFi.setAutoReconnect(true);    //断开WiFi后自动重新连接,ESP32不可用
  WiFi.setHostname(HOSTNAME.c_str());
  WiFi.begin(ssid, password);


  //等待5000ms，如果没有连接上，就继续往下
  //不然基本功能不可用
  byte count = 0;
  while (WiFi.status() != WL_CONNECTED && count < 10)
  {
    count ++;
    delay(500);
    Serial.print(".");
  }


  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("Connecting...OK.");
    SerialBT.println("Connecting...OK.");
  } else {
    Serial.println("Connecting...Failed");
    SerialBT.println("Connecting...Failed");
  }
}

void setup()
{
  Serial.begin(115200);
  SerialBT.begin(HOSTNAME); //Bluetooth
  HOSTNAME.concat(getMacAddress());


  // Sink current to drain charge from watchdog circuit
  pinMode(trigWDTPin, OUTPUT);
  digitalWrite(trigWDTPin, LOW);




  Serial.println();
  Serial.println(F("***********************************"));
  Serial.println("Initialize...");
  SerialBT.println(F("***********************************"));
  SerialBT.println("Initialize...");
  setupWIFI();
  setupOTA();

  HeartBeat();
  runner.init();
  Serial.println("Initialized scheduler");

  runner.addTask(t1);
  Serial.println("added t1");
  runner.addTask(t2);
  Serial.println("added t2");
  //  runner.addTask(t3);
  //  Serial.println("added t3");
  //  runner.addTask(t4);
  //  Serial.println("added t4");
  HeartBeat();


  t1.enable();  Serial.println("Enabled t1");
  t2.enable();  Serial.println("Enabled t2");
  //  t3.enable();  Serial.println("Enabled t3");
  //  t4.enable();  Serial.println("Enabled t4");

  HeartBeat();


  _init();
  HeartBeat();

  _loadConfig();




  modbus.begin(9600, SERIAL_8N2, 16, 17);


}

void t2CallsendViaNBIOT ()
{
  Serial.println("#####################t2CallsendViaNBIOT###################");

  meta = AISnb.getSignal();
  HeartBeat();
  Serial.print("RSSI:"); Serial.println(meta.rssi);

  json = "";

  json.concat(" {\"Tn\":\"");
  json.concat(deviceToken);
  json.concat("\"");


  json.concat(",\"DO\":");
  json.concat(device.DO);
  json.concat(",\"TempDO\":");
  json.concat(device.TempDO);
  json.concat(",\"pH1\":");
  json.concat(device.pH1);
  json.concat(",\"pH2\":");
  json.concat(device.pH2);
  json.concat(",\"TempH\":");
  json.concat(device.TempH);
  json.concat(",\"TSS1\":");
  json.concat(device.TSS1);
  json.concat(",\"Cond\":");
  json.concat(device.Cond);
  json.concat(",\"TempCond\":");
  json.concat(device.TempCond);
  json.concat(",\"TSS2\":");
  json.concat(device.TSS2);
  json.concat(",\"COD\":");
  json.concat(device.COD);
  json.concat(",\"BOD\":");
  json.concat(device.BOD);
  json.concat(",\"rssi\":");
  json.concat(meta.rssi);
  json.concat(",\"csq\":");
  json.concat(meta.csq);
  json.concat("}");

  Serial.println(json);
  SerialBT.println(json);

  //
  if (isValid) {
    UDPSend udp = AISnb.sendUDPmsgStr(serverIP, serverPort, json);
    Serial.print("udp.strsend:"); Serial.println(udp.strsend);
    Serial.print("udp.status:"); Serial.println(udp.status);

    SerialBT.print("udp.strsend:"); Serial.println(udp.strsend);
    SerialBT.print("udp.status:"); Serial.println(udp.status);
    isValid = true;
  }

}



void _writeEEPROM(String data) {
  Serial.print("Writing Data:");
  Serial.println(data);

  writeString(10, data);  //Address 10 and String type data
  delay(10);
}

void _loadConfig() {
  serverIP = read_String(10);
  serverIP.trim();
  Serial.print("IP:");
  Serial.println(serverIP);
}

void getMODBUS()
{
  float i = 0;

  modbus.begin(9600, SERIAL_8N2, 16, 17);

  result = node.readHoldingRegisters(0x0000, 64);
  delay(1000);
  // do something with data if read is successful
  if (result == node.ku8MBSuccess)
  {
    for (j = 0; j < 64; j++)
    {
      data[j] = node.getResponseBuffer(j);
      Serial.print(j); Serial.print(":");      Serial.print(data[j]); Serial.print(",");
      SerialBT.print(j); SerialBT.print(":");      SerialBT.print(data[j]); SerialBT.print(",");

    }
    SerialBT.println();
    Serial.println();
  }

}


float getDO()
{
  float param = 0.0;
  uint32_t value = 0;

  value = data[4];
  value = value << 16;
  value = value + data[5];
  param = HexTofloat(value);
  if (param <= 0)
    isValid = false;
  else
    isValid = true;
  return param;
}

float getTempDO()
{
  float param = 0.0;
  uint32_t value = 0;

  value = data[6];
  value = value << 16;
  value = value + data[7];
  param = HexTofloat(value);
  if (param <= 0)
    isValid = false;
  else
    isValid = true;
  return param;
}

float getPh1()
{
  float param = 0.0;
  uint32_t value = 0;

  value = data[12];
  value = value << 16;
  value = value + data[13];
  param = HexTofloat(value);
  if (param <= 0)
    isValid = false;
  else
    isValid = true;
  return param;
}

float getPh2()
{
  float param = 0.0;
  uint32_t value = 0;

  value = data[36];
  value = value << 16;
  value = value + data[37];
  param = HexTofloat(value);
  if (param <= 0)
    isValid = false;
  else
    isValid = true;
  return param;
}
float getTempH()
{
  float param = 0.0;
  uint32_t value = 0;

  value = data[14];
  value = value << 16;
  value = value + data[15];
  param = HexTofloat(value);
  if (param <= 0)
    isValid = false;
  else
    isValid = true;
  return param;
}

float getTSS1()
{
  float param = 0.0;
  uint32_t value = 0;

  value = data[20];
  value = value << 16;
  value = value + data[21];
  param = HexTofloat(value);

  return param;
}

float getCond()
{
  float param = 0.0;
  uint32_t value = 0;

  value = data[28];
  value = value << 16;
  value = value + data[29];
  param = HexTofloat(value);

  return param;
}


float getTempCond()
{
  float param = 0.0;
  uint32_t value = 0;

  value = data[30];
  value = value << 16;
  value = value + data[31];
  param = HexTofloat(value);

  return param;
}

float getTSS2()
{
  float param = 0.0;
  uint32_t value = 0;

  value = data[52];
  value = value << 16;
  value = value + data[53];
  param = HexTofloat(value);

  return param;
}
float getCOD()
{
  float param = 0.0;
  uint32_t value = 0;

  value = data[46];
  value = value << 16;
  value = value + data[47];
  param = HexTofloat(value);

  return param;
}
float getBOD()
{
  float param = 0.0;
  uint32_t value = 0;

  value = data[60];
  value = value << 16;
  value = value + data[61];
  SerialBT.println(value);
  param = HexTofloat(value);


  return param;
}
void readMeter()
{
  node.begin(ID, modbus);
  Serial.print("Start");
  getMODBUS();

  device.DO = getDO();//
  device.TempDO = getTempDO(); //

  device.pH1 = getPh1(); //

  device.TempH = getTempH(); //

  device.TSS1 = getTSS1(); //
  device.Cond = getCond();
  device.TempCond = getTempCond();
  device.TSS2 = getTSS2(); //
  device.COD = getCOD();

  device.BOD = getBOD();
  device.pH2 = getPh2();




  Serial.print("DO: ");  Serial.println(device.DO);
  Serial.print("TempDo: ");  Serial.println(device.TempDO);
  Serial.print("pH1: ");  Serial.println(device.pH1);
  Serial.print("TempH: ");  Serial.println(device.TempH);

  Serial.print("Tss1: ");  Serial.println(device.TSS1);
  Serial.print("COD: ");  Serial.println(device.COD);
  Serial.print("BOD: ");  Serial.println(device.BOD);
  Serial.print("pH2: ");  Serial.println(device.pH2);

  SerialBT.print("DO: ");  SerialBT.println(device.DO);
  SerialBT.print("TempDo: ");  SerialBT.println(device.TempDO);
  SerialBT.print("TempH: ");  SerialBT.println(device.TempH);
  SerialBT.print("pH1: ");  SerialBT.println(device.pH1);
  SerialBT.print("pH2: ");  SerialBT.println(device.pH2);
  SerialBT.print("COD: ");  SerialBT.println(device.COD);
  SerialBT.print("BOD: ");  SerialBT.println(device.BOD);
  SerialBT.print("Tss1: ");  SerialBT.println(device.TSS1);
  SerialBT.print("Cond: ");  SerialBT.println(device.Cond);
  SerialBT.print("TempCond: ");  SerialBT.println(device.TempCond);
  SerialBT.print("Tss2: ");  SerialBT.println(device.TSS2);
  SerialBT.println("");
  Serial.println("");
  HeartBeat();
}

void t1CallgetMeter() {     // Update read all data

  readMeter();

}
void t3CallHeartbeat() {
  //  HeartBeat();
}
//void t4Restart() {     // Update read all data
//  Serial.println("Restart");
//  ESP.restart();
//}
float HexTofloat(uint32_t x)
{
  return (*(float*)&x);
}



uint32_t FloatTohex(float x) {
  return (*(uint32_t*)&x);
}


void loop()
{
  runner.execute();

  ArduinoOTA.handle();
  unsigned long ms = millis();
  if (ms % 60000 == 0)
  {
    Serial.println("hello，OTA now");
    SerialBT.println("hello，OTA now");
  }

  if (ms % 600000 == 0)
  {

    Serial.println("Attach WiFi for，OTA "); Serial.println(WiFi.RSSI() );
    SerialBT.println("Attach WiFi for，OTA "); Serial.println(WiFi.RSSI() );

    setupWIFI();
    setupOTA();

  }
}


void WiFiEvent(WiFiEvent_t event)
{
  Serial.printf("[WiFi-event] event: %d\n", event);
  switch (event)
  {
    case SYSTEM_EVENT_WIFI_READY:               /**< ESP32 WiFi ready */
      break;
    case SYSTEM_EVENT_SCAN_DONE:                /**< ESP32 finish scanning AP */
      break;

    case SYSTEM_EVENT_STA_START:                /**< ESP32 station start */
      break;
    case SYSTEM_EVENT_STA_STOP:                 /**< ESP32 station stop */
      break;

    case SYSTEM_EVENT_STA_CONNECTED:            /**< ESP32 station connected to AP */
      break;

    case SYSTEM_EVENT_STA_DISCONNECTED:         /**< ESP32 station disconnected from AP */
      break;

    case SYSTEM_EVENT_STA_AUTHMODE_CHANGE:      /**< the auth mode of AP connected by ESP32 station changed */
      break;

    case SYSTEM_EVENT_STA_GOT_IP:               /**< ESP32 station got IP from connected AP */
    case SYSTEM_EVENT_STA_LOST_IP:              /**< ESP32 station lost IP and the IP is reset to 0 */
      break;

    case SYSTEM_EVENT_STA_WPS_ER_SUCCESS:       /**< ESP32 station wps succeeds in enrollee mode */
    case SYSTEM_EVENT_STA_WPS_ER_FAILED:        /**< ESP32 station wps fails in enrollee mode */
    case SYSTEM_EVENT_STA_WPS_ER_TIMEOUT:       /**< ESP32 station wps timeout in enrollee mode */
    case SYSTEM_EVENT_STA_WPS_ER_PIN:           /**< ESP32 station wps pin code in enrollee mode */
      break;

    case SYSTEM_EVENT_AP_START:                 /**< ESP32 soft-AP start */
    case SYSTEM_EVENT_AP_STOP:                  /**< ESP32 soft-AP stop */
    case SYSTEM_EVENT_AP_STACONNECTED:          /**< a station connected to ESP32 soft-AP */
    case SYSTEM_EVENT_AP_STADISCONNECTED:       /**< a station disconnected from ESP32 soft-AP */
    case SYSTEM_EVENT_AP_PROBEREQRECVED:        /**< Receive probe request packet in soft-AP interface */
    case SYSTEM_EVENT_AP_STA_GOT_IP6:           /**< ESP32 station or ap interface v6IP addr is preferred */
      break;

    case SYSTEM_EVENT_ETH_START:                /**< ESP32 ethernet start */
    case SYSTEM_EVENT_ETH_STOP:                 /**< ESP32 ethernet stop */
    case SYSTEM_EVENT_ETH_CONNECTED:            /**< ESP32 ethernet phy link up */
    case SYSTEM_EVENT_ETH_DISCONNECTED:         /**< ESP32 ethernet phy link down */
    case SYSTEM_EVENT_ETH_GOT_IP:               /**< ESP32 ethernet got IP from connected AP */
    case SYSTEM_EVENT_MAX:
      break;
  }
}

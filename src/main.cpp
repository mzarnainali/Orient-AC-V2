#pragma region includes
#include <Arduino.h>
#include <ArduinoOTA.h>
#include <arduino-timer.h>
#include <TM1637TinyDisplay.h>
#include <assert.h>
#include <IRrecv.h>
#include <IRac.h>
#include <IRutils.h>
#include <ESP8266WiFi.h>
// #include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <NTPClient.h>
#include <HLW8012.h>
// #include "LittleFS.h"
#include <Preferences.h>
#include <Arduino_JSON.h>
#include <WebSerial.h>
#include <zHeader.h>
#include <page_main.h>
#pragma endregion

#define WIFI_ON 0b00000001
#define WIFI_OFF 0b00000010
#define LEAF 0b00000100

#define U_DOT 0b00001000
#define L_DOT 0b00010000
#define COLON 0b00011000

// #define BLYNK_TEMPLATE_ID "TMPL6nYAVh2DF"
// #define BLYNK_TEMPLATE_NAME "Orient AC Template"
// #define BLYNK_AUTH_TOKEN "DS9zzzkRkxPLaliqbuQ2BvoGRlyh19yG"

int displayInterval,
    defaultInterval = 2000;
String g_ac_running_time = "88:88";
boolean remoteOn = false;
bool isSetupDone = false;
bool isAcOn = false;
unsigned long whenAcOnTime = 0;
unsigned long currentAcRunning = 0;
unsigned long totalAcRunning = 0;
unsigned long at_start_totalAcRunning = 0;

unsigned long tempStartTime = 0;
bool startTimeRecorded = false;
bool startTimeUpdated = false;

Preferences prefs;
// unsigned long totalTimeForLoop = 0;
int totalTimeForLoop = 0;
#pragma region LCD
#define CLK 12
#define DIO 2
TM1637TinyDisplay display(CLK, DIO);
// Create °C symbol
const uint8_t celsius[] = {
    SEG_A | SEG_B | SEG_F | SEG_G, // °
    SEG_A | SEG_D | SEG_E | SEG_F  // C
};
// Note that the counting goes from right to left. So the 11111111 segment corresponds to (dot)GFEDCBA.
const uint8_t allOFF = {0x00};

typedef struct
{
  uint8_t ScreenId;
  uint8_t order;
  uint8_t status;
} schedule_t;

schedule_t schedule[16]; // buffer for the data (max 16 entries)
const String s_delay = "delay";
void setupDisplay()
{
  pinMode(CLK, OUTPUT);
  pinMode(DIO, OUTPUT);
  display.begin();
}
void clearWatt()
{
  display.show(0x00, 1);
}
void clearWifi()
{
  display.show(0x00, 0);
}

void show_v()
{
  display.show(SEG_A, 1);
}

void show_a()
{
  display.show(SEG_B, 1);
}

void show_w()
{
  display.show(SEG_C, 1);
}

void show_kwh()
{
  display.show(SEG_D, 1);
}

void show_wifi_off()
{
  uint8_t _d = 0x00;
  if (display.isLeafMode)
  {
    _d = _d | LEAF;
  }
  display.show(WIFI_OFF | _d, 0);
  display.isWifiConnected = false;
}

void show_wifi_on()
{
  uint8_t _d = 0x00;
  if (display.isLeafMode)
  {
    _d = _d | LEAF;
  }

  display.show(WIFI_ON | _d, 0);
  display.isWifiConnected = true;
}

void hide_wifi()
{
  uint8_t _d = 0x00;
  if (display.isLeafMode)
  {
    _d = _d | LEAF;
  }

  display.show(_d, 0);
}

void show_leaf()
{
  uint8_t _d = 0x00;
  if (display.isWifiConnected)
  {
    _d = _d | WIFI_ON;
  }
  else
  {
    _d = _d | WIFI_OFF;
  }

  display.show(LEAF | _d, 0);
}

void hide_leaf()
{
  uint8_t _d = 0x00;
  if (display.isWifiConnected)
  {
    _d = _d | WIFI_ON;
  }
  else
  {
    _d = _d | WIFI_OFF;
  }

  display.show(_d, 0);
}

void show_uDot()
{
  uint8_t _d = display.getLastDisplayState();
  display.show(U_DOT | _d, 0);
}

void show_lDot()
{
  uint8_t _d = display.getLastDisplayState();
  display.show(L_DOT | _d, 0);
}

void show_colon()
{
  uint8_t _d = display.getLastDisplayState();
  display.show(COLON | _d, 0);
}

void hide_colon()
{
  uint8_t _d = display.getLastDisplayState();
  display.show(_d, 0);
}

#pragma endregion

#pragma region IR Sensor
String iPower, iMode, iFan, iSwingV, iSwingH, iLight, iClean, iTurbo, iFeel, iLightLast;
int iTemp;
const uint16_t kRecvPin = 0;
const uint16_t kCaptureBufferSize = 1024;
const uint8_t kTimeout = 15;
const uint8_t kTolerancePercentage = kTolerance;
#define LEGACY_TIMING_INFO false
IRrecv irrecv(kRecvPin, kCaptureBufferSize, kTimeout, true);
IRac ac(kRecvPin, true, false);
decode_results results;

void setupIR()
{
  assert(irutils::lowLevelSanityCheck() == 0);
  irrecv.setTolerance(kTolerancePercentage);
  irrecv.enableIRIn();
}

void processIr(String str, char delimiter)
{
  int delimiterIndex = 0;
  while (delimiterIndex != -1)
  {
    delimiterIndex = str.indexOf(delimiter);
    if (delimiterIndex != -1)
    {
      String token = str.substring(0, delimiterIndex);
      if (token.indexOf("Power:") != -1)
      {
        iPower = token.substring(token.indexOf("Power:") + 7);
        // Serial.println(iPower);
      }
      else if (token.indexOf("Mode:") != -1)
      {
        iMode = token.substring(token.indexOf("Mode:") + 9);
        iMode = iMode.substring(0, iMode.length() - 1);
        // Serial.println(iMode);
      }
      else if (token.indexOf("Temp:") != -1)
      {
        String xTemp = token.substring(token.indexOf("Temp:") + 6);
        iTemp = xTemp.substring(0, xTemp.length() - 1).toInt();
        // Serial.println(String(iTemp));
      }
      else if (token.indexOf("Fan:") != -1)
      {
        iFan = token.substring(token.indexOf("Fan:") + 8);
        iFan = iFan.substring(0, iFan.length() - 1);
        // Serial.println(iFan);
      }
      else if (token.indexOf("Swing(V):") != -1)
      {
        iSwingV = token.substring(token.indexOf("Swing(V):") + 10);
        // Serial.println(iSwingV);
      }
      else if (token.indexOf("Swing(H):") != -1)
      {
        iSwingH = token.substring(token.indexOf("Swing(H):") + 10);
        // Serial.println(iSwingH);
      }
      else if (token.indexOf("Light:") != -1)
      {
        iLight = token.substring(token.indexOf("Light:") + 7);
        // Serial.println(iLight);
      }
      else if (token.indexOf("Clean:") != -1)
      {
        iClean = token.substring(token.indexOf("Clean:") + 7);
        // Serial.println(iClean);
      }
      else if (token.indexOf("Turbo:") != -1)
      {
        iTurbo = token.substring(token.indexOf("Turbo:") + 7);
        // Serial.println(iTurbo);
      }
      else if (token.indexOf("IFeel:") != -1)
      {
        iFeel = token.substring(token.indexOf("IFeel:") + 7);
        // Serial.println(iFeel);
      }
      str = str.substring(delimiterIndex + 1);
    }
  }
  // Print the remaining part of the string
  if (str.length() > 0)
  {
    if (str.indexOf("IFeel:") != -1)
    {
      iFeel = str.substring(str.indexOf("IFeel:") + 7);
      // Serial.println(iFeel);
    }
  }
}

void sendAcSignal(bool power, int acTemp = 26)
{

  ac.next.protocol = decode_type_t::ELECTRA_AC; // Set a protocol to use.
  ac.next.model = 1;                            // Some A/Cs have different models. Try just the first.
  ac.next.mode = stdAc::opmode_t::kCool;        // Run in cool mode initially.  kAuto | kCool | kHeat | kDry | kFan
  ac.next.celsius = true;                       // Use Celsius for temp units. False = Fahrenheit
  ac.next.degrees = acTemp;                     // 25 degrees.
  ac.next.fanspeed = stdAc::fanspeed_t::kHigh;  // Start the fan at medium.  kAuto | kLow | kMedium | kHigh | kMax
  ac.next.swingv = stdAc::swingv_t::kOff;       // Don't swing the fan up or down.  kOff | kOn
  ac.next.swingh = stdAc::swingh_t::kOff;       // Don't swing the fan left or right.
  ac.next.light = true;                         // Turn off any LED/Lights/Display that we can.
  ac.next.beep = false;                         // Turn off any beep from the A/C if we can.
  ac.next.econo = false;                        // Turn off any economy modes if we can.
  ac.next.filter = false;                       // Turn off any Ion/Mold/Health filters if we can.
  ac.next.turbo = false;                        // Don't use any turbo/powerful/etc modes.
  ac.next.quiet = false;                        // Don't use any quiet/silent/etc modes.
  ac.next.sleep = -1;                           // Don't set any sleep time or modes.
  ac.next.clean = false;                        // Turn off any Cleaning options if we can.
  ac.next.clock = -1;                           // Don't set any current time if we can avoid it
  ac.next.power = power;                        // Initially start with the unit off.
  irrecv.disableIRIn();
  delay(1);
  ac.sendAc(); // Send the message.
  delay(1);
  irrecv.enableIRIn();

  iPower = "On";
  iMode = "Cool";
  iFan = "High";
  iSwingV = "Off";
  iSwingH = "Off";
  iLight = "On";
  iClean = "Off";
  iTurbo = "Off";
  iFeel = "Off";
  iLightLast = "Off";
  iTemp = acTemp;
}

#pragma endregion

#pragma region Temperature Sensor
const double VCC = 3.3;             // NodeMCU on board 3.3v vcc
const double R2 = 6200;             // 5000;  // 6200;    	1.2kOhm         // 1k ohm series resistor
const double adc_resolution = 1023; // 10-bit adc

/*
+3.3--ntc---adc---1.2k--gnd

*/
const double A = 0.001147648;     // 0.001129148; // thermistor equation parameters
const double B = 0.00023200;      // 0.000234125;  any increase reduce temprature  28 to 20
const double C = 0.0000000876741; // 0.0000000876741;

const int numReadings = 10;
int readings[numReadings]; // the readings from the analog input
int readIndex = 0;         // the index of the current reading
int total = 0;             // the running total
float g_room_Temp = 0;
int g_temp_adc = 0;
void setupTemp()
{
  for (int thisReading = 0; thisReading < numReadings; thisReading++)
    readings[thisReading] = 0;
}

float getTemp()
{
  float Vout, Rth, temperature, avg_adc_value = 0;
  total = total - readings[readIndex];
  readings[readIndex] = analogRead(A0);
  total = total + readings[readIndex];
  readIndex = readIndex + 1;
  if (readIndex >= numReadings)
  {
    readIndex = 0;
  }
  avg_adc_value = total / numReadings;

  Vout = (avg_adc_value * VCC) / adc_resolution;

  Rth = (R2 * (VCC - Vout)) / Vout;
  // Rth = (VCC * R2 / Vout) - R2;
  temperature = (1 / (A + (B * log(Rth)) + (C * pow((log(Rth)), 3)))); // Temperature in kelvin
  temperature = temperature - 273.15;                                  // Temperature in degree celsius
  g_temp_adc = avg_adc_value;
  return (temperature);
}
#pragma endregion

#pragma region OTA
void setupOTA()
{
  ArduinoOTA.setPort(8266);
  ArduinoOTA.setHostname("Orient AC");

  ArduinoOTA.onStart([]()
                     { Serial.println("Start"); });
  ArduinoOTA.onEnd([]()
                   {
    Serial.println("\nEnd");
    ESP.restart(); });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total)
                        { Serial.printf("Progress: %u%%\r", (progress / (total / 100))); });
  ArduinoOTA.onError([](ota_error_t error)
                     {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed"); });
  ArduinoOTA.begin();
}
#pragma endregion

#pragma region Current Time
const long utcOffsetInSeconds = 18000;
char daysOfTheWeek[7][12] = {
    "Sunday",
    "Monday",
    "Tuesday",
    "Wednesday",
    "Thursday",
    "Friday",
    "Saturday"};

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", utcOffsetInSeconds);
unsigned long previousMillis = 0;
bool acOn = false;
void setupTime()
{
  timeClient.begin();
}

String convertMill2Time(long milis)
{
  String minutes = String((milis / 60000) % 60);
  if (minutes.length() == 1)
  {
    minutes = "0" + minutes;
  }
  String hours = String(milis / 3600000);
  if (hours.length() == 1)
  {
    hours = "0" + hours;
  }
  String t = hours + ":" + minutes;
  return t;
}

void calcAcOnTime()
{
  if (isAcOn && startTimeUpdated)
  {
    currentAcRunning = millis() - whenAcOnTime;
    g_ac_running_time = convertMill2Time(currentAcRunning);

    totalAcRunning = at_start_totalAcRunning + currentAcRunning;
  }
}

void displayTime()
{
  timeClient.update();

  int h = timeClient.getHours();
  if (h > 12)
  {
    h = h - 12;
  }
  String hh = String(h);
  if (h < 10)
  {
    hh = "0" + hh;
  }
  int m = timeClient.getMinutes();
  String mm = String(m);
  if (m < 10)
  {
    mm = "0" + mm;
  }
  String tm = hh + mm;
  display.showString(tm.c_str());
  show_colon();
}
#pragma endregion

#pragma region Watt Meter
// GPIOs
#define SEL_PIN 16
#define CF1_PIN 5
#define CF_PIN 4
#define CURRENT_MODE HIGH
#define CURRENT_RESISTOR 0.001
#define VOLTAGE_RESISTOR_UPSTREAM (5 * 470000) // Real: 2280k
#define VOLTAGE_RESISTOR_DOWNSTREAM (1000)     // Real 1.009k
#define UPDATE_TIME 5000
HLW8012 hlw8012;

int Watt = 0;
int VA = 0;
float powerFactor = 0;
int Voltage = 0;
float Current = 0;
String Ampere = "0";
String Kwh = "0";
float current_kwh = 0;
float total_kwh = 0;
float at_start_total_kwh = 0;
volatile unsigned long i_pulse_count = 0;
void setupWattMeter()
{
  hlw8012.begin(CF_PIN, CF1_PIN, SEL_PIN, CURRENT_MODE, false, 500000);
  hlw8012.setResistors(CURRENT_RESISTOR, VOLTAGE_RESISTOR_UPSTREAM, VOLTAGE_RESISTOR_DOWNSTREAM);
  hlw8012.setCurrentMultiplier(10086);
  hlw8012.setVoltageMultiplier(428000);
  hlw8012.setPowerMultiplier(7779810);
}

int i = 1;
void updateWattMeter()
{
  switch (i)
  {
  case 1:
    Voltage = hlw8012.getVoltage(8000);
    if (Voltage > 0)
    {
      VA = Voltage * Current;
      hlw8012.setMode(MODE_CURRENT);
      i++;
    }
    else
    {
      VA = 0;
      hlw8012.setMode(MODE_VOLTAGE);
    }

    break;
  case 2:
    if (Voltage > 0)
    {
      Watt = hlw8012.getActivePower(500000);
      if (Watt > VA)
        powerFactor = 1.0f;
      else if (VA == 0)
        powerFactor = 0.0f;
      else
        powerFactor = (float)Watt / VA;
      ////
      if (powerFactor < 0)
        powerFactor = 0;
      else if (powerFactor > 1)
        powerFactor = 1;
    }
    else
    {
      Watt = 0;
      powerFactor = 0;
    }
    i++;
    break;
  case 3:
    if (Voltage > 0)
    {
      Current = hlw8012.getCurrent(100000);
      Ampere = String(Current, 2);
      VA = Voltage * Current;
    }
    else
    {
      Ampere = "0";
      VA = 0;
    }
    hlw8012.setMode(MODE_VOLTAGE);
    i++;
    break;
  default:
    if (Voltage > 0)
    {
      Watt = hlw8012.getActivePower(500000);
      if (Watt > VA)
        powerFactor = 1.0f;
      else if (VA == 0)
        powerFactor = 0.0f;
      else
        powerFactor = (float)Watt / VA;
      ////
      if (powerFactor < 0)
        powerFactor = 0;
      else if (powerFactor > 1)
        powerFactor = 1;
    }
    else
    {
      Watt = 0;
      powerFactor = 0;
    }
    i = 1;
  }
  if (isAcOn && startTimeUpdated)
  {
    current_kwh = getEnergy();
    total_kwh = at_start_total_kwh + current_kwh;
  }
  Kwh = String(current_kwh, 2);
}

unsigned long wattMillis = 0;
void updateWattMeter(int interval)
{
  unsigned long currentMillis = millis();
  if (currentMillis - wattMillis >= interval)
  {
    wattMillis = currentMillis;
    updateWattMeter();
  }
}

void IRAM_ATTR hlw8012_cf_interrupt()
{
  i_pulse_count++;
}

float getEnergy()
{
  const float pulse_count = i_pulse_count;
  // return pulse_count; // * 7779810 / 1000000.0f;// / 2.0f;
  return pulse_count * 2 / 1000000.0f; // / 2.0f;
}

void resetEnergy()
{
  i_pulse_count = 0;
}

#pragma endregion

#pragma region Timer
;
void speedTest(String d)
{
  String val = "";
  unsigned long startTime = millis();
  if (d == "V")
  {
    val = String(hlw8012.getVoltage(8000));
  }
  else if (d == "A")
  {
    val = String(hlw8012.getCurrent(100000), 2);
  }
  else if (d == "W")
  {

    val = String(hlw8012.getActivePower(500000));
    // WebSerial.println("pulseIn CHECK");
    // val = String(pulseIn(CF_PIN, HIGH, 100000));
    //  WebSerial.print("     _width1: " + String(_width));
  }
  else if (d == "T")
  {
    hlw8012.toggleMode();
  }
  else if (d == "1")
  {
    hlw8012.setMode(MODE_CURRENT);
  }
  else if (d == "2")
  {
    hlw8012.setMode(MODE_VOLTAGE);
  }

  unsigned long endTime = millis();
  totalTimeForLoop = endTime - startTime;

  WebSerial.println(String(d) + " | " + String(hlw8012.getMode()) + " => Total Time Take: " + String(totalTimeForLoop) + " Value: " + val);
  //  Watt = hlw8012.getActivePower();
  // Ampere = String(hlw8012.getCurrent(), 2);
}

auto timer = timer_create_default();
bool ledOn = true;
bool updateWattMeterTimer(void *)
{
  calcAcOnTime();
  updateWattMeter();
  return true;
}
bool twoSec(void *)
{
  return true;
}
bool update_temp_timer(void *)
{
  g_room_Temp = getTemp();
  return true;
}
const char *ssid = "Zain";
const char *password = "Z1234567";
bool check_wifi_connection(void *)
{
  if ((WiFi.status() != WL_CONNECTED))
  {
    WebSerial.println("Wifi Disconnected Timer Check");
    show_wifi_off();
    WiFi.begin(ssid, password);
  }
  {
    WebSerial.println("Wifi Connected Timer Check");
    show_wifi_on();
  }
  return true;
}

bool save_current_data(void *)
{
  if (isAcOn && startTimeUpdated)
  {
    if (!prefs.begin("my-app", false))
    {
      WebSerial.println("Failed to initialize preferences");
      return true;
    }
    prefs.putLong("currentAcRunning", currentAcRunning);
    if (current_kwh > 0)
    {
      prefs.putFloat("current_kwh", current_kwh);
    }

    prefs.putLong("totalAcRunning", totalAcRunning);
    prefs.putFloat("total_kwh", total_kwh);
    prefs.end();
  }
  return true;
}

void resetTotalMeter()
{
  prefs.begin("my-app", false);
  totalAcRunning = 0;
  total_kwh = 0;
  at_start_total_kwh = 0;
  at_start_totalAcRunning = 0;
  prefs.putLong("totalAcRunning", totalAcRunning);
  prefs.putFloat("total_kwh", total_kwh);
  prefs.end();
}

int txt = 0;
uint8_t _p = 1;
bool _test(void *)
{
  WebSerial.println("Screen Update " + String(txt));
  return true;
}
bool yeld(void *)
{
  yield();
  return true;
}

#pragma endregion

#pragma region Web Server
String myHostname = "Orient AC";
const char *ap_ssid = "Orient AC";
const char *ap_password = "123456789z";

IPAddress local_IP(192, 168, 5, 1);
IPAddress gateway(192, 168, 5, 1);
IPAddress subnet(255, 255, 255, 0);
AsyncWebServer server(80);

int cnt = 0;

JSONVar data;
String getJsonData()
{
  data["voltage"] = Voltage;
  data["ampere"] = Ampere;
  data["watt"] = Watt;
  data["VA"] = VA;
  data["powerFactor"] = powerFactor;
  data["kwh"] = Kwh;
  data["total_kwh"] = String(total_kwh, 2);
  data["iPower"] = iPower;
  data["iMode"] = iMode;
  data["iTemp"] = iTemp;
  data["iFan"] = iFan;
  data["iSwingV"] = iSwingV;
  data["iSwingH"] = iSwingH;
  data["iLight"] = iLight;
  data["iClean"] = iClean;
  data["iTurbo"] = iTurbo;
  data["iFeel"] = iFeel;
  data["temp"] = String(g_room_Temp, 2);
  data["temp_adc"] = g_temp_adc;
  data["acTime"] = g_ac_running_time;
  data["totAcTime"] = totalAcRunning;

  String jsonString = JSON.stringify(data);
  return jsonString;
}

void setupWifi()
{
  WiFi.setSleepMode(WIFI_NONE_SLEEP);
  WiFi.mode(WIFI_AP_STA);

  WiFi.hostname(myHostname.c_str());
  WiFi.begin(ssid, password);

  Serial.println("Configuring access point...");
  WiFi.softAPConfig(local_IP, gateway, subnet);
  WiFi.softAP(ap_ssid, ap_password);

  IPAddress myIP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(myIP);

  Serial.println("Connecting..");
  bool togle = true;
  while (WiFi.status() != WL_CONNECTED && cnt <= 16)
  {
    delay(500);
    Serial.print(".");
    display.showNumber(cnt);
    if (togle)
    {
      show_wifi_off();
    }
    else
    {
      hide_wifi();
    }
    togle = !togle;
    cnt++;
  }
  display.clear();
  show_wifi_off();
  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  WiFi.setAutoReconnect(true);
  WiFi.persistent(true);

  if (WiFi.status() == WL_CONNECTED)
  {
    display.isWifiConnected = true;
    show_wifi_on();
  }
}

void reciveData(uint8_t *data, size_t len)
{

  String _d = "";
  for (int i = 0; i < len; i++)
  {
    _d += char(data[i]);
  }
  WebSerial.println("Received Data: " + _d);
  // speedTest(_d);
  // display.showNumber(_d);
  if (_d == "C1")
  {
    display.clear();
  }
  else if (_d == "C2")
  {
    display.clearAll();
  }
  else if (_d == "C3")
  {
    uint8_t blank[] = {0x00, 0x00, 0x00, 0x00};
    display.setSegments(blank);
  }
  else if (_d == "C4")
  {
    uint8_t blank[] = {0xff, 0xff, 0xff, 0xff};
    display.setSegments(blank);
  }
  else if (_d == "C5")
  {

    display.showNumber("12345677");
  }
  else if (_d == "ACON")
  {
    sendAcSignal(true, 26);
  }
  else if (_d == "ACOFF")
  {
    sendAcSignal(false, 26);
  }
  else if (_d == "C.U")
  {
    prefs.begin("my-app", true);
    float val = prefs.getFloat("current_kwh", 999);
    prefs.end();
    WebSerial.println(val);
  }
  else if (_d == "C.T")
  {
    prefs.begin("my-app", true);
    long val = prefs.getLong("currentAcRunning", 999);
    prefs.end();
    WebSerial.println(val);
  }
  else if (_d == "T.U")
  {
    prefs.begin("my-app", true);
    float val = prefs.getFloat("total_kwh", 999);
    prefs.end();
    WebSerial.println(val);
  }
  else if (_d == "T.T")
  {
    prefs.begin("my-app", true);
    long val = prefs.getLong("totalAcRunning", 999);
    prefs.end();
    WebSerial.println(val);
  }
  else
  {
    display.showString(_d.c_str());
  }
}

void setupWebServer()
{
  WebSerial.begin(&server);
  WebSerial.msgCallback(reciveData);

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send_P(200, "text/html", P_MAIN_PAGE); });
  // Request for the latest sensor readings
  server.on("/update", HTTP_GET, [](AsyncWebServerRequest *request)
            {
    String json = getJsonData();
    request->send(200, "application/json", json);
    json = String(); });

  server.on("/acon", HTTP_GET, [](AsyncWebServerRequest *request)
            {
              int set_temp = 26;
              if (request->hasArg("temp")) {
                set_temp = request->arg("temp").toInt();
                if (set_temp < 16 || set_temp > 32)
                {
                  set_temp = 26;
                }
              }
              sendAcSignal(true, set_temp);
              request->send(200, "text/html", "OK"); });
  server.on("/acoff", HTTP_GET, [](AsyncWebServerRequest *request)
            {
              sendAcSignal(false,26);
              request->redirect("/"); });

  server.on("/reset", HTTP_GET, [](AsyncWebServerRequest *request)
            {
              resetTotalMeter();
              request->redirect("/"); });

  server.begin();
}

#pragma endregion

void setup()
{
  Serial.begin(115200);
  Serial.println("Booting");
  prefs.begin("my-app", true);
  currentAcRunning = prefs.getLong("currentAcRunning", 0);
  current_kwh = prefs.getFloat("current_kwh", 0);
  totalAcRunning = prefs.getLong("totalAcRunning", 0);
  total_kwh = prefs.getFloat("total_kwh", 0);
  at_start_total_kwh = total_kwh;
  at_start_totalAcRunning = totalAcRunning;
  prefs.end();
  if (currentAcRunning > 0)
  {
    g_ac_running_time = convertMill2Time(currentAcRunning);
  }
  setupDisplay();
  setupWifi();
  setupOTA();
  setupWebServer();
  setupTemp();
  setupIR();
  setupTime();
  setupWattMeter();
  attachInterrupt(digitalPinToInterrupt(CF_PIN), hlw8012_cf_interrupt, FALLING);

  timer.every(1000, updateWattMeterTimer);
  timer.every(500, update_temp_timer);
  timer.every(30000, check_wifi_connection);
  timer.every(30000, save_current_data);
  timer.every(5000, yeld);
  displayInterval = defaultInterval; // prefs.getUInt("counter", defaultInterval);
  isSetupDone = true;

  // prefs.begin("my-app", false);
  // prefs.putLong("currentAcRunning", 534);
  // prefs.putFloat("current_kwh", 34);
  // prefs.end();

  Serial.println("Setup Done!");
}

char displayScreen = 'T';
unsigned long lastDisplayMillis = 0;
String ztm = "";

void loop()
{
  // unsigned long startTime = millis();
  ArduinoOTA.handle();
  timer.tick();
  unsigned long currentMillis = millis();

  if (display.displayOn && currentMillis - lastDisplayMillis >= displayInterval)
  {
    displayInterval = defaultInterval;
    lastDisplayMillis = currentMillis;

    switch (displayScreen)
    {
    case 'T':
      clearWatt();
      displayTime();
      displayScreen = 'V';
      break;
    case 'V':
      hide_colon();
      display.showNumber(Voltage);
      show_v();
      displayScreen = 'A';
      break;
    case 'A':
      hide_colon();
      display.showNumber(Ampere);
      show_a();
      displayScreen = 'W';
      break;
    case 'W':
      hide_colon();
      display.showNumber(Watt);
      show_w();
      displayScreen = 'U';
      break;
    case 'U':
      hide_colon();
      if (Watt > 999 && Kwh.toInt() < 10)
      {
        display.clear();
      }
      display.showNumber(Kwh);
      show_kwh();
      displayScreen = 'Z';
      break;
    case 'Z':
      ztm = g_ac_running_time;
      ztm.replace(":", "");
      display.showString(ztm.c_str());
      show_colon();
      show_a();
      displayScreen = 'S';
      break;
    case 'S':
      clearWatt();
      hide_colon();
      display.showString(("S-" + String(iTemp)).c_str());
      displayScreen = 'R';
      break;
    case 'R':
      hide_colon();
      clearWatt();
      display.showString(("A-" + String(g_room_Temp, 0)).c_str());
      displayScreen = 'T';
      break;
    default:
      hide_colon();
      clearWatt();
      display.showString(("S-" + String(iTemp)).c_str());
      displayScreen = 'T';
      break;
    }
  }

  if (Watt > 0)
  {

    if (!startTimeRecorded)
    {
      // Record the time when Watt first becomes greater than 0
      tempStartTime = millis();
      startTimeRecorded = true;
      startTimeUpdated = false;
    }
    if (millis() - tempStartTime >= 5000 && !startTimeUpdated)
    {
      // Now update whenAcOnTime
      whenAcOnTime = tempStartTime;

      currentAcRunning = 0;
      current_kwh = 0;
      at_start_total_kwh = total_kwh;
      at_start_totalAcRunning = totalAcRunning;
      prefs.begin("my-app", false);
      prefs.putLong("currentAcRunning", 0);
      prefs.putFloat("current_kwh", 0);
      prefs.end();
      resetEnergy();
      startTimeUpdated = true;
    }
    if (!isAcOn)
    {
      isAcOn = true;
      display.displayOn = true;
      displayInterval = 10000;
      hide_colon();
      clearWatt();
      display.showString(("S-" + String(iTemp)).c_str());
      previousMillis = millis();
    }
  }
  else
  {
    isAcOn = false;
    startTimeRecorded = false;
    display.displayOn = false;
    display.clear();
    hide_leaf();
  }

  if (irrecv.decode(&results))
  {
    String description = IRAcUtils::resultAcToString(&results);
    if (description.length())
    {
      processIr(description, ',');
      display.clear();
      if (iPower == "On")
      {
        remoteOn = true;
        if (iLight == "Toggle")
        {
          iLightLast = iLight;
          display.displayOn = !display.displayOn;
        }
        else
        {
          display.displayOn = true;
        }
        /*if (!isAcOn)
        {
          whenAcOnTime = millis();
          resetEnergy();
          isAcOn = true;
        }*/
        displayInterval = 10000;
        hide_colon();
        clearWatt();
        display.showString(("S-" + String(iTemp)).c_str());
        previousMillis = millis();
      }
      else
      {
        remoteOn = false;
        display.displayOn = false;
        display.clear();
        hide_leaf();
      }
      display.isLeafMode = (iTurbo == "On") ? true : false;
      if (display.isLeafMode)
      {
        show_leaf();
      }
      else
      {
        hide_leaf();
      }
    }
    yield();
  }
  // unsigned long endTime = millis();
  // totalTimeForLoop = endTime - startTime;
  // Serial.println("Loop Time: " + String(totalTimeForLoop));
}
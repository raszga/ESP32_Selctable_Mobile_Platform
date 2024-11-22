/*
This is an ESP32 WIFI micro resitive joystick and buttons application.
It is based on the GITHUB treasure repository. The only different  part is the String
conversion to be broadcasted. 
The joystick reads 6 analogs AN1 and few buttons and sends them thru an UDP
protocol to the mobile platform (Mecanum 4W).
The Joystic is also the access point where the platform has to connect.
I used CPP style libraries so the te development code for 
the platform can be as clear as possible.
*/
// WIFI
#include <LiquidCrystal_I2C.h>
#include <WiFi.h>
#include <WiFiUdp.h>

// create udp instance
WiFiUDP udp;
//Setup addresses for the nettwork --------------------------------------------
IPAddress Mecanum(192, 168, 5, 230);   //Target Platform
IPAddress Truck4WD(192, 168, 5, 240);  //Target Platform
IPAddress local_IP(192, 168, 5, 220);
IPAddress ip(192, 168, 5, 220);       // desired IP address
IPAddress gateway(192, 168, 5, 200);  // gateway of my network
IPAddress subnet(255, 255, 255, 0);   // subnet mask of my network
IPAddress dns(192, 168, 5, 220);      // dns

char packetBuffer[255];
unsigned int localPort = 80;
const char *ssid = "GRSD_220";  // local network credentials
const char *password = "";
int resetBT = 26;  // joystick reset button
int selTgBT = 27;  //target platform selection button
int tgStage;
uint8_t ledB = 2;  // led indicator

int lcdColumns = 16;  // LCD param
int lcdRows = 2;
LiquidCrystal_I2C lcd(0x27, lcdColumns, lcdRows);  ///LCD instance
// ADC readings================================================================
#define CONVERSIONS_PER_PIN 10
uint8_t adc_pins[] = {
  36,
  39,
  34,
  35,
  32,
  33
};                                                            //ADC1 ESP32 wroom in board layout  6 channels available if use WIFI
uint8_t adc_pins_count = sizeof(adc_pins) / sizeof(uint8_t);  // number total of pins
// digital pins inputs- avoided TX,RX and I@C pins
uint8_t digi_pins[] = {
  25,
  26,
  27,
  14,
  12,
  13,
  23,
  22,
  19,
  18,
  5,
  17,
  16
};                                                              //DIGI pins for ESP32 wroom in board layout
uint8_t digi_pins_count = sizeof(digi_pins) / sizeof(uint8_t);  // number of digi pins
volatile bool adc_coversion_done = false;                       // Addc conversion done switch
adc_continuous_data_t *result = NULL;                           // pointer to results
// program variables***********************************************************
String outString;  // data string broadcasted after reading inputs
//==============================================================================
void setup() {
  ledcAttach(ledB, 12000, 8);
  ledcWrite(ledB, 0);
  tgStage = 0;                    // set target platform stage
  lcd.init();                     // initialize LCD
  lcd.backlight();                // turn on LCD backlight
  lcd.clear();                    //clear LCD
  lcd.setCursor(0, 0);            // set cursor to first column, first row
  lcd.print("Current Platform");  // print message
  lcd.setCursor(0, 1);
  if (tgStage == 0) { lcd.print("Mecanum"); }    // set cursor
  if (tgStage == 1) { lcd.print("Truck4WD"); }   // set cursor
  Serial.begin(115200);                          // serial comunication
  Serial.print("Setting up Access Point ... ");  // start WIFI arangements
  Serial.println(WiFi.softAPConfig(local_IP, gateway, subnet) ? "Ready" : "Failed!");
  Serial.print("Starting Access Point ... ");
  Serial.println(WiFi.softAP(ssid, password) ? "Ready" : "Failed!");
  Serial.print("IP address = ");
  Serial.println(WiFi.softAPIP());
  udp.begin(localPort);  // start UDP prorocol
  Serial.printf("UDP Client : %s:%i \n", WiFi.localIP().toString().c_str(), localPort);
  analogContinuousSetWidth(12);  // setuAnalog continous reading 0-4095 on 3.3 V
  analogContinuousSetAtten(ADC_11db);
  analogContinuous(adc_pins, adc_pins_count, CONVERSIONS_PER_PIN, 20000, &adcComplete);
  analogContinuousStart();
  // setup input digital pins as pullup they will be 1 if not connected to ground G
  for (int i = 0; i < digi_pins_count; i++) { pinMode(digi_pins[i], INPUT_PULLUP); }
}
//*****************************************************************************
void loop() {
  ledcWrite(ledB, 0);
  // start udp
  if (digitalRead(selTgBT) == 0) {
    delay(100);
    tgStage = !tgStage;
    lcd.init();
    lcd.clear();                    // clears the display to print new message
                                    //    delay(50);
    lcd.setCursor(0, 0);            // set cursor to first column, first row
    lcd.print("Current Platform");  // print message
    Serial.println("CHANGIN PLATFORM");
    delay(10);
    lcd.setCursor(0, 1);
    if (tgStage == 0) {
      lcd.print("Mechanum");
    }
    if (tgStage == 1) {
      lcd.print("Truck4WD");
    }
  }
  String S = readAnalogs();                                    // read analogs and diggitals as a string
  Serial.println(S);                                           // print readings
  if (tgStage == 0) { udp.beginPacket(Mecanum, localPort); }   // this has to be the platform address
  if (tgStage == 1) { udp.beginPacket(Truck4WD, localPort); }  // this has to be the platform address
  int Slength = S.length() + 1;
  char buf[Slength];
  S.toCharArray(buf, Slength);  // convert to send-able data
  udp.printf(buf);              // send data
  udp.endPacket();              // end packet
  delay(20);
  // soft reset button for the board
  if (digitalRead(resetBT) == 0) {
    delay(100);
    ESP.restart();
  }
}
//*****************************************************************************
// read imputs and pack them in a string
//*****************************************************************************
String readAnalogs() {
  ledcWrite(ledB, 64);
  // read analogs based on continous read example
  if (adc_coversion_done == true) {
    adc_coversion_done = false;
    if (analogContinuousRead(&result, 0)) {
      outString = "";
      // compose the string
      for (int i = 0; i < adc_pins_count; i++) {
        char s[4];
        int R = result[i].avg_read_raw;
        sprintf(s, "%4d", R);
        if (i == 0) {
          outString += String(s);
        } else {
          outString += "," + String(s);
        }
      }
      delay(3);
    } else {
      Serial.println("Error occurred during reading data. Set Core Debug Level to error or lower for more information.");
      return "Error";
    }
    // read digital pins
    for (int i = 0; i < digi_pins_count; i++) {
      char s[4];
      int R = digitalRead(digi_pins[i]);
      sprintf(s, "%4d", R);
      outString += "," + String(s);
    }
  }
  ledcWrite(ledB, 255);
  return outString;  // return results
}
//**************check if conversion is complete*********************************
void ARDUINO_ISR_ATTR adcComplete() {
  adc_coversion_done = true;
}
//*****************************************************************************

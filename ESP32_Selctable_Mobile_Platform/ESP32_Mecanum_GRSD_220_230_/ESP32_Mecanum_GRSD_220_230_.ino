#include <WiFi.h>
#include <stdio.h>
#include <string.h>
#include <WiFiUdp.h>
#include "ESP32_Mobile.h"
//=============================================================================
WiFiServer server(localPort);         // Set web server port number to 80
IPAddress ip(192, 168, 5, 230);       // desired IP address for platform*******
IPAddress gateway(192, 168, 5, 200);  // gateway of my network
IPAddress subnet(255, 255, 255, 0);   // subnet mask of my network
IPAddress dns(192, 168, 5, 220);      // dns
//-------------Mecanum Settings------------------------------------------------
// same color has the same signal
int V_Blue;
int V_Red;
double_t _Ang;      // driving vector direction
double_t _Amp;      //  driving vector amplitude
int RTT;            // rotational speed
int JX, JY;         // direction components
float K_Amp = 1.0;  // corection factor for amplitude
float K_RTT = 0.75; /* Rotational coeficient - 
                    remeber it includes wheel dia and rotational radius*/
//---------------------------------------------------------------------------------
// setup funtion moved in the library to clean main code file*********************
void setup() {
  // network settings paricular
  WiFi.begin(ssid, password);
  // connect with the phone to GRSD to fix the address
  WiFi.config(ip, dns, gateway, subnet);
  _setup();  // the rest of setup common
    // motor object alocation - hardware dependent
  // Left Rear
  MotLR.NAM = "LftR ";
  MotLR.GP0 = LedArr[0];
  MotLR.GP1 = LedArr[1];
  // Left Front ---------------
  MotLF.NAM = "LftF ";
  MotLF.GP0 = LedArr[2];
  MotLF.GP1 = LedArr[3];
  // Right Rear ---------------
  MotRR.NAM = "RghR ";
  MotRR.GP0 = LedArr[4];
  MotRR.GP1 = LedArr[5];
  // Right Front --------------
  MotRF.NAM = "RghF ";
  MotRF.GP0 = LedArr[6];
  MotRF.GP1 = LedArr[7];
}
//----------------------------------------------------------------------------------
///LOOP main
//**********************************************************************************
void loop() {
  int packetSize = udp.parsePacket();  // read udp
  ledcWrite(ledB, 0);
  if (packetSize) {
    ledcWrite(ledB, 255);
    udp.read(packetBuffer, 128);  // was 255
    rS = String(packetBuffer);    // recompose the joystick string of data
    xintJoy = TrS(rS);            // extract numerical values in an array
    // Joysticks
    ///J0
    J0DX = xintJoy._val[0] - J0DX_N;  // read joystick values based on neutral
    J0DX = mapJY(J0DX, J0DX_N);       // re-map and symetrize- see function
    J0DY = xintJoy._val[1] - J0DY_N;
    J0DX = mapJY(J0DY, J0DY_N);

    //J1
    J1DX = xintJoy._val[2] - J1DX_N;
    J1DX = mapJY(J1DX, J1DX_N);
    J1DY = xintJoy._val[3] - J1DY_N;
    J1DY = mapJY(J1DY, J1DY_N);

    //J2
    J2DX = xintJoy._val[4] - J2DX_N;
    J2DX = mapJY(J2DX, J2DX_N);
    J2DY = xintJoy._val[5] - J2DY_N;
    J2DY = mapJY(J2DY, J2DY_N);

    //Buttons
    J0_BT = xintJoy._val[6];
    J1_BT = xintJoy._val[8];
    J2_BT = xintJoy._val[7];
    TG0_BT = xintJoy._val[9];
    TG1_BT = xintJoy._val[10];
    udp.clear();  // end of reading


    // flip axis as needed - we use only 3 fizical axes
    // this pends on joystick config ( can be mechanically gated 3 joysticks)

    // process received data - for each axis
    fltJY = shift(-J1DY, fltJY);  // add to history array by shifting values
    JY = _avg(fltJY);             // average of data collected
    fltJY = shift(JY, fltJY);     // add the las point averaged


    fltJX = shift(J1DX, fltJX);
    JX = _avg(fltJX);
    fltJX = shift(JX, fltJX);

    // read rotate
    fltRTT = shift(J2DX, fltRTT);
    RTT = _avg(fltRTT);
    fltRTT = shift(RTT, fltRTT);

    // dead band check
    if (abs(JX) < 50) { JX = 0; }
    if (abs(JY) < 50) { JY = 0; }
    if (abs(RTT) < 50) { RTT = 0; }

    // apply rotational speed coeficient
    if ((JX != 0) | (JY != 0)) { RTT = int(RTT * K_RTT); }

    // vector angle calculation see function
    _Ang = Angle(JY, JX);

    // amplitude of vector
    _Amp = (sqrt(JX * JX + JY * JY));

    /* Mecanum Platform Wheels convention
    please see:
    https://seamonsters-2605.github.io/archive/mecanum/#:~:text=The%20front%2Dright%20and%20back,x%2B1%2F4%CF%80

  |===========================================|
  |================= Front ===================|
  |Left Blue = MotLF ===== MotRF = Rigth Red  |
  |===========================================|
  |===========================================|
  |Left Red = MotLR ====== MotRR = Rigth Blue |
  |================= Rear  ===================|  */

    V_Red = int((sin(_Ang - M_PI / 4) * _Amp) * K_Amp);
    V_Blue = int((sin(_Ang + M_PI / 4) * _Amp) * K_Amp);

    if (abs((V_Blue)) > 2047) { V_Blue = (sgn(V_Blue) * 2047); }
    if (abs((V_Red)) > 2047) { V_Red = (sgn(V_Red) * 2047); }

    prInt("JX ", JX);
    prInt("JY ", JY);
    prInt("RTT ", RTT);
    /*
    prInt("Ang ", int(_Ang * 180 / M_PI));  // deg here
    prInt("Amp ", int(_Amp));
    prInt("Red ", int(V_Red));
    prInt("Blu ", int(V_Blue));
    prInt("J0BT ",J0_BT);
    prInt("J1BT ",J1_BT);
    prInt("J2BT ",J2_BT);
    prInt("TG0 ",TG0_BT);
    prInt("TG1 ",TG1_BT);
    */
    int cmdMotLF = (V_Blue)-RTT;
    MotoMove(MotLF, cmdMotLF, 50, 1);

    int cmdMotRR = (V_Blue) + RTT;
    MotoMove(MotRR, cmdMotRR, 50, 1);

    int cmdMotRF = (V_Red) + RTT;
    MotoMove(MotRF, cmdMotRF, 50, 1);

    int cmdMotLR = (V_Red)-RTT;
    MotoMove(MotLR, cmdMotLR, 50, 1);


    Serial.println("#");
    delay(10);

    if (J0_BT == 0) {
      MotoMove(MotLF, 0, 50, 0);
      MotoMove(MotRR, 0, 50, 0);
      MotoMove(MotRF, 0, 50, 0);
      MotoMove(MotLR, 0, 50, 0);
      delay(500);
      ESP.restart();
    }
  } else {
    //MotoMove(MotLF, 0, 50, 0);
    //MotoMove(MotRR, 0, 50, 0);
    //MotoMove(MotRF, 0, 50, 0);
    //MotoMove(MotLR, 0, 50, 0);
  }
}

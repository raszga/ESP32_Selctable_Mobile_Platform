#include <WiFi.h>
#include <stdio.h>
#include <string.h>
#include <WiFiUdp.h>
#include "ESP32_Mobile.h"
const char *password = "";
const char *ssid = "GRSD_220";
char packetBuffer[255];
WiFiUDP udp;
//----------------------------------------------------
_intJoy xintJoy;  // variable// Joystick output array
String rS;        // joystick response string
// Driving structure motors and directions
pairPWM MotLR;  // left rear
pairPWM MotLF;  // left front
pairPWM MotRR;  // rigth rear
pairPWM MotRF;  // Rigt Front
pairPWM BzLed;  // Buzzer

// define direction of movemet variables
int J0DX;  //  ....
int J0DY;  //
int J1DX;  //  ....
int J1DY;  //
int J2DX;  //  ....
int J2DY;  //

//Histeresys
int J0DX_H = 50;  //  ....
int J0DY_H = 50;  //
int J1DX_H = 50;  //  ....
int J1DY_H = 50;  //
int J2DX_H = 50;  //  ....
int J2DY_H = 50;  //

//Neutral
int J0DX_N;  //  ....
int J0DY_N;  //
int J1DX_N;  //  ....
int J1DY_N;  //
int J2DX_N;  //  ....
int J2DY_N;  //

// button
int J0_BT;
int J1_BT;
int J2_BT;
int TG0_BT;
int TG1_BT;
int RT;                     // counter rotate//
int hist = 50;              // hys
_flt fltJX, fltJY, fltRTT;  // histoy of inputd FLT lngth
uint8_t ledB = 2;           // led indicator
//====================================================================
// SIGN function
//====================================================================
int sgn(int a) {
  int s = 0;
  if (a > 0) { s = 1; }
  if (a < 0) { s = -1; }
  return s;
}
//======================================================
// Motor movements Fw/Rw function
//======================================================
pairPWM MotoMove(pairPWM M, int cmd, uint8_t hst, bool Verb) {
  // limit max outpur according wit pwm resolution
  if (abs(cmd) > ((1 << LEDC_TIMER_12_BIT) - 1)) {
    M.DTY = sgn(cmd) * ((1 << LEDC_TIMER_12_BIT) - 1);
  } else {
    M.DTY = cmd;
  }
  if (Verb) {
    prInt(M.NAM, M.DTY);
  }
  if ((M.DTY < -hst) or (M.DTY > hst)) {
    if (M.DTY > 0) {
      ledcWrite(M.GP1, M.DTY);
      ledcWrite(M.GP0, 0);
    }  //cmd>0
    if (M.DTY < 0) {
      ledcWrite(M.GP0, -M.DTY);
      ledcWrite(M.GP1, 0);
    }  //cmd<0
  }    //if
  else {
    ledcWrite(M.GP0, 0);
    ledcWrite(M.GP1, 0);
  }  //else
  return M;
}  // END MotoMove---------------------

// motor short break and relase ---------------------------------------------
void MotoBreak(pairPWM M) {
  ledcWrite(M.GP0, 1024);
  ledcWrite(M.GP1, 1024);
}

// Moto report ----------------------------------------------------------------
void MotoReport(pairPWM MTO) {
  Serial.println("**********************************");
  Serial.println(MTO.NAM);
  prInt("PIN0", MTO.GP0);
  prInt("PIN1", MTO.GP1);
  prInt("SPEED", MTO.SPD);
  prInt("DUTY", MTO.DTY);
  prInt("SENSE", MTO.SNS);
}
// Pretty print int -----------------------------------------------------------
void prInt(String T, float i) {
  char s[5];
  sprintf(s, "%05d", int(i));
  Serial.print(T + String(s) + "| ");
}
//===============================================================================
// transfer string
_intJoy TrS(String S) {
  _intJoy tk;
  char sz[] = "1807,1787,1818,1778,1841,1842,   1,   1,   1,   1,   1,   1,   1,   1,   0,   1,   1,   1,   1";  // prototipe
  char buf[sizeof(sz)];
  S.toCharArray(buf, sizeof(buf));
  char *p = buf;
  char *str;
  int k = 0;
  while ((str = strtok_r(p, ",", &p)) != NULL) {
    tk._val[k] = atoi(str);
    k++;
  }
  //for (int j=0;j<k;j++){Serial.print(tk._val[j]);Serial.print("  ");}Serial.println();

  return tk;
}
// angle calculation
double_t Angle(int DX, int DY) {
  double_t A = 0;

  if ((DX != 0) && (DY != 0)) {

    A = (atan(fabs(DY) / fabs(DX)));  // first Q

    if (DX < 0 && DY > 0) {
      A = M_PI - A;
    }  // second Q2

    else if (DX < 0 && DY < 0) {
      A = M_PI + A;
    }  // thirdh Q3
    else if (DX > 0 && DY < 0) {
      A = (2.0 * M_PI) - A;
    }  //Q4
  }

  if ((DX == 0) && (DY > 0)) { A = M_PI / 2; }      // fw
  if ((DX == 0) && (DY < 0)) { A = 3 * M_PI / 2; }  // rw

  if ((DX > 0) && (DY == 0)) { A = 0; }     //
  if ((DX < 0) && (DY == 0)) { A = M_PI; }  //
  if ((DX == 0) && (DY == 0)) { A = 0.0; }  //

  return A;
}  //end of Angle
// soft centering the joystick around neutraal point
int mapJY(int JX, int JXN) {
  int tJX = JX;
  if (JX < 0) { JX = map(
                  tJX, -JXN, 0,
                  -2047, 0); }
  if (JX > 0) {
    JX = map(
      tJX, 0, (4095 - JXN),
      0, 2047);
  }
  return JX;
}

//*****************************************************************************
//------------Setup file-------------------------
//*****************************************************************************
void _setup() {

  ledcAttach(ledB, 12000, 8);  /// setup led params
  ledcWrite(ledB, 0);
  // evrythig to 0*************************************************************
  // directions
  Serial.begin(115200);
  // ***************************WIFI*******************************************
  int i = 0;
  Serial.begin(115200);
  while (WiFi.status() != WL_CONNECTED) {
    i++;
    delay(50);
    Serial.print(F("."));
    Serial.println(i);
  }
  udp.begin(localPort);
  Serial.printf("UDP server : %s:%i \n", WiFi.localIP().toString().c_str(), localPort);
  delay(50);
  // Alocate resources*********************************************************
  Serial.println("Connecting PWM");
  for (int i = 0; i < NrLeds; i++) {
    bool LedResult = ledcAttach(LedArr[i], LEDC_BASE_FREQ, LEDC_TIMER_12_BIT);
    Serial.print(i);
    Serial.print(LedResult);
    if (!LedResult) {
      Serial.println(" Error");
    } else {
      Serial.println(" Succes");
    }
  }
  //****************Detect Neutrals**********************
  int packetSize = udp.parsePacket();
  if (packetSize) {
    int len = udp.read(packetBuffer, 255);  // was 255
    if (len > 0) { packetBuffer[len - 1] = 0; }
    rS = String(packetBuffer);
    xintJoy = TrS(rS);
    J0DX_N = xintJoy._val[0];
    J0DY_N = xintJoy._val[1];
    J1DX_N = xintJoy._val[2];
    J1DY_N = xintJoy._val[3];
    J2DX_N = xintJoy._val[4];
    J2DY_N = xintJoy._val[5];
    delay(500);
    J0DX = xintJoy._val[0] - J0DX_N;
    J0DY = xintJoy._val[1] - J0DY_N;
    J1DX = xintJoy._val[2] - J1DX_N;  // Y direction
    J1DY = xintJoy._val[3] - J1DY_N;  //X dirrection
    J2DX = xintJoy._val[4] - J2DX_N;
    J2DY = xintJoy._val[5] - J2DY_N;  // Rotate

    // self neutral detect
    if ((abs(J0DX) > 50) | (abs(J0DY) > 50) | (abs(J1DX) > 50) | (abs(J1DY) > 50) | (abs(J2DX) > 50) | (abs(J2DY) > 50)) { ESP.restart(); }

    J0_BT = xintJoy._val[6];
    J1_BT = xintJoy._val[8];
    J2_BT = xintJoy._val[7];

    TG0_BT = xintJoy._val[14];
    TG1_BT = xintJoy._val[15];
  }
  MotoMove(MotLF, 0, 50, 0);
  MotoMove(MotRR, 0, 50, 0);
  MotoMove(MotRF, 0, 50, 0);
  MotoMove(MotLR, 0, 50, 0);
}

// Shift values in the input history
_flt shift(int x, _flt ret) {
  for (int k = 1; k < FLT; k++) { ret._[k] = ret._[k - 1]; }
  ret._[0] = x;
  return ret;
}
// averahe of the input history
int _avg(_flt f) {
  int s = 0;
  for (int k = 0; k < FLT; k++) {
    s += f._[k];
  }
  s = int(s / FLT);
  return s;
}
//EOF EOF ---------------------------------------------------------------------
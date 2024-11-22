
#include <Arduino.h>
#include <string.h>
#include <WiFi.h>
#include <stdio.h>
#include <string.h>
#include <WiFiUdp.h>


#ifndef ESP32_Mobile_INCLUDED
#define ESP32_Mobile_INCLUDED
#define LEDC_TIMER_12_BIT 11  // wasm12
#define LEDC_BASE_FREQ 200    // was 100

extern WiFiUDP udp;
extern const char *ssid;  // network
extern const char *password;
extern char packetBuffer[255];
// PWM pins allocated---
// we kept safe TCX/RX0, I2C, ADC 0-5
// for sensors and communication
const int LedArr[] = {
  25,                                                        //MLeftFront [0]
  26,                                                        //[1]
  27,                                                        //MLeftRrsr
  14,                                                        //[3]
  12,                                                        //MRightFront
  13,                                                        //[5]
  23,                                                        // MRightRear
  19,                                                        //[7]
  18,                                                        // Servo Hoist
  5,                                                         // Servo Stick
  17,                                                        // Servo Tilt
  16,                                                        // Servo Clamp
  4,                                                         // Servo Swing
  0,                                                         //
  2,                                                         //
  15                                                         //
};                                                           // pins
const int NrLeds = int(sizeof(LedArr) / sizeof(LedArr[0]));  // number of pwm pins
const unsigned int localPort = 80;
//-----------------------------------------------------------------------------
// joystick message structure a matrix of integers
struct _intJoy {
  int _val[50] = { 0 };
};
// Declare PWM pair (for motor eventualy) structure---------------------------
struct pairPWM {
  String NAM = "---------";  // Name
  uint8_t GP0;               //pin 0 FW
  uint8_t GP1;               // pin 1 Rw
  int DTY = 0;               // duty cycle
  uint16_t SPD = 0;          // speedRPM if exists
  uint8_t SNS = 0;           // _CW(1) or _CCW(2) or _FRN=0 or  _FRWill=3
};
//-----------------------------------------------------------------------------
// filtering data
const int FLT = 8;  // filter length
struct _flt {
  int _[FLT] = { 0 };
};  // input hitory
//------------------------------------------------------------------------------
// extern variables
extern _intJoy xintJoy;  // variable command joystick
extern String rS;        // joystick response string
// Driving structure motors and directions
extern pairPWM MotLR;  // left rear
extern pairPWM MotLF;  // left front
extern pairPWM MotRR;  // Rigth Rear
extern pairPWM MotRF;  // Rigth Front

// define direction of movemet variables
extern int J0DX;  //  ....
extern int J0DY;  //
extern int J1DX;  //  ....
extern int J1DY;  //
extern int J2DX;  //  ....
extern int J2DY;  //

//Histeresys
extern int J0DX_H;  //  ....
extern int J0DY_H;  //
extern int J1DX_H;  //  ....
extern int J1DY_H;  //
extern int J2DX_H;  //  ....
extern int J2DY_H;  //

//neutral positions
extern int J0DX_N;  //  ....
extern int J0DY_N;  //
extern int J1DX_N;  //  ....
extern int J1DY_N;  //
extern int J2DX_N;  //  ....
extern int J2DY_N;  //
// switches

extern int J0_BT;
extern int J1_BT;
extern int J2_BT;
extern int TG0_BT;
extern int TG1_BT;
extern int RT;                     // counterotate
extern int hist;                   // hysteresys
extern int dtPack[24];             // integer array from joystick
extern _flt fltJX, fltJY, fltRTT;  // histoy of inputd FLT lngth
extern uint8_t ledB;               // led indicator
// Functions-------------------------------------------------------------------
int sgn(int a);  // sign function
// motor move-----------------------------------------------------------------
pairPWM MotoMove(pairPWM M, int cmd, uint8_t hst, bool Verb);
// Motor report---------------------------------------------------------------
void MotoReport(pairPWM MTO);
void MotoBreak(pairPWM M);
// pretty print int -----------------------------------------------------------
void prInt(String T, float i);
//---Translate string----------------------------------------------------------
_intJoy TrS(String S);
// angle calculation-----------------------------------------------------------
double_t Angle(int DX, int DY);
// soft center-----------------------------------------------------------------
int mapJY(int JX, int JXN);
// move the stup file here ----------------------------------------------------
void _setup();
// shift-----------------------------------------------------------------------
_flt shift(int x, _flt ret);
//avg--------------------------------------------------------------------------
int _avg(_flt f);
#endif
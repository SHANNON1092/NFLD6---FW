//NEW FIRMWARE - DUAL CORE

//REV2_24 - FD() CHANGE
//REV_2.23 - FD() 
//REV_2.22 - SIGNAL STABLE UPDATED - "SS 0/ SS 1"
//REV_2.21 - MG() CMD MATRIX CHECK REMOVED 
//REV_2.20 - 
//REV_2.19 - ONX COMMAND IMPLEMENTING STARTED - WORKS FINE
//REV_2.18 - AG() ERROR FIXED (DP CHANGE) + WP() IMPLEMENTED
//REV_2.17 - BYPASSED FROM 2.14 - STARTING AZT FOR GS()- AZT WORKS FOR BOTH GN AND GG
//REV_2.14 - ATZ FINE TUNED FOR GN() ONLY
//REV_2.13 - ZT IMPROVEMRNT
//REV_2.12 - ZT IMPROVEMENT - WORKS FOR BOTH POSITIVE AND NEGATIVE VALUES
//REV_2.11 - MODIFIED ZT -  WORKS WELL FOR POSITIVE RANGE. 
//REV_2.10 - STARTED IMPLEMENTING ZERO TRACKING- WORKD WITHOUT DELAY
//REV_2.9 - STARTED IMPLEMENTING STABILITY CHECK CMD
//REV_2.8 - Grounded floating pins set as Inputs , Commented other filter code snippets, AV() created- alpha value of FL3 can be changed (AV 123456 0.004000) 
//REV_2.7 - "RS" implemented [RS pass sn], FL 4 password protected, FL 1-3 exponential
//REV_2.6 - Debugged
//REV_2.5 - "AD" CAHNGE ONLY SET WHEN "SR". DUAL CORE PAUSED FOR ZI,RT,RZ...
//REV_2.4 - "ERROR" SET AGAIN, CI/CM SPACE ERROR FIXED
//REV_2.3 - "ERROR" REMOVED IN \r, FL0 works.
//REV_2.2 - OTHER FILTERS REMOVED. FL5 -> FL1 (new butterworth) | FL6 -> FL2 (Old FIR) | FL7 -> FL3 (new Exponential).
//REV_2.1 -  FL1 TO FL7. NOW ONLY 3 FILTERS (FL5,FL6,FL7)
//REV_2.0 - DUAL CORE, NEW FILTER(EXPONENTIAL - FL7)(FL5 - EXPONENTIAL + BUTTERWORTH), NEW COMMAND (AV - ALPHA VALUE)

#include "LittleFS.h"
#include "WiFi.h"
#include "BluetoothSerial.h"
#include <stdlib.h>
//#include <Filters.h>
//#include <filters.h>
#include <Filters.h>
#include <AH/Timing/MillisMicrosTimer.hpp>
#include <AH/STL/cmath>
//#include <Filters/Butterworth.hpp>
#include <Filters/Notch.hpp>
//#include <Filters/SMA.hpp>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
//#include <Gaussian.h>
//#include "firstorderFilter.h"
#include "math.h"
//firstorderFilter myFilter;

uint32_t loopTimer; //Declaring the time-keeping variable
float F = 0.0009, y, filtered_y; // Signal Frequency set to 10 radians/sec (10/2/pi ~ 1.6 Hz)
float Ts = 0.99; //Sample Time

#define DOUT 22
#define PD_SCK 21
#define RATE 25
//#include <filters_defs.h>
//#include <Filters/SMA.hpp>  // SMA (Simple Moving Average)
//Half and full duplex change
#define RE 17  //4 //A5
#define DE 18  // 6 //A4
#define TempPin A0


////  Define Grounded pins as Inputs
#define IO34 34
#define IO35 35
#define IO32 32
#define IO33 33
#define IO26 26
#define IO27 27
#define IO14 14
#define IO12 12
#define IO13 13
#define IO15 15
#define IO2 2
#define IO5 5
#define IO4 4
#define IO19 19
#define IO23 23

////





bool B16Res = 0;  //Control Analog read resoluation in Temp pin. 1 = 16bit, 0 = 10bit


String HGG;
bool HG = 0;

//////////////////////////////////NEW FILTER EXP//////////////////////
TaskHandle_t SAMPTaskHandle = NULL;
TaskHandle_t FL_SAMPTaskHandle = NULL;
TaskHandle_t GNTaskHandle;

SemaphoreHandle_t filtdataSemaphore;

signed long sampdata = 0;
signed long sampdatar = 0;
volatile bool GGRequested = false;
signed long filtdata = 0;
signed long filtdataST = 0;
signed long filtdataNTC = 0;
/////////////////////////////////////


///////////////////////////////////exP
// Constants
float alpha ;  // Smoothing factor
float alpha2 ;
float alpha3 ;
// Variables
float smoothedValue = 0;
float currentValue = 0;

SemaphoreHandle_t sampdatarMutex;

//////////////////////////////////////////////Butterworth filter

// Sampling frequency 1                       ////////// old FL7/////
const double f_s = 3000;  // Hz   100
// Cut-off frequency (-3 dB)
const double f_c = 1;  // Hz    2.5
// Normalized cut-off frequency
const double f_n = 0.15 * f_c / f_s;

// Sampling frequency 2                       /////////   ////////////
///const double f_s1 = 150;  // Hz   100
// Cut-off frequency (-3 dB)
const double f_c1 = 0.15;  // Hz    2.5
// Normalized cut-off frequency
const double f_n1 = 2 * f_c1 / f_s;

// Sampling frequency 3                       /////////   ////////////
///const double f_s2 = 150;  // Hz   100
// Cut-off frequency (-3 dB)
const double f_c2 = 0.15;  // Hz    2.5
// Normalized cut-off frequency
const double f_n2 = 2 * f_c2 / f_s;

/*
// Sixth-order Butterworth filter
auto filter5 = butter<1>(f_n);
auto filter6 = butter<1>(f_n1);
auto filter7 = butter<1>(f_n2);
*/




//////////////////////////////////////////////FIR  Notch

// Sampling frequency                         ///////// 05 ////////////
///const double f_sn = 50;  // Hz  250
// Notch frequency (-∞ dB)
const double f_cn = 60;  // Hz   50
// Normalized notch frequency
const double f_nn = 2 * f_cn / f_s;
/*
// Very simple Finite Impulse Response notch filter
auto filter8 = simpleNotchFIR(f_nn);       // fundamental
auto filter8A = simpleNotchFIR(2 * f_nn);  // second harmonic
*/

// Sampling frequency                         ///////// 06 ////////////
///const double f_sn = 50;  // Hz  250
// Notch frequency (-∞ dB)
const double f_cn1 = 50;  // Hz   50
// Normalized notch frequency
const double f_nn1 = 2 * f_cn1 / f_s;

// Very simple Finite Impulse Response notch filter
auto filter9 = simpleNotchFIR(f_nn1);       // fundamental
auto filter9A = simpleNotchFIR(2 * f_nn1);  // second harmonic

// Sampling frequency                         ///////// ////////////
///const double f_sn = 50;  // Hz  250
// Notch frequency (-∞ dB)
const double f_cn2 = 80;  // Hz   50
// Normalized notch frequency
const double f_nn2 = 2 * f_cn2 / f_s;
/*
// Very simple Finite Impulse Response notch filter
auto filter10 = simpleNotchFIR(f_nn2);       // fundamental
auto filter10A = simpleNotchFIR(2 * f_nn2);  // second harmonic


/////////////////////// REJECTED //////////////////////////Simple Moving Average (SMA, also known as Running Average)

// Sampling frequency                         /////////FL11////////////
//const double f_s = 100;  // Hz
SMA<10> average11 = { 100 };

// Sampling frequency                         /////////FL12////////////
//const double f_s = 100;  // Hz
SMA<10> average12 = { 300 };

// Sampling frequency                         /////////FL13////////////
//const double f_s = 100;  // Hz
SMA<10> average13 = { 512 };

*/
// Sample timer
Timer<micros> timer = std::round(1e6 / f_s);  //1e6    1e15



// Sample timer
//Timer<micros> timer = std::round(1e6 / f_sn);




//const float cutoff_freq = 5;         //Cutoff frequency in Hz     0.25
//const float sampling_time = 0.008;   //Sampling time in seconds.
//IIR::ORDER order = IIR::ORDER::OD3;  // Order (OD1 to OD4)

// Low-pass filter
//Filter f(50, 0.0001, order);
//Filter f(cutoff_freq, sampling_time, order);

//#define TEMP    7
//#define A0X 21
//#define GAIN1 4
//#define GAIN0   3
//#define TRIG 3
//#define PUSH 7
//#include <EEPROM.h>
//#include <SPI.h>
//#define EEPROM_SIZE 300
int countpdwn = 0;
signed long count;
signed long count_;
signed long sign_;
signed long compliment_;
signed long compliment_2;
signed long data;
signed long data1, datax = 0;

double data2;
double data3;
signed long fx;
String ECZ, ECG, matrixGainS, matrixGainCtrl;
double g;
double h;
double b;


String AD_val,NT_val,NR_val,FL_valwp;

bool AD_val_en = 0;
bool NT_val_en = 0;
bool NR_val_en = 0;
bool FL_val_en = 0;



signed long a = 0;
signed long c;
signed long d;
signed long e;
double w;  //Decimal
int TimeCount = 0;

double ag;
double sv;
long A;
long S;

double g1;
double g2;

///////signal stable
int signal_stab = 0;


signed long county;
signed long count_y;
signed long sign_y;
signed long compliment_y;
signed long compliment_2y;
signed long datay;
signed long data1y;
signed long data2y;
signed long data3y;


signed long countz;
signed long count_z;
signed long sign_z;
signed long compliment_z;
signed long compliment_2z;
signed long dataz;
signed long data1z;
signed long data2z;
signed long data3z;

signed long maxval;
signed long minval;



///////////////STABILITY TEST
long GN1 = 0;
long GN2 = 0;

unsigned long GN1Time;
//unsigned long GN2Time;
bool isGN1 = false;


long Second_VSS;
long First_VSS;

int signalstable = 0;
String signalstablestr;
/////////////////////////////



/////FD VALUES (FACTORY DEFAULT)//////////////

String fd_fl = "68";
String fd_nr = "10";
String fd_nt = "1000";
String fd_um = "2";
String fd_dp = "3";
String fd_ds = "10";
String fd_tn = "0";
String fd_zn = "0";
String fd_ci = "-999999";
String fd_cm = "999999";
String fd_zi = "0";


/////////////////////////////////////////////




long n2o2;


//signed long FL3 = 0;
//signed long FL5 = 0;
int val = 0;
int val1 = 0;

int valy = 0;
int val1y = 0;

int valz = 0;
int val1z = 0;

int inti_GW;
int int_GW_Error = 0;
int int_GW_set = 0;
int int_GW();

void x_read();
//void y_read();
//void z_read();
double GS();
long GN();
void AD();
//void lin_GS();
void Set_NV_ST();
void NR();
void NT();
void SAMP_Value();
void TN();
void ZN();
void Done_p();
void Error_p();
void Ok_p();
void Password_check();
double FD();
void CE();
void ZR();
void TT();

int i;
int j;

int x_stat;
int y_stat;
int z_stat;
int x_status();
int y_status();
int z_status();

long ST();
long CZ();
long RT();
double CS();
double WP();
double AG();
//double SV();

int UD();
int UV();
double read_c();
//double read_sv(); //Set value for trigger
double read_cmaxx();
double read_cminn();
double read_fv();

//signed long CM();
//signed long CI();
//char command;
long read_zero();
long readbutton();
long triggering();
//long comparing();
//long comparing_n();
long g_comparing();
int FL_Val();
int FL_Val_check();

int addr = 0;
int DP();
int D;
long BR_Check();
long B;
int ud0;
int UV_check();
int Lin();
int Lin1();
double decimal();


long GW();  //Global write command
long gw7;
double cm;
double CM();
double cmax;



double CI();
double ci;
double cmin;


//signed long FL_SAMP();

bool bitt;
/////////////////////////////////////
const int buttonPin = 2;  // the number of the pushbutton pin
const int ledPin = 13;    // the number of the LED pin

int value = 0;
//int stat = 0;
int stat_off = 0;
// variables will change:
volatile int buttonState = 0;  // variable for reading the pushbutton status
int lastButtonState = LOW;

//////////////////////////////////////////////////////
String command;
String command1;
String command2;
String commandrs;
String commandav;


///////////for serial number 
int RSNUMlen;
String RSNUM;
String RSPASS;

String FLNUM;
String FLPASS;


int AVNUMlen;
String AVNUM;
String AVPASS;

int trigg = 0;
long comp = 0;
int rc = 1;

long Test();  //need to remove
long test;    // need to remove

String Lg, Lg1, Lg2, Lg3, Lg4, Lg5;  // Long_to String
String l_sign;
String GW0, GW1;
long gw_set = 0;
long E_bit = 0;

long cmaxx1;
long cminn;

///////Factory Cal////

long FZ();
long FG();
double FV();
double FS();
double read_cal_gain();
long read_factry_zero();
double read_fv();
int set_fcal();
int set_scal();
int read_cal_stat();
double cal_Select();
double GV();
double read_gravity();
signed long fctry_zero;
signed long fctry_gain;
signed long fctry_diff;
double cal_gain;
double fv;
long fctry_val1;
int factory_set;
int secondary_set;
double CAL_GAIN;
long gravity_val;
long gravity_val1;
long fset;
double gv;
double Gravity_Factor;
signed long FAC_ZERO;
int read_factoryD();
//CE 100
bool Access;
int CE_value;
//SZ/RZ
signed long sz = 0;
bool SZ_A;
bool st_value;
//SG
bool com;
//ZR 150

// GT
bool GT_A;
//CS
bool AG_sa, ZR_sa, CZ_sa, CG_sa, DP_sa, UV_sa, CM_sa, CI_sa, ZI_sa, TM_sa, UM_sa, ZT_sa, GV_sa, GV_sa1;
double ag7;     //AG
long ZR_value;  //ZR


long ZTF_value;    //ZT

long ZRC_value;  //ZR


long ZTFC_value;    //ZT

long ntemp1,ntemp2;
int ztenable =0;
int ztenable2 =0;

int ztenable3 =0;
long ntempr;
unsigned long ztcurrent;
unsigned long ztstart;
unsigned long previouszt = 0;

unsigned long zttdiff;
const long ztinterval = 2500; 


int countzt = 0;
int zti =0;

int dsval;

long cmaxx;
//NR 110
long NR_value;
//NT 120
long NT_value;

float XX = 0.0;
float XU = 0.0;


bool stability_check;
String Stability;
long G_value;
bool Error;
int fil_val;
bool P_chk = 0;
bool FZ_set = 0;
bool FG_set = 0;
bool FV_set = 0;
bool FP_C = 0, FP_N = 0;
int ZValue;
int Tare_Val;
long C_SZ;

///New/////
long ZI_value;
bool ZI_C;
bool ZR_C;
bool ZI_O;
long C_ZI;
long zi = 0;
bool set_zi = 1;
///////

int Set_kg();
int Set_lb();
int Set_N();
int UOM;

int kg = 0;
int lb = 0;
int N = 0;
int read_UOM();
int UM();

int TM();
//int comp_TM();
int read_tare_mode();
int tare_mode;
int T_allow;
bool ST_comp_bit;
double ST_comp;
long ST_comp_Error;

bool G_comp_bit;
long G_comp;
int Timecount = 0;
// String Check_Stability();
//int comp_n=0;
int AGF = 0;
long AGF2 = 0;
byte zt();
int ZT;
byte read_ZT();
String readF;

//Full Duplec convert
int DX_v = 1;
bool c_ADD = 0, EG_sa, EZ_sa, MG_sa, MC_sa;
//TC Correction

//int TempPin = 0;
float FP3FLT, FP6FLT;
float FP1INT, FP2INT, FP3INT, FP4INT, FP5INT, FP6INT, lnxl, LowTMPCount, TCError, CurrentTempCount, TCWITH;
bool FP1_C, FP2_C, FP3_C, FP4_C, FP5_C, FP6_C, CGCHECK = 0, GSCHECK = 0;
int TCCRL;
float LowADC, LowTempC, HighADC, HighTempC, LowTCount, HIGHTCount;
//void (*resetFunc)(void) = 0;
//signed long valuex;
float NTempVal, NOWTempVal, NTempVal2;
bool TCSSGCONT = false, signcontrol;  //TC For loop off in SG Command
float CalTempCount;
float xx;
//SMA<10> average = { 512 };

//485
int Address_Read;

//NL CODE
int NL_Count, LMax = 0;
String LN1_Val, LN2_Val, LN3_Val, LN4_Val, LN5_Val, LN6_Val, LN7_Val;

//Matrix
bool matrixOn;
float matrixGain;
double matrixGainD;

String RSCot;
bool RScnt = 0;


void setup() {
  // Disable Wi-Fi
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);

  // Disable Bluetooth
  btStop();
  //////////////////////////////////////////////
  // Create task for runtime function (runs continuously on main core)
  /////////////////////////////////////////////


  // delay (4000);
  LittleFS.begin();
  pinMode(PD_SCK, OUTPUT);
  pinMode(DOUT, INPUT);
  //  pinMode(PWDN, OUTPUT);
  pinMode(RATE, OUTPUT);
  //pinMode(A0X, OUTPUT);
  // pinMode(GAIN0, OUTPUT);
  //pinMode(GAIN1, OUTPUT);
  //  pinMode(TRIG, OUTPUT);
  //  pinMode(PUSH, INPUT_PULLUP);
  //EEPROM.begin(EEPROM_SIZE);
  ////////////////////////////485
  pinMode(RE, OUTPUT);
  pinMode(DE, OUTPUT);

//////////////////Grounded pins set as inputs

  pinMode(IO34, INPUT);
  pinMode(IO35, INPUT);
  pinMode(IO32, INPUT);
  pinMode(IO26, INPUT);
  pinMode(IO27, INPUT);
  pinMode(IO14, INPUT);
  pinMode(IO12, INPUT);
  pinMode(IO13, INPUT);
  pinMode(IO15, INPUT);
  pinMode(IO2, INPUT);
  pinMode(IO5, INPUT);
  pinMode(IO4, INPUT);
  pinMode(IO19, INPUT);
  pinMode(IO23, INPUT);

//////////////////



  if (B16Res == 1) { analogReadResolution(16); }


  digitalWrite(PD_SCK, LOW);
  //digitalWrite(PWDN, LOW);
  digitalWrite(RATE, LOW);  // 10SPS LOW
                            // digitalWrite(A0X, LOW);
  // digitalWrite(GAIN0, HIGH);
  // digitalWrite(GAIN1, HIGH);
  //delayMicroseconds(10);
  //digitalWrite(PWDN, HIGH);
  //digitalWrite(PUSH, HIGH);
  /////////////////////////////////////////////////////
  // initialize the LED pin as an output:
  //pinMode(ledPin, OUTPUT);
  // initialize the pushbutton pin as an input:
  //pinMode(buttonPin, INPUT);
  // Attach an interrupt to the ISR vector
  //attachInterrupt(0, pin_ISR, CHANGE); // interrupt pin need to be assigned

  value = 0;
  //stat = 0;
  stat_off = 0;

  //EEPROM.get(5, D);  // Deciml place
  float XX = LRead(5).toFloat();
  D = long(XX);

  FL_Val_check();
  read_c();     // assign f & ag parameters from EEPROM address 3 and 10 for calculate g
  read_zero();  //assign Zero cal SAMP to c
  read_fv();
  read_cal_gain();
  read_factry_zero();
  read_gravity();
  read_cal_stat();
  cal_Select();
  //ud0 = readIntFromEEPROM(35);
  //////////////////////////////////////////////////////////////////
  BR_Check();  //Baudrate check
  //float XX = LRead(240)
  B = LRead(30).toInt();  // Assign baudrate value from EEPROM
  //////////////////////////////////////////////////////////////////////
  UV_check();  // Step value check
  //ud0 = readIntFromEEPROM(35);
  //read_sv();
  read_cmaxx();
  read_cminn();
  //read_ZI();
  read_UOM();
  read_tare_mode();
  read_ZT();

  /////////////////////////////////////////////////////////////////
  //Serial.begin(B);
  //int Baud = 9600;
  Serial.begin(B);
  //////////////////////////////////////////////////////////////////
  //Serial.flush();// newly added
  //Serial.begin(9600);
  //Serial.println("Test");
  //ZI_compairing_n();
  //ZI_compairing_g();
  //SZ();
  ZValue = LRead(140).toInt();
  if (ZValue == 1) {  // Zero non-volatile bit
    //long sz_value;
    //EEPROM.get(210, sz_value);
    float XX = LRead(210).toFloat();
    long sz_value = long(XX);
    sz = sz_value;
  }
  Tare_Val = LRead(130).toInt();
  if (Tare_Val == 1) {
    //long a_value;
    float XX = LRead(220).toFloat();
    long a_value = long(XX);
    //EEPROM.get(220, a_value);
    a = a_value;
  }

  //                  EEPROM.get(110, NR_value);
  float XQ = LRead(110).toFloat();
  NR_value = long(XQ);
  //                  EEPROM.get(120, NT_value);
  float XW = LRead(120).toFloat();
  NT_value = long(XW);
  //                  EEPROM.get(70, ZI_value);
  float XE = LRead(70).toFloat();
  ZI_value = long(XE);

  Duplex();


  ////////////////////////TC/////////////////

  //EEPROM.get(840, TCCRL);  //int
  TCCRL = LRead(840).toInt();

  //EEPROM.get(800, CalTempCount);  //float
  CalTempCount = LRead(800).toFloat();

  //EEPROM.get(810, LowADC);  //float
  LowADC = LRead(810).toFloat();

  //EEPROM.get(820, LowTempC);  //float
  LowTempC = LRead(820).toFloat();

  //EEPROM.get(825, HighADC);  //float
  HighADC = LRead(825).toFloat();

  //EEPROM.get(835, HighTempC);  //float
  HighTempC = LRead(835).toFloat();

  //EEPROM.get(815, LowTCount);  //float
  LowTCount = LRead(815).toFloat();

  //EEPROM.get(830, HIGHTCount);  //float
  HIGHTCount = LRead(830).toFloat();

  //NL count
  NL_Count = LRead(108).toInt();
  Address_Read = LRead(0).toInt();

  //matrix on = 1
  matrixOn = LRead(109).toInt();
  matrixGain = LRead(111).toFloat();  //matrics gain
  matrixGainD = double(matrixGain);

  int AD_value = LRead(0).toInt();
  Address_Read = AD_value;



  dsval = LRead(35).toInt();


  XX = LRead(110).toFloat();
  NR_value = long(XX);

  //EEPROM.get(120, NT_value);
  XU = LRead(120).toFloat();
  NT_value = long(XU);

  ZTFC_value = LRead(70).toFloat();
  ZRC_value = LRead(150).toFloat();

  fil_val = LRead(60).toFloat();
  alpha = LRead(841).toFloat();
  //myFilter.setup(F/4, Ts); // Cutoff Frequency of Filter set to 10/4 = 2.5 radians/sec ~ 0.4 Hz

  startSAMP();
  startFL_SAMP();
}









void loop() {
  //
  //if (rc ==1){
  // GS();
  // readbutton();
  //  }
  unsigned long currentTime = millis();
  if (Serial.available() > 0)  // if there is data comming
  {
    //Serial.println("Test");
    //SCAN();
    command1 = Serial.readStringUntil('\r');
    command = command1.substring(0, 2);
    commandrs = command1.substring(0, 17);
    commandav = command1.substring(0, 18);
    //Serial.println(command);
    command2 = command1.substring(2, 12);
    //Serial.println(command2);
    String FN = command1.substring(0, 3);

    RSPASS = commandrs.substring(3, 9);
    RSNUM = commandrs.substring(10, 17);
    RSNUMlen = RSNUM.length();


    AVPASS = commandrs.substring(3, 9);
    AVNUM = commandrs.substring(10, 18);
    AVNUMlen = RSNUM.length();


    FLPASS = commandrs.substring(3, 9);
    FLNUM = commandrs.substring(10);
   // RSNUMlen = RSNUM.length();

    ///////////////////Newly added 485
    //Duplex_Check();      
    //Serial.print(sampdata);
    //int Address_Read = EEPROM.read(0);
    String N_address = command2.substring(0, 3);
    int ADD = N_address.toInt();



    if (((command == "OP") & (Address_Read == ADD)) | (c_ADD == 1)) {

      c_ADD = 1;
      if ((command == "OP") & (Address_Read == ADD)) {


        Duplex_Check();
        Ok_p();


      } else if (command == "DX") {
        DX();
      } else if (command == "AD") {
        AD();
      }else if(command == "SS"){
        Duplex_Check();
        Serial.print("SS ");
        signalstablestr = String(signalstable);
        Serial.print(signalstablestr);
        Serial.print("\r");
      }else if ((command == "ST") & (command2 == "")) {
        stopSAMP();
        stopFL_SAMP();
        ST();
        startSAMP();
        startFL_SAMP();
      }else if((command == "ON") & (Address_Read == ADD)){ //ONX
      ///
      bool a = true;

      if ((ZI_value > 0) & (set_zi == 1)) {
        ZI();
        set_zi = 0;
      }
      GS();
      ///

    }else if((command == "OG") & (Address_Read == ADD)){ //OGX
      ///
      if ((ZI_value > 0) & (set_zi == 1)) {
        ZI();
        set_zi = 0;
      }
      GN();
      ///

    }else if (command == "AV") {
        AV();

        /*String commandAV = command1.substring(3, 12);
        int commandlAV = commandAV.length();
        if (commandAV != "") {
          if (commandlAV == 8) {
            float alphanew = commandAV.toFloat();
            LWrite(841, String(alphanew,6));
            Duplex_Check();
            Serial.print("OK\r");
          } else {
            Duplex_Check();
            Error_p();
          }
      } else if (commandAV == "") {
        float AVReadD = LRead(841).toFloat();
        Duplex_Check();       
        Serial.print(AVReadD, 8);
        Serial.print("\r");
      }*/        
      }
      /*
      
      else if ((command == "SR") & (command2 == "")) {
        //  wdt_enable(WDTO_1S);
        delay(100);
        Duplex_Check();
        Serial.print("DONE\r");

      }
      
      */


      else if ((command == "SZ") & (command2 == "")) {
        stopSAMP();
        stopFL_SAMP();
        SZ();
        startSAMP();
        startFL_SAMP();
      }
      // else if (command == "TT") {
      //TT();
      // }
      else if (command == "NR") {
        NR();
      } else if (command == "NT") {
        NT();
      } else if ((command == "RZ") & (command2 == "")) {
        stopSAMP();
        stopFL_SAMP();
        RZ();
        startSAMP();
        startFL_SAMP();

      }

      else if ((command == "SG") & (command2 == "") & (DX_v == 1)) {

        if ((ZI_value > 0) & (set_zi == 1)) {
          ZI();
          set_zi = 0;
        }
        com = 0;
        while (com == 0) {
          GN();
          ser();
          Serial.println();
        }
      }
      else if ((command == "SL") & (command2 == "") ) {////print GN for 5 mins
        unsigned long startTime = millis(); // Get the current time
        unsigned long currentTime = millis();
        while ((currentTime - startTime) < 300000) { // Run for 5 minutes
          printTime(); // Print current time
          Serial.print("\t");
          GS();
          Serial.println();
          delayMicroseconds(1000000);
          currentTime = millis(); // Update current time
       }

      }//////////
      else if ((command == "GT") & (command2 == "")) {
        GT_A = 1;

        if ((ZI_value > 0) & (set_zi == 1)) {
          ZI();
          set_zi = 0;
        }
        GN();
      }


      else if (command == "ZR") {
        stopSAMP();
        stopFL_SAMP();
        ZR_C = 1;
        ZR();
        startSAMP();
        startFL_SAMP();
      } else if ((command == "GS") & (command2 == "")) {
        SAMP_Value();  

      } /*else if ((command == "FD") & (command2 == "")) {//////////PRINT THE FILTERD RAW DATA
        Duplex_Check();
       // Serial.print("filtdata = ");
        Serial.print(filtdata);
        Serial.println('\r'); 
      }*/
       else if ((command == "MS") & (command2 == "")) {
        SAMP_Value2();
      } else if (command == "CE") {
        CE();
      } else if (command == "TN") {
        TN();
      } else if (command == "ZN") {
        ZN();
      } else if ((command == "FD") & (command2 == "")) {
        //Serial.println(NL_Count);
        FD();
        //Duplex_Check();
      } else if (FP_N == 1 && FN == "LC") {  //////////////////////////////////////////////////////////LC  for clear NL EEPROM
        LWrite(101, String(0));
        LWrite(102, String(0));
        LWrite(103, String(0));
        LWrite(104, String(0));
        LWrite(105, String(0));
        LWrite(106, String(0));
        LWrite(107, String(0));
        LWrite(108, String(2));
        LMax = 0;
        NL_Count = 2;
        Duplex_Check();
        Done_p();
      }


      else if (command == "FS" && FP_N == 1) {
        String commandfs = command1.substring(3, 10);
        if (commandfs == "900732") {
          if (NL_Count == 7) {
            LWrite(101, String(LN1_Val));
            LWrite(102, String(LN2_Val));
            LWrite(103, String(LN3_Val));
            LWrite(104, String(LN4_Val));
            LWrite(105, String(LN5_Val));
            LWrite(106, String(LN6_Val));
            LWrite(107, String(LN7_Val));
            LWrite(108, String(NL_Count));
          } else if (NL_Count == 6) {
            LWrite(101, String(LN1_Val));
            LWrite(102, String(LN2_Val));
            LWrite(103, String(LN3_Val));
            LWrite(104, String(LN4_Val));
            LWrite(105, String(LN5_Val));
            LWrite(106, String(LN6_Val));
            LWrite(108, String(NL_Count));
          } else if (NL_Count == 5) {
            LWrite(101, String(LN1_Val));
            LWrite(102, String(LN2_Val));
            LWrite(103, String(LN3_Val));
            LWrite(104, String(LN4_Val));
            LWrite(105, String(LN5_Val));
            LWrite(108, String(NL_Count));
          } else if (NL_Count == 4) {
            LWrite(101, String(LN1_Val));
            LWrite(102, String(LN2_Val));
            LWrite(103, String(LN3_Val));
            LWrite(104, String(LN4_Val));
            LWrite(108, String(NL_Count));
          } else if (NL_Count == 3) {
            LWrite(101, String(LN1_Val));
            LWrite(102, String(LN2_Val));
            LWrite(103, String(LN3_Val));
            LWrite(108, String(NL_Count));
          } else if (NL_Count == 2) {
            LWrite(101, String(LN1_Val));
            LWrite(102, String(LN2_Val));
            LWrite(108, String(NL_Count));
          } else if (NL_Count == 1) {
            LWrite(101, String(LN1_Val));
            LWrite(108, String(2));
          }
          FP_N = 0;
          Duplex_Check();
          Done_p();

        } else {
          Duplex_Check();
          Error_p();
        }

        //SAVE LN VALES
        //NL_Count
      }



      else if (command == "LN") {
        String commandlnx = command1.substring(4, 17);
        int commandlL = commandlnx.length();
        String LN = command1.substring(0, 3);



        //////////////////////////////////////////////////LN1//////////////////////////////////////

        if (LN == "LN1") {

          if (commandlnx == "") {  //LN1
            String LN1P = LRead(101);
            //Serial.println(LN1P);
            int ln1GS = LN1P.substring(0, 6).toInt();
            int ln1GY = LN1P.substring(7, 13).toInt();
            //Serial.println(ln1GS);
            //Serial.println(ln1GY);
            Duplex_Check();
            if (ln1GS >= 0) {
              //String p =
              Serial.print('+');
              Serial.print(ln1GS);
              Serial.print(' ');
            } else {
              Serial.print('-');
              Serial.print(ln1GS);
              Serial.print(' ');
            }
            if (ln1GY >= 0) {
              Serial.print('+');  // + ln1GY + '\r');
              Serial.print(ln1GY);
              Serial.print('\r');
            } else {
              //Serial.print('-');          //    + -- + '\r');
              Serial.print('-');  // + ln1GY + '\r');
              Serial.print(ln1GY);
              Serial.print('\r');
            }
          }

          else if (commandlnx != "" && FP_N == 1) {
            if (isInt(commandlnx.substring(0, 6).toInt()) && isInt(commandlnx.substring(7, 13).toInt()) && commandlL == 13) {
              //LWrite(101, commandlnx);
              LN1_Val = commandlnx;
              if (NL_Count <= 1) {
                NL_Count = 1;
              }
              Duplex_Check();
              Ok_p();
              //Serial.println(commandlnx);
            } else {
              Duplex_Check();
              Error_p();
            }
          } else {
            Duplex_Check();
            Error_p();
          }
        }

        //////////////////////////////////////////////////LN2//////////////////////////////////////

        else if (LN == "LN2") {

          if (commandlnx == "") {  //LN1
            String LN2P = LRead(102);
            //Serial.println(LN1P);
            int ln2GS = LN2P.substring(0, 6).toInt();
            int ln2GY = LN2P.substring(7, 13).toInt();
            //Serial.println(ln1GS);
            //Serial.println(ln1GY);
            Duplex_Check();
            if (ln2GS >= 0) {
              //String p =
              Serial.print('+');
              Serial.print(ln2GS);
              Serial.print(' ');
            } else {
              Serial.print('-');
              Serial.print(ln2GS);
              Serial.print(' ');
            }
            if (ln2GY >= 0) {
              Serial.print('+');  // + ln1GY + '\r');
              Serial.print(ln2GY);
              Serial.print('\r');
            } else {
              //Serial.print('-');          //    + -- + '\r');
              Serial.print('-');  // + ln1GY + '\r');
              Serial.print(ln2GY);
              Serial.print('\r');
            }
          }

          else if (commandlnx != "" && FP_N == 1) {
            if (isInt(commandlnx.substring(0, 6).toInt()) && isInt(commandlnx.substring(7, 13).toInt()) && commandlL == 13) {
              //LWrite(102, commandlnx);
              LN2_Val = commandlnx;
              if (NL_Count <= 2) {
                NL_Count = 2;
              }
              Duplex_Check();
              Ok_p();
              //Serial.println(commandlnx);
            } else {
              Duplex_Check();
              Error_p();
            }
          } else {
            Duplex_Check();
            Error_p();
          }
        }
        //////////////////////////////////////////////////LN3//////////////////////////////////////

        else if (LN == "LN3") {

          if (commandlnx == "" && NL_Count >= 3) {  //LN1
            String LN3P = LRead(103);
            //Serial.println(LN1P);
            int ln3GS = LN3P.substring(0, 6).toInt();
            int ln3GY = LN3P.substring(7, 13).toInt();
            //Serial.println(ln1GS);15.95
            //Serial.println(ln1GY);
            Duplex_Check();
            if (ln3GS >= 0) {
              //String p =
              Serial.print('+');
              Serial.print(ln3GS);
              Serial.print(' ');
            } else {
              Serial.print('-');
              Serial.print(ln3GS);
              Serial.print(' ');
            }
            if (ln3GY >= 0) {
              Serial.print('+');  // + ln1GY + '\r');
              Serial.print(ln3GY);
              Serial.print('\r');
            } else {
              //Serial.print('-');          //    + -- + '\r');
              Serial.print('-');  // + ln1GY + '\r');
              Serial.print(ln3GY);
              Serial.print('\r');
            }
          }

          else if (commandlnx != "" && FP_N == 1 && (NL_Count == 2 || NL_Count == 3)) {
            if (isInt(commandlnx.substring(0, 6).toInt()) && isInt(commandlnx.substring(7, 13).toInt()) && commandlL == 13) {
              //LWrite(103, commandlnx);
              LN3_Val = commandlnx;
              if (NL_Count << 3) {
                NL_Count = 3;
              }
              Duplex_Check();
              Ok_p();
              //Serial.println(commandlnx);
            } else {
              Duplex_Check();
              Error_p();
            }
          } else {
            Duplex_Check();
            Error_p();
          }
        }

        //////////////////////////////////////////////////LN4//////////////////////////////////////

        else if (LN == "LN4") {

          if (commandlnx == "" && NL_Count >= 4) {  //LN1
            String LN4P = LRead(104);
            //Serial.println(LN1P);
            int ln4GS = LN4P.substring(0, 6).toInt();
            int ln4GY = LN4P.substring(7, 13).toInt();
            //Serial.println(ln1GS);
            //Serial.println(ln1GY);
            Duplex_Check();
            if (ln4GS >= 0) {
              //String p =
              Serial.print('+');
              Serial.print(ln4GS);
              Serial.print(' ');
            } else {
              Serial.print('-');
              Serial.print(ln4GS);
              Serial.print(' ');
            }
            if (ln4GY >= 0) {
              Serial.print('+');  // + ln1GY + '\r');
              Serial.print(ln4GY);
              Serial.print('\r');
            } else {
              //Serial.print('-');          //    + -- + '\r');
              Serial.print('-');  // + ln1GY + '\r');
              Serial.print(ln4GY);
              Serial.print('\r');
            }
          }

          else if (commandlnx != "" && FP_N == 1 && (NL_Count == 3 || NL_Count == 4)) {
            if (isInt(commandlnx.substring(0, 6).toInt()) && isInt(commandlnx.substring(7, 13).toInt()) && commandlL == 13) {
              //LWrite(104, commandlnx);
              LN4_Val = commandlnx;
              if (NL_Count << 4) {
                NL_Count = 4;
              }
              Duplex_Check();
              Ok_p();
              //Serial.println(commandlnx);
            } else {
              Duplex_Check();
              Error_p();
            }
          } else {
            Duplex_Check();
            Error_p();
          }
        }


        //////////////////////////////////////////////////LN5//////////////////////////////////////

        else if (LN == "LN5") {

          if (commandlnx == "" && NL_Count >= 5) {  //LN1
            String LN5P = LRead(105);
            //Serial.println(LN1P);
            int ln5GS = LN5P.substring(0, 6).toInt();
            int ln5GY = LN5P.substring(7, 13).toInt();
            //Serial.println(ln1GS);
            //Serial.println(ln1GY);
            Duplex_Check();
            if (ln5GS >= 0) {
              //String p =
              Serial.print('+');
              Serial.print(ln5GS);
              Serial.print(' ');
            } else {
              Serial.print('-');
              Serial.print(ln5GS);
              Serial.print(' ');
            }
            if (ln5GY >= 0) {
              Serial.print('+');  // + ln1GY + '\r');
              Serial.print(ln5GY);
              Serial.print('\r');
            } else {
              //Serial.print('-');          //    + -- + '\r');
              Serial.print('-');  // + ln1GY + '\r');
              Serial.print(ln5GY);
              Serial.print('\r');
            }
          }

          else if (commandlnx != "" && FP_N == 1 && (NL_Count == 4 || NL_Count == 5)) {
            if (isInt(commandlnx.substring(0, 6).toInt()) && isInt(commandlnx.substring(7, 13).toInt()) && commandlL == 13) {
              //LWrite(105, commandlnx);
              LN5_Val = commandlnx;
              if (NL_Count << 5) {
                NL_Count = 5;
              }
              Duplex_Check();
              Ok_p();
              //Serial.println(commandlnx);
            } else {
              Duplex_Check();
              Error_p();
            }
          } else {
            Duplex_Check();
            Error_p();
          }
        }


        //////////////////////////////////////////////////LN6//////////////////////////////////////

        else if (LN == "LN6") {

          if (commandlnx == "" && NL_Count >= 6) {  //LN1
            String LN6P = LRead(106);
            //Serial.println(LN1P);
            int ln6GS = LN6P.substring(0, 6).toInt();
            int ln6GY = LN6P.substring(7, 13).toInt();
            //Serial.println(ln1GS);
            //Serial.println(ln1GY);
            Duplex_Check();
            if (ln6GS >= 0) {
              //String p =
              Serial.print('+');
              Serial.print(ln6GS);
              Serial.print(' ');
            } else {
              Serial.print('-');
              Serial.print(ln6GS);
              Serial.print(' ');
            }
            if (ln6GY >= 0) {
              Serial.print('+');  // + ln1GY + '\r');
              Serial.print(ln6GY);
              Serial.print('\r');
            } else {
              //Serial.print('-');          //    + -- + '\r');
              Serial.print('-');  // + ln1GY + '\r');
              Serial.print(ln6GY);
              Serial.print('\r');
            }
          }
          ///////
          else if (commandlnx != "" && FP_N == 1 && (NL_Count == 5 || NL_Count == 6)) {
            if (isInt(commandlnx.substring(0, 6).toInt()) && isInt(commandlnx.substring(7, 13).toInt()) && commandlL == 13) {
              //LWrite(106, commandlnx);
              LN6_Val = commandlnx;
              if (NL_Count << 6) {
                NL_Count = 6;
              }
              Duplex_Check();
              Ok_p();
              //Serial.println(commandlnx);
            } else {
              Duplex_Check();
              Error_p();
            }
          } else {
            Duplex_Check();
            Error_p();
          }
        }


        //////////////////////////////////////////////////LN7//////////////////////////////////////

        else if (LN == "LN7") {

          if (commandlnx == "" && NL_Count == 7) {  //LN1
            String LN7P = LRead(107);
            //Serial.println(LN1P);
            int ln7GS = LN7P.substring(0, 6).toInt();
            int ln7GY = LN7P.substring(7, 13).toInt();
            //Serial.println(ln1GS);
            //Serial.println(ln1GY);
            Duplex_Check();
            if (ln7GS >= 0) {
              //String p =
              Serial.print('+');
              Serial.print(ln7GS);
              Serial.print(' ');
            } else {
              Serial.print('-');
              Serial.print(ln7GS);
              Serial.print(' ');
            }
            if (ln7GY >= 0) {
              Serial.print('+');  // + ln1GY + '\r');
              Serial.print(ln7GY);
              Serial.print('\r');
            } else {
              //Serial.print('-');          //    + -- + '\r');
              Serial.print('-');  // + ln1GY + '\r');
              Serial.print(ln7GY);
              Serial.print('\r');
            }
          }

          else if (commandlnx != "" && FP_N == 1 && (NL_Count == 6 || NL_Count == 7)) {
            if (isInt(commandlnx.substring(0, 6).toInt()) && isInt(commandlnx.substring(7, 13).toInt()) && commandlL == 13) {
              //LWrite(107, commandlnx);
              LN7_Val = commandlnx;
              if (NL_Count << 7) {
                NL_Count = 7;
              }
              Duplex_Check();
              Ok_p();
              //Serial.println(commandlnx);
            } else {
              Duplex_Check();
              Error_p();
            }
          } else {
            Duplex_Check();
            Error_p();
          }
        }

        else {
          Duplex_Check();
          Error_p();
        }
      }




      else if (command == "FP") {
        String NEG = command1.substring(4, 5);

        String ln = command1.substring(4, 15);
        int LowTMPCount = ln.toInt();
        lnxl = ln.length();
        String FP = command1.substring(0, 3);
        if (FP == "FP0") {
          String Password = command1.substring(4, 10);

          if (Password == "900731") {
            FP_C = 1;
            Duplex_Check();
            Ok_p();
          } else if (Password == "") {
            //Only FP0
            float XC = 0;
            for (int i = 0; i < 10; i++) {    //This is not aveage, this loop for getting last value
              XC = XC + analogRead(TempPin);  //need to correct
              delayMicroseconds(10);
            }
            XC /= 10;
            Duplex_Check();
            Serial.print(XC, 0);
            Serial.print("\r");
          }
          /////////////////////////////////////////////Linearization///////////////////////////////
          else if (Password == "900732") {
            FP_N = 1;
            Duplex_Check();
            Ok_p();
          } else {
            Duplex_Check();
            Error_p();
          }
        }


        //////////////////////////////////////////////////////////FP1
        else if (FP == "FP1") {
          String commandx = command1.substring(4, 10);
          if (commandx == "") {  //Only FP0
            float LowADC = 0;
            LowADC = LRead(810).toFloat();
            //EEPROM.get(810, LowADC);
            Duplex_Check();
            Serial.print(LowADC, 0);
            Serial.print("\r");
          }

          else if (commandx != "") {
            if (FP_C == 1 && lnxl == 6 && NEG != "-") {
              String FP10 = command1.substring(4, 5);
              String FP11 = command1.substring(5, 6);
              String FP12 = command1.substring(6, 7);
              String FP13 = command1.substring(7, 8);
              String FP14 = command1.substring(8, 9);
              String FP15 = command1.substring(9, 10);
              String FP1Print = FP10 + FP11 + FP12 + FP13 + FP14 + FP15;
              FP1INT = FP1Print.toFloat();
              FP1_C = 1;
              //Serial.print("lnxl");
              //Serial.print(lnxl);
              Duplex_Check();
              Ok_p();
            } else if (FP_C == 1 && lnxl == 7 && NEG == "-") {
              String FP10 = command1.substring(4, 5);
              String FP11 = command1.substring(5, 6);
              String FP12 = command1.substring(6, 7);
              String FP13 = command1.substring(7, 8);
              String FP14 = command1.substring(8, 9);
              String FP15 = command1.substring(9, 10);
              String FP16 = command1.substring(10, 11);
              String FP1Print = FP10 + FP11 + FP12 + FP13 + FP14 + FP15 + FP16;
              FP1INT = FP1Print.toFloat();
              FP1_C = 1;
              //Serial.print("lnxl");
              //Serial.print(lnxl);
              Duplex_Check();
              Ok_p();
            } else {
              Duplex_Check();
              Error_p();
            }

          } else {
            Duplex_Check();
            Error_p();
          }
        }
        ///////////////////////////////////////////////////////////// FP2

        else if (FP == "FP2") {
          String commandx = command1.substring(4, 10);
          if (commandx == "") {
            float LowTMP = 0;
            LowTMP = LRead(815).toFloat();
            //EEPROM.get(815, LowTMP);
            Duplex_Check();
            Serial.print(LowTMP, 0);
            Serial.print("\r");
          }

          else if (commandx != "") {
            //Serial.print(LowTMPCount);
            if (FP_C == 1 && 0 <= LowTMPCount && LowTMPCount <= 4095 && lnxl == 4) {
              String FP20 = command1.substring(4, 5);
              String FP21 = command1.substring(5, 6);
              String FP22 = command1.substring(6, 7);
              String FP23 = command1.substring(7, 8);
              String FP24 = command1.substring(8, 9);
              String FP2Print = FP20 + FP21 + FP22 + FP23 + FP24;
              FP2INT = FP2Print.toFloat();
              FP2_C = 1;
              //Serial.print("lnxl");
              //Serial.print(lnxl);
              Duplex_Check();
              Ok_p();
            } else {
              Duplex_Check();
              Error_p();
            }
          }

        }


        ////////////////////////////////////////////////////////////////FP3


        else if (FP == "FP3") {
          String commandx = command1.substring(4, 10);
          if (commandx == "") {
            float ATMP = 0;
            ATMP = LRead(820).toFloat();
            //EEPROM.get(820, ATMP);
            Duplex_Check();
            Serial.print(ATMP, 1);
            Serial.print("\r");
          }

          else if (commandx != "") {
            if (FP_C == 1 && lnxl == 4 && NEG != "-") {

              String FP30 = command1.substring(4, 5);
              String FP31 = command1.substring(5, 6);
              String FP32 = command1.substring(6, 7);
              String FP33 = command1.substring(7, 8);
              String FP34 = command1.substring(8, 9);
              String FP3Print = FP30 + FP31 + FP32 + FP33 + FP34;
              FP3INT = FP3Print.toFloat();
              FP3FLT = FP3INT / 10.0;
              FP3_C = 1;
              //Serial.print("lnxl");
              //Serial.print(lnxl);
              Duplex_Check();
              Ok_p();
            }

            else if (FP_C == 1 && lnxl == 5 && NEG == "-") {
              //String FP30 = command1.substring(4, 5);
              String FP31 = command1.substring(5, 6);
              String FP32 = command1.substring(6, 7);
              String FP33 = command1.substring(7, 8);
              String FP34 = command1.substring(8, 9);
              String FP3Print = FP31 + FP32 + FP33 + FP34;
              FP3INT = FP3Print.toFloat();
              FP3FLT = FP3INT / -10.0;
              FP3_C = 1;
              //Serial.print("lnxl");
              //Serial.print(lnxl);
              Duplex_Check();
              Ok_p();

            } else {
              Duplex_Check();
              Error_p();
            }
          }

        }
        ////////////////////////////////////////////////////////////FP4  TO  NEW CHANGE
        //////////////////////////////////////////////////////////FP4
        else if (FP == "FP4") {
          String commandx = command1.substring(4, 10);
          if (commandx == "") {  //Only FP0
            float HIGHADC = 0;
            //EEPROM.get(825, HIGHADC);
            HIGHADC = LRead(825).toFloat();
            Duplex_Check();
            Serial.print(HIGHADC, 0);
            Serial.print("\r");

          }

          else if (commandx != "") {
            if (FP_C == 1 && lnxl == 6 && NEG != "-") {
              String FP40 = command1.substring(4, 5);
              String FP41 = command1.substring(5, 6);
              String FP42 = command1.substring(6, 7);
              String FP43 = command1.substring(7, 8);
              String FP44 = command1.substring(8, 9);
              String FP45 = command1.substring(9, 10);
              String FP4Print = FP40 + FP41 + FP42 + FP43 + FP44 + FP45;
              FP4INT = FP4Print.toFloat();
              FP4_C = 1;
              //Serial.print("lnxl");
              //Serial.print(lnxl);
              Duplex_Check();
              Ok_p();
            }

            if (FP_C == 1 && lnxl == 7 && NEG == "-") {
              String FP40 = command1.substring(4, 5);
              String FP41 = command1.substring(5, 6);
              String FP42 = command1.substring(6, 7);
              String FP43 = command1.substring(7, 8);
              String FP44 = command1.substring(8, 9);
              String FP45 = command1.substring(9, 10);
              String FP46 = command1.substring(10, 11);

              String FP4Print = FP40 + FP41 + FP42 + FP43 + FP44 + FP45 + FP46;
              FP4INT = FP4Print.toFloat();
              FP4_C = 1;
              //Serial.print("lnxl");
              //Serial.print(lnxl);
              Duplex_Check();
              Ok_p();
            }


            else {
              Duplex_Check();
              Error_p();
            }

          } else {
            Duplex_Check();
            Error_p();
          }
        }

        ///////////////////////////////////////////////////////////// FP5

        else if (FP == "FP5") {
          String commandx = command1.substring(4, 10);
          if (commandx == "") {
            float HIGHTMP = 0;
            //EEPROM.get(830, HIGHTMP);
            HIGHTMP = LRead(830).toFloat();
            Duplex_Check();
            Serial.print(HIGHTMP, 0);
            Serial.print("\r");
          }

          else if (commandx != "") {
            if (FP_C == 1 && 0 <= LowTMPCount && LowTMPCount <= 4095 && lnxl == 4) {
              String FP50 = command1.substring(4, 5);
              String FP51 = command1.substring(5, 6);
              String FP52 = command1.substring(6, 7);
              String FP53 = command1.substring(7, 8);
              String FP54 = command1.substring(8, 9);
              String FP5Print = FP50 + FP51 + FP52 + FP53 + FP54;
              FP5INT = FP5Print.toFloat();
              FP5_C = 1;
              //Serial.print("lnxl");
              //Serial.print(lnxl);
              Duplex_Check();
              Ok_p();
            } else {
              Duplex_Check();
              Error_p();
            }
          }

        }


        ////////////////////////////////////////////////////////////////FP6


        else if (FP == "FP6") {
          String commandx = command1.substring(4, 10);
          if (commandx == "") {
            float ATMPH = 0;
            //EEPROM.get(835, ATMPH);
            ATMPH = LRead(835).toFloat();
            Duplex_Check();
            Serial.print(ATMPH, 1);
            Serial.print("\r");
          }


          else if (commandx != "") {
            if (FP_C == 1 && lnxl == 4 && NEG != "-") {

              String FP60 = command1.substring(4, 5);
              String FP61 = command1.substring(5, 6);
              String FP62 = command1.substring(6, 7);
              String FP63 = command1.substring(7, 8);
              String FP64 = command1.substring(8, 9);
              String FP6Print = FP60 + FP61 + FP62 + FP63 + FP64;
              FP6INT = FP6Print.toFloat();
              FP6FLT = FP6INT / 10.0;
              FP6_C = 1;
              //Serial.print("lnxl");
              //Serial.print(lnxl);
              Duplex_Check();
              Ok_p();
            }

            else if (FP_C == 1 && lnxl == 5 && NEG == "-") {
              //String FP30 = command1.substring(4, 5);
              String FP61 = command1.substring(5, 6);
              String FP62 = command1.substring(6, 7);
              String FP63 = command1.substring(7, 8);
              String FP64 = command1.substring(8, 9);
              String FP6Print = FP61 + FP62 + FP63 + FP64;
              FP6INT = FP6Print.toFloat();
              FP6FLT = FP6INT / -10.0;
              FP6_C = 1;
              //Serial.print("lnxl");
              //Serial.print(lnxl);
              Duplex_Check();
              Ok_p();

            } else {
              Duplex_Check();
              Error_p();
            }
          }

        }
        ///////////////////////////////////////////////////////////////FP9
        else if (FP == "FP9") {
          String commandx = command1.substring(4, 10);
          if (commandx == "") {
            int CONT;
            CONT = LRead(840).toInt();
            //EEPROM.get(840, CONT);
            Duplex_Check();
            Serial.print(CONT);
            Serial.print("\r");
          }

          else if (commandx != "" && lnxl == 1 && (NEG == "0" | NEG == "1")) {

            String FP90 = command1.substring(4, 5);
            //int FP9INT = FP90.toInt();
            //EEPROM.put(840, FP9INT);
            LWrite(840, FP90);
            Duplex_Check();
            Serial.print("DONE");
            Serial.print("\r");
          } else {
            Duplex_Check();
            Error_p();
          }
        }
        ///////////////////////////////////////////////////////////////

        else {
          Duplex_Check();
          Error_p();
        }
      }




      ////NEW//
      else if (command == "ZI") {
        stopSAMP();
        stopFL_SAMP();
        ZI_C = 1;
        ZR();
        startSAMP();
        startFL_SAMP();

      }

      else if ((command == "GN") & (command2 == "")) {
        bool a = true;

        if ((ZI_value > 0) & (set_zi == 1)) {
          ZI();
          set_zi = 0;
        }
        //   Stream.flush();
        //delay(5);
        GS();

        //Serial.println();
        //    a = false;
      }

      else if ((command == "RT") & (command2 == "")) {

        stopSAMP();
        stopFL_SAMP();
        RT();  //Reset Tare
        startSAMP();
        startFL_SAMP();

      }
      // else if ((command == "XR")& (command2 == "")) {
      //  x_read();
      //    }Task1

      else if ((command == "GG") & (command2 == "")) {

         if ((ZI_value > 0) & (set_zi == 1)) {
            ZI();
            set_zi = 0;
          }
          GN();
      
      }

      else if ((command == "CZ") & (command2 == "")) {
        stopSAMP();
        stopFL_SAMP();
        CZ();
        startSAMP();
        startFL_SAMP();
      } else if ((command == "CG") & (command2 == "")) {
        stopSAMP();
        stopFL_SAMP();        
        CG();
        startSAMP();
        startFL_SAMP();
      } else if ((command == "CS") & (command2 == "")) {
        stopSAMP();
        stopFL_SAMP();
        CS();
        startSAMP();
        startFL_SAMP();        
        //Serial.println("CEvalue");
      } else if(command == "WP" ){
        //stopSAMP();
        //stopFL_SAMP();
        WP();
        //startSAMP();
        //startFL_SAMP();
      } else if (command == "FZ") {
        FZ();
      } else if (command == "FG") {
        FG();
      } else if (command == "FS") {
        FS();
      } else if (command == "FV") {
        FV();
      } else if (command == "GV") {
        //set_fcal();
        GV();
      } else if (command == "ZT") {
        stopSAMP();
        stopFL_SAMP();
        ZI_C = 1;
        ZTF();
        startSAMP();
        startFL_SAMP();
      } else if ((command == "SN") & (command2 == "") & (DX_v == 1)) {  // |(rc==1)

        if ((ZI_value > 0) & (set_zi == 1)) {
          ZI();
          set_zi = 0;
        }

        bitt = 0;
        while (bitt == 0) {
          GS();
          ser();
        }
      } else if (command == "RS") {
        RS();
      }


      else if ((command == "AD") & (command2 == "")) {
        //  while (1) {
        //   Serial.println(sampdata);
        // }
      }
      // else if ((command == "RS")& (command2 == "")) {// |(rc==1)
      // signed long FL2,FL3 =0;
      //  //float FL2,FL3 =0;
      //        int ll,kk,x =0;
      //        if (bitt == 1){
      //          kk =1;
      //           ll =1;
      //           FL2=0;RS
      //     while (bitt=1){
      //     for (x=1;x<1001;x++){
      //           ////kk= ll++;
      //           ////Serial.println(kk);
      //      FL_sampdata;
      //      FL2=FL2+data1; // SAMP sum need to be correct
      //        }
      //        kk =1;
      //        ll =1;
      //        FL3 = FL2/1000;
      //      Serial.print(FL3);
      //      //Serial.println(data1);
      //      FL2=0;
      //      //data2 = 0;
      //     // readbutton();
      //
      //      ser();
      //     if (bitt == 0){
      //      //Serial.println("Out");
      //      bitt=1;
      //
      //        return;
      //      }
      //
      //       }
      //    }
      //
      //    }

      else if ((command == "ID") & (command2 == "")) {  //((command == "ST")& (command2 == ""))
        String id = "FL6D";
        Duplex_Check();
        Serial.print(id + '\r');
      } else if ((command == "IV") & (command2 == "")) {  //((command == "ST")& (command2 == ""))
        String iv = "2.00";
        Duplex_Check();
        Serial.print(iv + '\r');
      } else if (command == "FD") {
        FD();
      } else if (command == "AG") {
        AG();
      } else if (command == "DP") {
        DP();
      } else if (command == "BR") {
        BRX();
      } else if (command == "FL") {
        FL_Val();
      } else if (command == "UM") {
        UM();
      } else if (command == "TM") {
        TM();
      }/* else if (command == "PM") {
        ParameterLoad();
      } */ // else if (command == "ER") {
         // String readPM = LRead(60);
      //  Serial.print(readPM);
      //   Serial.print("\r");
      //  }


      //  else if (command == "DD") {
      //    five();
      //          }

      //    else if (command == "r") {
      // read_c();
      //    }
      //    else if (command == "M") {
      //M();
      //    }
      //   else if (command == "B") {
      //    BR_Check();
      //   }
      //   else if (command == "UD") {
      //   UD();
      //   }
      else if (command == "DS") {
        UV();
      }
      //         else if (command == "UC") {
      //   UV_check();
      //   }
      //         else if (command == "SA") {
      //   sampdata;
      //   Serial.print(data1);
      //   }
      //    else if (command == "LI") {
      //   lin_GS();
      //   }

      else if (command == "CM") {  // Max Value
        //GW();// global write
        //Serial.println(gw7);
        //double cmax = gw7;
        //return cmax;

        CM();
      }

      else if (command == "CI") {  // Min Value
        //GW();// global write
        //double cmin = gw7;
        //return cmin;
        CI();
      }
      /*           else if (command == "ZI") { // Min Value
      //GW();// global write
      //double cmin = gw7;
      //return cmin;
      ZI();
      }*/
      // else if (command == "RM") {
      //   Serial.print(cm,3);
      //    }
      //
      //    else if (command == "RI") {
      //      Serial.print(ci,3);
      //    }

      // else if (command == "TE") {
      //read_factoryD();
      // UM();
      //   long EE=EEPROM.get(210);
      // Serial.println(zi);
      //}
      //int_GW();
      //     // Test();
      //      //Serial.println(test);
      ////      fctry_val1 = EEPROM.get(55);
      ////      Serial.print(fctry_val1,5);
      ////read_gravity(); //Ok
      ////read_fv();; // Ok
      ////read_cal_gain(); //Ok
      ////read_factry_zero();// Ok
      ////read_gravity();
      ////Serial.println(gv);
      //read_cal_stat();
      //Serial.println(factory_set);
      //Serial.println(secondary_set);
      ////cal_Select();
      //  }
      //           else if (command == "RB") {
      //            //while(1){
      //   digitalWrite(TRIG, HIGH);
      //   //delay (50);
      //   buttonState = digitalRead(PUSH);
      //  Serial.print(buttonState);YU
      //  // }
      //           }
      //   else if (command == "SV") {
      // SV();
      //    }
      //        else if (command == "FC") {
      //rc = 0;
      //    }
      //        else if (command == "cm") {
      //cmaxx = EEPROM.get(25);
      //long_tostring(cmaxx);
      //Serial.print("CM"+l_sign+Lg+Lg1+Lg2+Lg3+Lg4+Lg5+'\r');
      //    }
      //        else if (command == "ci") {
      ////Serial.println(ci,3);
      //cminn = EEPROM.get(40);
      //long_tostring(cminn);
      //Serial.print("CI"+l_sign+Lg+Lg1+Lg2+Lg3+Lg4+Lg5+'\r');
      //    }
      //
      //    else if (command == "YU") {
      //FL_sampdata;
      //long int_CE=0;
      //long def_max=999999;
      //long def_min=-999999;
      //EEPROM.put(100,int_CE);
      //EEPROM.put(25, def_max);
      //EEPROM.put(40, def_min);
      //  }

      else if (command == "CT") {  //Current temp

        //float NOWTempVal;
        for (int i = 0; i < 10; i++) {
          NOWTempVal = analogRead(TempPin);  //need to correct
          delayMicroseconds(10);
        }

        float CurrentTemp = mapFloat(NOWTempVal, LowTCount, HIGHTCount, LowTempC, HighTempC);
        Duplex_Check();
        Serial.print(CurrentTemp, 1);
        Serial.print("\r");
      }

      else if (command == "CL") {  //CALIBRATED temp
        float CurrentTemp = mapFloat(CalTempCount, LowTCount, HIGHTCount, LowTempC, HighTempC);
        Duplex_Check();
        Serial.print(CurrentTemp, 1);
        Serial.print("\r");
      }

      else if (command == "TC") {  //CALIBRATED temp COUNT
        float CalTempCount2 = LRead(800).toFloat();
        ;
        //EEPROM.get(800, CalTempCount2);
        Duplex_Check();
        Serial.print(CalTempCount2, 0);
        Serial.print("\r");
      }

      else if (command == "SR") {  //Power on reset
        Duplex_Check();
        Serial.print("DONE\r");
        //Serial.print("\r");
        delay(500);
        resetFunc();
      }

      //////////////////////////////////////////////////////////////////// Manual Enter CZ
      else if (command == "EZ") {  //EEPROM Write Zero CZ
        String commandEZ = command1.substring(3, 11);
        String NecCon = commandEZ.substring(0, 1);
        int commandlEZ = commandEZ.length();

        if (commandEZ != "") {
          if (Access == 1) {
            if ((commandlEZ == 6 && NecCon != "-") || ((NecCon == "-") && (commandlEZ == 7))) {
              ECZ = commandEZ;
              EZ_sa = 1;
              Duplex_Check();
              Serial.print("OK\r");
            } else {
              Duplex_Check();
              Error_p();
            }
          } else {
            Duplex_Check();
            Error_p();
          }
        }

        else if (commandEZ == "") {
          String EZRead = LRead(15);
          Duplex_Check();
          Serial.print(EZRead);
          Serial.print("\r");
        }
      }

      //////////////////////////////////////////////////////////////////// Manual Enter CG

      else if (command == "EG") {  //EEPROM Write CG
        stopSAMP();
        stopFL_SAMP();
        String commandEG = command1.substring(3, 11);
        String NecCon = commandEG.substring(0, 1);
        int commandlEG = commandEG.length();

        if (commandEG != "") {
          if (Access == 1) {
            if ((commandlEG == 7 && NecCon != "-") || ((NecCon == "-") && (commandlEG == 7))) {
              ECG = commandEG;
              EG_sa = 1;
              Duplex_Check();
              Serial.print("OK\r");
            } else {
              Duplex_Check();
              Error_p();
            }

            //ECZ, ECG;

          } else {
            Duplex_Check();
            Error_p();
          }
        } else if (commandEG == "") {
          String EGRead = LRead(240);
          Duplex_Check();
          Serial.print(EGRead);
          Serial.print("\r");
        }
        startSAMP();
        startFL_SAMP();

      }

      //////////////////////////////////////////////////////////////////// Matrix on off
      else if (command == "MC") {
        String commandMC = command1.substring(3, 5);
        int commandlMC = commandMC.length();
        if (commandMC != "") {
          if (Access == 1) {
            if ((commandlMC == 1) && ((commandMC == "1") || (commandMC == "0"))) {
              matrixGainCtrl = commandMC;
              MC_sa = 1;
              Duplex_Check();
              Serial.print("OK\r");
            } else {
              Duplex_Check();
              Error_p();
            }
          } else {
            Duplex_Check();
            Error_p();
          }
        } else if (commandMC == "") {
          String MCRead = LRead(109);
          Duplex_Check();
          Serial.print(MCRead);
          Serial.print("\r");
        }
      }


      //////////////////////////////////////////////////////////////////// Matrix gain  matrixGain

      else if (command == "MG") {
        String commandMG = command1.substring(3, 16);
        String NecCon = commandMG.substring(0, 1);
        int commandlMG = commandMG.length();
        if (commandMG != "") {
          if (Access == 1) {
            if ((commandlMG == 10) || ((commandlMG == 11) && (NecCon == "-"))) {
              matrixGainS = commandMG;
              MG_sa = 1;
              Duplex_Check();
              Serial.print("OK\r");
            } else {
              Duplex_Check();
              Error_p();
            }
          } else {
            Duplex_Check();
            Error_p();
          }
        } else if (commandMG == "") {
          float MGRead = LRead(111).toFloat();
          double MGReadD = double(MGRead);
          Duplex_Check();

          Serial.print(MGReadD, 8);
          Serial.print("\r");
        }
      }




      ////////////////////////////////////////////////////////////////////  Rev
      else if (command == "RV") {  //(Test Code) Rev
        Duplex_Check();
        Serial.print("V2_REV 2.24\r"); 
      }


      else if ((command == "OP") & (Address_Read != ADD)) {
        c_ADD = 0;
      }else if ((command == "ON") & (Address_Read != ADD)) {
        c_ADD = 0;
      }else if ((command == "OG") & (Address_Read != ADD)) {
        c_ADD = 0;
      }
      else {

        Duplex_Check();
        Error_p();
      }

    /*signal_stab = Signal_stability();
    stability_check = 0;  */


    }else if((command == "ON") & (Address_Read == ADD)){ //ONX
      ///
      bool a = true;

      if ((ZI_value > 0) & (set_zi == 1)) {
        ZI();
        set_zi = 0;
      }
      GS();
      ///

    } else if((command == "OG") & (Address_Read == ADD)){ //ONX
      ///
      if ((ZI_value > 0) & (set_zi == 1)) {
        ZI();
        set_zi = 0;
      }
      GN();
      ///

    }else {
      c_ADD = 0;
    }
  }


/////////////////////////////for stability

   if (!isGN1 && (currentTime - GN1Time >= NT_value)) {

    stability_check = 1;
    GN();
    First_VSS = G_value;
    /*Duplex_Check();
    Serial.print("gn1 = ");
    Serial.print(First_VSS );
    Serial.println("\r");*/
    
    isGN1 = true;
    GN1Time = currentTime; 
    stability_check = 0;
  }
  else if (isGN1 && (currentTime - GN1Time >= NT_value)) {
    stability_check = 1;
    GN();
    stability_check = 0;
    Second_VSS = G_value;
    long D_GGSS = (Second_VSS - First_VSS);
    long R_valueSS = abs(D_GGSS);

    /*Duplex_Check();
    Serial.print("R_valueSS = ");
    Serial.print(R_valueSS );
    Serial.print("     NR_value = ");
    Serial.print(NR_value );
    Serial.println("\r");
    /*
    Duplex_Check();*/
   /*Serial.print("gn2 = ");
    Serial.print(Second_VSS );
    Serial.println("\r");*/
    if (R_valueSS < NR_value) {
      signalstable = 1;
    } else {
      signalstable = 0;
    }
    /*Duplex_Check();
    Serial.print("  XXXXX = ");
    Serial.print(signalstable );
    Serial.println("\r");*/
    isGN1 = false;
    GN1Time = currentTime;
  }
  ///////////////////////////////////////////////////
  if (DX_v == 0) {
    delay(13);              //Half Duplex not working without this delay.
    digitalWrite(DE, LOW);  ///////////////
    delayMicroseconds(10);
    digitalWrite(RE, LOW);
    delayMicroseconds(10);
  }

  if (DX_v == 1) {
    digitalWrite(DE, HIGH);
    digitalWrite(RE, LOW);
    //delayMicroseconds(50);
  }

  if (TCCRL == 1 && CGCHECK == 0 && GSCHECK == 0) {
    TC();
  }
}



//////////////////////////
void startSAMP() {
  if (SAMPTaskHandle == NULL) {
    xTaskCreatePinnedToCore(
    SAMP,
    "SAMPTask",
    10000,
    NULL,
    1,
    &SAMPTaskHandle,
    0  //  core 1
  );
  }
}

void startFL_SAMP() {
  if (FL_SAMPTaskHandle == NULL) {
    xTaskCreatePinnedToCore(
    FL_SAMP,
    "FL_SAMPTask",
    10000,
    NULL,
    0,
    &FL_SAMPTaskHandle,
    0  //  core 1
  );
  }
}

void stopSAMP() {
  if (SAMPTaskHandle != NULL) {
      // Delete the task
      vTaskDelete(SAMPTaskHandle);
      // Set the task handle to NULL indicating the task is not running
      SAMPTaskHandle = NULL;
  }
}

void stopFL_SAMP() {
  if (FL_SAMPTaskHandle != NULL) {
      // Delete the task
      vTaskDelete(FL_SAMPTaskHandle);
      // Set the task handle to NULL indicating the task is not running
      FL_SAMPTaskHandle = NULL;
  }
}



//Serial.println (sampdata);

//}

int x_status() {
  digitalWrite(PD_SCK, LOW);
  //delayMicroseconds(50);
  x_stat = digitalRead(DOUT);
  return x_stat;
}

void x_read() {
  count = 0;
  val1 = 0;
  data = 0;
  data2 = 0;
  data1 = 0;
  data3 = 0;
  digitalWrite(PD_SCK, LOW);  //newly
  //delayMicroseconds(1);
  for (i = 0; i < 24; i++) {
    digitalWrite(PD_SCK, HIGH);
    delayMicroseconds(1);
    val1 = digitalRead(DOUT);
    count = count << 1;
    if (val1 == HIGH) {
      count++;
      //Serial.print("1");
    } else {
      //Serial.print("0");
    }
    digitalWrite(PD_SCK, LOW);
    //delayMicroseconds(1);
  }
  ///////////////////////////////////////////////////////////////
  data = count & 0X00800000;     //|0X02000000;   // X axis signature  l1= 0b00000010,111111111111110010011000
  if (data == 0X00800000) {      //if sign bit = 1
    data1 = count | 0XFF000000;  //set 25th bit to 32 bit as 1 including sign bit also
  } else {
    data1 = count;
  }
  //Serial.println (data,BIN);
  //Serial.println (data);
  // Serial.println (data1,BIN);

  data2 = ((0.0000014153) * data1) - 0.1873569254;  //+11.3996;//+21000;   tare//(5/1516965) (known weight/ differnce between ADC count (0-5))* Current ADC reading
  //data3= data2;//-0.236
  //Serial.println (data3,3);

  // Serial.println(count);
  //Serial.println(data1);
  // Serial.print(data2, 4);
  digitalWrite(PD_SCK, HIGH);
  delay(1);
  digitalWrite(PD_SCK, LOW);  //newly
  delay(1);
}

void SAMP(void *pvParameters) {
  (void)pvParameters;

  while (1) {
    x_status();  // Check the status of X axis ADC
    digitalWrite(DOUT, HIGH);
    delayMicroseconds(1);
    if (digitalRead(DOUT) == LOW) {
      count = 0;
      val1 = 0;
      data = 0;
      data2 = 0;
      data1 = 0;
      data3 = 0;

      for (int i = 0; i < 24; i++) {
        digitalWrite(PD_SCK, HIGH);
        delayMicroseconds(1);
        val1 = digitalRead(DOUT);
        delayMicroseconds(1);
        count = count << 1;
        if (val1 == HIGH) {
          count++;
        }
        digitalWrite(PD_SCK, LOW);
      }

      data = count & 0X00800000;
      if (data == 0X00800000) {
        data1 = count | 0XFF000000;
      } else {
        data1 = count;
      }
      digitalWrite(PD_SCK, HIGH);
      delayMicroseconds(1);
      digitalWrite(PD_SCK, LOW);
      delayMicroseconds(1);
    }

      //xSemaphoreTake(sampdatarMutex, portMAX_DELAY);
      sampdatar = data1;
      //xSemaphoreGive(sampdatarMutex);
    delay(1);
    //taskYIELD();
  }
}



/*static float oneMinusAlpha = 1.0 - alpha;
static float alphaValue = alpha;

// Function to apply exponential smoothing
float FL_EXP(long& currentValue) {


  if (smoothedValue == 0) {
   // Serial.println("sm = 0");
    smoothedValue = currentValue;
  } else {
   // Serial.println("sm = 1");
   // Serial.println(millis());
    smoothedValue = alphaValue * currentValue + oneMinusAlpha * smoothedValue;
   // Serial.println(millis());
    //smoothedValue = currentValue;
  }
  return smoothedValue;
}*/



///////////////////At TM1 tare mode not work properly////////////////
long ST() {
  long a1;
  Check_Stability();
  stability_check = 0;
  if (Stability == "STABLE") {
    ST_comp_bit = 1;  // pass b value from GS function (Nett weight)
    GS();
    ST_comp_bit = 0;
    for(i=0;i<25;i++){
      filtdataST = filtdata;
    }
    

    if (tare_mode == 1 & ST_comp >= 0) {
      a = (filtdataST - zi - sz);
      Duplex_Check();
      Ok_p();
      Set_NV_ST();
      return a;
    } else if (tare_mode == 1 & ST_comp < 0) {

      Duplex_Check();
      Error_p();

    } else if (tare_mode == 0 & ST_comp >= 0) {
      a = (filtdataST - zi - sz);
      Duplex_Check();
      Ok_p();
      Set_NV_ST();
      return a;
    } else if (tare_mode == 0 & ST_comp <= 0) {
      a = (filtdataST - zi - sz);
      Duplex_Check();
      Ok_p();
      Set_NV_ST();
      return a;
    }


  } else {
    Duplex_Check();
    Error_p();
    //Serial.print("UNSATABLE\r");
    return 0;
  }
  filtdataST = 0;
  startSAMP();
  startFL_SAMP();
}


long RT() {
  if (a != 0) {
    a = 0;
    Duplex_Check();
    Ok_p();  //Serial.print("Done\r");
    //double tare =g*a;
    //Serial.println(tare,8);
    //int Tare_Val = EEPROM.read(130);
    if (Tare_Val == 1) {
      //EEPROM.put(220, a);
      LWrite(220, String(a));
    }
    return a;
  } else {
    Duplex_Check();
    Error_p();
    return 0;
  }
}
double GS() {  //Net weight function
  //double b;
  //b = g *(SAMP() -a);// AD counts - Tared count
  //Serial.println(b,D);
  long o1, o2;
  int o3, o4, o5, o6, o7, o8;
  double o9, n0;
  String gross;
  String gross1, gross2, gross3, gross4, gross5, gross6;
  String sign1;
  double b1, b2, b3, b4, b5, b6;  //Start from here
  double ub, ub1, ub2, ub3;
  int k = 0;
  int q = 0;
  //Serial.println(maxval);
  //minval=1000;// Remove the comment after giving to thusitha

  cal_Select();

  if (zi != 0) {
    FAC_ZERO = zi;  // IF ZI is 1
    //Serial.println(FAC_ZERO);
  }

  G_comp_bit = 1;  // pass h value from GN function
  GN();            // return h value without printing
  //comp_TM();

  if (a == 0 & sz == 0) {                         //if not tared & set zero
    b3 = (CAL_GAIN * FAC_ZERO) / Gravity_Factor;  //Zero cal count issue
    b5 = 0;
    b6 = 0;
    st_value = 0;
  } else if (a != 0 & sz == 0) {                  // & T_allow==1){ //if tared
    b5 = (CAL_GAIN * (a + zi)) / Gravity_Factor;  //g*a;// Tare count
    b3 = 0;
    b6 = 0;
    st_value = 1;
  }
  //    else if(a!=0 & sz == 0 & T_allow==0){ //if tared BUT ST not allowed
  //   b5 = (CAL_GAIN *FAC_ZERO)/Gravity_Factor ;//g*a;// Tare count
  //   b3 = 0;
  //   b6 = 0;
  //   st_value = 1;
  //
  //   }

  else if (a == 0 & sz != 0) {  // not tared but set zero

    b6 = (CAL_GAIN * (sz + zi)) / Gravity_Factor;
    b3 = 0;
    b5 = 0;
    st_value = 0;
  }

  else if (a != 0 & sz != 0) {  // & T_allow==1){ // set tared and set zero


    if (st_value != 0) {
      b6 = ((2 * (CAL_GAIN * sz)) - (CAL_GAIN * FAC_ZERO)) / Gravity_Factor;  //
      b5 = 0;
      b3 = 0;

    } else {
      b5 = (CAL_GAIN * (a + zi + sz)) / Gravity_Factor;
      b6 = 0;
      b3 = 0;
    }
  }
  //   else if (a!=0 & sz != 0 & T_allow==0){ // set tared and set zero
  //
  //
  //        if(st_value !=0){
  //           b6 = ((2*(CAL_GAIN*sz))-(CAL_GAIN*FAC_ZERO))/Gravity_Factor;//
  //           b5 = 0;
  //           b3 = 0;
  //
  //          }
  //        else {
  //           b5 = (CAL_GAIN*FAC_ZERO)/Gravity_Factor;
  //           b6 =0;
  //           b3 =0;
  //
  //          }
  //     }


  //b4 = g *(SAMP());// Relevent weight for SAMP
  b4 = (CAL_GAIN * (filtdata)) / Gravity_Factor;  // Relevent weight for SAMP
  b = (b4 - (b3 + b5 + b6));                       // AD counts - Tared count
  //G_comp_bit=1; // pass h value from GN function
  //GN(); // return h value without printing
  if (ST_comp_bit == 1) {
    ST_comp_bit = 0;
    ST_comp = b;
    return ST_comp;
  }

  g_comparing();  // compairing h value based on min and maz value CM CI


  //if(b>ci & b<cm){
  //   comp=2;
  //   Serial.println(comp);
  //   return comp;
  //  }
  //  else if(b<ci){
  //    comp=1;
  //     Serial.println(comp);
  //     return comp;
  //  }
  //  else if(b>cm){
  //      comp=3;
  //       Serial.println(comp);
  //     return comp;
  //  }
  //  else{
  //    Serial.println(b);
  //    Serial.println("error");
  //    }
  //delay (1);
  //comparing_n();
  //Serial.println(b,8);
  //triggering();// Trigger function
  //readbutton();
  //double bz= b-zi;
  double bz = b;
  //b =  g *(SAMP()-c -a);//s-->SAMP() //double check that we need to subtract zero cal samp count from actual samp -c
  double ten1 = pow(10, D);  //PLACE D,// decimal places as a 10^
  o1 = round(ten1);          //
  //Serial.println(o1);
  ub = bz * o1;           // weight*1000
  ub1 = round(ub / ud0);  //(weight*10000)/UV
  ub2 = ub1 * ud0;
  //ub3 = ub2/o1;
  //n0 = (ub3 * o1); // remove DP
  n0 = (ub2);


  //if (n0>999999){ // compare the sign1 //maxval
  //  Serial.print("ooooooo");
  //  Serial.print('\r');
  //  Serial.print('\n');
  //  k=1;
  //}
  //else if(n0<-999999){//minval
  //    Serial.print("uuuuuuu");
  //    Serial.print('\r');
  //    Serial.print('\n');
  //    k=1;
  //}
  //else if (999999>n0>0){
  //  sign1='+';
  //  k=0;
  //}
  //else if (0>n0>-999999){
  //sign1='-';
  //k=0;
  //}
  //else{
  //
  //}

  if (n0 > 0) {  // compare the sign1
    sign1 = '+';
  } else {
    sign1 = '-';
  }


  double absolute1 = round(n0);
  
  if(ztenable3 == 1){  // When Zero tracking is on - get the value from GG()
  o2 = n2o2;
  }else{
    o2 = abs(absolute1);
  }

  o3 = (o2 / 100000U) % 10;
  o4 = (o2 / 10000U) % 10;
  o5 = (o2 / 1000U) % 10;
  o6 = (o2 / 100U) % 10;
  o7 = (o2 / 10U) % 10;
  o8 = (o2 / 1U) % 10;

  String Gross = "N";
  String Decimal = ".";
  gross = String(o3);
  gross1 = String(o4);
  gross2 = String(o5);
  gross3 = String(o6);
  gross4 = String(o7);
  gross5 = String(o8);

  if (D == 0 & k == 0 & comp == 2) {
    Duplex_Check();
    Serial.print(Gross + sign1 + gross + gross1 + gross2 + gross3 + gross4 + gross5 + '\r');  //Serial.print(Gross+sign1+gross+gross1+gross2+gross3+gross4+gross5+'\r'+'\n');
  } else if (D == 1 & k == 0 & comp == 2) {
    Duplex_Check();
    Serial.print(Gross + sign1 + gross + gross1 + gross2 + gross3 + gross4 + Decimal + gross5 + '\r');
  } else if (D == 2 & k == 0 & comp == 2) {
    Duplex_Check();
    Serial.print(Gross + sign1 + gross + gross1 + gross2 + gross3 + Decimal + gross4 + gross5 + '\r');
  } else if (D == 3 & k == 0 & comp == 2) {
    Duplex_Check();
    Serial.print(Gross + sign1 + gross + gross1 + gross2 + Decimal + gross3 + gross4 + gross5 + '\r');

  } else if (D == 4 & k == 0 & comp == 2) {
    Duplex_Check();
    Serial.print(Gross + sign1 + gross + gross1 + Decimal + gross2 + gross3 + gross4 + gross5 + '\r');
  } else if (D == 5 & k == 0 & comp == 2) {
    Duplex_Check();
    Serial.print(Gross + sign1 + gross + Decimal + gross1 + gross2 + gross3 + gross4 + gross5 + '\r');
  } else if (comp == 3) {
    Duplex_Check();
    Serial.print(Gross + "oooooooo\r");
  } else if (comp == 1) {
    Duplex_Check();
    Serial.print(Gross + "uuuuuuuu\r");
  }
  bz = 0;
  //zi =0;
  G_comp_bit = 0;
  return 0;
}

//long triggering(){
//  if (b >sv| trigg ==1){
//  //Serial.println ("High");//**Please comment
//  digitalWrite(TRIG, HIGH);
//  trigg =1;
//  readbutton();//**please comment
//}
//else if (trigg == 0){
//  //Serial.println ("Low");
//  digitalWrite(TRIG, LOW);
//   readbutton();
//}
//}

//long readbutton(){
//delay (50);
//buttonState = digitalRead(PUSH);
////Serial.println(buttonState);
//if (buttonState == 0) {
////Serial.println("button press");
// digitalWrite(TRIG, LOW);
//trigg =0;
////Serial.println("0");
//return trigg;
//}
//else{
// }
// }

//double SV(){
////String AG = command2;
////ag = AG.toInt();
////Serial.println(ag);
////EEPROM.put(10, ag);
////return ag;
//int sv0;
//long sv1,sv2,sv3,sv4,sv5,sv6;
//double sv7;
//
//String SV = command2;
//String SV0=SV.substring(0,1);
//
//String SV1=SV.substring(1,2);
//char CV1 = SV[1];
//String SV2=SV.substring(2,3);
//char CV2 = SV[2];
//String SV3=SV.substring(3,4);
//char CV3 = SV[3];
//String SV4=SV.substring(4,5);
//char CV4 = SV[4];
//String SV5=SV.substring(5,6);
//char CV5 = SV[5];
//String SV6=SV.substring(6,7);
//char CV6 = SV[6];
//
//
//int l = SV.length();
//
//if (SV0==" "& l==7 & (isdigit( CV1&CV2&CV3&CV4&CV5&CV6)) ){// if space & length 7 & only digits
//
//Serial.print("Done\r");
//
//sv1 = (SV1.toInt())*(100000);
//sv2 = (SV2.toInt())*(10000);
//sv3 = (SV3.toInt())*(1000);
//sv4 = (SV4.toInt())*(100);
//sv5 = (SV5.toInt())*(10);
//sv6 = (SV6.toInt())*(1);
//
//S= (sv1+sv2+sv3+sv4+sv5+sv6); // A assigned for sv
////Serial.println(A);
//sv7 = S;  // sv assinged for A
////Serial.println(sv7);
//EEPROM.put(20, sv7);
////double ag10 = EEPROM.get(10);
////Serial.println(ag10);
//decimal();
////Serial.println(w);
//double sv8 = sv7/w;
//double sv9 = sv8;
////Serial.println(ag9,5);
//sv =sv9;
//
////Serial.println(ag);
//
//return sv;// sv return instead of A
//}
//else{
//  Serial.print("ERROR\r");
//}
//return sv;// sv return instead of A
//  }

long CZ() {
  if ((Access == 1) && (matrixOn == 0)) {

    //Serial.println("DZZ");
    //Serial.println("CS Running1");
    Check_Stability();
    //Serial.println("CS Running2");
    //Serial.println("cZZZ");
    stability_check = 0;
    if (Stability == "STABLE") {
 
        startSAMP();
        startFL_SAMP();       
        c = filtdata;
        stopSAMP();
        stopFL_SAMP();
      //EEPROM.put(addr,c);
      // EEPROM.put(1, c);
      Duplex_Check();
      Ok_p();
      Access = 0;
      CZ_sa = 1;
      return c;
    } else {
      Duplex_Check();
      Error_p();
    }
  } else {
    Duplex_Check();
    Error_p();
    return 0;
  }
  //Serial.println("cZ");
  return 0;
}


long CG() {
  if ((Access == 1) && (matrixOn == 0)) {
    Check_Stability();
    stability_check = 0;
    if (Stability == "STABLE") {
      CGCHECK = 1;
      
        startSAMP();
        startFL_SAMP();
        d = filtdata;
        stopSAMP();
        stopFL_SAMP();  


      for (int i = 0; i < 10; i++) {
        NTempVal = analogRead(TempPin);  //need to correct
        delayMicroseconds(10);
      }

      //EEPROM.put(800, NTempVal);
      LWrite(800, String(NTempVal));


      //EEPROM.put(2, d);
      //Serial.print(d);
      Duplex_Check();
      Ok_p();
      CG_sa = 1;
      Access = 0;
      return d;
    } else {
      Duplex_Check();
      Error_p();
      return 0;
    }
  } else {
    Duplex_Check();
    Error_p();
    return 0;
  }
}

double AG() {
  if (Access == 1 & command2 != "") {

    //String AG = command2;
    //ag = AG.toInt();
    //Serial.println(ag);
    //EEPROM.put(10, ag);
    //return ag;
    int ag0;
    long ag1, ag2, ag3, ag4, ag5, ag6;


    String AG = command2;
    String AG0 = AG.substring(0, 1);

    String AG1 = AG.substring(1, 2);
    char CH1 = AG[1];
    String AG2 = AG.substring(2, 3);
    char CH2 = AG[2];
    String AG3 = AG.substring(3, 4);
    char CH3 = AG[3];
    String AG4 = AG.substring(4, 5);
    char CH4 = AG[4];
    String AG5 = AG.substring(5, 6);
    char CH5 = AG[5];
    String AG6 = AG.substring(6, 7);
    char CH6 = AG[6];


    int l = AG.length();

    if (AG0 == " " & l == 7 & (isdigit(CH1 & CH2 & CH3 & CH4 & CH5 & CH6))) {  // if space & length 7 & only digits
      Duplex_Check();
      Done_p();

      ag1 = (AG1.toInt()) * (100000);
      ag2 = (AG2.toInt()) * (10000);
      ag3 = (AG3.toInt()) * (1000);
      ag4 = (AG4.toInt()) * (100);
      ag5 = (AG5.toInt()) * (10);
      ag6 = (AG6.toInt()) * (1);

      A = (ag1 + ag2 + ag3 + ag4 + ag5 + ag6);  // A assigned for ag
      //Serial.println(A);
      ag7 = A;  // ag assinged for A
      //Serial.println(ag7);
      AG_sa = 1;

      //double ag10 = EEPROM.get(10);
      //Serial.println(ag10);
      decimal();
      //Serial.println(w);
      double ag8 = ag7 / w;
      double ag9 = ag8;
      //Serial.println(ag9, 5);
      ag = ag9;

      //Serial.println(ag);

      return ag;  // ag return instead of A
    } else {
      Duplex_Check();
      Error_p();
      return 0;
    }
    Access = 0;
  }

  else if (command2 == "") {  // AG without space
    //D = 5;
    //EEPROM.get(10, AGF);
    float XX = LRead(10).toFloat();
    long AGF = long(XX);
    Duplex_Check();
    if (AGF > 0) {
      Serial.print("AG+");
    } else {
      Serial.print("AG-");
    }
    String lnagx = String(AGF);
    int lnag = lnagx.length();
    lnag = 6 - lnag;
    for (int i = 0; i < lnag; i++) {
      Serial.print("0");
    }
    Serial.print(AGF);
    Serial.print("\r");

    //long_tostring(AGF);
    //Serial.print("AG" + l_sign + Lg + Lg1 + Lg2 + Lg3 + Lg4 + Lg5);  //+'\r');
    //Serial.print('\r');
    //abort();
    //return 0;

  } else {
    Duplex_Check();
    Error_p();
    //return 0;
    // Serial.println("B");
  }
  return 0;
}

double WP() {
  if((AD_val_en == 1)||(NT_val_en == 1)||(NR_val_en == 1)||(FL_val_en ==1)){
    if(AD_val_en == 1){
      LWrite(0, String(AD_val));  
      AD_val_en = 0;


    }
    if(NT_val_en == 1){
      LWrite(120, String(NT_val));  
      NT_val_en = 0;


    }
    if(NR_val_en == 1){
      LWrite(110, String(NR_val));  
      NR_val_en = 0;


    }
    if(FL_val_en == 1){
      LWrite(60, String(FL_valwp));  
      FL_val_en = 0;

    }
    Duplex_Check();
    Done_p();
    
  }else{
    Duplex_Check();
    Error_p();
  }
  return 0;
}








double CS() {
  stopSAMP();
  stopFL_SAMP();
  bool cal_check = 0;
  if (Access == 1) {
    //long k;
    //e = 2;
    fx = d - c;
    //Serial.println(d);
    //Serial.println(c);
    //Serial.println(f);
    if (CG_sa == 1) {
      //Serial.print(f);
      //EEPROM.put(240, f);
      LWrite(240, String(fx));  //NANO 3
      //delay(30);
      CG_sa = 0;
    }

    if (RScnt == 1) {  //Serial Number
      LWrite(2, RSCot);
      RScnt == 0;
    }

    if (EG_sa == 1) {
      LWrite(240, String(ECG));  //ECZ, ECG;
      EG_sa = 0;
    }

    if (MG_sa == 1) {
      LWrite(111, String(matrixGainS));
      MG_sa = 0;
    }

    if (MC_sa == 1) {
      LWrite(109, String(matrixGainCtrl));
      MC_sa = 0;
    }

    if (EZ_sa == 1) {
      LWrite(220, String(0));
      LWrite(210, String(0));
      LWrite(15, String(ECZ));
      EZ_sa == 0;
    }

    if (CZ_sa == 1) {
      long ALL_Z = 0;
      // EEPROM.put(15, c);  //Save zero cal count
      LWrite(15, String(c));
      // EEPROM.put(220, ALL_Z);  //Clear volatile ST
      LWrite(220, String(ALL_Z));
      //  EEPROM.put(210, ALL_Z);  //Clear volatile SZ
      LWrite(210, String(ALL_Z));
      //EEPROM.commit();
      zi = 0;
      cal_check = 1;
      CZ_sa = 0;
    }
    //k = EEPROM.get(240);
    //Serial.println(k,10);
    //g = 2 / f;
    if (AG_sa == 1) {
      //Serial.println(ag7);
      int agsave = ag7;
      //EEPROM.put(10, ag7);
      LWrite(10, String(ag7));
      //delay(300);
      AG_sa = 0;
    }
    if (DP_sa == 1) {
      //EEPROM.put(5, D);
      LWrite(5, String(D));  //NANO 7
      DP_sa = 0;
      //wdt_enable(WDTO_1S);
    }

    if (UV_sa == 1) {
      //EEPROM.put(35, ud0);
      LWrite(35, String(ud0));
      //EEPROM.commit();
      UV_sa = 0;
    }
    if (TM_sa == 1) {
      //EEPROM.put(80, tare_mode);
      LWrite(80, String(tare_mode));
      TM_sa = 0;
    }
    if (UM_sa == 1) {
      //EEPROM.put(75, UOM);
      LWrite(75, String(UOM));
      UM_sa = 0;
    }

    if (GV_sa == 1) {
      //EEPROM.put(65, gravity_val);
      LWrite(65, String(gravity_val));

      GV_sa = 0;
      read_gravity();
    }

    if (GV_sa1 == 1) {
      //EEPROM.put(65, fset);
      LWrite(65, String(fset));
      GV_sa1 = 0;
      read_gravity();
    }

    if (ZT_sa == 1) {
      //EEPROM.put(85, ZT);  // Zero tracking
      LWrite(85, String(ZT));
      ZT_sa = 0;
    }

    if (CM_sa == 1) {
      //EEPROM.put(25, cmaxx);
      LWrite(25, String(cmaxx));
      CM_sa = 0;
    }
    if (CI_sa == 1) {
      // EEPROM.put(40, cminn);
      LWrite(40, String(cminn));
      CI_sa = 0;
    }
    if (ZI_sa == 1) {
      //EEPROM.put(70, ZI_value);
      LWrite(70, String(ZI_value));
      ZI_sa = 0;
    }
    if (ZR_sa == 1) {
      // EEPROM.put(150, ZR_value);
      LWrite(150, String(ZR_value));
      ZR_sa = 0;
    }
    //////////////////////////////////////////////////////FP////////////

    if (FP1_C == 1) {
      //EEPROM.put(810, FP1INT);
      LWrite(810, String(FP1INT));
      //Serial.println(FP1INT);
      FP1_C = 0;
    }
    if (FP2_C == 1) {
      //EEPROM.put(815, FP2INT);
      LWrite(815, String(FP2INT));
      FP2_C = 0;
    }
    if (FP3_C == 1) {
      LWrite(820, String(FP3FLT));
      //EEPROM.put(820, FP3FLT);
      FP3_C = 0;
    }
    if (FP4_C == 1) {
      //EEPROM.put(825, FP4INT);
      LWrite(825, String(FP4INT));
      FP4_C = 0;
    }
    if (FP5_C == 1) {
      LWrite(830, String(FP5INT));
      //EEPROM.put(830, FP5INT);
      FP5_C = 0;
    }
    if (FP6_C == 1) {
      LWrite(835, String(FP6FLT));
      //EEPROM.put(835, FP6FLT);
      FP6_C = 0;
    }


    /////////////////////////////////////////////////////


    if (cal_check == 1) {
      double ste = fx;
      //Serial.println(ste);
      //Serial.println(ag);
      g = ag / ste;

      set_scal();
      int Nvalue = CE_value + 1;
      //Serial.println(Nvalue);
      //EEPROM.put(100, Nvalue);
      LWrite(100, String(Nvalue));
      //delay(500);
      Access = 0;
      Duplex_Check();
      Done_p();
      return g;
    }
    int Nvalue = CE_value + 1;
    //Serial.println(Nvalue);
    //EEPROM.put(100, Nvalue);
    LWrite(100, String(Nvalue));
    //delay(1);
    //Serial.println(Nvalue);
    //EEPROM.commit();
    //delay(500);
    Access = 0;
    Duplex_Check();
    Done_p();
    return 0;  // NOT SURE THIS PLACE
  } else {
    Duplex_Check();
    Error_p();
    return 0;
  }
  //EEPROM.commit();
  //break;
  delay(10);
}

long GN() {  //Gross weight
  //h = g *sampdata;
  //Serial.println(h,D);
  long n2;  //n1,
  int n3, n4, n5, n6, n7, n8;
  double n9, n0;
  long n1;
  String net;
  String net1, net2, net3, net4, net5, net6;
  String sign;
  double b3, b4;
  double gba, gba1, gba2;

  /*Duplex_Check();
    Serial.println("OUT ***");
    Serial.print("\r");*/

  //h = g *(sampdata-c);
  cal_Select();


  if (zi != 0) {  // Value has been assignied initially
    FAC_ZERO = zi;
  }




  //////////////////////////////////////include in rev 36
  if ((a != 0) & (sz == 0)) {
    st_value = 1;
  }
  if ((a == 0) & (sz != 0)) {
    st_value = 0;
  }

  /////////////////////////////////////////////
  if (ZI_O == 1) {

    //b3 = g*c;
    b3 = CAL_GAIN * FAC_ZERO;
    b4 = g * (filtdata);
    //  long FL_SAMP2;
    //   long FL_SAMP2;
    //  for (int i = 0; i < 20; i++) {
    //     FL_SAMP2 = filtdata;
    //  }
    b4 = CAL_GAIN * (filtdata);
    //h = b4-b3;
    h = (b4 - b3) / Gravity_Factor;  //Equation 3 for Gross weight
  }
  ////////////Set Zero//////////////
  else if ((sz != 0) & (GT_A != 1)) {  //if Zero set &  (GT)not requested

    // b3 = g*sz;
    b3 = CAL_GAIN * (sz + zi);  //Actual count where SZ press
    /// b4 = g *(filtdata);
    //    long FL_SAMP2;
    //  for (int i = 0; i < 20; i++) {
    // FL_SAMP2 = filtdata;
    //   }
    b4 = CAL_GAIN * (filtdata);
    //h = b4-b3;
    h = (b4 - b3) / Gravity_Factor;  // Equation 1 @ set Zero

  }

  ///////////////////////////////////////
  else if (GT_A == 1) {  // Tare weight request
    //b4 = g*c;

    if (a == 0) {  //  Set tare is not press
      ///b3 = g*c;
      b4 = CAL_GAIN * FAC_ZERO;
      b3 = CAL_GAIN * FAC_ZERO;
    }
    ///else if((a != 0)&(sz!=0)){
    /// if(st_value !=0){
    ///b3 = CAL_GAIN*a;
    /// b4 = CAL_GAIN*FAC_ZERO;
    /// }
    /// else{
    ///    b3 = CAL_GAIN*a;
    ///     b4 = CAL_GAIN*sz;
    ///  }
    ///}
    else {
      // b3 = g*a;
      b4 = CAL_GAIN * FAC_ZERO;
      ///b3 = CAL_GAIN*a; // Where Set tare press
      b3 = CAL_GAIN * (a + zi);  // Where Set tare press
    }
    //h = (b3-b4);
    h = (b3 - b4) / Gravity_Factor;  //Equation 2 @ Get trae count

  }
  ////////
  else {
    //b3 = g*c;
    b3 = CAL_GAIN * FAC_ZERO;
    //b4 = g *(filtdata);
    b4 = CAL_GAIN * (filtdata);
    //h = b4-b3;
    h = (b4 - b3) / Gravity_Factor;  //Equation 3 for Gross weight
  }


  //Serial.println(h,D);

  //h = (g *sampdata);//s-->sampdata //double check that we need to subtract zero cal samp count from actual samp -c
  //Serial.println(h,5);

    /*Duplex_Check();
    Serial.println("OUT ***");
    Serial.print("\r");*/

   /*Duplex_Check();
   Serial.println("IN ***");
   //Serial.println(n2o2);
   Serial.print("\r");*/

  //if (ST_comp_bit==1){
  //  ST_comp_bit=0;
  //  ST_comp =h;
  //  return ST_comp;
  //}

  g_comparing();  //CI & CM value
  //ZI_compairing();
  //Serial.print(h);
  //Serial.print(zi);
  double hz = h;

  double ten = pow(10, D);  //PLACE D,
  n1 = round(ten);
  gba = hz * n1;

  gba1 = round(gba / ud0);  //Update count
  gba2 = gba1 * ud0;
  ////n1 = 10^D;
  //Serial.println(n1);

  //n0 = (h * n1); // remove DP
  n0 = (gba2);
  //Serial.println(n0);


  if (n0 > 0) {  // compare the sign
    sign = '+';
  } else {
    sign = '-';
  }

  double absolute = round(n0);
  n2 = abs(absolute);
  //n2= abs(n0);
  //Duplex_Check();
  //Serial.println(n2);

  /*Duplex_Check();
  Serial.println("n2 bef ");
  Serial.println(n2);
  Serial.print("ZR val ");
  Serial.println(ZRC_value);
  Serial.print("ZT val ");
  Serial.print(ZTFC_value);
  Serial.println("\r");*/


///////////// Zero tracking ////////////////////////////////////////////////////////////////////////////////

  if((n2 < (ZTFC_value/2)) && (n2 > 0) && (signalstable == 1)&& (ztenable2 == 0)&&(ntempr >= ntemp1)){
    ztenable3 = 1;
    /*Duplex_Check();
    Serial.println("IN IF");
    Serial.print("ZT2 = ");
    Serial.println(ztenable2);
    Serial.print("ZT1 = ");
    Serial.println(ztenable);
    Serial.print("\r");*/
    //countzt = countzt + 1;
    if((ztenable == 0)&& (countzt == 70)){ // if only 70 samples are taken, take the n2 value for zero tracking
      //delay(1000);
     ntemp1 = n2;
     ntempr = ntemp1;
     ztenable = 1;
     ztstart = millis();
     /*Duplex_Check();
     Serial.print("ZSTART ");
     Serial.println(ztstart);*/
     //previouszt = ztstart;
     countzt = 0;
      /*Duplex_Check();
      Serial.print("CT CHAN");
      Serial.println(countzt);
      Serial.print("\r");*/

    }
      /*Duplex_Check();
      Serial.print("count = ");
      Serial.println(countzt);
      Serial.print("\r");*/
    if((ztenable == 0)&&(countzt < 70)&& (countzt >=0)){      // count 70 samples. till that, zero tracking is not enable
      n2 = n2;
      /*Duplex_Check();
      Serial.print("in loop ");
      Serial.println(countzt);
      Serial.print("\r");*/
      countzt = countzt + 1;
    }    
    
    ztcurrent = millis();
    /*Duplex_Check();
    Serial.print("ztcurrent ");
    Serial.println(ztcurrent);
    Serial.print("\r");*/
    //delay(100);
    zttdiff = ztcurrent - ztstart;
    /*Duplex_Check();
    Serial.print("DIFF ");
    Serial.println(zttdiff);
    Serial.print("\r");*/
    if(zttdiff <= ztinterval){
      n2 = ntemp1;
    }else if (ztenable == 1){
      int timesdiff = round(zttdiff / ztinterval);
      ntemp1 = ntemp1 - (dsval*timesdiff);
      n2 = ntemp1;
      ztstart = millis();
    }
    

  
    if(n2 == 0){
      ztenable2 = 1;
      ztenable3 = 0;
      SZZT();
      n2 = 0;

    }else{
      ztenable2 =0;
      //n2 = ntemp1;
    }
    //n2 = ntemp1;
    //delay(2500);
    //Serial.println("\r");
    n2o2 = n2;                //  send the zero trak
    n3 = (n2 / 100000U) % 10;
    n4 = (n2 / 10000U) % 10;
    n5 = (n2 / 1000U) % 10;
    n6 = (n2 / 100U) % 10;
    n7 = (n2 / 10U) % 10;
    n8 = (n2 / 1U) % 10;

    String Gross = "G";
    String Decimal = ".";
    net = String(n3);
    net1 = String(n4);
    net2 = String(n5);
    net3 = String(n6);
    net4 = String(n7);
    net5 = String(n8);

    if (G_comp_bit == 1) {
    G_comp_bit = 0;
    G_comp = h;
   /* Duplex_Check();
    Serial.println("IN ***");
    Serial.print("\r");*/
    return G_comp;

    if (ZI_O == 1) {
      ZI_O = 0;
      C_ZI = gba;
      return C_ZI;  // return value to SZ to calculate zero gain
    }

    if (SZ_A == 1) {
      SZ_A = 0;
      C_SZ = gba;
      return C_SZ;  // return value to SZ to calculate zero gain
    }





   }

    if (stability_check == 1) {
    G_value = n2;
    return G_value;
    }

    if (GT_A == 1) {
      if (D == 0) {
        Duplex_Check();
        Serial.print("T" + sign + net + net1 + net2 + net3 + net4 + net5 + '\r');  //  Serial.print(Gross+sign+net+net1+net2+net3+net4+net5+'\r'+'\n');
        GT_A = 0;
      } else if (D == 1) {
        Duplex_Check();
        Serial.print("T" + sign + net + net1 + net2 + net3 + net4 + "." + net5 + '\r');
        GT_A = 0;
      } else if (D == 2) {
        Duplex_Check();
        Serial.print("T" + sign + net + net1 + net2 + net3 + "." + net4 + net5 + '\r');
        GT_A = 0;
      } else if (D == 3) {
        Duplex_Check();
        Serial.print("T" + sign + net + net1 + net2 + "." + net3 + net4 + net5 + '\r');
        GT_A = 0;
      } else if (D == 4) {
        Duplex_Check();
        Serial.print("T" + sign + net + net1 + "." + net2 + net3 + net4 + net5 + '\r');
        GT_A = 0;
      } else if (D == 5) {
        Duplex_Check();
        Serial.print("T" + sign + net + "." + net1 + net2 + net3 + net4 + net5 + '\r');
        GT_A = 0;
      }
    } else if ((stability_check != 1) & (SZ_A != 1) & (HG == 0)) {
      if (D == 0 & comp == 2) {
        Duplex_Check();
        Serial.print(Gross + sign + net + net1 + net2 + net3 + net4 + net5 + '\r');  //  Serial.print(Gross+sign+net+net1+net2+net3+net4+net5+'\r'+'\n');
      } else if (D == 1 & comp == 2) {
        Duplex_Check();
        Serial.print(Gross + sign + net + net1 + net2 + net3 + net4 + Decimal + net5 + '\r');
      } else if (D == 2 & comp == 2) {
        Duplex_Check();
        Serial.print(Gross + sign + net + net1 + net2 + net3 + Decimal + net4 + net5 + '\r');
      } else if (D == 3 & comp == 2) {
        Duplex_Check();
        Serial.print(Gross + sign + net + net1 + net2 + Decimal + net3 + net4 + net5 + '\r');
      } else if (D == 4 & comp == 2) {
        Duplex_Check();
        Serial.print(Gross + sign + net + net1 + Decimal + net2 + net3 + net4 + net5 + '\r');
      } else if (D == 5 & comp == 2) {
        Duplex_Check();
        Serial.print(Gross + sign + net + Decimal + net1 + net2 + net3 + net4 + net5 + '\r');
      } else if (comp == 3) {
        Duplex_Check();
        Serial.print(Gross + "oooooooo\r");
      } else if (comp == 1) {
        Duplex_Check();
        Serial.print(Gross + "uuuuuuuu\r");
      }
    } else if ((stability_check != 1) & (SZ_A != 1) & (HG == 1)) {
      if (D == 0 & comp == 2) {
        //Duplex_Check();
        HGG = (Gross + sign + net + net1 + net2 + net3 + net4 + net5 + '\r');  //  Serial.print(Gross+sign+net+net1+net2+net3+net4+net5+'\r'+'\n');
      } else if (D == 1 & comp == 2) {
        //Duplex_Check();
        HGG = (Gross + sign + net + net1 + net2 + net3 + net4 + Decimal + net5 + '\r');
      } else if (D == 2 & comp == 2) {
        //Duplex_Check();
        HGG = (Gross + sign + net + net1 + net2 + net3 + Decimal + net4 + net5 + '\r');
      } else if (D == 3 & comp == 2) {
        //Duplex_Check();
        HGG = (Gross + sign + net + net1 + net2 + Decimal + net3 + net4 + net5 + '\r');
      } else if (D == 4 & comp == 2) {
        //Duplex_Check();
        HGG = (Gross + sign + net + net1 + Decimal + net2 + net3 + net4 + net5 + '\r');
      } else if (D == 5 & comp == 2) {
        //Duplex_Check();
        HGG = (Gross + sign + net + Decimal + net1 + net2 + net3 + net4 + net5 + '\r');
      } else if (comp == 3) {
        ///Duplex_Check();
        HGG = (Gross + "oooooooo\r");
      } else if (comp == 1) {
        //Duplex_Check();
        HGG = (Gross + "uuuuuuuu\r");
      }
    }
    hz = 0;
    //zi =0;
    delayMicroseconds(10);
    return 0;
  }else{
    ztenable3 = 0;
    ztenable = 0;
    if((ztenable2 == 1)&&(n2 < (ZTFC_value/2)) && (n2 > 0) && (signalstable == 1)){
      //ztenable2 = 1;
      n2 = 0;
    }else{
      ztenable2 = 0;
    }

    n3 = (n2 / 100000U) % 10;
    n4 = (n2 / 10000U) % 10;
    n5 = (n2 / 1000U) % 10;
    n6 = (n2 / 100U) % 10;
    n7 = (n2 / 10U) % 10;
    n8 = (n2 / 1U) % 10;

    String Gross = "G";
    String Decimal = ".";
    net = String(n3);
    net1 = String(n4);
    net2 = String(n5);
    net3 = String(n6);
    net4 = String(n7);
    net5 = String(n8);

    if (G_comp_bit == 1) {
    G_comp_bit = 0;
    G_comp = h;
   /* Duplex_Check();
    Serial.println("IN ***");
    Serial.print("\r");*/
    return G_comp;
   }

    if (ZI_O == 1) {
      ZI_O = 0;
      C_ZI = gba;
      return C_ZI;  // return value to SZ to calculate zero gain
    }

    if (SZ_A == 1) {
      SZ_A = 0;
      C_SZ = gba;
      return C_SZ;  // return value to SZ to calculate zero gain
    }




   if (stability_check == 1) {
    G_value = n2;
    return G_value;
    }

    if (GT_A == 1) {
      if (D == 0) {
        Duplex_Check();
        Serial.print("T" + sign + net + net1 + net2 + net3 + net4 + net5 + '\r');  //  Serial.print(Gross+sign+net+net1+net2+net3+net4+net5+'\r'+'\n');
        GT_A = 0;
      } else if (D == 1) {
        Duplex_Check();
        Serial.print("T" + sign + net + net1 + net2 + net3 + net4 + "." + net5 + '\r');
        GT_A = 0;
      } else if (D == 2) {
        Duplex_Check();
        Serial.print("T" + sign + net + net1 + net2 + net3 + "." + net4 + net5 + '\r');
        GT_A = 0;
      } else if (D == 3) {
        Duplex_Check();
        Serial.print("T" + sign + net + net1 + net2 + "." + net3 + net4 + net5 + '\r');
        GT_A = 0;
      } else if (D == 4) {
        Duplex_Check();
        Serial.print("T" + sign + net + net1 + "." + net2 + net3 + net4 + net5 + '\r');
        GT_A = 0;
      } else if (D == 5) {
        Duplex_Check();
        Serial.print("T" + sign + net + "." + net1 + net2 + net3 + net4 + net5 + '\r');
        GT_A = 0;
      }
    } else if ((stability_check != 1) & (SZ_A != 1) & (HG == 0)) {
      if (D == 0 & comp == 2) {
        Duplex_Check();
        Serial.print(Gross + sign + net + net1 + net2 + net3 + net4 + net5 + '\r');  //  Serial.print(Gross+sign+net+net1+net2+net3+net4+net5+'\r'+'\n');
      } else if (D == 1 & comp == 2) {
        Duplex_Check();
        Serial.print(Gross + sign + net + net1 + net2 + net3 + net4 + Decimal + net5 + '\r');
      } else if (D == 2 & comp == 2) {
        Duplex_Check();
        Serial.print(Gross + sign + net + net1 + net2 + net3 + Decimal + net4 + net5 + '\r');
      } else if (D == 3 & comp == 2) {
        Duplex_Check();
        Serial.print(Gross + sign + net + net1 + net2 + Decimal + net3 + net4 + net5 + '\r');
      } else if (D == 4 & comp == 2) {
        Duplex_Check();
        Serial.print(Gross + sign + net + net1 + Decimal + net2 + net3 + net4 + net5 + '\r');
      } else if (D == 5 & comp == 2) {
        Duplex_Check();
        Serial.print(Gross + sign + net + Decimal + net1 + net2 + net3 + net4 + net5 + '\r');
      } else if (comp == 3) {
        Duplex_Check();
        Serial.print(Gross + "oooooooo\r");
      } else if (comp == 1) {
        Duplex_Check();
        Serial.print(Gross + "uuuuuuuu\r");
      }
    } else if ((stability_check != 1) & (SZ_A != 1) & (HG == 1)) {
      if (D == 0 & comp == 2) {
        //Duplex_Check();
        HGG = (Gross + sign + net + net1 + net2 + net3 + net4 + net5 + '\r');  //  Serial.print(Gross+sign+net+net1+net2+net3+net4+net5+'\r'+'\n');
      } else if (D == 1 & comp == 2) {
        //Duplex_Check();
        HGG = (Gross + sign + net + net1 + net2 + net3 + net4 + Decimal + net5 + '\r');
      } else if (D == 2 & comp == 2) {
        //Duplex_Check();
        HGG = (Gross + sign + net + net1 + net2 + net3 + Decimal + net4 + net5 + '\r');
      } else if (D == 3 & comp == 2) {
        //Duplex_Check();
        HGG = (Gross + sign + net + net1 + net2 + Decimal + net3 + net4 + net5 + '\r');
      } else if (D == 4 & comp == 2) {
        //Duplex_Check();
        HGG = (Gross + sign + net + net1 + Decimal + net2 + net3 + net4 + net5 + '\r');
      } else if (D == 5 & comp == 2) {
        //Duplex_Check();
        HGG = (Gross + sign + net + Decimal + net1 + net2 + net3 + net4 + net5 + '\r');
      } else if (comp == 3) {
        ///Duplex_Check();
        HGG = (Gross + "oooooooo\r");
      } else if (comp == 1) {
        //Duplex_Check();
        HGG = (Gross + "uuuuuuuu\r");
      }
    }
    hz = 0;
    //zi =0;
    delayMicroseconds(10);
    return 0;

  }
}

long g_comparing() {
  if (h > ci & h < cm) {
    comp = 2;
    //Serial.println(comp);
    return comp;
  } else if (h < ci) {
    comp = 1;
    //Serial.println(comp);
    return comp;
  } else if (h > cm) {
    comp = 3;
    // Serial.println(comp);
    return comp;
  } else {
    return 0;
    //Serial.println(b);
    //Serial.println("error");
  }
}




double read_c() {
  //f =  EEPROM.get(240);
  //ag = EEPROM.get(10);
  //Serial.println(ag);
  //g = ag / f;
  //Serial.println(g,10);
  //return g;

  //EEPROM.get(240, f);
  float XX = LRead(240).toFloat();
  fx = long(XX);

  double ag7 = LRead(10).toDouble();
  //EEPROM.get(10, ag7);  //EEPROM.put(10, ag7);
  decimal();
  double ag8 = ag7 / w;
  double ag9 = ag8;
  ag = ag9;

  double ste = fx;
  g = ag / ste;
  return g;
}

//double read_sv(){
//  double sv7 = EEPROM.get(20);
//  decimal();
//  double sv8 = sv7/w;
//double sv9 = sv8;
//sv =sv9;
//
//return sv;// sv return instead of A
//
//  }


long read_zero() {
  //EEPROM.get(15, c);
  float XX = LRead(15).toFloat();
  c = long(XX);
  return c;
}

//long five() {
//  int va;
//  int vb;
//String ge = command2;
// //String ge = re.substring(3,5);
// Serial.print(ge);
//va = ge.toInt();
//vb = va -3;
//  //Serial.println(re.toInt());
//Serial.print(vb);
////Serial.println(f);
////return g;
//}
//
//int M(){
//  int L;
//  L = readIntFromEEPROM(10);
//Serial.print(L);
//  }

int DP() {
  int dp0;
  int M;
  String dp = command2;

  String dp1 = dp.substring(0, 1);

  String dp2 = dp.substring(1, 2);
  char chdp2 = dp[1];
  //Serial.println(dp2);

  //  String dp3=dp.substring(2,3);
  //  char chdp3 = dp[3];

  int ldp = dp.length();
  dp0 = dp.toInt();
  //Serial.println(ldp);
  if (Access == 1) {
    if (dp0 == 0 & dp1 == " " & ldp == 2 & (isdigit(chdp2))) {  //(dp1==" "& ldp==2 & (isdigit(chdp2))& dp0 ==0 ){// if space & length 7 & only digits
      D = 0;
      DP_sa = 1;

      //Serial.println("0");
      Duplex_Check();
      Ok_p();
      return D;
    } else if (dp0 == 1 & dp1 == " " & ldp == 2 & (isdigit(chdp2))) {  //(dp1==" "& ldp==2 & (isdigit(chdp2))& dp0 ==1){
      D = 1;
      DP_sa = 1;
      // M = readIntFromEEPROM(7);
      //Serial.println(M);
      Duplex_Check();
      Ok_p();
      return D;
    } else if (dp0 == 2 & dp1 == " " & ldp == 2 & (isdigit(chdp2))) {  // (dp1==" "& ldp==2 & (isdigit(chdp2))& dp0 ==2){
      D = 2;
      DP_sa = 1;
      Duplex_Check();
      Ok_p();
      return D;
    } else if (dp0 == 3 & dp1 == " " & ldp == 2 & (isdigit(chdp2))) {
      D = 3;
      DP_sa = 1;
      Duplex_Check();
      Ok_p();
      //M = readIntFromEEPROM(7);
      //Serial.println(M);
      return D;
    } else if (dp0 == 4 & dp1 == " " & ldp == 2 & (isdigit(chdp2))) {
      D = 4;
      DP_sa = 1;
      Duplex_Check();
      Ok_p();
      return D;
    } else if (dp0 == 5 & dp1 == " " & ldp == 2 & (isdigit(chdp2))) {
      D = 5;
      DP_sa = 1;
      Duplex_Check();
      Ok_p();
      return D;
    } else {
      Duplex_Check();
      Error_p();
    }

    Access = 0;
  } else if (dp1 == "" & dp2 == "") {  // DP without space
                                       //D = 5;
    //EEPROM.get(5, M);
    delay(1);
    //Serial.print(M);
    M = LRead(5).toInt();
    Duplex_Check();
    Serial.print("DP ");
    Serial.print(M);
    Serial.print('\r');  //+'\r');
    return 0;
    //String DP_Place = M.toString();
    //Serial.print("DP " + M + '\r');  //+'\r');
    // Serial.print('\r');
  } else {
    Duplex_Check();
    Error_p();
    return 0;
  }
}
//EEPROM.put(20, D);
//M = readIntFromEEPROM(20);
//Serial.println(M);
//return ag;
// L = readIntFromEEPROM(10);
//Serial.println(L);
// }

long BRX() {
  int br0;
  long b;
  String br = command2;
  //Serial.println(AG);
  String br1 = br.substring(0, 1);
  String br2 = br.substring(1, 2);
  char chbr2 = br[1];

  int lbr = br.length();
  br0 = br.toInt();
  //Serial.println(br0);

  if (br0 == 0 & br1 == " " & lbr == 2 & (isdigit(chbr2))) {  //(dp1==" "& ldp==2 & (isdigit(chdp2))& dp0 ==0 )
    B = 9600;
    //Serial.println(B);
    //EEPROM.put(30, B);
    LWrite(30, String(B));
    Duplex_Check();
    Ok_p();
    return B;
  } else if (br0 == 1 & br1 == " " & lbr == 2 & (isdigit(chbr2))) {
    B = 19200;
    //Serial.println(B);
    //EEPROM.put(30, B);
    LWrite(30, String(B));
    Duplex_Check();
    Ok_p();
    // M = readIntFromEEPROM(7);
    //Serial.println(M);
    return B;
  } else if (br0 == 2 & br1 == " " & lbr == 2 & (isdigit(chbr2))) {
    B = 38400;
    //Serial.println(B);
    Duplex_Check();
    Ok_p();
    //EEPROM.put(30, B);
    LWrite(30, String(B));
    return B;
  } else if (br0 == 3 & br1 == " " & lbr == 2 & (isdigit(chbr2))) {
    B = 57600;
    //Serial.println(B);
    //EEPROM.put(30, B);
    LWrite(30, String(B));
    Duplex_Check();
    Ok_p();
    //M = readIntFromEEPROM(7);
    //Serial.println(M);
    return B;
  } else if (br0 == 4 & br1 == " " & lbr == 2 & (isdigit(chbr2))) {
    B = 115200;
    //Serial.println(B);
    Duplex_Check();
    Ok_p();
    //EEPROM.put(30, B);
    LWrite(30, String(B));
    return B;
  } else if (br1 == "" & br2 == "") {
    //EEPROM.get(30, b);
    // String baud = String(b);
    String baud = LRead(30);
    Duplex_Check();
    Serial.print("BR " + baud + '\r');
    //  Serial.print();
  }
  //        else if (br0 == 5){
  //  B = 230400;
  //  Serial.println(B);
  //  EEPROM.put(30, B);
  //  return B;
  //  }
  else {
    Duplex_Check();
    Error_p();
    return 0;
  }
  //EEPROM.put(20, D);
  //M = readIntFromEEPROM(20);
  //Serial.println(M);
  //return ag;
  // L = readIntFromEEPROM(10);
  //Serial.println(L);
  return 0;
}

long BR_Check() {
  //long a;
  //EEPROM.get(30, a);
  //Serial.println(a);
  float XX = LRead(30).toFloat();
  long a = long(XX);
  if (a == 9600 | a == 19200 | a == 38400 | a == 57600 | a == 115200)  // | a == 38400 | a == 57600 | a == 115200)
  {
    B = a;
    //Serial.println("a = 9600");
    return B;
  }

  else {
    B = 9600;
    //Serial.println("Set to 9600");
    //EEPROM.put(30, B);
    LWrite(30, String(B));
    return 0;
  }
  return 0;
}
//void AD(){
//  int AD;
//  String ad = command2;
// //Serial.println(ad);
//AD = ad.toInt();
//Serial.print(AD);
//    }




//int UD(){
//
// double b,b1,b2,b3,b4,b5;
//
//b = g *(sampdata -a) ;// AD counts - Tared count
////Serial.println(b,3);
//b4 = pow(10,D);
////Serial.println(b4);
//b3 = b * b4;
////Serial.println(b3);
//b1 = round (b3/ud0);
////Serial.println(b1);
//b2 = (b1 *ud0) ;
//b5 = (b2 / b4);
//Serial.print(b5,D);
//    }

//
// int Lin(){
//long f1 = 119018;
//g1 = 0.2/f1;
//Serial.print(g1);
//return g1;
// }
// int Lin1(){
//long f2 = 1211648;
//g2 = 1.8/f2;
//Serial.print(g2);
//return g2;
//  }

//void lin_GS() {
//   double b1;
//   long a1;
////  x_status();     //Check the staus of X axis ADC
////  if (x_stat == 0) { // if data ready to read
////    x_read();   // Read the data
////  }
//Lin();
//Lin1();
//a1 = sampdata;
//Serial.print(a1);
//
//if ( a1 <= 259100)
//{
//b1 = g1 *(a1 -a);// AD counts - Tared count
//Serial.print("Low");
//
//Serial.print(b1,D);
//
//}
//else{
//
//b1 = g1 *(a1 -a);
//Serial.print(b1,D);
//Serial.print("High");
//  }
//}

bool ser() {
  //Serial.println("IN");
  if (Serial.available())  // if there is data comming
  {
    //SCAN();
    command1 = Serial.readStringUntil('\r');
    command = command1.substring(0, 2);
    //Serial.println(command);
    command2 = command1.substring(2, 10);
    //Serial.println(command2);


    if ((command != "RC") & (command != "SG")) {
      //Serial.print("DONE");

      rc = 0;
      bitt = 1;
      com = 1;
      return com, bitt;
    }
  }
  return 0;
}
double decimal() {
  double power;
  if (D == 0) {
    power = 1;
    w = power;
    return w;
  } else if (D == 1) {
    power = 10;
    w = power;
    return w;
  } else if (D == 2) {
    power = 100;
    w = power;
    return w;
  } else if (D == 3) {
    power = 1000;
    w = power;
    return w;
  } else if (D == 4) {
    power = 10000;
    w = power;
    return w;
  } else if (D == 5) {
    power = 100000;
    w = power;
    return w;
  } else {
    return 0;
  }
}

//signed long CM(){
//
//int cm0;
//long cm1,cm2,cm3,cm4,cm5,cm6;
//double cm7;
//signed long Max;
//String CM = command2;
//String CM0=CM.substring(0,1);
//
//String CM1=CM.substring(1,2);
//char MC1 = CM[1];             // MC Max Character
//String CM2=CM.substring(2,3);
//char MC2 = CM[2];
//String CM3=CM.substring(3,4);
//char MC3 = CM[3];
//String CM4=CM.substring(4,5);
//char MC4 = CM[4];
//String CM5=CM.substring(5,6);
//char MC5 = CM[5];
//String CM6=CM.substring(6,7);
//char MC6 = CM[6];
//
//
//int l1 = CM.length();
//
//if (CM0==" "& l1==7 & (isdigit( MC1&MC2&MC3&MC4&MC5&MC6)) ){// if space & length 7 & only digits
//
//Serial.println("done");
//
//cm1 = (CM1.toInt())*(100000);
//cm2 = (CM2.toInt())*(10000);
//cm3 = (CM3.toInt())*(1000);
//cm4 = (CM4.toInt())*(100);
//cm5 = (CM5.toInt())*(10);
//cm6 = (CM6.toInt())*(1);
//
//Max= (cm1+cm2+cm3+cm4+cm5+cm6); // A assigned for ag
//Serial.println(Max);
//maxval = Max;  // ag assinged for A
//Serial.println(maxval);
////EEPROM.put(10, maxval);
//return maxval;
//  }
//else{
//  Serial.println("ERROR");
//}
//}S
//
//signed long CI(){
//
//}
signed long FL2, FL3, FL4, FL5 = 0, FL6, FL7;
void FL_SAMP(void *pvParameters) {
  (void)pvParameters;
  
  while (1) {
    //xSemaphoreTake(sampdatarMutex, portMAX_DELAY); 
    sampdata = sampdatar;
    //xSemaphoreGive(sampdatarMutex);  
    if (fil_val == 66) {   //FL1
      currentValue = sampdata;
      alpha2 = 0.005;
      static float oneMinusAlpha2 = 1.0 - alpha2;

      if (filtdataNTC == 0) {
        filtdataNTC = currentValue;
      } else {
        filtdataNTC = alpha2 * currentValue + oneMinusAlpha2 * filtdataNTC;
      }
    } else if (fil_val == 67) {  //FL2
      currentValue = sampdata;
      alpha3 = 0.0045;
      static float oneMinusAlpha3= 1.0 - alpha3;

      if (filtdataNTC == 0) {
        filtdataNTC = currentValue;
      } else {
        filtdataNTC = alpha3 * currentValue + oneMinusAlpha3 * filtdataNTC;
      }
    } else if (fil_val == 68) {  //FL3 - FINE TUNED
      currentValue = sampdata;
      static float oneMinusAlpha = 1.0 - alpha;

      if (filtdataNTC == 0) {
        filtdataNTC = currentValue;
      } else {
        filtdataNTC = alpha * currentValue + oneMinusAlpha * filtdataNTC;
      }
    } else if (fil_val == 50) {
      filtdataNTC = filter9(filter9A(sampdata));
     // return FL6;
    } /*else if (fil_val == 70) {
      filtdataNTC = filter9(filter9A(sampdata));
     // return FL6;
    } else if (fil_val == 71) {
      filtdataNTC = filter10(filter10A(sampdata));
      //return FL6;
    } else if (fil_val == 72) {
      filtdataNTC = average11(sampdata);
     // return FL7;
    } else if (fil_val == 73) {
      filtdataNTC = average12(sampdata);
     // return FL7;
    } else if (fil_val == 74) {
      filtdataNTC = average13(sampdata);
    //  return FL7;
    }*/else if(fil_val == 2){
      filtdataNTC = sampdata;
    }
    else{
      filtdata = filtdataNTC;
    }
    filtdata = filtdataNTC - TCError ;
    //taskYIELD();
  }

}



long GW(String GW) {  // Global write command

  int gw0;
  long gw1, gw2, gw3, gw4, gw5, gw6, gw8, B;
  long B_1;
  E_bit = 0;
  //String GW = command2;
  GW0 = GW.substring(0, 1);

  GW1 = GW.substring(1, 2);
  char CH1 = GW[1];
  String GW2 = GW.substring(2, 3);
  char CH2 = GW[2];
  String GW3 = GW.substring(3, 4);
  char CH3 = GW[3];
  String GW4 = GW.substring(4, 5);
  char CH4 = GW[4];
  String GW5 = GW.substring(5, 6);
  char CH5 = GW[5];
  String GW6 = GW.substring(6, 7);
  char CH6 = GW[6];
  String GW8 = GW.substring(7, 8);  //newly addded
  char CH8 = GW[7];
  //String GW9=GW.substring(8,9); //newly addded
  //char CH9 = GW[8];

  int l = GW.length();
  //Serial.println(l);
  //Serial.println(CH9);
  if (GW0 == " " & l == 7 & (isdigit(CH1 & CH2 & CH3 & CH4 & CH5 & CH6))) {  // if space & length 7 & only digits

    //Serial.print("Done\r");

    gw1 = (GW1.toInt()) * (100000);
    gw2 = (GW2.toInt()) * (10000);
    gw3 = (GW3.toInt()) * (1000);
    gw4 = (GW4.toInt()) * (100);
    gw5 = (GW5.toInt()) * (10);
    gw6 = (GW6.toInt()) * (1);

    B = (gw1 + gw2 + gw3 + gw4 + gw5 + gw6);  // A assigned for gw (global write)
    gw7 = B;                                  // gw assinged for A
    //Serial.println(gw7);
    return gw7;

    //return gw_set=1;
    ////////////////////////////////////////////
    //code that need to include under each command

    //EEPROM.put(10, gw7);
    //decimal();
    //double sv2_1 = gw7/w;
    //double sv2_3 = sv2_1;
    //sv2 =sv2_3;

    //////////////////////////////////////////
  } else if (GW0 == " " & GW1 == "+" & (isdigit(CH2 & CH3 & CH4 & CH5 & CH6 & CH8)) & l == 8) {  // & (isdigit( CH2&CH3&CH4&CH5&CH6&CH8))){//& l==8 if space & length 7 & only digits

    //Serial.print("+\r");

    gw2 = (GW2.toInt()) * (100000);
    gw3 = (GW3.toInt()) * (10000);
    gw4 = (GW4.toInt()) * (1000);
    gw5 = (GW5.toInt()) * (100);
    gw6 = (GW6.toInt()) * (10);
    gw8 = (GW8.toInt()) * (1);

    B = (gw2 + gw3 + gw4 + gw5 + gw6 + gw8);  //concider gw1 as "+" sign

    gw7 = B;  // gw assinged for A
    //Serial.println(gw7);
    return gw7;
    //return gw_set=1;
  } else if (GW0 == " " & GW1 == "-" & l == 8 & (isdigit(CH2 & CH3 & CH4 & CH5 & CH6 & CH8))) {  // if space & length 7 & only digits

    //Serial.print("-\r");

    gw2 = (GW2.toInt()) * (100000);
    gw3 = (GW3.toInt()) * (10000);
    gw4 = (GW4.toInt()) * (1000);
    gw5 = (GW5.toInt()) * (100);
    gw6 = (GW6.toInt()) * (10);
    gw8 = (GW8.toInt()) * (1);

    B_1 = (gw2 + gw3 + gw4 + gw5 + gw6 + gw8);  //concider gw1 as "-" sign
    B = -1 * (B_1);
    gw7 = B;  // gw assinged for A
    //Serial.println(gw7);
    return gw7;
    //return gw_set=1;
  }

  else if (GW0 == "" & GW1 == "" & l == 0) {
    //Serial.println("got");
    gw_set = 1;
    //    Error = 1;
    //   Error_p();
    return gw_set;
  }

  else {
    Duplex_Check();
    //Serial.println("HERE !!!!");
    Error_p();
    Error = 1;
    E_bit = 1;
    return E_bit;
  }
  //return gw7;// ag return instead of A
}

double CM() {

  GW(command2);
  if (Access == 1 & command2 != "") {
    //Serial.println(gw_set);
    //if(gw7>0 & gw7<=999999 & gw_set==0 & E_bit==0){
    if (gw7 >= 1 & gw7 <= 999999 & gw_set == 0 & E_bit == 0) {
      Duplex_Check();
      Ok_p();
      cmaxx = gw7;
      CM_sa = 1;
      decimal();
      double cm_1 = cmaxx / w;
      double cm_2 = cm_1;
      cm = cm_2;
      //Serial.print(cm,8);
      //Serial.print('\r');
      gw7 = 1111111;  // commented on 2 jan 2022
      return cm;
    } else if (gw7 <= 0 & gw_set == 0 & E_bit == 0) {
      //Serial.print("OUT\r");
      Duplex_Check();
      Error_p();
      gw7 = 1111111;
    }else {
    Duplex_Check();
    Error_p();
    return 0;
   // Duplex_Check();
    //Serial.println("IN ERROR");
  }
    Access = 0;
  } else if (gw_set == 1 & E_bit == 0) {
    //long cmaxx2;
    float XX = LRead(25).toFloat();
    cmaxx1 = long(XX);
    //EEPROM.get(25, cmaxx1);
    long_tostring(cmaxx1);
    Duplex_Check();
    Serial.print("CM" + l_sign + Lg + Lg1 + Lg2 + Lg3 + Lg4 + Lg5 + '\r');
    gw_set = 0;
    return 0;
  } else {
    Duplex_Check();
    Error_p();
    return 0;
  }
}

double CI() {

  GW(command2);
  if (Access == 1 & command2 != "") {
    //if( gw7>=-999999 & gw7<0 & gw_set==0 & E_bit==0){
    if (gw7 >= -999999 & gw7 <= 0 & gw_set == 0 & E_bit == 0) {
      Duplex_Check();
      Ok_p();
      cminn = gw7;
      CI_sa = 1;
      decimal();
      double ci_1 = cminn / w;
      double ci_2 = ci_1;
      ci = ci_2;
      //Serial.print(ci);
      //Serial.print('\r');
      gw7 = -1111111;
      return ci;
    }
    //else if(gw7>=0& gw_set==0 & E_bit==0){
    else if (gw7 >= 1 & gw_set == 0 & E_bit == 0) {
      //Serial.print("OUT\r");
      Duplex_Check();
      Error_p();
      gw7 = -1111111;
      return 0;
    }else if (E_bit == 1) {
      Duplex_Check();
      //Serial.println("Space error");
      Error_p();
      return 0;
    }
    Access = 0;
  } else if (gw_set == 1 & E_bit == 0) {
    //EEPROM.get(40, cminn);
    float XX = LRead(40).toFloat();
    cminn = long(XX);
    long_tostring(cminn);
    Duplex_Check();
    Serial.print("CI" + l_sign + Lg + Lg1 + Lg2 + Lg3 + Lg4 + Lg5 + '\r');
    gw_set = 0;
    return 0;
  } else {
    Duplex_Check();
    Error_p();
    return 0;
   // Duplex_Check();
    //Serial.println("IN ERROR");
  }
}


long comparing() {
  if (b > ci & b < cm) {
    comp = 2;
    //Serial.println(comp);
    return comp;
  } else if (b < ci) {
    comp = 1;
    //Serial.println(comp);
    return comp;
  } else if (b > cm) {
    comp = 3;
    // Serial.println(comp);
    return comp;
  } else {
    return 0;
    //Serial.println(b);
    //Serial.println("error");
  }
}

void long_tostring(int l_value) {
  int l_val3, l_val4, l_val5, l_val6, l_val7, l_val8;
  long l_value1;

  if (l_value > 0) {  // compare the sign
    l_sign = '+';
  } else {
    l_sign = '-';
  }
  l_value1 = abs(l_value);
  l_val3 = (l_value1 / 100000U) % 10;
  l_val4 = (l_value1 / 10000U) % 10;
  l_val5 = (l_value1 / 1000U) % 10;
  l_val6 = (l_value1 / 100U) % 10;
  l_val7 = (l_value1 / 10U) % 10;
  l_val8 = (l_value1 / 1U) % 10;

  Lg = String(l_val3);
  Lg1 = String(l_val4);
  Lg2 = String(l_val5);
  Lg3 = String(l_val6);
  Lg4 = String(l_val7);
  Lg5 = String(l_val8);
}

//void double_tostring(){
//
//}

double read_cmaxx() {
  float XX = LRead(25).toFloat();
  cmaxx1 = long(XX);
  //EEPROM.get(25, cmaxx1);
  decimal();
  double cm_1 = cmaxx1 / w;
  double cm_2 = cm_1;
  cm = cm_2;
  //Serial.print(cm,8);
  //Serial.print('\r');
  //cmaxx=1111111; // commented on 2 jan 2022
  gw_set = 0;
  return cm;
}

double read_cminn() {
  float XX = LRead(40).toFloat();
  cminn = long(XX);
  //EEPROM.get(40, cminn);
  decimal();
  double ci_1 = cminn / w;
  double ci_2 = ci_1;
  ci = ci_2;
  //Serial.print(ci);
  //Serial.print('\r');
  //gw7=-1111111;
  gw_set = 0;
  return ci;
}



//  if (b>cm){   //| trigg ==1){
//  //Serial.println ("High");//**Please comment
//  //digitalWrite(TRIG, HIGH);
//  comp =1;
//  return comp;
//  //Serial.println("HIGH");
//  //readbutton();//**please comment
//}
//else if (b<cm){
//  //Serial.println ("Low");
//  comp=0;
//   return comp;
//  //digitalWrite(TRIG, LOW);
//   //readbutton();
//}
//else if (b<ci){
//  //Serial.println ("Low");
//  comp_n=1;
//     return comp_n;
//  //digitalWrite(TRIG, LOW);
//   //readbutton();
//}
//else if (b>ci){
//  //Serial.println ("Low");
//  comp_n=0;
//  return comp_n;
//  //digitalWrite(TRIG, LOW);
//   //readbutton();
//}


//}


//long Test() {
//GW();
//test = gw7;
//return test;
//}


void Password_check() {
  String P = command1.substring(2, 9);
  String Password = command1.substring(3, 9);

  GW(P);

  if (Password == "900731") {
    P_chk = 1;
  } else {
    P_chk = 0;
    Duplex_Check();
    Error_p();
  }
}

long FZ() {
  Password_check();
  //Serial.println(P_chk);
  if (P_chk == 1) {
    fctry_zero = filtdata;
    //EEPROM.put(addr,c);
    // EEPROM.put(1, c);
    Duplex_Check();
    Done_p();
    FZ_set = 1;
    return fctry_zero;
  } else {
    return 0;
  }
}

long FG() {
  Password_check();
  if (P_chk == 1) {
    fctry_gain = filtdata;
    //EEPROM.put(2, d);
    Duplex_Check();
    Done_p();
    FG_set = 1;
    return fctry_gain;
  } else {
    return 0;
  }
}

double FS() {
  //long k;
  //e = 2;
  Password_check();
  if (P_chk == 1) {
    bool FZ_b, FG_b, FV_b = 0;
    if (FZ_set == 1) {
      //EEPROM.put(50, fctry_zero);
      LWrite(50, String(fctry_zero));
      FZ_set = 0;
      FZ_b = 1;
      gravity_val = 1;  ////NEW()
      //EEPROM.put(65, gravity_val);  ////NEW()
      LWrite(65, String(gravity_val));
      //return 0;
    }
    if (FG_set == 1) {
      fctry_diff = fctry_gain - fctry_zero;
      //EEPROM.put(45, fctry_diff);
      LWrite(45, String(fctry_diff));
      FG_set = 0;
      FG_b = 1;
      //return 0;
    }
    if (FV_set == 1) {
      double fte = fctry_diff;
      //Serial.println(ste);
      //Serial.println(ag);
      //cal_gain = gw7 / fte; // assign value to gw7
      cal_gain = fv / fte;
      FV_set = 0;
      FV_b = 1;
      //return 0;
    }

    //Serial.println(d);
    //Serial.println(c);
    //Serial.println(f);

    //Save zero cal count
    if (DP_sa == 1) {
      //EEPROM.put(230, D);
      LWrite(230, String(D));
      //k = EEPROM.get(240);
      //Serial.println(k,10);
      //g = 2 / f;
      DP_sa = 0;
      //return 0;
    }
    // assign value to gw7
    //Serial.println(g);
    if ((FZ_b == 1) & (FG_b == 1) & (FV_b == 1)) {
      Duplex_Check();
      Done_p();
      set_fcal();
      return cal_gain;
    }
    Duplex_Check();
    Done_p();
    return 0;
  }
}


double FD() {
  long def_max = 999999;
  long def_min = -999999;
  if (Access == 1) {
    //D = readIntFromEEPROM(230); // Factory Decimal place
    int ST_C = 1;
    //EEPROM.put(70, 0);
    LWrite(70, String(0));
    //EEPROM.put(80, 0);
    LWrite(80, String(0));
    //EEPROM.put(35, ST_C);  //Step count  DS 1 
    LWrite(35, String(ST_C));  
    //EEPROM.put(130, 0);
    LWrite(130, String(0));
    //EEPROM.put(140, 0);
    LWrite(140, String(0));
    //EEPROM.put(210, 0);
    LWrite(210, String(0));
    //EEPROM.put(220, 0);
    LWrite(220, String(0));

//////////////////////////
    LWrite(60, fd_fl);
    //delay(10);
    LWrite(110, fd_nr);
    //delay(10);
    LWrite(120, fd_nt);
    //delay(10);
    LWrite(75, fd_um);
    //delay(10);
    LWrite(5, fd_dp);
//////////////////////////



    //EEPROM.commit();
    //delay(300);
    read_factoryD();
    //EEPROM.put(7, D);
    LWrite(7, String(D));
    gv = 1;
    //EEPROM.put(65, gv);
    LWrite(65, String(gv));

    float XX = LRead(55).toFloat();
    long fctry_val = long(XX);
    //EEPROM.get(55, fctry_val);
    decimal();
    double fv_1 = fctry_val / w;
    double fv_2 = fv_1;
    fv = fv_2;
    //EEPROM.get(45, fctry_diff);
    float XR = LRead(45).toFloat();
    fctry_diff = long(XR);

    //EEPROM.get(50, fctry_zero);  //Save zero cal count
    float XT = LRead(50).toFloat();
    fctry_zero = long(XT);

    double fte = fctry_diff;

    cal_gain = fv / fte;  // assign value to gw7
                          //EEPROM.put(25, def_max);
    LWrite(25, String(def_max));
    //EEPROM.put(40, def_min);
    LWrite(40, String(def_min));
    Set_N();
    read_cmaxx();
    read_cminn();
    read_UOM();
    Duplex_Check();
    Done_p();
    set_fcal();



    int NvalueFD = CE_value + 1;
    //Serial.println(Nvalue);
    //EEPROM.put(100, Nvalue);
    LWrite(109, String(0)); // MC ZERO
    LWrite(100, String(NvalueFD));
    resetFunc();
    Access = 0;
    return cal_gain, gv;
  } else {
    Duplex_Check();
    Error_p();
    return 0;
  }
}
/*
///////////////////////////FACTORY DEFAULT SETTINGS///////////////////////////////
double FD() {

  if (Access == 1) {
    LWrite(60, fd_fl);
    //delay(10);
    LWrite(110, fd_nr);
    //delay(10);
    LWrite(120, fd_nt);
    //delay(10);
    LWrite(75, fd_um);
    //delay(10);
    LWrite(5, fd_dp);
    //delay(10);
    LWrite(35, fd_ds);
    //delay(10);
    LWrite(130, fd_tn);
    //delay(10);
    LWrite(140, fd_zn);
    //delay(10);
    LWrite(40, fd_ci);
    //delay(10);
    LWrite(25, fd_cm);
    //delay(10);
    LWrite(70, fd_zi);
    //delay(10);
    Duplex_Check();
    Done_p();
    Access = 0;
    resetFunc();
    
  } else {
    Duplex_Check();
    Error_p();
    return 0;
  }
}


/////////////////////////////////////////////////////////////////////////////////

*/




int read_factoryD() {
  //EEPROM.get(230, D);  // Factory Decimal place \D int
  D = LRead(230).toInt();
  //Serial.print(D);
  return D;
}

double FV() {
  GW(command2);
  if (FP_C == 1) {                   // FP0 123456 reqired
    if (gw_set == 0 & E_bit == 0) {  //& gw_set==0 & E_bit==0
      long fctry_val = gw7;
      //EEPROM.put(55, fctry_val);
      LWrite(55, String(fctry_val));
      decimal();
      double fv_1 = fctry_val / w;
      double fv_2 = fv_1;
      fv = fv_2;
      Duplex_Check();
      Ok_p();
      FV_set = 1;
      FP_C = 0;
      return fv;
    } else {
      Duplex_Check();
      Error_p();
      return 0;
    }
  }
  //Serial.println(fv);
  else if (gw_set == 1 & E_bit == 0) {
    // EEPROM.get(55, fctry_val1);
    float XX = LRead(55).toFloat();
    fctry_val1 = long(XX);
    long_tostring(fctry_val1);
    Duplex_Check();
    Serial.print("FV" + l_sign + Lg + Lg1 + Lg2 + Lg3 + Lg4 + Lg5 + '\r');
    gw_set = 0;

  } else {
    Duplex_Check();
    Error_p();
  }
  return 0;
}

double read_fv() {
  //EEPROM.get(55, fctry_val1);
  float XX = LRead(55).toFloat();
  fctry_val1 = long(XX);
  long_tostring(fctry_val1);
  decimal();
  double fv_1 = fctry_val1 / w;
  double fv_2 = fv_1;
  fv = fv_2;
  //Serial.print(cm,8);
  //Serial.print('\r');
  //cmaxx=1111111; // Check EEPROM
  gw_set = 0;
  return fv;
}
double read_cal_gain() {
  // EEPROM.get(45, fctry_diff);
  float XX = LRead(45).toFloat();
  fctry_diff = long(XX);
  double fte = fctry_diff;
  read_fv();
  cal_gain = fv / fte;  // assign value to gw7
  //Serial.print("DONE\r");
  //Serial.println(cal_gain,8);
  return cal_gain;
}

long read_factry_zero() {
  // EEPROM.get(50, fctry_zero);
  float XX = LRead(50).toFloat();
  fctry_zero = long(XX);
  //Serial.println(fctry_zero);
  return fctry_zero;
}

int set_fcal() {
  factory_set = 1;
  //EEPROM.put(90, factory_set);
  LWrite(90, String(factory_set));
  secondary_set = 0;
  //EEPROM.put(95, secondary_set);
  LWrite(95, String(secondary_set));
  // Set_N();
  return factory_set;
  return secondary_set;
}

int set_scal() {
  factory_set = 0;
  //EEPROM.put(90, factory_set);
  LWrite(90, String(factory_set));
  secondary_set = 1;
  //EEPROM.put(95, secondary_set);
  LWrite(95, String(secondary_set));
  //Set_kg();
  return factory_set;
  return secondary_set;
}

int read_cal_stat() {
  //EEPROM.get(90, factory_set);
  factory_set = LRead(90).toInt();
  //EEPROM.get(95, secondary_set);
  secondary_set = LRead(95).toInt();
  return factory_set;
  return secondary_set;
}

double cal_Select() {
  if (factory_set == 1 & secondary_set == 0 & matrixOn == 0) {
    CAL_GAIN = cal_gain;
    Gravity_Factor = gv;
    FAC_ZERO = fctry_zero;
    //Serial.println(CAL_GAIN);
    //Serial.println(Gravity_Factor);
    //Serial.println(FAC_ZERO);
    //Set_N();
    return Gravity_Factor;  // check gravity factor
    return CAL_GAIN;
  } else if (factory_set == 0 & secondary_set == 1 & matrixOn == 0) {
    CAL_GAIN = g;
    Gravity_Factor = 1;
    FAC_ZERO = c;
    // Serial.println(CAL_GAIN);
    //Serial.println(Gravity_Factor);
    //Serial.println(FAC_ZERO);
    //Set_kg();
    return Gravity_Factor;
    return CAL_GAIN;
  }
  ///////////////////////////////////////////// MATRIX IN ///////////////////
  else if (matrixOn == 1) {
    CAL_GAIN = matrixGainD;
    Gravity_Factor = 1;
    FAC_ZERO = c;
    // Serial.println(CAL_GAIN);
    //Serial.println(Gravity_Factor);
    //Serial.println(FAC_ZERO);
    //Set_kg();
    return Gravity_Factor;
    return CAL_GAIN;
  }
}

double GV() {
  GW(command2);
  if (Access == 1 & command2 != "") {
    if (gw7 != 111111 & gw_set == 0 & E_bit == 0) {  //& gw_set==0
      double den = 100000;
      gravity_val = gw7;
      GV_sa = 1;
      //EEPROM.put(65, gravity_val);
      //decimal();
      double gravity_vall1 = gravity_val / den;
      double gravity_vall2 = gravity_vall1;
      gv = gravity_vall2;
      //Serial.println(gv);
      //set_fcal();
      Set_kg();
      read_UOM();
      Duplex_Check();
      Ok_p();
      return gv;
    } else if (gw7 == 111111 & gw_set == 0 & E_bit == 0) {
      //Serial.println("GOT it");
      fset = 1;
      GV_sa1 = 1;
      //EEPROM.put(65,fset);
      double fset1 = fset;
      gv = fset1;
      //set_fcal();  /// avialable on rev 29
      //Serial.println(fset1);
      Set_N();
      read_UOM();
      Duplex_Check();
      Ok_p();
      return gv;
    } else {
      Duplex_Check();
      Error_p();
      return 0;
    }
    Access = 0;

  } else if (gw_set == 1 & E_bit == 0) {
    //EEPROM.get(65, gravity_val1);
    float XX = LRead(65).toFloat();
    gravity_val1 = long(XX);
    long_tostring(gravity_val1);
    Duplex_Check();
    Serial.print("GV" + l_sign + Lg + Lg1 + Lg2 + Lg3 + Lg4 + Lg5 + '\r');
    //Serial.println(gv);
    gw_set = 0;
    return 0;
  } else {
    Duplex_Check();
    Error_p();
    return 0;
  }
}

double read_gravity() {
  double den1 = 100000;
  //EEPROM.get(65, gravity_val1);
  float XX = LRead(65).toFloat();
  gravity_val1 = long(XX);
  if (gravity_val1 == 1) {
    gv = gravity_val1;
    //Serial.println(gv);
    return gv;
  } else {
    double gravity_vall1 = gravity_val1 / den1;
    double gravity_vall2 = gravity_vall1;
    gv = gravity_vall2;
    // Serial.println(gv);
    return gv;
  }
}

long ZI() {


  Check_Stability();
  stability_check = 0;
  if (Stability == "STABLE") {
    ZI_O = 1;
    GN();
    long ZI_value_min = -ZI_value;  // Allowble range in ZI
    long ZI_value_max = ZI_value;

    if ((ZI_value_min <= C_ZI) & (C_ZI <= ZI_value_max)) {
      zi = filtdata;
      // return zi;
    } else {
      zi = 0;

      // return zi;
    }
  } else {
    zi = 0;

    //return zi;
  }
  return zi;
}


long SZ() {  //Set Zero function

  Check_Stability();
  stability_check = 0;
  if (Stability == "STABLE") {
    SZ_A = 1;
    GN();



    //long ZR_value;
    //EEPROM.get(150, ZR_value);
    float XX = LRead(150).toFloat();
    long ZR_value = long(XX);
    //long ZR_value_min = ZR_value - ((ZR_value*2)/100);
    // long ZR_value_max = ZR_value + ((ZR_value*2)/100);

    long ZR_value_min = -ZR_value;
    long ZR_value_max = ZR_value;

    //////////////////////////////////////////////////////////////////////////////////////////

    if ((ZR_value_min <= C_SZ) & (C_SZ <= ZR_value_max)) {
      sz = (filtdata - zi - a);
      //   EEPROM.put(addr,sampdata);
      //   b = EEPROM.read(addr);
      //int ZValue = EEPROM.read(140);
      if (ZValue == 1) {

        //EEPROM.put(210, sz);
        LWrite(210, String(sz));
      }
      Duplex_Check();
      Ok_p();
      return sz;
    }
    //  else if(ZR_value == 0){
    //
    //    //int ZValue = EEPROM.read(140);
    //  sz = filtdata;
    //  if(ZValue==1){
    //    EEPROM.put(210, sz);
    //    }
    //
    //  //   EEPROM.put(addr,sampdata);
    //  //   b = EEPROM.read(addr);
    //  Ok_p();
    //    return sz;
    //    }
    else {
      Duplex_Check();
      Error_p();
      return 0;
    }
  } else {
    Duplex_Check();
    Error_p();
    return 0;
  }
}


//////////////////////////FOR AZT


long SZZT() {  //Set Zero function


  if (signalstable == 1) {
    SZ_A = 1;
    GN();



    //long ZR_value;
    //EEPROM.get(150, ZR_value);
    float XX = LRead(150).toFloat();
    long ZR_value = long(XX);
    //long ZR_value_min = ZR_value - ((ZR_value*2)/100);
    // long ZR_value_max = ZR_value + ((ZR_value*2)/100);

    long ZR_value_min = -ZR_value;
    long ZR_value_max = ZR_value;

    //////////////////////////////////////////////////////////////////////////////////////////

    if ((ZR_value_min <= C_SZ) & (C_SZ <= ZR_value_max)) {
      sz = (filtdata - zi - a);
      //   EEPROM.put(addr,sampdata);
      //   b = EEPROM.read(addr);
      //int ZValue = EEPROM.read(140);
      if (ZValue == 1) {

        //EEPROM.put(210, sz);
        LWrite(210, String(sz));
      }
      //Duplex_Check();
      //Ok_p();
      return sz;
    }

    else {
      //Duplex_Check();
      //Error_p();
      return 0;
    }
  } else {
    //Duplex_Check();
    //Error_p();
    return 0;
  }
}

///////////////




long RZ() {  /// Reset Zero
  if (sz != 0) {
    sz = 0;
    Duplex_Check();
    Ok_p();  //Serial.print("Done\r");
    //double tare =g*a;
    //Serial.print(tare,8);
    // int ZValue = EEPROM.read(140);
    if (ZValue == 1) {
      //EEPROM.put(210, sz);
      LWrite(210, String(sz));
      //Serial.println(sz);
    }

    return sz;
  } else {
    Duplex_Check();
    Error_p();
    return 0;
  }
}
void CE() {

  String CE_0 = command2.substring(0, 1);
  //EEPROM.get(100, CE_value);
  String Svalue = LRead(100);
  CE_value = Svalue.toInt();

  //delay(1);
  //String Svalue = String(CE_value);
  if (command2 == "") {

    //Serial.print(Svalue);
    Duplex_Check();
    Serial.print("E+" + Svalue + "\r");

  }

  else if (CE_0 == " ") {

    int CEvalue = command2.toInt();
    //Serial.println(CEvalue);
    //Serial.println(CE_value);
    if (CEvalue == CE_value) {
      Duplex_Check();
      Ok_p();
      Access = 1;

    } else {
      Duplex_Check();
      Error_p();
    }
  } else {
    Duplex_Check();
    Error_p();
  }
}

//unvoid

void ZR() {

  if (Access == 1 & command2 != "") {
    GW(command2);
    // if(Access == 1& command2 != ""){
    // ZR_value = gw7;
    if (ZI_C == 1) {
      ZI_value = gw7;
    }
    if (ZR_C == 1) {
      ZR_value = gw7;
    }
    if (Error != 1 & gw_set != 1) {
      if (0 <= ZR_value <= 999999) {
        //ZR_sa =1;
        if (ZI_C == 1) {
          ZI_C = 0;
          //EEPROM.put(70, ZI_value);
          ZI_sa = 1;
        } else if (ZR_C == 1) {
          ZR_sa = 1;
          ZR_C = 0;
        }
        Duplex_Check();
        Ok_p();
      }

      else {
        ZR_C = 0;
        ZI_C = 0;

        Duplex_Check();
        Error_p();
      }

    } else {
      ZR_C = 0;
      ZI_C = 0;
      Error = 0;
    }
    Access = 0;
  } else if (command2 == "") {
    if (ZI_C == 1) {
      ZI_C = 0;
      //EEPROM.get(70, ZI_value);
      float XX = LRead(70).toFloat();
      ZI_value = long(XX);
      long_tostring(ZI_value);
      Duplex_Check();
      Serial.print("I" + l_sign + Lg + Lg1 + Lg2 + Lg3 + Lg4 + Lg5 + '\r');
    } else if (ZR_C == 1) {
      ZR_C = 0;
      //EEPROM.get(150, ZR_value);
      float XX = LRead(150).toFloat();
      ZR_value = long(XX);
      //String ZR_valueS = String(ZR_value);
      long_tostring(ZR_value);
      Duplex_Check();
      Serial.print("R" + l_sign + Lg + Lg1 + Lg2 + Lg3 + Lg4 + Lg5 + '\r');
    }
  } else {
    Duplex_Check();
    Error_p();
    ZR_C = 0;
    ZI_C = 0;
  }
}


///////////////////////////////////////////////////////////ZT() NEW

void ZTF() {

  if (Access == 1 & command2 != "") {
    GW(command2);
    // if(Access == 1& command2 != ""){
    // ZR_value = gw7;
    if (ZI_C == 1) {
      ZI_value = gw7;
    }
    if (ZR_C == 1) {
      ZTF_value = gw7;
    }
    if (Error != 1 & gw_set != 1) {
      if (0 <= ZTF_value <= 999999) {
        //ZR_sa =1;
        if (ZI_C == 1) {
          ZI_C = 0;
          //EEPROM.put(70, ZI_value);
          ZI_sa = 1;
        } else if (ZR_C == 1) {
          ZR_sa = 1;
          ZR_C = 0;
        }
        Duplex_Check();
        Ok_p();
      }

      else {
        ZR_C = 0;
        ZI_C = 0;

        Duplex_Check();
        Error_p();
      }

    } else {
      ZR_C = 0;
      ZI_C = 0;
      Error = 0;
    }
    Access = 0;
  } else if (command2 == "") {
    if (ZI_C == 1) {
      ZI_C = 0;
      //EEPROM.get(70, ZI_value);
      float XX = LRead(70).toFloat();
      ZI_value = long(XX);
      long_tostring(ZI_value);
      Duplex_Check();
      Serial.print("I" + l_sign + Lg + Lg1 + Lg2 + Lg3 + Lg4 + Lg5 + '\r');
    } else if (ZR_C == 1) {
      ZR_C = 0;
      //EEPROM.get(150, ZTF_value);
      float XX = LRead(85).toFloat();
      ZTF_value = long(XX);
      //String ZTF_valueS = String(ZTF_value);
      long_tostring(ZTF_value);
      Duplex_Check();
      Serial.print("R" + l_sign + Lg + Lg1 + Lg2 + Lg3 + Lg4 + Lg5 + '\r');
    }
  } else {
    Duplex_Check();
    Error_p();
    ZR_C = 0;
    ZI_C = 0;
  }
}

///////////////////////////////////////////////////////////////









void SAMP_Value() {
  //Serial.print("Run");
  long signtess = filtdata;
  String GS_value = String(signtess);
  String sign;

  if (signtess > 0) {  // compare the sign
    sign = '+';
  } else {
    //sign = '-';
  }
  Duplex_Check();
  Serial.print("S" + sign + GS_value + '\r');
}

void SAMP_Value2() {
  //Serial.print("Run");
  long ct = 0, ctSum = 0;
  for (int i = 1; i <= 100; i++) {
    ct = filtdata;
    ctSum += ct;
  }
  ctSum /= 100;
  //long signtess = filtdata;
  String GS_value = String(ctSum);
  String sign;

  if (ctSum > 0) {  // compare the sign
    sign = '+';
  } else {
    sign = '-';
  }
  Duplex_Check();
  Serial.print("S" + sign + GS_value + '\r');
}

void TT() {

  String P = command1.substring(2, 9);
  String Password = command1.substring(3, 9);
  GW(P);

  if (Password == "123456") {
    long rs1, rs2, rs3, rs4, rs5, rs6, rs7, rs8;


    String RS = command1.substring(9, 18);
    String RS0 = RS.substring(0, 1);
    String RS1 = RS.substring(1, 2);
    char CH1 = RS[1];
    String RS2 = RS.substring(2, 3);
    char CH2 = RS[2];
    String RS3 = RS.substring(3, 4);
    char CH3 = RS[3];
    String RS4 = RS.substring(4, 5);
    char CH4 = RS[4];
    String RS5 = RS.substring(5, 6);
    char CH5 = RS[5];
    String RS6 = RS.substring(6, 7);
    char CH6 = RS[6];
    String RS7 = RS.substring(7, 8);
    char CH7 = RS[7];
    String RS8 = RS.substring(8, 9);
    char CH8 = RS[8];

    int l = RS.length();

    if (RS0 == " " & l == 9 & (isdigit(CH1 & CH2 & CH3 & CH4 & CH5 & CH6 & CH7 & CH8))) {  // if space & length 7 & only digits
      Duplex_Check();
      Done_p();

      rs1 = (RS1.toInt()) * (10000000);
      rs2 = (RS2.toInt()) * (1000000);
      rs3 = (RS3.toInt()) * (100000);
      rs4 = (RS4.toInt()) * (10000);
      rs5 = (RS5.toInt()) * (1000);
      rs6 = (RS6.toInt()) * (100);
      rs7 = (RS7.toInt()) * (10);
      rs8 = (RS8.toInt()) * (1);

      long SN = (rs1 + rs2 + rs3 + rs4 + rs5 + rs6 + rs7 + rs8);  // A assigned for ag


      //EEPROM.put(200, SN);
      LWrite(200, String(SN));

    } else {
      Duplex_Check();
      Error_p();
    }
  }


  else if (command2 == "") {  // DP without space
    //D = 5;
    //long S_value;
    //EEPROM.get(200, S_value);
    String SerialNumber = LRead(200);
    Duplex_Check();
    Serial.print("S:" + SerialNumber + "\r");
    //Serial.print('\r');
  } else {
    Duplex_Check();
    Error_p();
  }
}

void NR() {


  String NRvalue = LRead(110);
  //String NRvalue = String(NR_value);
  if (command2 == "") {
    Duplex_Check();
    Serial.print("NR " + NRvalue + "\r");

  }

  else if (command2 != "") {

    GW(command2);
    NR_value = gw7;
    if (Error != 1) {
      if (0 <= NR_value <= 65535) {
        //EEPROM.put(110, NR_value);
        //LWrite(110, String(NR_value));
        NR_val_en = 1;
        NR_val = NR_value;
        Duplex_Check();
        Ok_p();


      } else {
        Duplex_Check();
        Error_p();
      }
    } else {

      Error = 0;
    }

  } else {
    Duplex_Check();
    Error_p();
  }
}


void NT() {


  //EEPROM.get(120, NT_value);

  String NTvalue = LRead(120);
  if (command2 == "") {
    Duplex_Check();
    Serial.print("NT " + NTvalue + "\r");


  }

  else if (command2 != "") {

    GW(command2);
    NT_value = gw7;
    if (Error != 1) {
      if (0 <= NT_value <= 65535) {
        //EEPROM.put(120, NT_value);
        //LWrite(120, String(NT_value));
        NT_val_en = 1;
        NT_val = NT_value;
        Duplex_Check();
        Ok_p();


      } else {
        Duplex_Check();
        Error_p();
      }

    } else {
      Error = 0;
    }


  } else {
    Duplex_Check();
    Error_p();
  }
}

String Check_Stability() {
  //Serial.println("DDDDD");
  //EEPROM.get(110, NR_value);
  /*float XX = LRead(110).toFloat();
  NR_value = long(XX);

  //EEPROM.get(120, NT_value);
  float XU = LRead(120).toFloat();
  NT_value = long(XU);*/

  //delay(1);
  //Serial.println(NT_value);
  //Serial.println(NR_value);
  stability_check = 1;
  GN();
  delay(NT_value);
  long First_V = G_value;
  GN();
  long Second_V = G_value;
  long D_GG = (Second_V - First_V);
  long R_value = abs(D_GG);
  if (R_value < NR_value) {
    Stability = "STABLE";
    return Stability;
  } else {
    Stability = "UNSTABLE";
    return Stability;
  }
  return "0";
}


int Signal_stability() {


  stability_check = 1;
  GN();
  long First_VSS = G_value;
  delay(NT_value);
  
  GN();
  long Second_VSS = G_value;
  long D_GGSS = (Second_VSS - First_VSS);
  long R_valueSS = abs(D_GGSS);
  if (R_valueSS < NR_value) {
    return 1;
  } else {
    return 0;
  }
  
}








void TN() {
  if (command2 != "") {
    if (Access == 1) {
      String NV_Tare = command1.substring(2, 4);
      String NV_Tare0 = NV_Tare.substring(0, 1);
      char CH1 = NV_Tare[1];
      int l = command1.length();

      if (NV_Tare0 == " " & l == 4) {
        if ((CH1 == '0') | (CH1 == '1')) {
          int Tare_S = CH1 - '0';
          Tare_Val = Tare_S;
          //EEPROM.put(130, Tare_S);
          LWrite(130, String(Tare_S));
          Duplex_Check();
          Ok_p();
        } else {
          Duplex_Check();
          Error_p();
        }
      } else {
        Duplex_Check();
        Error_p();
      }
      Access = 0;
    } else {
      Duplex_Check();
      Error_p();
    }
  } else if (command2 == "") {
    int TValue = LRead(130).toInt();
    String Tare_ST = String(TValue);
    Duplex_Check();
    Serial.print("TN " + Tare_ST + "\r");
  } else {
    Duplex_Check();
    Error_p();
  }
}


void ZN() {
  if (command2 != "") {
    if (Access == 1) {
      String ZN_Zero = command1.substring(2, 4);
      String ZN_Zero0 = ZN_Zero.substring(0, 1);
      char CH1 = ZN_Zero[1];
      int l = command1.length();

      if (ZN_Zero0 == " " & l == 4) {
        if ((CH1 == '0') | (CH1 == '1')) {
          int Zero_S = CH1 - '0';

          ZValue = Zero_S;
          //EEPROM.put(140, Zero_S);
          LWrite(140, String(Zero_S));
          Duplex_Check();
          Ok_p();
        } else {
          Duplex_Check();
          Error_p();
        }
      } else {
        Duplex_Check();
        Error_p();
      }
      Access = 0;
    } else {
      Duplex_Check();
      Error_p();
    }
  } else if (command2 == "") {
    int ZValue = LRead(140).toInt();
    String Zero_ST = String(ZValue);
    Duplex_Check();
    Serial.print("ZN " + Zero_ST + "\r");
  } else {
    Duplex_Check();
    Error_p();
  }
}
//signed long lastDebounceTime = 0;  // the last time the output pin was toggled
//unsigned long debounceDelay = 50; // the debounce time; increase if the output flickers
//int reading =digitalRead(PUSH);
//Serial.println (reading);
// If the switch changed, due to noise or pressing:
// if (reading != lastButtonState) {
//      // reset the debouncing timer
//    lastDebounceTime = millis();
//    Serial.println ("reset");
//  }
//if ((millis() - lastDebounceTime) > debounceDelay) {
//    // whatever the reading is at, it's been there for longer than the debounce
//    // delay, so take it as the actual current state:
//    // if the button state has changed:
//    if (reading != buttonState) {
//      buttonState = reading;
//      Serial.println("in");
//  // only toggle the LED if the new button state is HIGH
//     if (buttonState == HIGH) {
//        //ledState = !ledState;
//        Serial.println("button press");
//        trigg =0;
//        Serial.println("0");
//        return trigg;
//      }
//    }
//}
//
//  // set the LED:
//  //digitalWrite(ledPin, ledState);
// return trigg;
//  // save the reading. Next time through the loop, it'll be the lastButtonState:
//  lastButtonState = reading;
//}

int int_GW() {
  //int int_GW;
  int_GW_Error = 0;
  String int_gw0 = command2;
  String int_gw1 = int_gw0.substring(0, 1);  // Space

  String int_gw2 = int_gw0.substring(1, 2);  //1St character
  int int_gw2_2 = int_gw2.toInt();
  char chint_gw2 = int_gw0[1];

  String int_gw3 = int_gw0.substring(2, 3);  //2nd character
  int int_gw2_3 = int_gw3.toInt();
  char chint_gw3 = int_gw0[2];

  String int_gw4 = int_gw0.substring(3, 4);  //3rd character
  int int_gw2_4 = int_gw4.toInt();
  char chint_gw4 = int_gw0[3];

  inti_GW = int_gw0.toInt();
  String value_p = String(inti_GW);
  int lint_gw0 = int_gw0.length();

  if (int_gw1 == " " & int_gw2_2 <= 5 & int_gw2_3 == 0 & int_gw2_4 == 0 & lint_gw0 == 4) {  // & (isdigit(chint_gw2& chint_gw3 & chint_gw4))){

    //Serial.print(value_p+'\r');
    return inti_GW;
  } else if (int_gw1 == " " & int_gw2_2 <= 9 & int_gw2_3 <= 9 & int_gw4 == "" & lint_gw0 == 3) {  // & (isdigit(chint_gw2& chint_gw3 & chint_gw4))){

    // Serial.print(value_p+'\r');
    return inti_GW;
  } else if (int_gw1 == " " & int_gw2_2 <= 5 & int_gw3 == "" & int_gw4 == "" & lint_gw0 == 2) {

    //Serial.print(value_p+'\r');
    return inti_GW;
  } else if (int_gw0 == "") {
    int_GW_set = 1;
    //Serial.print("OK SET\r");
    return int_GW_set;

  } else {
    int_GW_Error = 1;
    //Serial.print("ERROR\r");
    return int_GW_Error;
  }
}

int UV() {
  //String V = command2;
  // Serial.print(V);
  //ud0 = V.toInt();
  // need to complete.
  //UV_sa=1;
  //
  //Serial.print(a);
  int_GW();
  if (Access == 1 & command2 != "") {
    if (inti_GW == 1 | inti_GW == 2 | inti_GW == 5 | inti_GW == 10 | inti_GW == 20 | inti_GW == 50 | inti_GW == 100 | inti_GW == 200 | inti_GW == 500 & int_GW_Error == 0 & int_GW_set == 0) {
      ud0 = inti_GW;
      UV_sa = 1;
      //EEPROM.put(35, ud0);
      Duplex_Check();
      Ok_p();
      return ud0;
    } else {
      Duplex_Check();
      Error_p();
      return 0;
    }
    Access = 0;
  } else if (int_GW_set == 1 & int_GW_Error == 0) {
    int a;
    // EEPROM.get(35, a);
    String uv_val1 = LRead(35);
    Duplex_Check();
    Serial.print("DS " + uv_val1 + '\r');
    int_GW_set = 0;
    return 0;
  } else {
    Duplex_Check();
    Error_p();
    return 0;
  }
}

int UV_check() {
  int cu = LRead(35).toInt();
  //EEPROM.get(35, cu);


  if (cu == 1 | cu == 2 | cu == 5 | cu == 10 | cu == 20 | cu == 50 | cu == 100 | cu == 200 | cu == 500) {
    ud0 = cu;

    return ud0;
  } else {
    ud0 = 1;
    //EEPROM.put(35, ud0);
    LWrite(35, String(ud0));
    return ud0;
  }
  //Serial.print(ud0);
}

int FL_Val() {


  //Serial.println(read_fl_val);
  //String filter=command2;
  //String filter1=filter.substring(0,1);
  //String filter2=filter.substring(1,2);
  //char chfl2 = filter[1];
  //
  //int lfl =filter.length();
  //int fl0 = filter.toInt();
  //int_GW();



  String FLCONTset = command2.substring(0, 1);  // Space
  String FLCONTset2 = command2.substring(1, 3);
  String FLCONTset3 = command2.substring(0, 2);
  int FLCONT = command2.substring(1, 4).toInt();


  //Serial.print("FLCONTset ");
  //Serial.println(FLCONTset);
  //Serial.print("FLCONTset2 ");
  //Serial.println(FLCONTset2);
  //Serial.print("FLCONT ");
  //Serial.println(FLCONT);



  if ((FLCONT <= 7) && (FLCONTset == " ") && (FLCONTset2 != " ")) {
    //Serial.print("IN ");
    if (FLCONTset2 == "0") {  //(fl0  ==0 & filter1==" " & lfl==2 & (isdigit( chfl2)) ){
      fil_val = 2;
      //EEPROM.put(60, fil_val);
      //LWrite(60, String(fil_val));
      FL_val_en = 1;
      FL_valwp = fil_val;
      //Serial.print(fil_val);
      Duplex_Check();
      Ok_p();
      return fil_val;
    } else if (FLCONTset2 == "1") {  //(fl0  ==1 & filter1==" " & lfl==2 & (isdigit( chfl2)) ){
      fil_val = 66;
      // EEPROM.put(60, fil_val);
      //LWrite(60, String(fil_val));
      FL_val_en = 1;
      FL_valwp = fil_val;
      //Serial.print(fil_val);
      Duplex_Check();
      Ok_p();
      return fil_val;
    } else if (FLCONTset2 == "2") {  //(fl0  ==2 & filter1==" " & lfl==2 & (isdigit( chfl2)) ){
      //Serial.print("IN 2");
      fil_val = 67;
      // EEPROM.put(60, fil_val);
      //LWrite(60, String(fil_val));
      FL_val_en = 1;
      FL_valwp = fil_val;      
      //Serial.print(fil_val);
      Duplex_Check();
      Ok_p();
      return fil_val;
    } else if (FLCONTset2 == "3") {  //(fl0  ==3 & filter1==" " & lfl==2 & (isdigit( chfl2)) ){
      fil_val = 68;
      // EEPROM.put(60, fil_val);
      //LWrite(60, String(fil_val));
      FL_val_en = 1;
      FL_valwp = fil_val;      
      //Serial.print(fil_val);
      Duplex_Check();
      Ok_p();
      return fil_val;
    }
    ////////////////////////////////////////////////////////////////////////////NEW FILTER 5


    /*////////////////////////////////////////////////////////////////////////////NEW FILTER 6    FL5
    else if (FLCONTset2 == "5") {  //(fl0  ==3 & filter1==" " & lfl==2 & (isdigit( chfl2)) ){
      fil_val = 66;
      // EEPROM.put(60, fil_val);
      LWrite(60, String(fil_val));
      //Serial.print(fil_val);
      Duplex_Check();
      Ok_p();
      return fil_val;
    }


    ////////////////////////////////////////////////////////////////////////////NEW FILTER 7      FL6
    else if (FLCONTset2 == "6") {  //(fl0  ==3 & filter1==" " & lfl==2 & (isdigit( chfl2)) ){
      fil_val = 67;
      // EEPROM.put(60, fil_val);
      LWrite(60, String(fil_val));
      //Serial.print(fil_val);
      Duplex_Check();
      Ok_p();
      return fil_val;
    }

    ////////////////////////////////////////////////////////////////////////////NEW FILTER 8      FL7
    else if (FLCONTset2 == "7") {  //(fl0  ==3 & filter1==" " & lfl==2 & (isdigit( chfl2)) ){
      fil_val = 68;
      // EEPROM.put(60, fil_val);
      LWrite(60, String(fil_val));
      //Serial.print(fil_val);
      Duplex_Check();
      Ok_p();
      return fil_val;
    }

    ////////////////////////////////////////////////////////////////////////////NEW FILTER 9      FL8
    else if (FLCONTset2 == "8") {  //(fl0  ==3 & filter1==" " & lfl==2 & (isdigit( chfl2)) ){
      fil_val = 69;
      // EEPROM.put(60, fil_val);
      LWrite(60, String(fil_val));
      //Serial.print(fil_val);
      Duplex_Check();
      Ok_p();
      return fil_val;
    }

    ////////////////////////////////////////////////////////////////////////////NEW FILTER 10      FL9
    else if (FLCONTset2 == "9") {  //(fl0  ==3 & filter1==" " & lfl==2 & (isdigit( chfl2)) ){
      fil_val = 70;
      // EEPROM.put(60, fil_val);
      LWrite(60, String(fil_val));
      //Serial.print(fil_val);
      Duplex_Check();
      Ok_p();
      return fil_val;
    }

    ////////////////////////////////////////////////////////////////////////////NEW FILTER 11     FL10
    else if (FLCONTset2 == "10") {  //(fl0  ==3 & filter1==" " & lfl==2 & (isdigit( chfl2)) ){
      fil_val = 71;
      // EEPROM.put(60, fil_val);
      LWrite(60, String(fil_val));
      //Serial.print(fil_val);
      Duplex_Check();
      Ok_p();
      return fil_val;
    }

    ////////////////////////////////////////////////////////////////////////////NEW FILTER 12     FL11
    else if (FLCONTset2 == "11") {  //(fl0  ==3 & filter1==" " & lfl==2 & (isdigit( chfl2)) ){
      fil_val = 72;
      // EEPROM.put(60, fil_val);
      LWrite(60, String(fil_val));
      //Serial.print(fil_val);
      Duplex_Check();
      Ok_p();
      return fil_val;
    }

    ////////////////////////////////////////////////////////////////////////////NEW FILTER 13      FL12
    else if (FLCONTset2 == "12") {  //(fl0  ==3 & filter1==" " & lfl==2 & (isdigit( chfl2)) ){
      fil_val = 73;
      // EEPROM.put(60, fil_val);
      LWrite(60, String(fil_val));
      //Serial.print(fil_val);
      Duplex_Check();
      Ok_p();
      return fil_val;
    }

    ////////////////////////////////////////////////////////////////////////////NEW FILTER 14     FL13
    else if (FLCONTset2 == "13") {  //(fl0  ==3 & filter1==" " & lfl==2 & (isdigit( chfl2)) ){
      fil_val = 74;
      // EEPROM.put(60, fil_val);
      LWrite(60, String(fil_val));
      //Serial.print(fil_val);
      Duplex_Check();
      Ok_p();
      return fil_val;
    }*/


    else {
      Duplex_Check();
      Error_p();
      return 0;
    }
  }else if ((FLPASS == "900731" )&&(FLNUM == "4")) {  //(fl0  ==3 & filter1==" " & lfl==2 & (isdigit( chfl2)) ){
      fil_val = 50;
      // EEPROM.put(60, fil_val);
      //LWrite(60, String(fil_val));
      FL_val_en = 1;
      FL_valwp = fil_val;
      //Serial.print(fil_val);
      Duplex_Check();
      Ok_p();
      return fil_val;
  }


  else if (FLCONTset3 == "") {
    //Serial.println("in Read");
    //int read_fl_val;
    //Serial.print("OUT ");
    String read_fl_val = LRead(60);
    //EEPROM.get(60, read_fl_val);

    if (read_fl_val == "2") {
      //String fl_val1 = read_fl_val;
      Duplex_Check();
      Serial.print("FL 0\r");
    } else if (read_fl_val == "66") {
      //String fl_val1 = read_fl_val;
      Duplex_Check();
      Serial.print("FL 1\r");
    } else if (read_fl_val == "67") {
      //String fl_val1 = read_fl_val;
      Duplex_Check();
      Serial.print("FL 2\r");
    } else if (read_fl_val == "68") {
      //String fl_val1 = read_fl_val;
      Duplex_Check();
      Serial.print("FL 3\r");
    }
    else if (read_fl_val == "50") {
      //String fl_val1 = read_fl_val;
      Duplex_Check();
      Serial.print("FL 4\r");
    }
    /*
    //////////////////////////////////////////////////////////////////////NEW FILTER 5  FL4
    else if (read_fl_val == "50") {
      //String fl_val1 = read_fl_val;
      Duplex_Check();
      Serial.print("FL 4\r");
    }
    //////////////////////////////////////////////////////////////////////NEW FILTER 6  FL5
    else if (read_fl_val == "66") {
      //String fl_val1 = read_fl_val;
      Duplex_Check();
      Serial.print("FL 5\r");
    }
    //////////////////////////////////////////////////////////////////////NEW FILTER 7  FL6
    else if (read_fl_val == "67") {
      //String fl_val1 = read_fl_val;
      Duplex_Check();
      Serial.print("FL 6\r");
    }

    //////////////////////////////////////////////////////////////////////NEW FILTER 7  FL7
    else if (read_fl_val == "68") {
      //String fl_val1 = read_fl_val;
      Duplex_Check();
      Serial.print("FL 7\r");
    }

    //////////////////////////////////////////////////////////////////////NEW FILTER 7  FL8
    else if (read_fl_val == "69") {
      //String fl_val1 = read_fl_val;
      Duplex_Check();
      Serial.print("FL 8\r");
    }

    //////////////////////////////////////////////////////////////////////NEW FILTER 7  FL9
    else if (read_fl_val == "70") {
      //String fl_val1 = read_fl_val;
      Duplex_Check();
      Serial.print("FL 9\r");
    }

    //////////////////////////////////////////////////////////////////////NEW FILTER 7  FL10
    else if (read_fl_val == "71") {
      //String fl_val1 = read_fl_val;
      Duplex_Check();
      Serial.print("FL 10\r");
    }

    //////////////////////////////////////////////////////////////////////NEW FILTER 7  FL11
    else if (read_fl_val == "72") {
      //String fl_val1 = read_fl_val;
      Duplex_Check();
      Serial.print("FL 11\r");
    }

    //////////////////////////////////////////////////////////////////////NEW FILTER 7  FL12
    else if (read_fl_val == "73") {
      //String fl_val1 = read_fl_val;
      Duplex_Check();
      Serial.print("FL 12\r");
    }

    //////////////////////////////////////////////////////////////////////NEW FILTER 7  FL13
    else if (read_fl_val == "74") {
      //String fl_val1 = read_fl_val;
      Duplex_Check();
      Serial.print("FL 13\r");
    }*/

    //int_GW_set = 0;
    return 0;
  } else {
    Duplex_Check();
    Error_p();
    return 0;
  }
}


int FL_Val_check() {
  int A_filter = LRead(60).toInt();
  //int A_filter;
  //EEPROM.get(60, A_filter);
  if (A_filter == 2 | A_filter == 4 | A_filter == 6 | A_filter == 11 | A_filter == 50 | A_filter == 66 | A_filter == 67 | A_filter == 68 | A_filter == 69 | A_filter == 70 | A_filter == 71 | A_filter == 72 | A_filter == 73 | A_filter == 74) {
    fil_val = A_filter;
    return fil_val;
  } else {
    fil_val = 11;
    //EEPROM.put(60, fil_val);
    LWrite(60, String(fil_val));
    return fil_val;
  }
}










void Done_p() {
  Serial.print("DONE\r");
}
void Error_p() {
  Serial.print("ERROR\r");
}
void Ok_p() {
  Serial.print("OK\r");
}

/*  double ZI(){
  GW(command2);
  if(Access == 1){
  if(gw7>=0 & gw7<=999999 & gw_set==0& E_bit==0){
  ZI_val1=gw7;
  decimal();
  ZI_sa =1;
  double ZI_val2 = ZI_val1/w;
  double ZI_val3= ZI_val2;
  ZI_val= ZI_val3;
  //EEPROM.put(70, ZI_val);
  gw7=1111111;
  Ok_p();
  return  ZI_val;
  }
  Access = 0;
  }
  else if(gw_set==1& E_bit==0){
  long ZI_val4 = EEPROM.get(70);
  long_tostring(ZI_val4);
  Serial.print("ZI"+l_sign+Lg+Lg1+Lg2+Lg3+Lg4+Lg5+'\r');
  //Serial.println(gv);
  gw_set=0;
  }
  else{
  Error_p(); // error need to be managed
  }
  }
  double ZI_compairing(){
  if(h<=ZI_val & h>=0){
  //Check_Stability();
  // stability_check =0;
  // if(Stability == "STABLE"){
  zi = h;
  //Serial.println(ZI_val);
  // Ok_p();
   return zi;
    }
  //}
    else{
     zi = ZI_val; // Error_p();
     return zi;
    }
  }

  double ZI_compairing_n(){
  if(b<=ZI_val & b>=0){
  //Check_Stability();
   //stability_check =0;
   //if(Stability == "STABLE"){
  zi = b;
  //Serial.println(ZI_val);
  // Ok_p();
   return zi;
    }
  // }
    else{
     zi = ZI_val; // Error_p();
     return zi;
    }
  }

  double read_ZI(){
  long ZI_val5 = EEPROM.get(70);
  if(ZI_val5<0 & ZI_val5>999999){
  ZI_val5 =0;
  ZI_val1=ZI_val5;
  return ZI_val;
  }
  else{
  ZI_val1=ZI_val5;
    decimal();
    double ZI_val21 = ZI_val1/w;
  double ZI_val31= ZI_val21;
  ZI_val= ZI_val31;
  return ZI_val;
  }
  }*/

int Set_kg() {
  UOM = 1;
  //EEPROM.put(75, UOM);  //kg location
  LWrite(75, String(UOM));
  return UOM;
}
int Set_lb() {
  UOM = 2;
  //EEPROM.put(75, UOM);  //kg location
  LWrite(75, String(UOM));
  return UOM;
}

int Set_N() {
  UOM = 3;
  //EEPROM.put(75, UOM);  //kg location
  LWrite(75, String(UOM));
  return UOM;
}

int read_UOM() {
  //EEPROM.get(75, UOM);
  UOM = LRead(75).toInt();
  //EEPROM.commit();
  //delay(300);
  return UOM;
}

int UM() {
  int_GW();
  if (Access == 1 & command2 != "") {
    if (inti_GW == 1 & int_GW_Error == 0 & int_GW_set == 0) {
      UOM = 1;
      //EEPROM.put(75,UOM);
      UM_sa = 1;
      Duplex_Check();
      Ok_p();
      return UOM;
    } else if (inti_GW == 2 & int_GW_Error == 0 & int_GW_set == 0) {
      UOM = 2;
      UM_sa = 1;
      //EEPROM.put(75,UOM);
      Duplex_Check();
      Ok_p();
      return UOM;
    } else if (inti_GW == 3 & int_GW_Error == 0 & int_GW_set == 0) {
      UOM = 3;
      UM_sa = 1;
      //EEPROM.put(75,UOM);
      Duplex_Check();
      Ok_p();
      return UOM;
    } else {
      Duplex_Check();
      Error_p();
      return 0;
    }
    Access = 0;
  } else if (int_GW_set == 1 & int_GW_Error == 0) {
    //int read_UOM_val;
    int read_UOM_val = LRead(75).toInt();
    // EEPROM.get(75, read_UOM_val);
    //Serial.print( read_UOM_val);
    //Ok_p();
    if (read_UOM_val == 1) {
      Duplex_Check();
      Serial.print("UM kg\r");
    } else if (read_UOM_val == 2) {
      Duplex_Check();
      Serial.print("UM lb\r");
    } else if (read_UOM_val == 3) {
      Duplex_Check();
      Serial.print("UM N\r");
    }
    int_GW_set = 0;
  } else {
    Duplex_Check();
    Error_p();
  }
  return 0;
}


int TM() {
  int_GW();
  if (Access == 1 & command2 != "") {
    if (inti_GW == 0 & int_GW_Error == 0 & int_GW_set == 0) {
      tare_mode = 0;
      TM_sa = 1;
      //EEPROM.put(80,tare_mode);
      Duplex_Check();
      Ok_p();
      return tare_mode;
    } else if (inti_GW == 1 & int_GW_Error == 0 & int_GW_set == 0) {
      tare_mode = 1;
      TM_sa = 1;
      //EEPROM.put(80,tare_mode);
      Duplex_Check();
      Ok_p();
      return tare_mode;
    } else {
      Duplex_Check();
      Error_p();
    }
    Access = 0;
  } else if (int_GW_set == 1 & int_GW_Error == 0) {
    int read_tare_mode_val = LRead(80).toInt();
    //EEPROM.get(80, read_tare_mode_val);
    //Serial.print( read_UOM_val);
    //Ok_p();
    if (read_tare_mode_val == 0) {
      Duplex_Check();
      Serial.print("TM 0\r");
    } else if (read_tare_mode_val == 1) {
      Duplex_Check();
      Serial.print("TM 1\r");
    }
    int_GW_set = 0;
  } else {
    Duplex_Check();
    Error_p();
  }
  return 0;
}
int read_tare_mode() {
  //EEPROM.get(80, tare_mode);
  tare_mode = LRead(80).toInt();
  return tare_mode;
}

//int comp_TM() {
//  if(tare_mode==0 & h>=0 ){
//    T_allow =1;
//    return  T_allow;
//  }
//   else if(tare_mode==0 & h<=0){
//     T_allow =1;
//     return  T_allow;
//   }
//  else if(tare_mode==1 & h>=1){
//     T_allow =1;
//     return  T_allow;
//  }
//   else if (tare_mode==1 & h<=1){
//    T_allow =0;
//    return  T_allow;
//   }
//}

void Set_NV_ST() {
  if (Tare_Val == 1) {
    //EEPROM.put(220, a);
    LWrite(220, String(a));
  }
}

byte zt() {
  int_GW();
  if (Access == 1 & command2 != "") {
    if (inti_GW <= 255 & inti_GW >= 0 & int_GW_Error == 0 & int_GW_set == 0) {
      ZT = inti_GW;
      ZT_sa = 1;
      Duplex_Check();
      Ok_p();
      return ZT;
    } else {
      Duplex_Check();
      Error_p();
    }
    Access = 0;
  } else if (int_GW_set == 1 & int_GW_Error == 0) {
    //byte ZT_val = EEPROM.read(85);
    String ZT_val1;  // = LRead(85);                        ////commented in half
    Duplex_Check();
    Serial.print("ZT " + ZT_val1 + '\r');
    int_GW_set = 0;
  } else {
    Duplex_Check();
    Error_p();
  }
}


byte read_ZT() {  //////Problem
  byte ZT_val2 = LRead(85).toInt();
  ZT = ZT_val2;
  return ZT;
}



float TC() {
  float CalFactot, CountERR;
  if (TCSSGCONT == true) {
    CurrentTempCount = analogRead(TempPin);  //need to correct
    //Serial.println(CurrentTempCount);
  } else {
    for (int i = 0; i < 10; i++) {
      CurrentTempCount = analogRead(TempPin);  //need to correct
      delayMicroseconds(10);
    }
    //Serial.println(CurrentTempCount);
  }


  //CurrentTempCount from get weight.
  //  EEPROM.get(810, LowADC);
  //  EEPROM.get(820, LowTempC);
  //  EEPROM.get(825, HighADC);
  //  EEPROM.get(835, HighTempC);
  //Serial.println(LowADC);
  //Serial.println(CountERR);
  //Serial.println(HighADC);
  //Serial.println(CurrentTempCount);


  // CalFactot = (HighADC - LowADC) / (HighTempC - LowTempC);  //Temp Count for 1C
  CalFactot = (HighADC - LowADC) / (HIGHTCount - LowTCount);  //Temp Count for 1C
  CountERR = CurrentTempCount - CalTempCount;
  TCError = CountERR * CalFactot;
  TCSSGCONT = false;

  // if (CalTempCount > CurrentTempCount) {
  //  signcontrol = true;
  // } else {
  // signcontrol = false;
  ////}
  //Serial.println(CountERR);
  return 0;
}

float mapFloat(float x, float in_min, float in_max, int out_min, int out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}




/*
void ParameterLoad() {

  //EEPROM.put(5, 3);  ////////////////x7            //ok
  LWrite(5, String(3));

  //EEPROM.put(35, 1);  //ok
  LWrite(35, String(1));

  //EEPROM.put(90, 0);  //0                          //ok
  LWrite(90, String(0));

  //EEPROM.put(95, 0);  //0                          //ok
  LWrite(95, String(0));

  //EEPROM.put(100, 0);  //0                         //ok
  LWrite(100, String(0));

  //EEPROM.put(60, 3);  //ok
  LWrite(60, String(3));

  //EEPROM.put(230, 3);  //ok
  LWrite(230, String(3));

  // EEPROM.put(10, 0);  //0                          //ok
  LWrite(10, String(0));

  //EEPROM.put(240, 0);  //0///////////x3            //ok
  LWrite(240, String(0));

  //EEPROM.put(15, 0);  //0                          //ok
  LWrite(15, String(0));

  //EEPROM.put(30, 9600);  //ok
  LWrite(30, String(9600));

  //EEPROM.put(20, 0);  //0                          //ok
  LWrite(20, String(0));

  //EEPROM.put(25, 999999);  //ok
  LWrite(25, String(999999));

  //EEPROM.put(40, -999999);  //ok
  LWrite(40, String(-999999));

  //EEPROM.put(65, 1);  //ok
  LWrite(65, String(1));

  //EEPROM.put(150, 0);  //0                         //ok
  LWrite(150, String(0));

  //EEPROM.put(200, 0);  //0                         //ok
  LWrite(200, String(0));

  //EEPROM.put(110, 100);  //ok
  LWrite(110, String(100));

  //EEPROM.put(120, 10);  //ok
  LWrite(120, String(10));

  //EEPROM.put(45, 0);  //0                          //ok
  LWrite(45, String(0));

  //EEPROM.put(50, 0);  //0                          //ok
  LWrite(50, String(0));

  //EEPROM.put(55, 0);  //0                          //ok
  LWrite(55, String(0));

  //EEPROM.put(210, 0);  //0                         //ok
  LWrite(210, String(0));

  //EEPROM.put(220, 0);  //0                         //ok
  LWrite(220, String(0));

  //EEPROM.put(70, 0);  //0                          //ok
  LWrite(70, String(0));

  //EEPROM.put(85, 0);  //0                          //ok
  LWrite(85, String(0));

  //EEPROM.put(75, 3);  //ok
  LWrite(75, String(3));

  //EEPROM.put(140, 0);  //0                         //ok
  LWrite(140, String(0));

  //EEPROM.put(130, 0);  //0                         //ok
  LWrite(130, String(0));

  //EEPROM.put(80, 0);  //0                          //ok
  LWrite(80, String(0));

  //EEPROM.put(235, 1);  //0  ///////////x0          //OK
  LWrite(235, String(1));

  //EEPROM.put(1, 1);  //OK
  LWrite(1, String(1));
  Duplex_Check();
  Serial.println("DONE\r");
}*/

/////////////////////////////////////////////////////little fs Rev9
void LWrite(int filename, String Lvalue) {
  File file;
  file.close();
  String Sfilename = String(filename);
  String Rfilename = ("/" + Sfilename + ".txt");
  file = LittleFS.open(String(Rfilename), "w");
  if (file) {
    file.print(Lvalue);
  }
  file.close();
  delay(100);
}

String LRead(int filename) {
  String readF;
  String Sfilename = String(filename);
  String Rfilename = ("/" + Sfilename + ".txt");
  File file = LittleFS.open(String(Rfilename));
  while (file.available()) {
    readF = (file.readString());
  }
  file.close();
  return readF;
}


void Duplex() {
  //DX_v = EEPROM.read(1);
  DX_v = LRead(1).toInt();
  if (DX_v == 0) {
    //pinMode(3, OUTPUT);//H
    digitalWrite(RE, LOW);
    digitalWrite(DE, LOW);
    delayMicroseconds(100);
  }
}


void Duplex_Check() {
  // Serial.println(DX_v);
  if (DX_v == 0) {
    digitalWrite(RE, HIGH);
    delayMicroseconds(50);
    digitalWrite(DE, HIGH);
    delayMicroseconds(50);
  }
}

/////////////////////////////////////////dx

void DX() {


  String DX_0 = command2.substring(0, 1);
  //int DX_value = LRead(1);
  //int DX_value = EEPROM.read(1);
  String DXvalue = LRead(1);
  if (command2 == "") {

    Duplex_Check();
    Serial.print("D+" + DXvalue + "\r");
    // delayprint();

  }

  else if (command2 != "") {

    int DXvalue = command2.toInt();
    if ((DXvalue == 0) | (DXvalue == 1)) {
      LWrite(1, String(DXvalue));
      // EEPROM.write(1, DXvalue);
      Duplex_Check();
      Ok_p();

    } else {
      Duplex_Check();
      Error_p();
    }
  } else {
    Duplex_Check();
    Error_p();
  }
}


void AD() {


  String AD_0 = command2.substring(0, 3);
  //int AD_value = EEPROM.read(0);
  int AD_value = LRead(0).toInt();
  String ADvalue = String(AD_value);
  if (command2 == "") {

    Duplex_Check();
    Serial.print("A+" + ADvalue + "\r");
    //delayprint();

  }

  else if (command2 != "") {

    int ADvalue = command2.toInt();
    if (0 <= ADvalue <= 255) {
      //LWrite(0, String(ADvalue));
      AD_val = ADvalue;
      AD_val_en = 1;
      //Address_Read = ADvalue;
      //EEPROM.write(0, ADvalue);
      Duplex_Check();
      Ok_p();

    } else {
      Duplex_Check();
      Error_p();
    }
  } else {
    Duplex_Check();
    Error_p();
  }
}

boolean isInt(float num) {
  return num == (int)num;
}


void resetFunc() {
  ESP.restart();
}

void RS() {

  if (command2 == "") {
    ////EEPROM.get(240, S_number);
    //long S_number;
    String Serial_N = LRead(2);
    Duplex_Check();
    Serial.print("Sn: " + Serial_N + '\r');
  } else if (commandrs.length() == 17) {
      /*Duplex_Check();
      Serial.println(commandrs);
      Serial.println(RSPASS);
      Serial.println(RSNUM);
      Serial.println(RSNUMlen);*/
      if((RSPASS == "123456") && (RSNUMlen == 7)){
        LWrite(2, String(RSNUM));
        Duplex_Check();
        Ok_p();
        
      }else {
        Duplex_Check();
        Serial.print("ERROR\r");
      }


  }else {
    Duplex_Check();
    Serial.print("ERROR\r");
  }
}
////////////////////////////
/*else if (command == "AV") {
        String commandAV = command1.substring(3, 12);
        int commandlAV = commandAV.length();
        if (commandAV != "") {
          if (commandlAV == 8) {
            float alphanew = commandAV.toFloat();
            LWrite(841, String(alphanew,6));
            Duplex_Check();
            Serial.print("OK\r");
          } else {
            Duplex_Check();
            Error_p();
          }
      } else if (commandAV == "") {
        float AVReadD = LRead(841).toFloat();
        Duplex_Check();       
        Serial.print(AVReadD, 8);
        Serial.print("\r");
      }        
      }*/

/////////////////////////////AV
void AV() {

  if (command2 == "") {
      float AVReadD = LRead(841).toFloat();
      Duplex_Check();       
      Serial.print(AVReadD, 8);
      Serial.print("\r");
  } else if (commandav.length() == 18) {
      /*Duplex_Check();
      Serial.println(commandrs);
      Serial.println(AVPASS);
      Serial.println(AVNUM);
      Serial.println(AVNUMlen);*/
      if((AVPASS == "123456") && (AVNUMlen == 7)){
        float alphanew = AVNUM.toFloat();
        LWrite(841, String(alphanew,6));
        Duplex_Check();
        Serial.print("OK\r");       
      }else {
        Duplex_Check();
        Serial.print("ERROR\r");
      }
  }else {
    Duplex_Check();
    Serial.print("ERROR\r");
  }
}


////////////////////////////






void printTime() {
  // Get current time
  unsigned long currentTime = millis();

  // Calculate hours, minutes, and seconds
  unsigned long seconds = currentTime / 1000;
  unsigned long minutes = seconds / 60;
  unsigned long hours = minutes / 60;
  seconds = seconds % 60;
  minutes = minutes % 60;

  // Print time in HH:MM:SS format
  Serial.print(hours);
  Serial.print(":");
  if (minutes < 10) Serial.print("0");
  Serial.print(minutes);
  Serial.print(":");
  if (seconds < 10) Serial.print("0");
  Serial.print(seconds);
}
bool debug = true;

/////////////
// sensing //
/////////////
// which analog pin to connect
#define THERMISTORPIN1 A0
#define THERMISTORPIN2 A1
#define THERMISTORPIN3 A3
#define THERMISTORPIN4 A2 // bug fix due to soldering error
#define THERMISTORPIN5 A4
#define THERMISTORPIN6 A5
#define THERMISTORPIN7 A6
#define THERMISTORPIN8 A7

// resistance at 25 degrees C
#define THERMISTORNOMINAL 100000
// temp. for nominal resistance (almost always 25 C)
#define TEMPERATURENOMINAL 25
// how many samples to take and average, more takes longer
// but is more 'smooth
#define NUMSAMPLES 25
// The beta coefficient of the thermistor (usually 3000-4000)
#define BCOEFFICIENT 3950
// the value of the 'other' resistor
#define SERIESRESISTOR 10000

int samples1[NUMSAMPLES];
int samples2[NUMSAMPLES];
int samples3[NUMSAMPLES];
int samples4[NUMSAMPLES];
int samples5[NUMSAMPLES];
int samples6[NUMSAMPLES];
int samples7[NUMSAMPLES];
int samples8[NUMSAMPLES];

double temp_LH_Peltier;
double temp_LH;
double temp_RH_Peltier;
double temp_RH;
double temp_LF_Peltier;
double temp_LF;
double temp_RF_Peltier;
double temp_RF;

#include <filters.h>
const float cutoff_freq   = 5.0;  //Cutoff frequency in Hz
const float sampling_time = 0.005; //Sampling time in seconds.
IIR::ORDER  order  = IIR::ORDER::OD3; // Order (OD1 to OD4)

// Low-pass filter
Filter f0(cutoff_freq, sampling_time, order);
Filter f1(cutoff_freq, sampling_time, order);
Filter f2(cutoff_freq, sampling_time, order);
Filter f3(cutoff_freq, sampling_time, order);
Filter f4(cutoff_freq, sampling_time, order);
Filter f5(cutoff_freq, sampling_time, order);
Filter f6(cutoff_freq, sampling_time, order);
Filter f7(cutoff_freq, sampling_time, order);

///////////////
// actuation //
///////////////
// left hand
const int LH_inaPin = 25;
const int LH_inbPin = 24;
const int LH_pwmPin = 8;
const int LH_diagaPin = 23;
const int LH_diagbPin = 22;
const int LH_fan = 36;

// right hand
const int RH_inaPin = 29;
const int RH_inbPin = 28;
const int RH_pwmPin = 9;
const int RH_diagaPin = 27;
const int RH_diagbPin = 26;
const int RH_fan = 37;

// left foot
const int LF_inaPin = 33;
const int LF_inbPin = 32;
const int LF_pwmPin = 10;
const int LF_diagaPin = 31;
const int LF_diagbPin = 30;
const int LF_fan = 34;

// right foot
const int RF_inaPin = 4;
const int RF_inbPin = 5;
const int RF_pwmPin = 11;
const int RF_diagaPin = 6;
const int RF_diagbPin = 7;
const int RF_fan = 35;

/////////
// PID //
/////////
#include <AutoPID.h>

double LH_pwm = 0;
double RH_pwm = 0;
double LF_pwm = 0;
double RF_pwm = 0;

double set_temp_LH = 0;
double set_temp_RH = 0;
double set_temp_LF = 0;
double set_temp_RF = 0;

// FEET
// COOLING SETTINGS
#define OUTPUT_MIN_cf -255 // cool
#define OUTPUT_MAX_cf 0 // heat - don't do it
#define KP_cf 200
#define KI_cf 8
#define KD_cf 0
AutoPID PID_RF_c(&temp_RF, &set_temp_RF, &RF_pwm, OUTPUT_MIN_cf, OUTPUT_MAX_cf, KP_cf, KI_cf, KD_cf);
AutoPID PID_LF_c(&temp_LF, &set_temp_LF, &LF_pwm, OUTPUT_MIN_cf, OUTPUT_MAX_cf, KP_cf, KI_cf, KD_cf);
// HEATING SETTINGS
//pid settings and gains
#define OUTPUT_MIN_hf -255 // cool
#define OUTPUT_MAX_hf 255 // heat
#define KP_hf 75
#define KI_hf 3
#define KD_hf 0
AutoPID PID_RF_h(&temp_RF, &set_temp_RF, &RF_pwm, OUTPUT_MIN_hf, OUTPUT_MAX_hf, KP_hf, KI_hf, KD_hf);
AutoPID PID_LF_h(&temp_LF, &set_temp_LF, &LF_pwm, OUTPUT_MIN_hf, OUTPUT_MAX_hf, KP_hf, KI_hf, KD_hf);

// HANDS
// COOLING SETTINGS
#define OUTPUT_MIN_ch -255 // cool
#define OUTPUT_MAX_ch 0 // heat - don't do it
#define KP_ch 30
#define KI_ch 0
#define KD_ch 1
AutoPID PID_RH_c(&temp_RH, &set_temp_RH, &RH_pwm, OUTPUT_MIN_ch, OUTPUT_MAX_ch, KP_ch, KI_ch, KD_ch);
AutoPID PID_LH_c(&temp_LH, &set_temp_LH, &LH_pwm, OUTPUT_MIN_ch, OUTPUT_MAX_ch, KP_ch, KI_ch, KD_ch);
// HEATING SETTINGS
//pid settings and gains
#define OUTPUT_MIN_hh -10 // cool
#define OUTPUT_MAX_hh 255 // heat
#define KP_hh 15
#define KI_hh 0.1
#define KD_hh 0.5
AutoPID PID_RH_h(&temp_RH, &set_temp_RH, &RH_pwm, OUTPUT_MIN_hh, OUTPUT_MAX_hh, KP_hh, KI_hh, KD_hh);
AutoPID PID_LH_h(&temp_LH, &set_temp_LH, &LH_pwm, OUTPUT_MIN_hh, OUTPUT_MAX_hh, KP_hh, KI_hh, KD_hh);

// Serial comms
const byte numChars = 32;
char receivedChars[numChars];
char tempChars[numChars];        // temporary array for use when parsing
boolean newData = false;
int printCounter = 0;

//============

void setup() {
  Serial.begin(115200);
  analogReference(EXTERNAL);

  // setup left hand
  pinMode(LH_inaPin, OUTPUT);
  pinMode(LH_inbPin, OUTPUT);
  pinMode(LH_pwmPin, OUTPUT);
  pinMode(LH_diagaPin, INPUT);
  pinMode(LH_diagbPin, INPUT);
  pinMode(LH_fan, OUTPUT);

  // setup right hand
  pinMode(RH_inaPin, OUTPUT);
  pinMode(RH_inbPin, OUTPUT);
  pinMode(RH_pwmPin, OUTPUT);
  pinMode(RH_diagaPin, INPUT);
  pinMode(RH_diagbPin, INPUT);
  pinMode(RH_fan, OUTPUT);

  // setup left foot
  pinMode(LF_inaPin, OUTPUT);
  pinMode(LF_inbPin, OUTPUT);
  pinMode(LF_pwmPin, OUTPUT);
  pinMode(LF_diagaPin, INPUT);
  pinMode(LF_diagbPin, INPUT);
  pinMode(LF_fan, OUTPUT);

  // setup right foot
  pinMode(RF_inaPin, OUTPUT);
  pinMode(RF_inbPin, OUTPUT);
  pinMode(RF_pwmPin, OUTPUT);
  pinMode(RF_diagaPin, INPUT);
  pinMode(RF_diagbPin, INPUT);
  pinMode(RF_fan, OUTPUT);

  //if temperature is more than X degrees below or above setpoint, OUTPUT will be set to min or max respectively
  PID_RH_c.setBangBang(20);
  PID_RH_h.setBangBang(10);
  PID_LH_c.setBangBang(20);
  PID_LH_h.setBangBang(10);
  PID_RF_c.setBangBang(1);
  PID_RF_h.setBangBang(2);
  PID_LF_c.setBangBang(1);
  PID_LF_h.setBangBang(2);

  //set PID update interval
  PID_RH_c.setTimeStep(10);
  PID_RH_h.setTimeStep(10);
  PID_LH_c.setTimeStep(10);
  PID_LH_h.setTimeStep(10);
  PID_RF_c.setTimeStep(10);
  PID_RF_h.setTimeStep(10);
  PID_LF_c.setTimeStep(10);
  PID_LF_h.setTimeStep(10);
  //Serial.println("Enter data in this style <70, 0, -120, -255>  ");
}

//============

void loop() {
  recvWithStartEndMarkers();
  if (newData == true) {
    strcpy(tempChars, receivedChars);
    // this temporary copy is necessary to protect the original data
    //   because strtok() used in parseData() replaces the commas with \0
    parseData();
    if(debug == true) {
      showParsedData();
    }
    newData = false;
  }

  // pull the sensor readings
  getSensorData();

  // then heat/cool each hand/foot accordingly
  if (set_temp_LH == 0) { // kill the motor driver
    LH_pwm = 0;
  }
  else if (set_temp_LH <= 30) {
    PID_LH_c.run(); //call every loop, updates automatically at certain time interval
  }
  else if (set_temp_LH > 30)  {
    PID_LH_h.run(); //call every loop, updates automatically at certain time interval
  }

  if (set_temp_RH == 0) { // kill the motor driver
    RH_pwm = 0;
  }
  else if (set_temp_RH <= 30) {
    PID_RH_c.run(); //call every loop, updates automatically at certain time interval
  }
  else if (set_temp_RH > 30)  {
    PID_RH_h.run(); //call every loop, updates automatically at certain time interval
  }

  if (set_temp_LF == 0) { // kill the motor driver
    LF_pwm = 0;
  }
  else if (set_temp_LF <= 28) {
    PID_LF_c.run(); //call every loop, updates automatically at certain time interval
  }
  else if (set_temp_LF > 28)  {
    PID_LF_h.run(); //call every loop, updates automatically at certain time interval
  }

  if (set_temp_RF == 0) { // kill the motor driver
    RF_pwm = 0;
  }
  else if (set_temp_RF <= 28) {
    PID_RF_c.run(); //call every loop, updates automatically at certain time interval
  }
  else if (set_temp_RF > 28)  {
    PID_RF_h.run(); //call every loop, updates automatically at certain time interval
  }

  runThermals();

  printCounter++;
  if (debug == true && printCounter > 50) {
    if (set_temp_RF <= 28) {
      //Serial.print("---RF: Cooling---   ");
    }
    else if (set_temp_RF > 28) {
      //Serial.print("---RF: Heating---   ");
    }
    Serial.print("LH set temp: ");
    Serial.print(set_temp_LH);
    Serial.print(", LH PWM: ");
    Serial.print(LH_pwm);
    Serial.print(", RH set temp: ");
    Serial.print(set_temp_RH);
    Serial.print(", RH PWM: ");
    Serial.print(RH_pwm);
    Serial.print(", LF set temp: ");
    Serial.print(set_temp_LF);
    Serial.print(", LF PWM: ");
    Serial.print(LF_pwm);
    Serial.print(", RF set temp: ");
    Serial.print(set_temp_RF);
    Serial.print(", RF PWM: ");
    Serial.println(RF_pwm);

    Serial.print("temp_LH_Peltier: ");
    Serial.print(temp_LH_Peltier);
    Serial.print(", temp_LH: ");
    Serial.print(temp_LH);
    Serial.print(", temp_RH_Peltier: ");
    Serial.print(temp_RH_Peltier);
    Serial.print(", temp_RH: ");
    Serial.print(temp_RH);
    Serial.print(", temp_LF_Peltier: ");
    Serial.print(temp_LF_Peltier);
    Serial.print(", temp_LF: ");
    Serial.print(temp_LF);
    Serial.print(", temp_RF_Peltier: ");
    Serial.print(temp_RF_Peltier);
    Serial.print(", temp_RF: ");
    Serial.println(temp_RF);

    printCounter = 0;
  }
}

//============

void getSensorData() {
  uint8_t i;
  float average1;
  float average2;
  float average3;
  float average4;
  float average5;
  float average6;
  float average7;
  float average8;

  // take N samples in a row
  for (i = 0; i < NUMSAMPLES; i++) {
    samples1[i] = analogRead(THERMISTORPIN1);
    samples2[i] = analogRead(THERMISTORPIN2);
    samples3[i] = analogRead(THERMISTORPIN3);
    samples4[i] = analogRead(THERMISTORPIN4);
    samples5[i] = analogRead(THERMISTORPIN5);
    samples6[i] = analogRead(THERMISTORPIN6);
    samples7[i] = analogRead(THERMISTORPIN7);
    samples8[i] = analogRead(THERMISTORPIN8);
  }

  // average all the samples out
  average1 = 0;
  average2 = 0;
  average3 = 0;
  average4 = 0;
  average5 = 0;
  average6 = 0;
  average7 = 0;
  average8 = 0;
  for (i = 0; i < NUMSAMPLES; i++) {
    average1 += samples1[i];
    average2 += samples2[i];
    average3 += samples3[i];
    average4 += samples4[i];
    average5 += samples5[i];
    average6 += samples6[i];
    average7 += samples7[i];
    average8 += samples8[i];
  }
  average1 /= NUMSAMPLES;
  average2 /= NUMSAMPLES;
  average3 /= NUMSAMPLES;
  average4 /= NUMSAMPLES;
  average5 /= NUMSAMPLES;
  average6 /= NUMSAMPLES;
  average7 /= NUMSAMPLES;
  average8 /= NUMSAMPLES;

  // convert the value to resistance, then to temperature
  average1 = 1023 / average1 - 1;
  average1 = SERIESRESISTOR / average1;
  float steinhart1;
  steinhart1 = average1 / THERMISTORNOMINAL;     // (R/Ro)
  steinhart1 = log(steinhart1);                  // ln(R/Ro)
  steinhart1 /= BCOEFFICIENT;                   // 1/B * ln(R/Ro)
  steinhart1 += 1.0 / (TEMPERATURENOMINAL + 273.15); // + (1/To)
  steinhart1 = 1.0 / steinhart1;                 // Invert
  steinhart1 -= 273.15;                         // convert absolute temp to C

  average2 = 1023 / average2 - 1;
  average2 = SERIESRESISTOR / average2;
  float steinhart2;
  steinhart2 = average2 / THERMISTORNOMINAL;     // (R/Ro)
  steinhart2 = log(steinhart2);                  // ln(R/Ro)
  steinhart2 /= BCOEFFICIENT;                   // 1/B * ln(R/Ro)
  steinhart2 += 1.0 / (TEMPERATURENOMINAL + 273.15); // + (1/To)
  steinhart2 = 1.0 / steinhart2;                 // Invert
  steinhart2 -= 273.15;                         // convert absolute temp to C

  average3 = 1023 / average3 - 1;
  average3 = SERIESRESISTOR / average3;
  float steinhart3;
  steinhart3 = average3 / THERMISTORNOMINAL;     // (R/Ro)
  steinhart3 = log(steinhart3);                  // ln(R/Ro)
  steinhart3 /= BCOEFFICIENT;                   // 1/B * ln(R/Ro)
  steinhart3 += 1.0 / (TEMPERATURENOMINAL + 273.15); // + (1/To)
  steinhart3 = 1.0 / steinhart3;                 // Invert
  steinhart3 -= 273.15;                         // convert absolute temp to C

  average4 = 1023 / average4 - 1;
  average4 = SERIESRESISTOR / average4;
  float steinhart4;
  steinhart4 = average4 / THERMISTORNOMINAL;     // (R/Ro)
  steinhart4 = log(steinhart4);                  // ln(R/Ro)
  steinhart4 /= BCOEFFICIENT;                   // 1/B * ln(R/Ro)
  steinhart4 += 1.0 / (TEMPERATURENOMINAL + 273.15); // + (1/To)
  steinhart4 = 1.0 / steinhart4;                 // Invert
  steinhart4 -= 273.15;                         // convert absolute temp to C

  average5 = 1023 / average5 - 1;
  average5 = SERIESRESISTOR / average5;
  float steinhart5;
  steinhart5 = average5 / THERMISTORNOMINAL;     // (R/Ro)
  steinhart5 = log(steinhart5);                  // ln(R/Ro)
  steinhart5 /= BCOEFFICIENT;                   // 1/B * ln(R/Ro)
  steinhart5 += 1.0 / (TEMPERATURENOMINAL + 273.15); // + (1/To)
  steinhart5 = 1.0 / steinhart5;                 // Invert
  steinhart5 -= 273.15;                         // convert absolute temp to C

  average6 = 1023 / average6 - 1;
  average6 = SERIESRESISTOR / average6;
  float steinhart6;
  steinhart6 = average6 / THERMISTORNOMINAL;     // (R/Ro)
  steinhart6 = log(steinhart6);                  // ln(R/Ro)
  steinhart6 /= BCOEFFICIENT;                   // 1/B * ln(R/Ro)
  steinhart6 += 1.0 / (TEMPERATURENOMINAL + 273.15); // + (1/To)
  steinhart6 = 1.0 / steinhart6;                 // Invert
  steinhart6 -= 273.15;                         // convert absolute temp to C

  average7 = 1023 / average7 - 1;
  average7 = SERIESRESISTOR / average7;
  float steinhart7;
  steinhart7 = average7 / THERMISTORNOMINAL;     // (R/Ro)
  steinhart7 = log(steinhart7);                  // ln(R/Ro)
  steinhart7 /= BCOEFFICIENT;                   // 1/B * ln(R/Ro)
  steinhart7 += 1.0 / (TEMPERATURENOMINAL + 273.15); // + (1/To)
  steinhart7 = 1.0 / steinhart7;                 // Invert
  steinhart7 -= 273.15;                         // convert absolute temp to C

  average8 = 1023 / average8 - 1;
  average8 = SERIESRESISTOR / average8;
  float steinhart8;
  steinhart8 = average8 / THERMISTORNOMINAL;     // (R/Ro)
  steinhart8 = log(steinhart8);                  // ln(R/Ro)
  steinhart8 /= BCOEFFICIENT;                   // 1/B * ln(R/Ro)
  steinhart8 += 1.0 / (TEMPERATURENOMINAL + 273.15); // + (1/To)
  steinhart8 = 1.0 / steinhart8;                 // Invert
  steinhart8 -= 273.15;                         // convert absolute temp to C

  temp_LH_Peltier = f0.filterIn(steinhart1);
  temp_LH = f1.filterIn(steinhart2);
  temp_RH_Peltier = f2.filterIn(steinhart3);
  temp_RH = f3.filterIn(steinhart4);
  temp_LF_Peltier = f4.filterIn(steinhart5);
  temp_LF = f5.filterIn(steinhart6);
  temp_RF_Peltier = f6.filterIn(steinhart7);
  temp_RF = f7.filterIn(steinhart8);
}

//============

void runThermals() {
  // Left hand
  if (LH_pwm >= 0) {
    digitalWrite(LH_inaPin, HIGH);
    digitalWrite(LH_inbPin, LOW);
    analogWrite(LH_pwmPin, LH_pwm);
  }
  else  {
    digitalWrite(LH_inaPin, LOW);
    digitalWrite(LH_inbPin, HIGH);
    analogWrite(LH_pwmPin, abs(LH_pwm));
  }

  if (set_temp_LH == 0) {
    digitalWrite(LH_fan, LOW);
  }
  else if (temp_LH >= set_temp_LH) {
    digitalWrite(LH_fan, HIGH);
  }
  else {
    digitalWrite(LH_fan, LOW);
  }

  // Right hand
  if (RH_pwm >= 0) {
    digitalWrite(RH_inaPin, HIGH);
    digitalWrite(RH_inbPin, LOW);
    analogWrite(RH_pwmPin, RH_pwm);
  }
  else  {
    digitalWrite(RH_inaPin, LOW);
    digitalWrite(RH_inbPin, HIGH);
    analogWrite(RH_pwmPin, abs(RH_pwm));
  }

  if (set_temp_RH == 0) {
    digitalWrite(RH_fan, LOW);
  }
  else if (temp_RH >= set_temp_RH) {
    digitalWrite(RH_fan, HIGH);
  }
  else {
    digitalWrite(RH_fan, LOW);
  }

  // Left foot
  if (LF_pwm >= 0) {
    digitalWrite(LF_inaPin, HIGH);
    digitalWrite(LF_inbPin, LOW);
    analogWrite(LF_pwmPin, LF_pwm);
  }
  else  {
    digitalWrite(LF_inaPin, LOW);
    digitalWrite(LF_inbPin, HIGH);
    analogWrite(LF_pwmPin, abs(LF_pwm));
  }

  if (temp_LF >= set_temp_LF) {
    digitalWrite(LF_fan, HIGH);
  }
  else {
    digitalWrite(LF_fan, LOW);
  }

  // Right foot
  if (RF_pwm >= 0) {
    digitalWrite(RF_inaPin, HIGH);
    digitalWrite(RF_inbPin, LOW);
    analogWrite(RF_pwmPin, RF_pwm);
    digitalWrite(RF_fan, LOW);
  }
  else  {
    digitalWrite(RF_inaPin, LOW);
    digitalWrite(RF_inbPin, HIGH);
    analogWrite(RF_pwmPin, abs(RF_pwm));
    digitalWrite(RF_fan, HIGH);
  }

  if (temp_RF >= set_temp_RF) {
    digitalWrite(RF_fan, HIGH);
  }
  else {
    digitalWrite(RF_fan, LOW);
  }
  
}

//============

void recvWithStartEndMarkers() {
  static boolean recvInProgress = false;
  static byte ndx = 0;
  char startMarker = '<';
  char endMarker = '>';
  char rc;

  while (Serial.available() > 0 && newData == false) {
    rc = Serial.read();

    if (recvInProgress == true) {
      if (rc != endMarker) {
        receivedChars[ndx] = rc;
        ndx++;
        if (ndx >= numChars) {
          ndx = numChars - 1;
        }
      }
      else {
        receivedChars[ndx] = '\0'; // terminate the string
        recvInProgress = false;
        ndx = 0;
        newData = true;
      }
    }

    else if (rc == startMarker) {
      recvInProgress = true;
    }
  }
}

//============

void parseData() {      // split the data into its parts

  char * strtokIndx; // this is used by strtok() as an index

  //strtokIndx = strtok(tempChars,",");      // get the first part - the string
  //strcpy(messageFromPC, strtokIndx); // copy it to messageFromPC

  strtokIndx = strtok(tempChars, ",");     // get the first part - the string
  set_temp_LH = atoi(strtokIndx); // copy it to messageFromPC

  strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
  set_temp_RH = atoi(strtokIndx);     // convert this part to an integer

  strtokIndx = strtok(NULL, ",");     // get the first part - the string
  set_temp_LF = atoi(strtokIndx); // copy it to messageFromPC

  strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
  set_temp_RF = atoi(strtokIndx);     // convert this part to an integer

  //strtokIndx = strtok(NULL, ",");
  //floatFromPC = atof(strtokIndx);     // convert this part to a float

}

//============

void showParsedData() {
  Serial.print("set_temp_LH ");
  Serial.println(set_temp_LH);
  Serial.print("set_temp_RH ");
  Serial.println(set_temp_RH);
  Serial.print("set_temp_LF ");
  Serial.println(set_temp_LF);
  Serial.print("set_temp_RF ");
  Serial.println(set_temp_RF);
}

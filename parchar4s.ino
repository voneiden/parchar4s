#include "parchar4s.h"
// Pins
// 0 AS0, SERIAL
// 1 AS1, CLK
// 2 AS2
// 3 LOAD
// 4/A2 ADC

#define LED_SERIAL 0
#define LED_CLOCK 1
#define LED_LATCH 3
#define MUX_S0 0
#define MUX_S1 1
#define MUX_S2 2
#define ADC_PIN A2
#define ENABLE_DP 0b10000000

// MAX7219 op codes
#define OP_SCANLIMIT 11
#define OP_INTENSITY 10
#define OP_SHUTDOWN 12
#define OP_DISPLAY_TEST 15

#define MUX_GRID_1 0b00000011
#define MUX_GRID_2 0b00000000
#define MUX_GRID_3 0b00000001
#define MUX_GRID_4 0b00000010


#define MUX_TEST_1 0b00000101
#define MUX_TEST_2 0b00000111 // TODO!
#define MUX_TEST_3 0b00000110
#define MUX_TEST_4 0b00000111 // TODO!

const static byte charTable[] = {0b01111110,0b00110000,0b01101101,0b01111001,0b00110011,0b01011011,0b01011111,0b01110000,0b01111111,0b01111011};

void setup() {
  //delay(10); // Wait a bit just in case..
  // Setup digital pins
  pinMode(0, OUTPUT);
  pinMode(1, OUTPUT);
  pinMode(2, OUTPUT);
  pinMode(LED_LATCH, OUTPUT);
  //ledOut(OP_DISPLAY_TEST, 0);
  ledOut(OP_SCANLIMIT, 3); // Use 3 digits
  ledOut(OP_INTENSITY, 0x1); // Max brightness
  ledOut(OP_SHUTDOWN, 1); // Turn on!; 
}
/*
int ADC_read(bool s0, bool s1, bool s2) {
  digitalWrite(MUX_S0, s0);
  digitalWrite(MUX_S1, s1);
  digitalWrite(MUX_S2, s2);
  //delay(10); // TODO verify this delay
  return analogRead(ADC_PIN);
}
*/

int ADC_read(byte address) {
  digitalWrite(MUX_S0, address & 0b00000001);
  digitalWrite(MUX_S0, address & 0b00000010);
  digitalWrite(MUX_S0, address & 0b00000100);
  return analogRead(ADC_PIN);
}

int get_display_ADC_reading() {
  // Check voltage on grid
  // A3, A0, A1, A2
  int gridVoltages[4];
  gridVoltages[0] = ADC_read(MUX_GRID_1);
  gridVoltages[1] = ADC_read(MUX_GRID_2) - gridVoltages[0];
  gridVoltages[2] = ADC_read(MUX_GRID_3) - gridVoltages[1] - gridVoltages[0];
  gridVoltages[3] = ADC_read(MUX_GRID_4) - gridVoltages[2] - gridVoltages[1] - gridVoltages[0];
  
  // Check voltage on test pins
  // A5, A7, A6, A4
  int testVoltages[4];
  testVoltages[0] = ADC_read(MUX_TEST_1);
  testVoltages[1] = ADC_read(MUX_TEST_2) - testVoltages[0];
  testVoltages[2] = ADC_read(MUX_TEST_3) - testVoltages[1] - testVoltages[0];
  testVoltages[3] = ADC_read(MUX_TEST_4) - testVoltages[2] - testVoltages[1] - testVoltages[0];

  
  // Nothing attached
  if (testVoltages[0] < 10 ||
      testVoltages[1] < 10 ||
      testVoltages[2] < 10 ||
      testVoltages[3] < 10) {
        return max(gridVoltages[0], max(gridVoltages[1], max(gridVoltages[2], gridVoltages[3])));
  } else {
    int max_difference = abs(gridVoltages[0] - testVoltages[0]);
    max_difference = max(max_difference, abs(gridVoltages[1] - testVoltages[1]));
    max_difference = max(max_difference, abs(gridVoltages[2] - testVoltages[2]));
    return max(max_difference, abs(gridVoltages[3] - testVoltages[3]));
  }
}
void reading_to_buffer(int adc_reading, byte reading_buffer[4]) {
  char i = 3;
  // Scale the voltage reading according to on-board resistors TODO
  int voltage_reading = adc_reading * 1700 / 1024;
  while (i >= 0) {
    byte digit = voltage_reading % 10;
    //buffer[i] = '0' + digit;
    reading_buffer[i] = digit;
    i--;
    voltage_reading /= 10;
  }
}

void display_max7219(byte reading_buffer[4]) {
  //
  byte bi = 1;
  byte di = 3;
  byte number = 0;
  if (reading_buffer[0]) {
    bi = 2;
  }

  do {
    
    number = charTable[reading_buffer[di+bi]];
    if (bi==di) {
      number |= ENABLE_DP;
    }
    //setDigit(di, number, bi==di);
    ledOut(di, number);
    --di;
  }
  while (di != 1);
  

  //ledOut(1, charTable[number]);
  //setDigit(0, number, false);
  //setDigit(1, buffer[1], false);
  //setDigit(2, buffer[2], false);
}

static void setDigit(byte digit, byte number, boolean dp) {
  byte value = charTable[number];
  if (dp) {
    value |= ENABLE_DP;
  }
  ledOut(digit+1, value);
}
static void ledOut(byte address, byte value) {
  digitalWrite(LED_LATCH, LOW);
  shiftOut(LED_SERIAL, LED_CLOCK, MSBFIRST, address);
  shiftOut(LED_SERIAL, LED_CLOCK, MSBFIRST, value);
  digitalWrite(LED_LATCH, HIGH);
}

void loop() {
  int ADC_reading = get_display_ADC_reading(); // 288 bytes
  //int ADC_reading = analogRead(ADC_PIN);

  byte reading_buffer[4] = {0,0,0,0};
  reading_to_buffer(ADC_reading, reading_buffer); // 758 bytes
  //byte reading_buffer[4] = {(byte) ADC_reading,0,0,0};
  /*
  if (reading_buffer[1] > 2) {
    digitalWrite(LED_LATCH, HIGH);
  } else {
    digitalWrite(LED_LATCH, LOW);
  }*/
  display_max7219(reading_buffer); // 80 bytes
  //setDigit(0, reading_buffer[0], false);
  //delay(500);
}

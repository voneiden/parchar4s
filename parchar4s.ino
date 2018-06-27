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

const static byte charTable[] = {0b01111110,0b00110000,0b01101101,0b01111001,0b00110011,0b01011011,0b01011111,0b01110000,0b01111111,0b01111011};

void setup() {
  //delay(10); // Wait a bit just in case..
  // Setup digital pins
  pinMode(0, OUTPUT);
  pinMode(1, OUTPUT);
  pinMode(2, OUTPUT);
  pinMode(LED_LATCH, OUTPUT);
  digitalWrite(LED_LATCH, HIGH);
  // Setup the MAX7219
  ledOut(OP_DISPLAY_TEST, 0);
  // Use 3 digits
  ledOut(OP_SCANLIMIT, 3);

  // Max brightness
  ledOut(OP_INTENSITY, 0x1);

  // Startup values
  setDigit(0, 6, false);
  setDigit(1, 6, false);
  setDigit(2, 6, false);
  
  // Turn on!
  ledOut(OP_SHUTDOWN, 1);

  
  
}
int ADC_read(bool s0, bool s1, bool s2) {
  digitalWrite(MUX_S0, s0);
  digitalWrite(MUX_S1, s1);
  digitalWrite(MUX_S2, s2);
  //delay(10); // TODO verify this delay
  return analogRead(ADC_PIN);
}

int get_display_ADC_reading() {
  // Check voltage on grid
  // A3, A0, A1, A2
  int gridVoltages[4];
  gridVoltages[0] = ADC_read(0, 1, 1);
  gridVoltages[1] = ADC_read(0, 0, 0) - gridVoltages[0];
  gridVoltages[2] = ADC_read(0, 0, 1) - gridVoltages[1] - gridVoltages[0];
  gridVoltages[3] = ADC_read(0, 1, 0) - gridVoltages[2] - gridVoltages[1] - gridVoltages[0];
  
  // Check voltage on test pins
  // A5, A7, A6, A4
  int testVoltages[4];
  testVoltages[0] = ADC_read(1, 0, 1);
  testVoltages[1] = ADC_read(1, 1, 1) - testVoltages[0];
  testVoltages[2] = ADC_read(1, 1, 0) - testVoltages[1] - testVoltages[0];
  testVoltages[3] = ADC_read(1, 1, 1) - testVoltages[2] - testVoltages[1] - testVoltages[0];

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
  int voltage_reading = adc_reading * 17 / 10.24;
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
  byte bi = 0;
  //byte di = 2;
  //byte number = 0;
  if (reading_buffer[0]) {
    bi = 1;
  }

  /*
  while (di != 255) {
    number = reading_buffer[di+bi];
    setDigit(di, number, bi==di);
    di--;
  }
  */
  byte number = reading_buffer[bi];
  setDigit(0, number, false);
  //setDigit(1, buffer[1], false);
  //setDigit(2, buffer[2], false);
}

void setDigit(byte digit, byte number, boolean dp) {
  byte value = charTable[number];
  if (dp) {
    value |= ENABLE_DP;
  }
  ledOut(digit+1, value);
}
void ledOut(byte address, byte value) {
  digitalWrite(LED_LATCH, LOW);
  shiftOut(LED_SERIAL, LED_CLOCK, MSBFIRST, address);
  shiftOut(LED_SERIAL, LED_CLOCK, MSBFIRST, value);
  digitalWrite(LED_LATCH, HIGH);
}

void loop() {
  int ADC_reading = get_display_ADC_reading();
  byte reading_buffer[4] = {0,0,0,0};
  reading_to_buffer(ADC_reading, reading_buffer);
  display_max7219(reading_buffer);
  //delay(500);
}

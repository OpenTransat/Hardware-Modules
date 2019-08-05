/*
Sailwing Position Sensor Based on Hall Effect Sensors

OpenTransat
http://opentransat.com
http://github.com/opentransat
http://fb.com/opentransat
Released under the Creative Commons Attribution ShareAlike 4.0 International License
https://creativecommons.org/licenses/by-sa/4.0/
*/

#define HALL A0
#define MUXA A1
#define MUXB A2
#define MUXC A3
#define MUXD A4

#define ROUND(x) ((x)>=0?(int)((x)+0.5):(int)((x)-0.5))
const int HALL_ZERO[16] = {448, 445, 440, 441, 441, 441, 444, 444, 445, 440, 445, 445, 440, 443, 442, 442};

//#define SERIAL_DEBUG
#define SERIAL_BAUD 9600
#define PRINT_DELAY 250

#define NUM 16
#define HALL_MAX 500
int h[NUM];

void setup() {
  pinMode(HALL, INPUT);
  pinMode(MUXA, OUTPUT);
  digitalWrite(MUXA, LOW);
  pinMode(MUXB, OUTPUT);
  digitalWrite(MUXB, LOW);
  pinMode(MUXC, OUTPUT);
  digitalWrite(MUXC, LOW);
  pinMode(MUXD, OUTPUT);
  digitalWrite(MUXD, LOW);
  Serial.begin(SERIAL_BAUD);
}

int readHall(char i) {
  digitalWrite(MUXA, bitRead(i, 0));
  digitalWrite(MUXB, bitRead(i, 1));
  digitalWrite(MUXC, bitRead(i, 2));
  digitalWrite(MUXD, bitRead(i, 3));
  delayMicroseconds(800); //power on hall sensor (DRV5055 datasheet: 175-330us)
  return analogRead(HALL) - HALL_ZERO[i];
}

void readAll() {
  for (char i = 0; i < NUM; i++) {
    h[i] = readHall(i);
  }  
}

char findMaxi() {
  char i = 0;
  for (char j = 0; j < NUM; j++) {
    if (h[j] > h[i])
      i = j;
  }
  return i;
}

float correction(float diff) {
  if (diff > 300)
    return 0;
  else if (diff > 285)
    return 1;
  else if (diff > 266)
    return 2;
  else if (diff > 238)
    return 3;
  else if (diff > 206)
    return 4;
  else if (diff > 190)
    return 5;
  else if (diff > 153)
    return 6;
  else if (diff > 118)
    return 7;
  else if (diff > 83)
    return 8;
  else if (diff > 34)
    return 9;
  else
    return 10;
}
    
int findAngle() {
  char i = findMaxi();
  float angle = i*22.5;
  float diff;
  char i1 = (i - 1 + 16) % 16;
  char i2 = (i + 1 + 16) % 16;
  if (h[i1] > h[i2]) {
    diff = h[i] - h[i1];
    angle -= correction(diff);
  }
  else {
    diff = h[i] - h[i2];
    angle += correction(diff);
  }
  if (angle < 0)
    angle += 360;

  return (int)ROUND(angle);
}

void printNMEA(int a) {
  char buf[128];
  sprintf(buf, "H,%d", a);
  Serial.print("$");
  Serial.print(buf);
  Serial.print('*');
  PrintHex8(checksum(buf));
  Serial.println();
}

void loop() {
  readAll();
  int a = findAngle();
  printNMEA(a);
  #ifdef SERIAL_DEBUG
    printAll();
  #endif
  delay(PRINT_DELAY);
}

void printAll() { //debugging
  for (char i = 0; i < NUM; i++) {
    Serial.print(h[i]);
    Serial.print(" ");
  }
  Serial.println();
  delay(100);
}

unsigned char checksum(char* s) {
  unsigned char ret = 0;
  while (*s != '\0') {
    ret ^= *s;
    s++;
  }
  return ret;
}

void PrintHex8(char X) {
  if (X < 16)
    Serial.print("0");
  Serial.print(X, HEX);
}


//increase I2C BUFFER to 128 bytes:
//twi.h: TWI_BUFFER_LENGTH 128
//Wire.h: BUFFER_LENGTH 128

#include "Globals.h"
#include <Wire.h>
#include <SPI.h>
#include <EEPROM.h>

#define ROUND(x) ((x)>=0?(int)((x)+0.5):(int)((x)-0.5))
#define RESET() digitalWrite(GPIO, LOW)
#define DELAY_RESET() { delay_sec(4); digitalWrite(GPIO, LOW); }

//#define SERIAL_DEBUG //show command received
//#define DEBUG_COMPASS //uncomment for calibrating
//#define CALIB_COMPASS //uncomment for calibrating

#define DEFAULT_COMPASS 1
#define SERIAL_BAUD 38400 //accepted baud for 8MHz: 9600, 14400, 19200, 38400 https://cache.amobbs.com/bbs_upload782111/files_22/ourdev_508497.html
#define DETECT_CHANGE_EQUAL_MAX 50
#define DETECT_CHANGE_FLOAT_FACTOR 10000

#define LED 24 //PA2
#define MUX1_ABC 70 //PE2
#define MUX1_INH 5 //PE3
#define MUX2_ABC 25 //PA3
#define MUX2_INH 26 //PA4
#define HEARTBEAT 71 //PD6
#define GPIO 38 //PD7

char hb;
bool save_calib = false;

//EEPROM data with checking to store the next waypoint
struct EEPROM_data {
  uint8_t compass_num;
  uint8_t crc;
};
EEPROM_data ee;
#define EEPROM_ADDR 0

void saveEEPROM() {
  ee.crc = ee.compass_num;
  EEPROM.put(EEPROM_ADDR, ee);
}

void loadEEPROM() {
  EEPROM.get(EEPROM_ADDR, ee);
  if (ee.compass_num < 1 || ee.compass_num > 4 || ee.crc != ee.compass_num) {
    ee.compass_num = DEFAULT_COMPASS;
    saveEEPROM();
  }
}

void setCompass(uint8_t x) { //0 = none, 1, 2, 3, 4
  ee.compass_num = x; // global var
  switch (x) {
    case 0:
      digitalWrite(MUX1_INH, HIGH);
      digitalWrite(MUX1_INH, HIGH);
      digitalWrite(MUX1_ABC, LOW);
      digitalWrite(MUX2_ABC, LOW);
      break;
    case 1:
      digitalWrite(MUX2_INH, HIGH);
      digitalWrite(MUX2_ABC, LOW);
      digitalWrite(MUX1_ABC, LOW);
      digitalWrite(MUX1_INH, LOW);
      break;       
    case 2:
      digitalWrite(MUX2_INH, HIGH);
      digitalWrite(MUX2_ABC, LOW);
      digitalWrite(MUX1_ABC, HIGH);    
      digitalWrite(MUX1_INH, LOW);
      break;       
    case 3:
      digitalWrite(MUX1_INH, HIGH);
      digitalWrite(MUX1_ABC, LOW);    
      digitalWrite(MUX2_ABC, HIGH);    
      digitalWrite(MUX2_INH, LOW);
      break;       
    case 4:
      digitalWrite(MUX1_INH, HIGH);
      digitalWrite(MUX1_ABC, LOW);    
      digitalWrite(MUX2_ABC, LOW);    
      digitalWrite(MUX2_INH, LOW);
      break;
    default:
      break;
  }
}

void saveCompass(uint8_t x) {
  ee.compass_num = x;
  saveEEPROM();
}

void setup() {
  //first, all inputs with pullups to minimize power consumption
  DDRA = 0b00000000;
  PORTA = 0b11111111;
  DDRB = 0b00000000;
  PORTB = 0b11111111;
  DDRC = 0b00000000;
  PORTC = 0b11111111;
  DDRD = 0b00000000;
  PORTD = 0b11111111;
  DDRE = 0b00000000;
  PORTE = 0b11111111;
  DDRF = 0b00000000;
  PORTF = 0b11111111;
  DDRG = 0b00000000;
  PORTG = 0b11111111;
  DDRG = 0b00000000;
  PORTG = 0b11111111;
  DDRH = 0b00000000;
  PORTH = 0b11111111;
  DDRJ = 0b00000000;
  PORTJ = 0b11111111;
  DDRK = 0b00000000;
  PORTK = 0b11111111;
  DDRL = 0b00000000;
  PORTL = 0b11111111;
  
  pinMode(MUX1_INH, OUTPUT);
  digitalWrite(MUX1_INH, HIGH); //all compasses turned off by default

  pinMode(MUX2_INH, OUTPUT);
  digitalWrite(MUX2_INH, HIGH); //all compasses turned off by default

  pinMode(MUX1_ABC, OUTPUT);
  pinMode(MUX2_ABC, OUTPUT);

  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);

  pinMode(HEARTBEAT, OUTPUT);
  hb = LOW;
  digitalWrite(HEARTBEAT, hb);
  pinMode(GPIO, OUTPUT);
  digitalWrite(GPIO, HIGH);

  Serial.begin(SERIAL_BAUD);
  Serial.println("******************* start *******************");

  loadEEPROM();
  startCompass(ee.compass_num);
}

void startCompass(char cmp) {
  delay_sec(1); //delay to propagate power, otherwise the compass freezes at *warm start*
  readcmd(); //reads command E<num>; from the main controller to change compass and store it in EEPROM
  setCompass(cmp);
  delay_sec(1); //delay to propagate power, otherwise the compass freezes at *warm start*
  Wire.begin();
  initCompass();  
}

void heartbeat() {
  hb = !hb;
  digitalWrite(HEARTBEAT, hb);  
}

void delay_sec(int n) {
  while(n--) {
    delay(1000);
    heartbeat();
  }
}

void fixAHRS() {
  Yaw -= 270.0f;
  if (Yaw < 0) Yaw += 360.0f;
  float p = Pitch;
  Pitch = -Roll;
  Roll = p;
}

void printHuman() {
  Serial.print((uint8_t)ee.compass_num);
  Serial.print(" ");
  Serial.print(Yaw);
  Serial.print(" ");
  Serial.print(Pitch);
  Serial.print(" ");
  Serial.print(Roll);
  Serial.print(" ");
  Serial.println(temperature);
}

void printNMEA() {
  char buf[128];
  char Yaw_str[16];
  dtostrf(Yaw, 0, 4, Yaw_str);
  sprintf(buf, "C,%u,%s,%d,%d,%d", ee.compass_num, Yaw_str, ROUND(Pitch), ROUND(Roll), ROUND(temperature));
  Serial.print("$");
  Serial.print(buf);
  Serial.print('*');
  PrintHex8(checksum(buf));
  Serial.println();
}

void detectChange() {
  static float Yaw_prev = 0;
  static int counter = 0;
  
  if ((long)(Yaw*DETECT_CHANGE_FLOAT_FACTOR) == (long)(Yaw_prev*DETECT_CHANGE_FLOAT_FACTOR))
    counter++;
  else
    counter = 0;

  if (counter > DETECT_CHANGE_EQUAL_MAX)
    DELAY_RESET();

  Yaw_prev = Yaw;
}

void runCompass() {
  #ifdef CALIB_COMPASS
    calibCompass();
    readcmd();
  #else
    readCompass();
    detectChange();
    fixAHRS();
    //printHuman();
    printNMEA();
    smartDelay(100); //read command characters
  #endif  
}

void loop() {
  heartbeat();
  runCompass();
}

void smartDelay(unsigned long ms) {
  unsigned long start = millis();
  do {
    while (Serial.available()) {
      readcmd();
    }
  } while (millis() - start < ms);
}

void blinkN(char n) {
  while(n--) {
    digitalWrite(LED, HIGH);
    delay(200);
    digitalWrite(LED, LOW);
    delay(200);
  }
  delay_sec(2);
}

void compass_power_test() {
  setCompass(1);
  blinkN(1);
  setCompass(2);
  blinkN(2);
  setCompass(3);
  blinkN(3);
  setCompass(4);
  blinkN(4);
  blinkN(5);
  RESET();
  blinkN(6);
  blinkN(7);
}

////////////////////////////////////////////////////////////

String cmd = "";
int cmdlen;
#define cmdmax 200

void readcmd() {
  while (Serial.available() > 0) {
    char ch = (char)Serial.read();
    if (ch == '\n' || ch == '\r') continue;
    if (ch == 'S')
      save_calib = true;      
    if (ch == ';') { //end of command, process
      execmd();
      cmd = "";
      cmdlen = 0;
    }
    else if (ch == '$') { //start a new command (optional)
      cmd = "";
      cmdlen = 0;
    }
    else {
      cmd += ch;
      cmdlen++;
    }
    if (cmdlen > cmdmax-2) { //too long command
      cmd = "";
      cmdlen = 0;      
    }
  }
}

void execmd() {
  #ifdef SERIAL_DEBUG
    Serial.print("COMMAND: "); Serial.println(cmd);
  #endif
  //find first digit
  int i = 0;
  while(i < cmd.length() && !isDigit(cmd[i]) && cmd[i] != '-') i++;
  //
  String cmd1 = cmd.substring(0, i);
  String cmd2_str = cmd.substring(i);
  int cmd2 = cmd2_str.toInt();
  if (cmd1 == "C" && cmd2 >= 0 && cmd2 <= 4) { //set compass
    startCompass(cmd2);
  }
  if (cmd1 == "E" && cmd2 >= 0 && cmd2 <= 4) { //change compass, store it in EEPROM and reset
    if (cmd2 != ee.compass_num) {
      saveCompass(cmd2);
      delay_sec(1);
      RESET();
    }
  }
  if (cmd1 == "RESET" && cmd2 == 0) { //reset
    delay_sec(1);
    RESET();
  }
}

//////////////////////////////////////////////////////

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


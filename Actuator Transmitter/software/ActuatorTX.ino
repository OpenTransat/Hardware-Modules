/*
Actuator Transmitter (inside the hull)

OpenTransat
http://opentransat.com
http://github.com/opentransat
http://fb.com/opentransat
Released under the Creative Commons Attribution ShareAlike 4.0 International License
https://creativecommons.org/licenses/by-sa/4.0/
*/

#include <RH_ASK.h>
#include <SPI.h> // Not actualy used but needed to compile

uint8_t buflen = RH_ASK_MAX_MESSAGE_LEN;

#define WAKEUP_SIGNAL 1500 //continuous signal to wake up receiver on the sailwing (if the sailwing has 1s sleep cycle => wakeup signal = 1.5s)
#define PRINT_BAUD 9600
//#define SERIAL_DEBUG

//pins
#define LED 13 //PB5
#define RF_TX 3 //PD3
#define RF_RX 2 //PD2
#define RF_TX_ON 5 //PD5
#define RF_RX_ON 4 //PD4

typedef struct {
  float volts;
  float amps;
} vamps;

//RH_ASK
RH_ASK driver(500, RF_RX, RF_TX); //speed, rx, tx
//Fosc = 2MHz => Tosc = 0.5us
//1000bps => 1ms bit period
//8 samples per bit period => 125us per sample = 500*Tosc
//OK, integral number of Tosc

int msg;

String cmd = "";
int cmdlen;
#define cmdmax 200


void setup() {
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);
  pinMode(RF_TX_ON, OUTPUT);
  digitalWrite(RF_TX_ON, LOW); //transmitter on only when needed
  pinMode(RF_RX_ON, OUTPUT);
  digitalWrite(RF_RX_ON, HIGH); //receiver always on

  delay(100); //serial doesn't print without the delay
  
  Serial.begin(PRINT_BAUD);
  
  #ifdef SERIAL_DEBUG
    Serial.println("setup");
  #endif
  
  //RH_ASK init
  driver.init();

  //init LED
  digitalWrite(LED, HIGH);
  delay(100);
  digitalWrite(LED, LOW);
  delay(100);
  digitalWrite(LED, HIGH);
  delay(100);
  digitalWrite(LED, LOW);
  delay(100);
}

void loop() {
  readcmd(); //non-blocking
  smartDelay(0); //receive characters
}

void response(char chanID, unsigned int msg) {
  char buf[128];
  sprintf(buf, "%c,%d", chanID, msg);
  Serial.print('$');
  Serial.print(buf);
  Serial.print('*');
  PrintHex8(checksum(buf));
  #ifdef SERIAL_DEBUG
    Serial.print(" ");
    Serial.print(msg >> 8);
    Serial.print(" ");
    Serial.print(msg % 256);
  #endif
  Serial.println();
}

//read incoming characters within delay
void smartDelay(unsigned long ms) {
  unsigned long startMillis = millis();
  unsigned long currentMillis;
  uint8_t buf[buflen];
  do {
    if (driver.recv(buf, &buflen)) {
      int i;
      for (i = 0; i < buflen-2; i += 3){
        char chanID = buf[i];
        unsigned int msg = (buf[i+2] << 8) | buf[i+1];
        response(chanID, msg);
      }
    }
    currentMillis = millis();
  } while (currentMillis >= startMillis /* check millis overflow */ && currentMillis - startMillis < ms);
}

void send_message(char chanID, unsigned int *msg){
  //Since the reading is somewhere between 0 and 1023 (from the analog read) and we are only
  //sending 8-bit packets, each reading must be split as two 8-bit elements and channel ID is
  //sent as a single character, 8-bits large by default.
  uint8_t buf[3];
  buf[0] = chanID;    // Label this channel, to distinguish it from other channels
  buf[1] = *msg;      // Least significant byte (8 bits), rest is truncated from int to uint8_t
  buf[2] = (*msg)>>8; // Most significant byte

  digitalWrite(RF_TX, HIGH); //wake up receiver on the sailwing
  delay(WAKEUP_SIGNAL);
  digitalWrite(RF_TX, LOW);
  delay(100);
  
  digitalWrite(LED, HIGH);       // Flash a light to show transmitting
  driver.send(buf, 3);     // Sending the message
  driver.waitPacketSent(3000);      // Wait until the whole message is gone                
  digitalWrite(LED, LOW);      // Turn the LED off.
  smartDelay(100);                    // A short gap.
}

void readcmd() {
  while (Serial.available() > 0) {
    char ch = (char)Serial.read();
    if (ch == '\n' || ch == '\r') continue;
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
    Serial.print("Received command: "); Serial.println(cmd);
  #endif
  //find first digit
  int i = 0;
  while(i < cmd.length() && !isDigit(cmd[i]) && cmd[i] != '-') i++;
  //
  String cmd1 = cmd.substring(0, i);
  String cmd2_str = cmd.substring(i);
  int cmd2 = cmd2_str.toInt();
  if (cmd1 == "A" && cmd2 >= 0 && cmd2 <= 65535) {
    digitalWrite(RF_TX_ON, HIGH);
    send_message('A', &cmd2);
    digitalWrite(RF_TX_ON, LOW);
  }
  else if (cmd1 == "S" && cmd2 >= 0 && cmd2 <= 65535) {
    digitalWrite(RF_TX_ON, HIGH);
    send_message('S', &cmd2);
    digitalWrite(RF_TX_ON, LOW);
  }
  else if (cmd1 == "P" && cmd2 >= 0 && cmd2 <= 65535) {
    digitalWrite(RF_TX_ON, HIGH);
    send_message('P', &cmd2);
    digitalWrite(RF_TX_ON, LOW);
  }
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


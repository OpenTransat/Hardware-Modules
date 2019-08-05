/*
Actuator Controller (inside the sailwing)

OpenTransat
http://opentransat.com
http://github.com/opentransat
http://fb.com/opentransat
Released under the Creative Commons Attribution ShareAlike 4.0 International License
https://creativecommons.org/licenses/by-sa/4.0/
*/

//Atmel Studio programming: change SPI clock to max 62.5kHz because the i/o clock is prescaled by 1/8 while sleeping

#include <RH_ASK.h> //RH_ASK.cpp: using timer1 (default), timer2 will be for interrupts
#include <SPI.h>                // Not actualy used but needed to compile
#include <Adafruit_INA219.h>
#include <avr/sleep.h>
#include <avr/power.h>

uint8_t buflen = RH_ASK_MAX_MESSAGE_LEN;

//pins
#define LED 13 //PB5
#define HEARTBEAT 6 //PD6
#define IN1 7 //PD7
#define IN2 8 //PB0
#define IN3 9 //PB1
#define IN4 10 //PB2
#define POT A7 //analog input ADC7
#define THERM A0 //analog input PC0,ADC0
#define PWRSENSOR_ON 1 //PD1,TX
#define ACT_ON 0 //PD0,RX (switches EMS & motor driver DC+)
#define RF_TX 3 //PD3
#define RF_RX 2 //PD2
#define RF_TX_ON 5 //PD5
#define RF_RX_ON 4 //PD4
#define ENC_CS A1 //PC1, EMS:6
#define ENC_CLK A3 //PC3, EMS:2
#define ENC_DO A2 //PC2, EMS:4

#define ACT_TIMEOUT 45000UL //number of milliseconds when the actuator is not moving
#define ENC_TRIALS 10 //how many times to read the encoder when parity fails
#define ENC_VALID_MIN 100
#define ENC_VALID_MAX 900
#define ENC_ZERO 521
#define ENC_MIN (ENC_ZERO - 70)
#define ENC_MAX (ENC_ZERO + 70)

//#define SERIAL_DEBUG //WARNING: IN1/IN2 can burn the motor driver if connected!
#ifdef SERIAL_DEBUG
  #include <SoftwareSerial.h>
  SoftwareSerial swSerial(IN1, IN2); // RX, TX
#endif

//ina219
#define INA219_ADDR 0x45 //A0=+3V3, A1=+3V3
#define INA219_SOL1_ADDR 0x40 //PowerSensor: A1=GND2, A0=GND2
#define INA219_SOL2_ADDR 0x44 //PowerSensor: A1=3V3-SOL, A0=GND2

Adafruit_INA219 ina219(INA219_ADDR); //power sensor on Navigator
Adafruit_INA219 ina219_sol1(INA219_SOL1_ADDR); //PowerSensor: solar panel 1
Adafruit_INA219 ina219_sol2(INA219_SOL2_ADDR); //PowerSensor: solar panel 2

typedef struct {
  float volts;
  float mamps;
} vamps;

unsigned int encodeVamps(vamps va, float volts_factor, float mamps_factor) {
  return ((unsigned int)round(va.volts * volts_factor) << 8) | (unsigned int)round(va.mamps * mamps_factor);
}

//RH_ASK
RH_ASK driver(500, RF_RX, RF_TX); //speed, rx, tx
//Fosc = 2MHz => Tosc = 0.5us
//1000bps => 1ms bit period
//8 samples per bit period => 125us per sample = 500*Tosc
//OK, integral number of Tosc


enum tPullPush {
  ACT_PULL,
  ACT_PUSH
};

enum tfeedback {
  FB_ENC,
  FB_POT
};

void setup() {
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);
  pinMode(HEARTBEAT, OUTPUT);


  #ifdef SERIAL_DEBUG
    swSerial.begin(9600);  
  #else
    pinMode(IN1, OUTPUT);
    digitalWrite(IN1, LOW);
    pinMode(IN2, OUTPUT);
    digitalWrite(IN2, LOW);
  #endif
  
  pinMode(IN3, OUTPUT);
  digitalWrite(IN3, LOW);
  pinMode(IN4, OUTPUT);
  digitalWrite(IN4, LOW);

  pinMode(ENC_CS, OUTPUT);
  digitalWrite(ENC_CS, LOW);
  pinMode(ENC_CLK, OUTPUT);
  digitalWrite(ENC_CLK, LOW); //ENC_CLK=HIGH draws 15mA current even if encoder is disconnected!
  
  pinMode(ENC_DO, INPUT);

  pinMode(POT, INPUT);
  pinMode(THERM, INPUT);

  pinMode(PWRSENSOR_ON, OUTPUT);
  digitalWrite(PWRSENSOR_ON, LOW);
  
  pinMode(ACT_ON, OUTPUT);
  digitalWrite(ACT_ON, LOW);
  pinMode(RF_TX_ON, OUTPUT);
  digitalWrite(RF_TX_ON, LOW); //transmitter on only when needed
  pinMode(RF_RX_ON, OUTPUT);
  digitalWrite(RF_RX_ON, LOW);

  ina219.begin(); 
  ina219.setCalibration_32V_2A(); //0.1 ohm resistor
  ina219.enterPowerSave();

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

  init_timer2(); //interrupt to wake up
}

void loop() {
  heartbeat();
  smartDelay(500);
}

void heartbeat() {
  static bool hb = LOW;
  hb = !hb;
  //digitalWrite(LED, hb);
  digitalWrite(HEARTBEAT, hb);
}

int enc_read() { //output: 0..1023, -1 = parity error
  digitalWrite(ENC_CLK, HIGH);
  digitalWrite(ENC_CS, HIGH);
  delayMicroseconds(1);
  digitalWrite(ENC_CS, LOW);
  byte b[16];
  int pos = 0;
  byte parity = 0;

  for (int i = 0; i < 16; i++) {
    digitalWrite(ENC_CLK, LOW);
    delayMicroseconds(1);
    digitalWrite(ENC_CLK, HIGH);
    b[i] = digitalRead(ENC_DO) == HIGH ? 1 : 0;

    if (i < 15)
      parity += b[i];
  }

  parity %= 2;

  if (b[15] == parity) {
    for (int i = 0; i < 10; i++) {
      pos += b[i] * (1 << 10-(i+1));
    }
    digitalWrite(ENC_CS, LOW);
    digitalWrite(ENC_CLK, LOW); //minimize current draw
    return pos;
  }
  digitalWrite(ENC_CS, LOW);
  digitalWrite(ENC_CLK, LOW); //minimize current draw
  return -1;
}

int enc_read_try(int trials) {
  int enc;

  while ((enc = enc_read()) < 0) {
    if (--trials <= 0)
      break;
    delay(20);
  }

  return enc;
}

void act_pull() {
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN4, HIGH);   
}

void act_push() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN4, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
}

void act_stop() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW); 
}

#define POT_ARR_NUM 51
int pot_arr[POT_ARR_NUM] =
{880, 876, 861, 856, 852,
840, 833, 830, 816, 810,
798, 787, 780, 776, 763,
758, 743, 736, 727, 723,
711, 703, 691, 683, 672,
661 /*middle index: 24*/,
655, 642, 632, 620, 610,
601, 589, 582, 569, 558,
546, 533, 527, 514, 503,
490, 478, 469, 456, 449,
435, 424, 413, 402, 389};

void response(char chanID, unsigned int msg) {
  if (chanID == 'A') { //set actuator at a specific angle given by msg (100 + -20..20)
    
    if (msg > 1023) //allowed input is 0-1023
      return;

    delay(200); //some delay before the first message

    digitalWrite(RF_TX_ON, HIGH); //transmitter on
    send_message2('A', &msg); //instant confirmation that the message was received
    digitalWrite(RF_TX_ON, LOW); //transmitter off

    digitalWrite(ACT_ON, HIGH); //connect driver
    delay(20);

    //try reading encoder
    int enc = enc_read_try(ENC_TRIALS);
    int pot = analogRead(POT);
    int pot_go, enc_go;
    tfeedback fb = FB_ENC;
    
    if (enc < ENC_VALID_MIN || enc > ENC_VALID_MAX) { //bad encoder, use potentiometer
      fb = FB_POT;
      
      int i = (long)msg + 24 - 100;
      if (i < 0)
        i = 0;
      else if (i > POT_ARR_NUM-1)
        i = POT_ARR_NUM-1;
      pot_go = pot_arr[i]; //convert msg to potentiometer value
      
    } else { //ok, use encoder
      fb = FB_ENC;
      enc_go = ENC_ZERO - (((long)msg - 100) * 1023 / 360); //convert msg to encoder value
      if (enc_go > ENC_MAX)
        enc_go = ENC_MAX;
      if (enc_go < ENC_MIN)
        enc_go = ENC_MIN;
    }
    
    tPullPush pp;

    if (fb == FB_ENC && enc < enc_go || fb == FB_POT && pot < pot_go) {
      act_push();
      pp = ACT_PUSH;
    }
    else {
      act_pull();
      pp = ACT_PULL;
    }

    unsigned long startMillis = millis();

    do {
      enc = enc_read_try(ENC_TRIALS);
      pot = analogRead(POT);
      
      unsigned long currentMillis = millis();
      if (currentMillis < startMillis) break;
      if (currentMillis - startMillis > ACT_TIMEOUT)
        break;

    } while(pp == ACT_PUSH && (fb == FB_ENC && enc < enc_go || fb == FB_POT && pot < pot_go)
         || pp == ACT_PULL && (fb == FB_ENC && enc > enc_go || fb == FB_POT && (pot > pot_go || pot == 0 /* potentiometer temporarily disconnected */)));
    
    act_stop(); //stop actuator
    digitalWrite(ACT_ON, LOW); //disconnect driver

    //output encoder & potentiometer value for debugging
    digitalWrite(RF_TX_ON, HIGH); //transmitter on

    if (fb == FB_ENC) { //the order depends on whether encoder or potentiometer was used
      msg = enc;
      send_message2('E', &msg);
      //msg = pot;
      //send_message2('P', &msg);
    }
    else {
      msg = enc;
      send_message2('P', &msg);
      //msg = pot;
      //send_message2('E', &msg);
    }
    digitalWrite(RF_TX_ON, LOW); //transmitter off
  }
  else if (chanID == 'S') { //sensors: battery, left/right panel, temperature
    //read ina219
    ina219.begin(); 
    ina219.setCalibration_32V_2A(); //0.1 ohm resistor
    vamps va = read_INA(ina219);
    unsigned int ina = encodeVamps(va, 10, 10);
    ina219.enterPowerSave();

    //read temperature
    float read_temp = analogRead(THERM);
    unsigned int temp = round((read_temp - 370.0)/5.07);

    //read power sensor
    digitalWrite(PWRSENSOR_ON, HIGH);
    delay(2000); //charging capacitor
    ina219_sol1.begin();
    ina219_sol1.setCalibration_32V_2A(); //0.1 ohm resistor
    va = read_INA(ina219_sol1);
    unsigned int ina_sol1 = encodeVamps(va, 10, 1);
    ina219_sol2.begin();
    ina219_sol2.setCalibration_32V_2A(); //0.1 ohm resistor
    va = read_INA(ina219_sol2);
    unsigned int ina_sol2 = encodeVamps(va, 10, 1);
    digitalWrite(PWRSENSOR_ON, LOW);

    //transmitting sensor data
    digitalWrite(RF_TX_ON, HIGH); //transmitter on
    msg = ina;
    send_message('B', &msg);
    msg = ina_sol1;
    send_message('R', &msg);
    msg = ina_sol2;
    send_message('L', &msg);
    send_message('T', &temp);
    digitalWrite(RF_TX_ON, LOW); //transmitter off
  }
  else if (chanID == 'P') { //position: EMS+POT
    delay(200); //some delay before the first message
    digitalWrite(RF_TX_ON, HIGH); //transmitter on

    msg = analogRead(POT);
    send_message2('P', &msg);

    digitalWrite(ACT_ON, HIGH); //actuator+EMS circuit on
    delay(20);
    msg = enc_read();
    digitalWrite(ACT_ON, LOW); //actuator+EMS circuit off
    send_message2('E', &msg);

    digitalWrite(RF_TX_ON, LOW); //transmitter off
  }
}

bool listen_mode = false;

//read incoming characters within delay
void smartDelay(unsigned long ms) {
  unsigned long startMillis = millis();
  unsigned long currentMillis;
  uint8_t buf[buflen];
  do {
    static unsigned long sum = 0;
    static unsigned long count = 0;
    static unsigned long start_counting = millis();

    if (!listen_mode && millis() > start_counting + 100) { //100ms RX module startup time, then increment count and sum
      count++;
      if (digitalRead(RF_RX))
        sum++;
    }

    if (!listen_mode && millis() > start_counting + 150) { //after 150ms: keep listening or go to sleep
      if (sum > count - count/10) { //keep listening for characters
        listen_mode = true;
      }

      if (!listen_mode)
        enterSleep();      

      sum = count = 0;
      start_counting = millis();
     }

    if (listen_mode && millis() > start_counting + 2500UL) { //not receiving characters for 2.5sec: go to sleep
      listen_mode = false;
      enterSleep();
      sum = count = 0;
      start_counting = millis();
    }

    if (listen_mode && driver.recv(buf, &buflen)) {
      sum = count = 0;
      start_counting = millis(); //receiving characters => prevent going to sleep
      
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
  
  digitalWrite(LED, HIGH);       // Flash a light to show transmitting
  driver.send(buf, 3);     // Sending the message
  driver.waitPacketSent(3000);      // Wait until the whole message is gone                
  digitalWrite(LED, LOW);      // Turn the LED off.
  smartDelay(100);                    // A short gap.
}

vamps read_INA(Adafruit_INA219 ina) {
  float shuntvoltage = ina.getShuntVoltage_mV();
  float busvoltage = ina.getBusVoltage_V();
  float volts = busvoltage + shuntvoltage / 1000;
  float ma = ina.getCurrent_mA();
  return {volts, ma};
}

void enterSleep()
{ 
  set_sleep_mode(SLEEP_MODE_PWR_SAVE);
  sleep_enable();

  digitalWrite(RF_RX_ON, LOW); //receiver off before going to sleep  
  digitalWrite(LED, LOW);
  
  power_adc_disable();
  power_spi_disable();
  power_timer0_disable();
  power_timer1_disable();
//  power_timer2_disable();  //used to wake up
  power_twi_disable();  
  
  clock_prescale_8();
  
  sleep_mode(); //going to sleep
  sleep_disable(); //runs after timer interrupt
  
  clock_prescale_normal();
  
  power_all_enable();
  digitalWrite(RF_RX_ON, HIGH); //turn on receiver
}
/*
void init_timer1() {
  TCCR1A = 0x00; 
  TCNT1 = 0; 
  TCCR1B = 0x03; //prescaler 1/64 => 2sec cycle (1/1024... 32sec cycle)
  TIMSK1=0x01;
}
*/
void init_timer2() {
  TCCR2A = 0x00; 
  TCNT2 = 0; 
  TCCR2B = 0x07; //prescaler 1/1024 => 1/2MHz * 256 * 1024 = 0.131072ms, but with clock prescaler 8: 1sec
  TIMSK2 = 0x01;
}

ISR(TIMER2_OVF_vect) {
}

void clock_prescale_normal() {
  CLKPR = _BV(CLKPCE);  // enable change of the clock prescaler
  CLKPR = 0;
}

void clock_prescale_8() {
  CLKPR = _BV(CLKPCE);  // enable change of the clock prescaler
  CLKPR = 3;
}

void send_message2(char chanID, unsigned int *msg) {
  send_message(chanID, msg);
  send_message(chanID, msg);
}


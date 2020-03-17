#include <Arduino.h>
#include "Adafruit_FONA.h"
#include "RotaryDial.h"


#define ROTARY_DIAL_TIMEOUT 3000 //If no numbers are dialed for this period, newline is sent
#define ROTARY_DIAL_PULSE_PIN 2 //PIN 2 supports interrupts on Arduino Nano
#define HOOK 3
#define FONA_RST 4
#define FONA_RI 7
#define FONA_TX 8
#define FONA_RX 9
#define FONA_KEY 10

#define FONA_RI_INTERRUPT 4


void(* resetFunc) (void) = 0;


// this is a large buffer for replies
char replybuffer[255];
char dialString[20];
int dialStringIndex=0;

// We default to using software serial. If you want to use hardware serial
// (because softserial isnt supported) comment out the following three lines 
// and uncomment the HardwareSerial line
#include "SoftwareSerial.h"
SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);
SoftwareSerial *fonaSerial = &fonaSS;

// Use this for FONA 800 and 808s
Adafruit_FONA fona = Adafruit_FONA(FONA_RST);

uint8_t readline(char *buff, uint8_t maxbuff, uint16_t timeout = 0);
uint8_t type;

void KEYtoggle()
{
  digitalWrite(FONA_KEY, HIGH);
  delay(4000);
  digitalWrite(FONA_KEY, LOW);
}




#define ONHOOK 0
#define OFFHOOK 1

#define STATE_IDLE  0
#define STATE_OFFHOOK  1
#define STATE_INCALL  2
#define STATE_ONHOOK 3
int hookStatus = STATE_ONHOOK;
int currentState = STATE_IDLE;

unsigned long lastdigit = 0;
int newline = true;


void flushSerial() {
  while (Serial.available())
    Serial.read();
}

char readBlocking() {
  while (!Serial.available());
  return Serial.read();
}
uint16_t readnumber() {
  uint16_t x = 0;
  char c;
  while (! isdigit(c = readBlocking())) {
    //Serial.print(c);
  }
  Serial.print(c);
  x = c - '0';
  while (isdigit(c = readBlocking())) {
    Serial.print(c);
    x *= 10;
    x += c - '0';
  }
  return x;
}

uint8_t readline(char *buff, uint8_t maxbuff, uint16_t timeout) {
  uint16_t buffidx = 0;
  boolean timeoutvalid = true;
  if (timeout == 0) timeoutvalid = false;

  while (true) {
    if (buffidx > maxbuff) {
      //Serial.println(F("SPACE"));
      break;
    }

    while (Serial.available()) {
      char c =  Serial.read();

      //Serial.print(c, HEX); Serial.print("#"); Serial.println(c);

      if (c == '\r') continue;
      if (c == 0xA) {
        if (buffidx == 0)   // the first 0x0A is ignored
          continue;

        timeout = 0;         // the second 0x0A is the end of the line
        timeoutvalid = true;
        break;
      }
      buff[buffidx] = c;
      buffidx++;
    }

    if (timeoutvalid && timeout == 0) {
      //Serial.println(F("TIMEOUT"));
      break;
    }
    delay(1);
  }
  buff[buffidx] = 0;  // null term
  return buffidx;
}

void printMenu(void) {
  Serial.println(F("-------------------------------------"));
  Serial.println(F("[?] Print this menu"));
  Serial.println(F("[b] read the Battery V and % charged"));
  Serial.println(F("[n] get Network status"));
  Serial.println(F("[v] set audio Volume"));
  Serial.println(F("[V] get Volume"));
  Serial.println(F("[e] set External audio (FONA800 & 808)"));
  Serial.println(F("[P] PWM/Buzzer out (FONA800 & 808)"));
  Serial.println(F("[Z] power off with Key"));
  Serial.println(F("[z] power on with Key"));

  // Phone
  Serial.println(F("[c] make phone Call"));
  Serial.println(F("[A] get call status"));
  Serial.println(F("[h] Hang up phone"));
  Serial.println(F("[p] Pick up phone"));

  // SMS
  Serial.println(F("[N] Number of SMSs"));
  Serial.println(F("[r] Read SMS #"));
  Serial.println(F("[R] Read All SMS"));
  Serial.println(F("[d] Delete SMS #"));
  Serial.println(F("[s] Send SMS"));
  Serial.println(F("[u] Send USSD"));
  
  Serial.println(F("-------------------------------------"));
}

// IRQ handler for the hook change event 
uint8_t HOOKEVT=0;
void HOOKChg()
{
  HOOKEVT=1;  
}

/**************************************/
/*     setup()                        */
/**************************************/
void setup() {

  // this waits forever for a serial connection to start
  // ***** remove this for auto-run *****
  //while (!Serial);

 
  // configure FONA power key
  pinMode(FONA_KEY, OUTPUT);
  KEYtoggle();

  // show debug
  Serial.begin(115200);
  Serial.println(F("FONA basic test"));
  Serial.println(F("Initializing....(May take 3 seconds)"));

  // Setup comms to FONA
  fonaSerial->begin(4800);
  if (! fona.begin(*fonaSerial)) {
    Serial.println(F("Couldn't find FONA"));
    KEYtoggle();
    delay(3000);
    resetFunc(); // FONA device did not respond. Reboot.
    //while (1);
  }
  type = fona.type();
  Serial.println(F("FONA is OK"));

  // Setup rotary dial features
  RotaryDial::setup(ROTARY_DIAL_PULSE_PIN); 
  
  // Using external audio and mic
  fona.setAudio(FONA_EXTAUDIO);
  fona.setMicVolume(FONA_EXTAUDIO, 10);

  // Monitor the phone hook with an interrupt. 
  pinMode(HOOK, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(HOOK),HOOKChg,CHANGE);
  delay(1000);
  HOOKEVT=0; // reset the hook event
}

/**************************************/
/*     LOOP()                         */
/**************************************/
void loop() {
  char phone[32]={0};
  flushSerial();
  while(fona.available()) {
    Serial.write(fona.read());
  }

  // Check HOOK status
  if(HOOKEVT){ // only when the hook event changes
    delay(300); // debounce with delay
    if(digitalRead(HOOK)==OFFHOOK){
      // off hook
      if(fona.getCallStatus()==3){ // incoming call
        fona.pickUp();
      }
      else {
        fona.playToolkitTone(1, 10000); // dial tone
      }
    }
    else{
      // on hook. 
      fona.playToolkitTone(2, 100); // Stop any playing tones
    }
    HOOKEVT=0; //reset
  } // end HOOKEVT

   
  /* Rotary Dial Reading code follows...*/
  if(RotaryDial::available()) {
    dialString[dialStringIndex]= '0'+RotaryDial::read();
    dialString[++dialStringIndex]=0;
    lastdigit = millis();
    newline = false;
  } else {
    if(!newline && millis() - lastdigit > ROTARY_DIAL_TIMEOUT) {
      Serial.println(" OK");
      newline = true;
      // TODO: phone number is complete. Check format here.
      // Dial the number:
      if(fona.getCallStatus()==0)
      {
        if (!fona.callPhone(dialString)) {
          fona.playToolkitTone(2, 10000);
          Serial.println(F("Failed"));
        } else {
          Serial.println(F("Sent!"));
        }
      }
    }
  }
 

  // Process any commands coming from the debug console
  char command=' ';
  if (Serial.available()){
    command = Serial.read();
    Serial.println(command);
    switch (command) {
      case '?': {
          printMenu();
          break;
        }
  
      case 'Z': {
        // Power OFF
          digitalWrite(FONA_KEY, LOW);
          delay(2000);
          digitalWrite(FONA_KEY, HIGH);
          delay(3000);
          break;
        }
      case 'z': {
          // Power ON
          digitalWrite(FONA_KEY, LOW);
          delay(2000);
          digitalWrite(FONA_KEY, HIGH);
          delay(3000);
          if (! fona.begin(*fonaSerial)) {
            Serial.println(F("Couldn't find FONA"));
          }
          break;
        }
      case 'n': {
          // read the network/cellular status
          uint8_t n = fona.getNetworkStatus();
          Serial.print(F("Network status "));
          Serial.print(n);
          Serial.print(F(": "));
          if (n == 0) Serial.println(F("Not registered"));
          if (n == 1) Serial.println(F("Registered (home)"));
          if (n == 2) Serial.println(F("Not registered (searching)"));
          if (n == 3) Serial.println(F("Denied"));
          if (n == 4) Serial.println(F("Unknown"));
          if (n == 5) Serial.println(F("Registered roaming"));
          break;
        }
  
      /*** Audio ***/
      case 'v': {
          // set volume
          flushSerial();
          if ( (type == FONA3G_A) || (type == FONA3G_E) ) {
            Serial.print(F("Set Vol [0-8] "));
          } else {
            Serial.print(F("Set Vol % [0-100] "));
          }
          uint8_t vol = readnumber();
          Serial.println();
          if (! fona.setVolume(vol)) {
            Serial.println(F("Failed"));
          } else {
            Serial.println(F("OK!"));
          }
          break;
        }
  
      case 'V': {
          uint8_t v = fona.getVolume();
          Serial.print(v);
          if ( (type == FONA3G_A) || (type == FONA3G_E) ) {
            Serial.println(" / 8");
          } else {
            Serial.println("%");
          }
          break;
        }
  
      case 'e': {
          // Set External output
          if (! fona.setAudio(FONA_EXTAUDIO)) {
            Serial.println(F("Failed"));
          } else {
            Serial.println(F("OK!"));
          }
  
          fona.setMicVolume(FONA_EXTAUDIO, 10);
          break;
        }
  
  
      /*** PWM ***/
      case 'P': {
          // PWM Buzzer output @ 2KHz max
          flushSerial();
          Serial.print(F("PWM Freq, 0 = Off, (1-2000): "));
          uint16_t freq = readnumber();
          Serial.println();
          if (! fona.setPWM(freq)) {
            Serial.println(F("Failed"));
          } else {
            Serial.println(F("OK!"));
          }
          break;
        }
  
      /*** Call ***/
      case 'c': {
          // call a phone!
          char number[30];
          flushSerial();
          Serial.print(F("Call #"));
          readline(number, 30);
          Serial.println();
          Serial.print(F("Calling ")); Serial.println(number);
          if (!fona.callPhone(number)) {
            Serial.println(F("Failed"));
          } else {
            Serial.println(F("Sent!"));
          }
  
          break;
        }
      case 'A': {
          // get call status
          int8_t callstat = fona.getCallStatus();
          switch (callstat) {
            case 0: Serial.println(F("Ready")); break;
            case 1: Serial.println(F("Could not get status")); break;
            case 3: Serial.println(F("Ringing (incoming)")); break;
            case 4: Serial.println(F("Ringing/in progress (outgoing)")); break;
            default: Serial.println(F("Unknown")); break;
          }
          break;
        }
        
      case 'h': {
          // hang up!
          if (! fona.hangUp()) {
            Serial.println(F("Failed"));
          } else {
            Serial.println(F("OK!"));
          }
          break;
        }
  
      case 'p': {
          // pick up!
          if (! fona.pickUp()) {
            Serial.println(F("Failed"));
          } else {
            Serial.println(F("OK!"));
          }
          break;
        }
  
      /*** SMS ***/
      case 'N': {
          // read the number of SMS's!
          int8_t smsnum = fona.getNumSMS();
          if (smsnum < 0) {
            Serial.println(F("Could not read # SMS"));
          } else {
            Serial.print(smsnum);
            Serial.println(F(" SMS's on SIM card!"));
          }
          break;
        }
      case 'r': {
          // read an SMS
          flushSerial();
          Serial.print(F("Read #"));
          uint8_t smsn = readnumber();
          Serial.print(F("\n\rReading SMS #")); Serial.println(smsn);
  
          // Retrieve SMS sender address/phone number.
          if (! fona.getSMSSender(smsn, replybuffer, 250)) {
            Serial.println("Failed!");
            break;
          }
          Serial.print(F("FROM: ")); Serial.println(replybuffer);
  
          // Retrieve SMS value.
          uint16_t smslen;
          if (! fona.readSMS(smsn, replybuffer, 250, &smslen)) { // pass in buffer and max len!
            Serial.println("Failed!");
            break;
          }
          Serial.print(F("***** SMS #")); Serial.print(smsn);
          Serial.print(" ("); Serial.print(smslen); Serial.println(F(") bytes *****"));
          Serial.println(replybuffer);
          Serial.println(F("*****"));
  
          break;
        }
      case 'R': {
          // read all SMS
          int8_t smsnum = fona.getNumSMS();
          uint16_t smslen;
          int8_t smsn;
  
          if ( (type == FONA3G_A) || (type == FONA3G_E) ) {
            smsn = 0; // zero indexed
            smsnum--;
          } else {
            smsn = 1;  // 1 indexed
          }
  
          for ( ; smsn <= smsnum; smsn++) {
            Serial.print(F("\n\rReading SMS #")); Serial.println(smsn);
            if (!fona.readSMS(smsn, replybuffer, 250, &smslen)) {  // pass in buffer and max len!
              Serial.println(F("Failed!"));
              break;
            }
            // if the length is zero, its a special case where the index number is higher
            // so increase the max we'll look at!
            if (smslen == 0) {
              Serial.println(F("[empty slot]"));
              smsnum++;
              continue;
            }
  
            Serial.print(F("***** SMS #")); Serial.print(smsn);
            Serial.print(" ("); Serial.print(smslen); Serial.println(F(") bytes *****"));
            Serial.println(replybuffer);
            Serial.println(F("*****"));
          }
          break;
        }
  
      case 'd': {
          // delete an SMS
          flushSerial();
          Serial.print(F("Delete #"));
          uint8_t smsn = readnumber();
  
          Serial.print(F("\n\rDeleting SMS #")); Serial.println(smsn);
          if (fona.deleteSMS(smsn)) {
            Serial.println(F("OK!"));
          } else {
            Serial.println(F("Couldn't delete"));
          }
          break;
        }
  
      case 's': {
          // send an SMS!
          char sendto[21], message[141];
          flushSerial();
          Serial.print(F("Send to #"));
          readline(sendto, 20);
          Serial.println(sendto);
          Serial.print(F("Type out one-line message (140 char): "));
          readline(message, 140);
          Serial.println(message);
          if (!fona.sendSMS(sendto, message)) {
            Serial.println(F("Failed"));
          } else {
            Serial.println(F("Sent!"));
          }
  
          break;
        }
      default: {
          printMenu();
          break;
        }
    }
  }


}  // end loop()
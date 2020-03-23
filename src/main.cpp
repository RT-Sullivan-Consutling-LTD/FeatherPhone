#include <Arduino.h>
#include "Adafruit_FONA.h"
#include "RotaryDial.h"


#define ROTARY_DIAL_TIMEOUT 3000 //If no numbers are dialed for this period, newline is sent
#define ROTARY_DIAL_PULSE_PIN 2 //PIN 2 supports interrupts on Arduino Nano
#define HOOK 3
#define ROTARYREADY 5 
#define FONA_RST 4
#define FONA_RI 7
#define FONA_TX 8
#define FONA_RX 9
#define FONA_KEY 10

#define FONA_RI_INTERRUPT 4

#define ONHOOK  1
#define OFFHOOK 0
#define ON      1 
#define OFF     2

#define STATE_IDLE      0
#define STATE_OFFHOOK   1
#define STATE_INCALL    2
#define STATE_ONHOOK    3

// Declare a reset function
void(* resetFunc) (void) = 0;

// this is a large buffer for replies
char replybuffer[255];
char dialString[20]; // used for building up outbound dialed number
int  dialStringIndex=0;
uint8_t type;  // FONA device type
unsigned long lastdigit = 0;
int newline = true;

// We default to using software serial. If you want to use hardware serial
// (because softserial isnt supported) comment out the following three lines 
// and uncomment the HardwareSerial line
#include "SoftwareSerial.h"
SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);
SoftwareSerial *fonaSerial = &fonaSS;

// Use this for FONA 800 and 808s
Adafruit_FONA fona = Adafruit_FONA(FONA_RST);

// Function prototypes
uint8_t readline(char *buff, uint8_t maxbuff, uint16_t timeout = 0);

/**************************************/
/*     support functions follow       */
/**************************************/
uint8_t FONApwr(uint8_t mode)
{
  if(mode==0){
    // Power OFF
    digitalWrite(FONA_KEY, LOW);
    delay(2000);
    digitalWrite(FONA_KEY, HIGH);
    delay(3000);
  }
  if(mode==1){
    // Power ON
    digitalWrite(FONA_KEY, LOW);
    delay(2000);
    digitalWrite(FONA_KEY, HIGH);
    delay(3000);
    if (! fona.begin(*fonaSerial)) {
      Serial.println(F("Couldn't find FONA"));
      return -1;
    }
  }
  
  return 0;
}

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
  Serial.println(F("[a] read the ADC 2.8V max (FONA800 & 808)"));
  Serial.println(F("[b] read the Battery V and % charged"));
  Serial.println(F("[C] read the SIM CCID"));
  Serial.println(F("[U] Unlock SIM with PIN code"));
  Serial.println(F("[i] read RSSI"));
  Serial.println(F("[n] get Network status"));
  Serial.println(F("[v] set audio Volume"));
  Serial.println(F("[V] get Volume"));
  Serial.println(F("[H] set Headphone audio (FONA800 & 808)"));
  Serial.println(F("[e] set External audio (FONA800 & 808)"));
  Serial.println(F("[T] play audio Tone"));
  Serial.println(F("[P] PWM/Buzzer out (FONA800 & 808)"));
  Serial.println(F("[Z] power off with Key"));
  Serial.println(F("[z] power on with Key"));

  // FM (SIM800 only!)
  Serial.println(F("[f] tune FM radio (FONA800)"));
  Serial.println(F("[F] turn off FM (FONA800)"));
  Serial.println(F("[m] set FM volume (FONA800)"));
  Serial.println(F("[M] get FM volume (FONA800)"));
  Serial.println(F("[q] get FM station signal level (FONA800)"));

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
  
  // Time
  Serial.println(F("[y] Enable network time sync (FONA 800 & 808)"));
  Serial.println(F("[Y] Enable NTP time sync (GPRS FONA 800 & 808)"));
  Serial.println(F("[t] Get network time"));

  // GPRS
  Serial.println(F("[G] Enable GPRS"));
  Serial.println(F("[g] Disable GPRS"));
  Serial.println(F("[l] Query GSMLOC (GPRS)"));
  Serial.println(F("[w] Read webpage (GPRS)"));
  Serial.println(F("[W] Post to website (GPRS)"));

  // GPS
  if ((type == FONA3G_A) || (type == FONA3G_E) || (type == FONA808_V1) || (type == FONA808_V2)) {
    Serial.println(F("[O] Turn GPS on (FONA 808 & 3G)"));
    Serial.println(F("[o] Turn GPS off (FONA 808 & 3G)"));
    Serial.println(F("[L] Query GPS location (FONA 808 & 3G)"));
    if (type == FONA808_V1) {
      Serial.println(F("[x] GPS fix status (FONA808 v1 only)"));
    }
    Serial.println(F("[E] Raw NMEA out (FONA808)"));
  }
  
  Serial.println(F("[S] create Serial passthru tunnel"));
  Serial.println(F("-------------------------------------"));
  Serial.println(F(""));

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

  // show debug
  Serial.begin(115200);
  Serial.println(F("FONA basic test"));
  Serial.println(F("Initializing....(May take 3 seconds)"));

  // Setup comms to FONA
  fonaSerial->begin(4800);
  if (! fona.begin(*fonaSerial)) {
    Serial.println(F("Couldn't find FONA"));
    FONApwr(ON);
    delay(3000);
    resetFunc(); // FONA device did not respond. Reboot.
    //while (1);
  }
  type = fona.type();
  Serial.println(F("FONA is OK"));

  // Setup rotary dial features
   RotaryDial::setup(ROTARY_DIAL_PULSE_PIN); 
  // pinMode(ROTARYREADY, INPUT); // Dial pulse ready pin
  // digitalWrite(ROTARYREADY, HIGH); // Pullup 

  // Monitor the phone hook with an interrupt. 
  pinMode(HOOK, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(HOOK),HOOKChg,CHANGE);
  delay(1000);
  HOOKEVT=0; // reset the hook event


  delay(3000);
  // Using external audio and mic
  fona.setAudio(FONA_EXTAUDIO);
  fona.setMicVolume(FONA_EXTAUDIO, 30);
  fona.setVolume(20);
  flushSerial();
}

/**************************************/
/*     LOOP()                         */
/**************************************/
void loop() {

  // if FONA has something to say...
  while(fona.available()) {
    Serial.write(fona.read()); // echo to serial
  }

  // Check HOOK status
  if(HOOKEVT){ // only when the hook event changes
    Serial.println("Hook event");
    delay(300); // debounce with delay
    if(digitalRead(HOOK)==OFFHOOK){
      // Answer inbound call
      if(fona.getCallStatus()==3){ // 3=incoming call
        fona.pickUp();
      }
      else {
        // Normal off-hook event. 
        if(fona.getCallStatus()!=0){ // cleanup
          fona.hangUp();
        }
        fona.playToolkitTone(1, 10000); // play dial tone 10s
      }
    }
    else{
      // on hook. 
      fona.playToolkitTone(2, 100); // Stop any playing tones
      fona.hangUp();
    }
    HOOKEVT=0; //reset
  } // end HOOKEVT

  // if(digitalRead(HOOK)==ONHOOK && fona.getCallStatus()!=0){
  //   Serial.print("Call status:");
  //   Serial.println(fona.getCallStatus());
  //   fona.hangUp();

  // }

  /* Rotary Dial Reading code follows...*/
  if(RotaryDial::available()) {
    dialString[dialStringIndex]= '0'+RotaryDial::read();
    Serial.print(F("Dialed: "));
    Serial.print(dialString);
    Serial.print("\r\n");
    dialStringIndex++;
    lastdigit = millis();
    newline = false;
  } else {
    if(!newline && millis() - lastdigit > ROTARY_DIAL_TIMEOUT) {
      Serial.print("OK. dialing: ");
      Serial.print(dialString);
      Serial.print("\r\n");
      newline = true;
      // TODO: phone number is complete. Check format here.

      // Dial the number:
      if(digitalRead(HOOK)==OFFHOOK && fona.getCallStatus()==0)
      {
        // if (!fona.callPhone(dialString)) {
        //   fona.playToolkitTone(2, 20000);
        //   Serial.println(F("Failed"));
        // } else {
        //   Serial.println(F("Sent!"));
        // }
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
        digitalWrite(FONA_KEY, LOW);
        delay(2000);
        digitalWrite(FONA_KEY, HIGH);
        delay(3000);
        break;
      }
    case 'z': {
        digitalWrite(FONA_KEY, LOW);
        delay(2000);
        digitalWrite(FONA_KEY, HIGH);
        delay(3000);
        if (! fona.begin(*fonaSerial)) {
          Serial.println(F("Couldn't find FONA"));
        }
        break;
      }
    case 'a': {
        // read the ADC
        uint16_t adc;
        if (! fona.getADCVoltage(&adc)) {
          Serial.println(F("Failed to read ADC"));
        } else {
          Serial.print(F("ADC = ")); Serial.print(adc); Serial.println(F(" mV"));
        }
        break;
      }

    case 'b': {
        // read the battery voltage and percentage
        uint16_t vbat;
        if (! fona.getBattVoltage(&vbat)) {
          Serial.println(F("Failed to read Batt"));
        } else {
          Serial.print(F("VBat = ")); Serial.print(vbat); Serial.println(F(" mV"));
        }


        if (! fona.getBattPercent(&vbat)) {
          Serial.println(F("Failed to read Batt"));
        } else {
          Serial.print(F("VPct = ")); Serial.print(vbat); Serial.println(F("%"));
        }

        break;
      }

    case 'U': {
        // Unlock the SIM with a PIN code
        char PIN[5];
        flushSerial();
        Serial.println(F("Enter 4-digit PIN"));
        readline(PIN, 3);
        Serial.println(PIN);
        Serial.print(F("Unlocking SIM card: "));
        if (! fona.unlockSIM(PIN)) {
          Serial.println(F("Failed"));
        } else {
          Serial.println(F("OK!"));
        }
        break;
      }

    case 'C': {
        // read the CCID
        fona.getSIMCCID(replybuffer);  // make sure replybuffer is at least 21 bytes!
        Serial.print(F("SIM CCID = ")); Serial.println(replybuffer);
        break;
      }

    case 'i': {
        // read the RSSI
        uint8_t n = fona.getRSSI();
        int8_t r;

        Serial.print(F("RSSI = ")); Serial.print(n); Serial.print(": ");
        if (n == 0) r = -115;
        if (n == 1) r = -111;
        if (n == 31) r = -52;
        if ((n >= 2) && (n <= 30)) {
          r = map(n, 2, 30, -110, -54);
        }
        Serial.print(r); Serial.println(F(" dBm"));

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

    case 'H': {
        // Set Headphone output
        if (! fona.setAudio(FONA_HEADSETAUDIO)) {
          Serial.println(F("Failed"));
        } else {
          Serial.println(F("OK!"));
        }
        fona.setMicVolume(FONA_HEADSETAUDIO, 15);
        break;
      }
    case 'e': {
        // Set External output
        if (! fona.setAudio(FONA_EXTAUDIO)) {
          Serial.println(F("Failed"));
        } else {
          Serial.println(F("OK!"));
        }

        fona.setMicVolume(FONA_EXTAUDIO, 15);
        break;
      }

    case 'T': {
        // play tone
        flushSerial();
        Serial.print(F("Play tone #"));
        uint8_t kittone = readnumber();
        Serial.println();
        // play for 1 second (1000 ms)
        if (! fona.playToolkitTone(kittone, 1000)) {
          Serial.println(F("Failed"));
        } else {
          Serial.println(F("OK!"));
        }
        break;
      }

    /*** FM Radio ***/

    case 'f': {
        // get freq
        flushSerial();
        Serial.print(F("FM Freq (eg 1011 == 101.1 MHz): "));
        uint16_t station = readnumber();
        Serial.println();
        // FM radio ON using headset
        if (fona.FMradio(true, FONA_HEADSETAUDIO)) {
          Serial.println(F("Opened"));
        }
        if (! fona.tuneFMradio(station)) {
          Serial.println(F("Failed"));
        } else {
          Serial.println(F("Tuned"));
        }
        break;
      }
    case 'F': {
        // FM radio off
        if (! fona.FMradio(false)) {
          Serial.println(F("Failed"));
        } else {
          Serial.println(F("OK!"));
        }
        break;
      }
    case 'm': {
        // Set FM volume.
        flushSerial();
        Serial.print(F("Set FM Vol [0-6]:"));
        uint8_t vol = readnumber();
        Serial.println();
        if (!fona.setFMVolume(vol)) {
          Serial.println(F("Failed"));
        } else {
          Serial.println(F("OK!"));
        }
        break;
      }
    case 'M': {
        // Get FM volume.
        uint8_t fmvol = fona.getFMVolume();
        if (fmvol < 0) {
          Serial.println(F("Failed"));
        } else {
          Serial.print(F("FM volume: "));
          Serial.println(fmvol, DEC);
        }
        break;
      }
    case 'q': {
        // Get FM station signal level (in decibels).
        flushSerial();
        Serial.print(F("FM Freq (eg 1011 == 101.1 MHz): "));
        uint16_t station = readnumber();
        Serial.println();
        int8_t level = fona.getFMSignalLevel(station);
        if (level < 0) {
          Serial.println(F("Failed! Make sure FM radio is on (tuned to station)."));
        } else {
          Serial.print(F("Signal level (dB): "));
          Serial.println(level, DEC);
        }
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

    case 'u': {
      // send a USSD!
      char message[141];
      flushSerial();
      Serial.print(F("Type out one-line message (140 char): "));
      readline(message, 140);
      Serial.println(message);

      uint16_t ussdlen;
      if (!fona.sendUSSD(message, replybuffer, 250, &ussdlen)) { // pass in buffer and max len!
        Serial.println(F("Failed"));
      } else {
        Serial.println(F("Sent!"));
        Serial.print(F("***** USSD Reply"));
        Serial.print(" ("); Serial.print(ussdlen); Serial.println(F(") bytes *****"));
        Serial.println(replybuffer);
        Serial.println(F("*****"));
      }
    }

    /*** Time ***/

    case 'y': {
        // enable network time sync
        if (!fona.enableNetworkTimeSync(true))
          Serial.println(F("Failed to enable"));
        break;
      }

    case 'Y': {
        // enable NTP time sync
        if (!fona.enableNTPTimeSync(true, F("pool.ntp.org")))
          Serial.println(F("Failed to enable"));
        break;
      }

    case 't': {
        // read the time
        char buffer[23];

        fona.getTime(buffer, 23);  // make sure replybuffer is at least 23 bytes!
        Serial.print(F("Time = ")); Serial.println(buffer);
        break;
      }


    /*********************************** GPS (SIM808 only) */

    case 'o': {
        // turn GPS off
        if (!fona.enableGPS(false))
          Serial.println(F("Failed to turn off"));
        break;
      }
    case 'O': {
        // turn GPS on
        if (!fona.enableGPS(true))
          Serial.println(F("Failed to turn on"));
        break;
      }
    case 'x': {
        int8_t stat;
        // check GPS fix
        stat = fona.GPSstatus();
        if (stat < 0)
          Serial.println(F("Failed to query"));
        if (stat == 0) Serial.println(F("GPS off"));
        if (stat == 1) Serial.println(F("No fix"));
        if (stat == 2) Serial.println(F("2D fix"));
        if (stat == 3) Serial.println(F("3D fix"));
        break;
      }

    case 'L': {
        // check for GPS location
        char gpsdata[120];
        fona.getGPS(0, gpsdata, 120);
        if (type == FONA808_V1)
          Serial.println(F("Reply in format: mode,longitude,latitude,altitude,utctime(yyyymmddHHMMSS),ttff,satellites,speed,course"));
        else 
          Serial.println(F("Reply in format: mode,fixstatus,utctime(yyyymmddHHMMSS),latitude,longitude,altitude,speed,course,fixmode,reserved1,HDOP,PDOP,VDOP,reserved2,view_satellites,used_satellites,reserved3,C/N0max,HPA,VPA"));
        Serial.println(gpsdata);

        break;
      }

    case 'E': {
        flushSerial();
        if (type == FONA808_V1) {
          Serial.print(F("GPS NMEA output sentences (0 = off, 34 = RMC+GGA, 255 = all)"));
        } else {
          Serial.print(F("On (1) or Off (0)? "));
        }
        uint8_t nmeaout = readnumber();

        // turn on NMEA output
        fona.enableGPSNMEA(nmeaout);

        break;
      }

    /*********************************** GPRS */

    case 'g': {
        // turn GPRS off
        if (!fona.enableGPRS(false))
          Serial.println(F("Failed to turn off"));
        break;
      }
    case 'G': {
        // turn GPRS on
        if (!fona.enableGPRS(true))
          Serial.println(F("Failed to turn on"));
        break;
      }
    case 'l': {
        // check for GSMLOC (requires GPRS)
        uint16_t returncode;

        if (!fona.getGSMLoc(&returncode, replybuffer, 250))
          Serial.println(F("Failed!"));
        if (returncode == 0) {
          Serial.println(replybuffer);
        } else {
          Serial.print(F("Fail code #")); Serial.println(returncode);
        }

        break;
      }
    case 'w': {
        // read website URL
        uint16_t statuscode;
        int16_t length;
        char url[80];

        flushSerial();
        Serial.println(F("NOTE: in beta! Use small webpages to read!"));
        Serial.println(F("URL to read (e.g. www.adafruit.com/testwifi/index.html):"));
        Serial.print(F("http://")); readline(url, 79);
        Serial.println(url);

        Serial.println(F("****"));
        if (!fona.HTTP_GET_start(url, &statuscode, (uint16_t *)&length)) {
          Serial.println("Failed!");
          break;
        }
        while (length > 0) {
          while (fona.available()) {
            char c = fona.read();

            // Serial.write is too slow, we'll write directly to Serial register!
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
            loop_until_bit_is_set(UCSR0A, UDRE0); /* Wait until data register empty. */
            UDR0 = c;
#else
            Serial.write(c);
#endif
            length--;
            if (! length) break;
          }
        }
        Serial.println(F("\n****"));
        fona.HTTP_GET_end();
        break;
      }

    case 'W': {
        // Post data to website
        uint16_t statuscode;
        int16_t length;
        char url[80];
        char data[80];

        flushSerial();
        Serial.println(F("NOTE: in beta! Use simple websites to post!"));
        Serial.println(F("URL to post (e.g. httpbin.org/post):"));
        Serial.print(F("http://")); readline(url, 79);
        Serial.println(url);
        Serial.println(F("Data to post (e.g. \"foo\" or \"{\"simple\":\"json\"}\"):"));
        readline(data, 79);
        Serial.println(data);

        Serial.println(F("****"));
        if (!fona.HTTP_POST_start(url, F("text/plain"), (uint8_t *) data, strlen(data), &statuscode, (uint16_t *)&length)) {
          Serial.println("Failed!");
          break;
        }
        while (length > 0) {
          while (fona.available()) {
            char c = fona.read();

#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
            loop_until_bit_is_set(UCSR0A, UDRE0); /* Wait until data register empty. */
            UDR0 = c;
#else
            Serial.write(c);
#endif

            length--;
            if (! length) break;
          }
        }
        Serial.println(F("\n****"));
        fona.HTTP_POST_end();
        break;
      }
    /*****************************************/

    case 'S': {
        Serial.println(F("Creating SERIAL TUBE"));
        while (1) {
          while (Serial.available()) {
            delay(1);
            fona.write(Serial.read());
          }
          if (fona.available()) {
            Serial.write(fona.read());
          }
        }
        break;
      }

    default: {
        Serial.println(F("Unknown command"));
        printMenu();
        break;
      }
  }
  }


}  // end loop()
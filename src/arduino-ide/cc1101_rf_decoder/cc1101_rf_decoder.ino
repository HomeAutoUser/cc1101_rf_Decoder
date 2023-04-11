/*
  Copyright (c) 2023, HomeAutoUser & elektron-bbs
  All rights reserved.

              pos | neg     H         H
                 ---       ---       ---                         ---
                |   |     |   |  L  |   |  L      Pause max     |   |
             ----   -------   -------   -------------------------   ------
   Counter:  0  1   2     3   4     5   6                   0   1   2

  Arduino Nano
  cc1101_rf_signal_Decoder_with_GDO_v18_digitalWriteFast:
  Der Sketch verwendet 8088 Bytes (26%) des Programmspeicherplatzes. Das Maximum sind 30720 Bytes.
  Globale Variablen verwenden 742 Bytes (36%) des dynamischen Speichers, 1306 Bytes für lokale Variablen verbleiben. Das Maximum sind 2048 Bytes.

  cc1101_rf_signal_Decoder_with_GDO_v16:
  Der Sketch verwendet 8174 Bytes (26%) des Programmspeicherplatzes. Das Maximum sind 30720 Bytes.
  Globale Variablen verwenden 742 Bytes (36%) des dynamischen Speichers, 1306 Bytes für lokale Variablen verbleiben. Das Maximum sind 2048 Bytes.
  ohne MsgData.reserve(255):
  Der Sketch verwendet 8162 Bytes (26%) des Programmspeicherplatzes. Das Maximum sind 30720 Bytes.
  Globale Variablen verwenden 742 Bytes (36%) des dynamischen Speichers, 1306 Bytes für lokale Variablen verbleiben. Das Maximum sind 2048 Bytes.

  ESP8266
  . Variables and constants in RAM (global, static), used 29456 / 80192 bytes (36%)
  ║   SEGMENT  BYTES    DESCRIPTION
  ╠══ DATA     1536     initialized variables
  ╠══ RODATA   1080     constants
  ╚══ BSS      26840    zeroed variables
  . Instruction RAM (IRAM_ATTR, ICACHE_RAM_ATTR), used 60723 / 65536 bytes (92%)
  ║   SEGMENT  BYTES    DESCRIPTION
  ╠══ ICACHE   32768    reserved space for flash instruction cache
  ╚══ IRAM     27955    code in IRAM
  . Code in flash (default, ICACHE_FLASH_ATTR), used 241644 / 1048576 bytes (23%)
  ║   SEGMENT  BYTES    DESCRIPTION
  ╚══ IROM     241644   code in flash

  https://forum.arduino.cc/t/how-exactly-slow-is-digitalread/325458/3
*/

/* Definitions for program code */
#define CC110x                    // ohne CC110x auskommentieren
//#define DEBUG           1         // zum aktivieren der Debugausgaben
//#define ReceiveMhz868   1         // Empfang auf 868, sonst 433 Mhz

/* Definitions END */
#include <Arduino.h>
#include "SimpleFIFO.h"
#include <digitalWriteFast.h>       // https://github.com/ArminJo/digitalWriteFast

#ifdef CC110x
#include <SPI.h>
#include "cc110x.h"
#endif

#define MsgLenMin         24        // Nachricht Mindestlänge
#define MsgLenMax         254       // Nachricht Maximallänge
#define PatMaxCnt         8         // Pattern, maximale Anzahl (Anzahl 8 -> FHEM SIGNALduino kompatibel)
#define PatTol            0.20      // Patterntoleranz

#define FIFO_LENGTH       160       // 90 von SIGNALduino FW
SimpleFIFO<int, FIFO_LENGTH> FiFo;  // store FIFO_LENGTH # ints

const char compile_date[]       = __DATE__ " " __TIME__;
#ifdef CC110x
static const char TXT_VERSION[] = "V 0.19 SIGNALduino compatible cc1101_rf_Decoder - compiled at ";
#else
static const char TXT_VERSION[] = "V 0.19 SIGNALduino compatible rf_Decoder - compiled at ";
#endif
bool valid;

int t_maxP = 32000;             // Zeitdauer maximum für gültigen Puls in µs
int t_minP = 75;                // Zeitdauer minimum für gültigen Puls in µs
unsigned long lastTime = 0;     // Zeit, letzte Aktion

int ArPaT[PatMaxCnt];           // Pattern Array für Zeiten
signed long ArPaSu[PatMaxCnt];  // Pattern Summe, aller gehörigen Pulse
byte ArPaCnt[PatMaxCnt];        // Pattern Counter, der Anzahl Pulse
byte PatNmb = 0;                // Pattern aktuelle Nummer 0 - 9
byte MsgLen;                    // Todo, kann durch message.valcount ersetzt werden
int first;                      // Pointer to first buffer entry
int last;                       // Pointer to last buffer entry

String MsgData;
byte TiOv = 0;                  // Marker - Time Overflow (SIGNALduino Kürzel p; )
byte PatMAX = 0;                // Marker - maximale Pattern erreicht und neuer unbekannter würde folgen (SIGNALduino Kürzel e; )

char input[50];                 // input from Serial/telnet
int count;                      // count bytes input from Serial/telnet

/* predefinitions of the functions */
inline void doDetect();
void MSGBuild();
void PatReset();
void decode(const int pulse);
void findpatt(int val);

/* alle definierten pin´s der Boards */
#ifdef ARDUINO_ARCH_ESP32         /* ESP32 */
#define PIN_RECEIVE   13            // D13 | G13
#define PIN_SEND      4             // D4  | G4
#define LED           2             // LED => ESP32 (OK msg & WIFI)
#define SerialSpeed   115200
#elif ARDUINO_ARCH_ESP8266        /* ESP8266 */
#define PIN_RECEIVE   5             // D1
#define PIN_SEND      4             // D2
#define LED           16            // LED => ESP8266 (OK msg & WIFI)
#define SerialSpeed   57600
//#define SerialSpeed   115200
//#define SerialSpeed   921600
#include <ESP8266WiFi.h>          /* ESP8266 | need for ESP8266 function system_get_free_heap_size (include failed, so separate) */
#elif ARDUINO_RADINOCC1101        /* radinoCC1101 */
#define PIN_RECEIVE   7             // GDO2     => Radino (Pin RX in)
#define PIN_SEND      9             // GDO0     => Radino (Pin TX out)
#define LED           13            // LED      => Radino (OK msg)
#define digitalPinToInterrupt(p) ((p) == 0 ? 2 : ((p) == 1 ? 3 : ((p) == 2 ? 1 : ((p) == 3 ? 0 : ((p) == 7 ? 4 : NOT_AN_INTERRUPT)))))
#define PIN_MARK433   4
#define SS            8
#define SerialSpeed   57600
#else                             /* Arduino Nano */
#define PIN_RECEIVE   2             // GDO2     => RX
#define PIN_SEND      3             // GDO0     => TX
#define LED           9             // LED      => Arduino Nano (OK msg)
#define SerialSpeed 57600
//#define SerialSpeed   1000000
#endif


#if defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_ESP8266)
IRAM_ATTR void Interupt() {     /* Pulseauswertung */
#else
void Interupt() {
#endif
  const unsigned long Time = micros();
  const signed long duration = Time - lastTime;
  lastTime = Time;
  if (duration >= t_minP) {         // kleinste zulässige Pulslänge
    int sDuration;
    if (duration < t_maxP) {        // größte zulässige Pulslänge, max = 32000
      sDuration = int(duration);    // das wirft bereits hier unnötige Nullen raus und vergrössert den Wertebereich
    } else {
      sDuration = t_maxP;           // Maximalwert
    }
    if (digitalReadFast(PIN_RECEIVE)) { // Wenn jetzt high ist, dann muss vorher low gewesen sein, und dafuer gilt die gemessene Dauer.
      sDuration = -sDuration;
    }
    FiFo.enqueue(sDuration);        // add an sDuration
  } // else => trash
}


void setup() {
  MsgData.reserve(255); // neu
  Serial.begin(SerialSpeed);
  Serial.println(F("Serial, OK"));
  pinModeFast(LED, OUTPUT);
  digitalWriteFast(LED, HIGH);  // LED on

#ifdef CC110x
  pinMode(SS, OUTPUT);                  /* CC1101 - Init START */
  delay(50);
  SPI.begin();
  SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));  // SCLK frequency, burst access max. 6,5 MHz
  digitalWrite(SS, HIGH);               // Deselect (SPI) CC1101
  delayMicroseconds(10);
  digitalWrite(SS, LOW);                // Select (SPI) CC1101
  delayMicroseconds(10);
  digitalWrite(SS, HIGH);               // Deselect (SPI) CC1101
  delayMicroseconds(40);
  digitalWrite(SS, LOW);                // Select (SPI) CC1101
  while (digitalRead(MISO) > 0);        // Wait until SPI MISO line goes low
  CC1101_cmdStrobe(0x30);               // Reset CC1101 chip
  delay(125);
  uint8_t chipVersion = CC1101_readReg(0x31, 0xC0);
  if (chipVersion == 0x14 || chipVersion == 0x04 || chipVersion == 0x03 || chipVersion == 0x05 || chipVersion == 0x07 || chipVersion == 0x17) {
#ifdef ReceiveMhz868
    CC1101_writeRegFor(Registers[1].reg_val, Registers[1].length);
#else
    CC1101_writeRegFor(Registers[0].reg_val, Registers[0].length);
#endif
    CC1101_cmdStrobe(0x3A);             // Flush the RX FIFO buffer. Only issue SFRX in IDLE or RXFIFO_OVERFLOW states
    CC1101_cmdStrobe(0x34);             // Enable RX. Perform calibration first if coming from IDLE and MCSM0.FS_AUTOCAL=1
#endif // ENDE ifdef CC110x
    delay(125);
    pinMode(PIN_RECEIVE, INPUT_PULLUP);   /* Lege den Interruptpin als Inputpin mit Pullupwiderstand fest */
    attachInterrupt(digitalPinToInterrupt(PIN_RECEIVE), Interupt, CHANGE); /* "Bei wechselnder Flanke auf dem Interruptpin" --> "Führe die Interupt Routine aus" */
#ifdef CC110x
    Serial.println(F("CC110x OK, Interrupt OK"));
#else
    Serial.println(F("without CC110x, Interrupt OK"));
#endif // Ende ifdef CC110x
#ifdef CC110x
  } else {
    Serial.println(F("CC110x nOK, Interrupt nAktiv"));
  } /* CC1101 - Init END */
#endif // Ende ifdef CC110x
  digitalWriteFast(LED, LOW);  // LED off
}


void loop() {
  while (FiFo.count() > 0 ) {         // Puffer auslesen und an Dekoder uebergeben
    int aktVal = FiFo.dequeue();      // get next element
    decode(aktVal);
  }

  const unsigned long t_now = micros();
  const signed long t_dif = t_now - lastTime;
  if ( t_dif > t_maxP ) {
    TiOv = 1;
    MSGBuild();
  }

  while (Serial.available() && Serial.availableForWrite() > 0) {
    char ch = Serial.read();
    String sResponse = "";  // Response
    if (ch != 10 && ch != 13) {
      if (count < 50 - 1) {
        input[count++] = ch;
      } else {
        Serial.flush();     // clear input buffer, else you get strange characters
        input[0] = '\0';    // clear input
        count = 0;
      }
    } else {
      input[count++] = '\0';
      switch (input[0]) {
        case 'C':
          if (input[1] == 'G') {  //  get config
            Serial.print(F("MS=0;MU=1;MC=0;Mred=0\n"));
          }
          break;
        case 'P':  // Ping
          Serial.print(F("OK\n"));
          break;
        case 'R':  // free RAM
#if defined (ARDUINO_AVR_NANO) || defined (ARDUINO_RADINOCC1101) || defined (ARDUINO_AVR_PRO)
          extern int __heap_start, *__brkval;
          int v;
          Serial.print( (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval) );
#endif
#ifdef ARDUINO_ARCH_ESP8266
          Serial.print( system_get_free_heap_size() );
#endif
#ifdef ARDUINO_ARCH_ESP32
          Serial.print( ESP.getFreeHeap() );
#endif
          Serial.print(F("\n"));
          break;
        case 't':  // Uptime
          Serial.print(12345);
          Serial.print(F("\n"));
          break;
        case 'V':  // Version
          Serial.print(TXT_VERSION); Serial.print(compile_date); Serial.print("\n");
          break;
        default:
          // Statement(s)
          break;  // Wird nicht benötigt, wenn Statement(s) vorhanden sind
      }
      input[0] = '\0';  // clear input
      count = 0;
    }
  }
}

void decode(const int pulse) {    /* Pulsübernahme und Weitergabe */
  if (MsgLen > 0) {
    last = first;
  } else {
    last = 0;
  }
  first = pulse;
  doDetect();
}

inline void doDetect() {      /* Pulsprüfung und Weitergabe an Patternprüfung */
  valid = (MsgLen == 0 || last == 0 || (first ^ last) < 0);   // true if a and b have opposite signs
  valid &= (MsgLen == MsgLenMax) ? false : true;
  valid &= ( (first > -t_maxP) && (first < t_maxP) );         // if low maxPulse detected, start processMessage()
#ifdef DEBUG
  Serial.print(F("PC:")); Serial.print(PatNmb); Serial.print(F(" ML:")); Serial.print(MsgLen); Serial.print(F(" v:")); Serial.print(valid);
  Serial.print(F(" | ")); Serial.print(first); Serial.print(F("    ")); Serial.println(last);
#endif
  if (valid) {
    findpatt(first);
  } else {
#ifdef DEBUG
    Serial.println(F("-- RESET --"));
#endif
    MSGBuild();
  }
}

void MSGBuild() {     /* Nachrichtenausgabe */
  if (MsgLen >= MsgLenMin) {
    digitalWriteFast(LED, HIGH);  // LED on
    uint8_t CP_PaNum = 0;
    int16_t PulseAvgMin = 32767;
    String msg = "";
#ifdef CC110x
    uint8_t rssi = CC1101_readReg(0x34, 0xC0); // not converted
#endif
    msg += char(2); msg += F("MU");
    for (uint8_t i = 0; i <= PatNmb; i++) {
      int16_t PulseAvg = ArPaSu[i] / ArPaCnt[i];
      msg += F(";P"); msg += i; msg += F("="); msg += PulseAvg;
      // search Clockpulse (CP=) - das funktioniert noch nicht richtig! --> kein richtiger Einfall ;-) TODO
      if (ArPaSu[i] > 0) { // HIGH-Pulse
        if (PulseAvg < PulseAvgMin) { // kürzeste Zeit
          PulseAvgMin = PulseAvg;     // kürzeste Zeit übernehmen
          CP_PaNum = i;
        }
        if (ArPaCnt[i] > ArPaCnt[CP_PaNum]) {
          CP_PaNum = i;               // ClockPulse übernehmen
        }
      }
    }
    msg += F(";D="); msg += MsgData; msg += F(";CP="); msg += CP_PaNum;
#ifdef CC110x
    msg += F(";R="); msg += rssi;
#endif
    msg += ';';
    if (MsgLen == MsgLenMax) {  /* max. Nachrichtenlänge erreicht */
      msg += F("O;");
    } else if (TiOv != 0) {     /* Timeoverflow größer 32000 -> Zwangstrennung */
      msg += F("p;");
    } else if (PatMAX == 1) {   /* max. Pattern erreicht und neuer unbekannter würde folgen */
      msg += F("e;");
    }
    msg += F("w="); msg += valid; msg += ';';    /* letzter Puls zu vorherigen Puls msg valid bzw. unvalid /  */
    msg += char(3); msg += char(10);
    Serial.print(msg);
    digitalWriteFast(LED, LOW);  // LED off
  }
  PatReset();
}

void findpatt(int val) {      /* Patterneinsortierung */
  for (uint8_t i = 0; i < PatMaxCnt; i++) {
    if (MsgLen == 0) {  /* ### nach RESET ### */
      MsgData = i;
      ArPaCnt[i] = 1;
      ArPaT[i] = val;
      ArPaSu[i] = val;
      MsgLen++;
#ifdef DEBUG
      Serial.print(i); Serial.print(F(" | ")); Serial.print(ArPaT[i]); Serial.print(F(" msgL0: ")); Serial.print(val);
      Serial.print(F(" l: ")); Serial.print(last); Serial.print(F(" PatN: ")); Serial.print(PatNmb); Serial.print(F(" msgL: ")); Serial.print(MsgLen); Serial.print(F(" Fc: ")); Serial.println(FiFo.count());
#endif
      break;
      /* ### in Tolleranz und gefunden ### */
    } else if ( (val > 0 && val > ArPaT[i] * (1 - PatTol) && val < ArPaT[i] * (1 + PatTol)) ||
                (val < 0 && val < ArPaT[i] * (1 - PatTol) && val > ArPaT[i] * (1 + PatTol)) ) {
      MsgData += i;
      ArPaCnt[i]++;
      ArPaSu[i] += val;
      MsgLen++;
#ifdef DEBUG
      Serial.print(i); Serial.print(F(" | ")); Serial.print(ArPaT[i]); Serial.print(F(" Pa T: ")); Serial.print(val);
      Serial.print(F(" l: ")); Serial.print(last); Serial.print(F(" PatN: ")); Serial.print(PatNmb); Serial.print(F(" msgL: ")); Serial.print(MsgLen); Serial.print(F(" Fc: ")); Serial.println(FiFo.count());
#endif
      break;
    } else if (i < (PatMaxCnt - 1) && ArPaT[i + 1] == 0 ) { /* ### nächste freie Pattern ### */
      MsgData += i + 1;
      PatNmb++;
      ArPaCnt[i + 1]++;
      ArPaT[i + 1] = val;
      ArPaSu[i + 1] += val;
      MsgLen++;
#ifdef DEBUG
      Serial.print(i); Serial.print(F(" | ")); Serial.print(ArPaT[i]); Serial.print(F(" Pa f: ")); Serial.print(val);
      Serial.print(F(" l: ")); Serial.print(last); Serial.print(F(" PatN: ")); Serial.print(PatNmb); Serial.print(F(" msgL: ")); Serial.print(MsgLen); Serial.print(F(" Fc: ")); Serial.println(FiFo.count());
#endif
      break;
    } else if (i == (PatMaxCnt - 1)) {  /* ### Anzahl vor definierter Pattern ist erreicht ### */
#ifdef DEBUG
      Serial.print(F("PC max! | MsgLen: ")); Serial.print(MsgLen); Serial.print(F(" | MsgLenMin: ")); Serial.println(MsgLenMin);
#endif
      PatMAX = 1;
      MSGBuild();
      break;
    }
  }
}

void PatReset() {     /* Zurücksetzen nach Nachrichtenbau oder max. Länge */
  MsgData = "";
  MsgLen = 0;
  PatMAX = 0;
  PatNmb = 0;
  TiOv = 0;

  for (uint8_t i = 0; i < PatMaxCnt; i++) {
    ArPaCnt[i] = 0;
    ArPaSu[i] = 0;
    ArPaT[i] = 0;
  }
}

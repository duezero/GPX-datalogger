/*
    GPX datalogger v1.1   7-10-2018 by DoppioZero
    basic gpx datalogger

    -v1.1 memory optimization for use with ATmega328p
    -v1.0 first release

   -buttons must have hardware debounce (to GND)
   -data cannot be stored on SDcard until DATA LED starts blink
   -SD card is initialised ONLY after power-on
   -if SDcard is not present before power on, SD LED will blink fast until SDcard is inserted


  --MY SETUP--
   ATMEGA328p or ATMEGA2560 or ATMEGA1284p
   GPS NEO6M
   MICROSD CAR READER
*/

#include <SPI.h>
#include <SD.h>
#include <TinyGPS++.h>

/************ enable/disable serial debug output ************/
//#define DEBUG
/************************************************************/

#ifdef DEBUG
#define DEBUG_PRINT(x) Serial.print(x)
#define DEBUG_PRINTDEC(x) Serial.print(x, DEC)
#define DEBUG_PRINTLN(x) Serial.println(x)
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTDEC(x)
#define DEBUG_PRINTLN(x)
#endif

/********** ENABLE PROMIDI1284P OR MEGA2560 OR UNO328P **********/
//#define PROMIDI
//#define MEGA2560
#define UNO328P
/****************************************************************/

#ifdef PROMIDI
#define CHIPSELECT 4
#define DATALEDNUM 0
#define SDLEDNUM 1
#define STARTPIN 20
#define STOPPIN 19
#define SERIALX_BEGIN(x)  Serial1.begin(x)
#define SERIALX_AVAILABLE() Serial1.available()
#define SERIALX_READ() Serial1.read()
#endif

#ifdef MEGA2560
#define CHIPSELECT 53 // MEGA2560
#define DATALEDNUM 7
#define SDLEDNUM 8
#define STARTPIN 17
#define STOPPIN 15
#define SERIALX_BEGIN(x)  Serial1.begin(x)
#define SERIALX_AVAILABLE() Serial1.available()
#define SERIALX_READ() Serial1.read()
#endif

#ifdef UNO328P
#define CHIPSELECT 10// UNO328P
#define DATALEDNUM 7
#define SDLEDNUM 8
#define STARTPIN 2
#define STOPPIN 3
#define SSRXPIN 4
#define SSTXPIN 5
#include <SoftwareSerial.h>
#define SERIALX_BEGIN(x)  ss.begin(x)
#define SERIALX_AVAILABLE() ss.available()
#define SERIALX_READ() ss.read()
#endif

uint8_t chipSelect = CHIPSELECT;
byte DATALED = DATALEDNUM; // valid data LED -- GREEN OR BLUE
bool DATALEDstatus = 0;
byte SDLED = SDLEDNUM; // SD writing LED -- RED
bool SDLEDstatus = 0;
char trackName[16] = "1-track.gpx";  //file name
char TrckNAME[28] = "";
//char TrckNUM[25] = "";
char TrckDATA[47] = "";
char TrckTIME[34] = "";
char LatSIGN = '+';
char LngSIGN = '+';
byte i = 2; // track name index
bool openLog = false;
bool closeLog = false;
byte startPIN = STARTPIN;  //start logging button
byte stopPIN = STOPPIN;    //stop logging button
bool startLog = false;
bool stopLog = false;
byte gpsSecond = 0;
unsigned long int LAT = 0;
unsigned long int LONG = 0;
unsigned long int LATint = 0;
unsigned long int LATdec = 0;
unsigned long int LONGint = 0;
unsigned long int LONGdec = 0;
int yr = 0; //year
float elev = 0; // altitude
float floatLAT, floatLONG, previousLAT, previousLONG;
bool POINTdist = true;
bool validDATA = false;
byte mth, dy, hr, mn, sc = 0;
unsigned long int previousMillis = 0;
unsigned long int previousMillisDATALED = 0;
unsigned long int previousMillisSDLED = 0;

File myTracks;
// The TinyGPS++ object
TinyGPSPlus gps;

#ifdef UNO328P
const byte RXPin = SSRXPIN, TXPin = SSTXPIN;
SoftwareSerial ss(RXPin, TXPin);
#endif

void setup() {
  pinMode(startPIN, INPUT_PULLUP);  //buttons with hardware debounce
  pinMode(stopPIN, INPUT_PULLUP);
  pinMode(DATALED, OUTPUT);
  pinMode(SDLED, OUTPUT);

  // Open serial communications and wait for port to open:
  Serial.begin(115200);
  delay(500);
  SERIALX_BEGIN(9600);
  delay(500);
  DEBUG_PRINT("Initializing SD card...");

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    DEBUG_PRINTLN("Card failed, or not present");
    while (!SD.begin(chipSelect)) {
      digitalWrite(SDLED, HIGH);
      delay(50);
      digitalWrite(SDLED, LOW);
      delay(50);
      digitalWrite(SDLED, HIGH);
      delay(50);
      digitalWrite(SDLED, LOW);
      delay(50);
      digitalWrite(SDLED, HIGH);
      delay(50);
      digitalWrite(SDLED, LOW);
      delay(50);
    }
  }
  else DEBUG_PRINTLN("card initialized.");

}

void loop() {
  /* GPS READING */
  while (SERIALX_AVAILABLE() > 0) {
    gps.encode(SERIALX_READ());
    elev = gps.altitude.meters();
  }
  /*READING START STOP LOG BUTTONS*/
  if (digitalRead(startPIN) == LOW && startLog == false && validDATA == true ) {
    startLog = true;
    openLog = true;
    DEBUG_PRINTLN("startLog");
  }
  if (digitalRead(stopPIN) == LOW && closeLog == false && startLog == true) {
    closeLog = true;
    DEBUG_PRINTLN("stopLog");
  }
  /* READING ONCE PER SECOND GPS DATA*/
  if (gps.time.second() != gpsSecond) {
    yr = gps.date.year();
    mth = gps.date.month();
    dy = gps.date.day();
    hr = gps.time.hour();
    mn = gps.time.minute();
    sc = gps.time.second();
    floatLAT = gps.location.lat();
    floatLONG = gps.location.lng();
    if (gps.location.rawLat().negative) {
      LatSIGN = '-';
    }
    else LatSIGN = '+';
    if (gps.location.rawLng().negative) {
      LngSIGN = '-';
    }
    else LngSIGN = '+';
    /* EXTRACTING INTEGER PART AND DECIMAL PART FROM FLOAT LONG AND LAT */
    if (floatLAT >= 0) {
      LAT = floatLAT * 1000000;
    }
    else  LAT = floatLAT * 1000000 * -1;
    if (floatLONG >= 0) {
      LONG = floatLONG * 1000000;
    }
    else  LONG = floatLONG * 1000000 * -1;
    LATint = LAT / 1000000; // integer part
    LATdec = LAT - LATint * 1000000; //decimal part
    LONGint = LONG / 1000000;
    LONGdec = LONG - LONGint * 1000000;
    DEBUG_PRINTLN(gps.hdop.hdop());
    /* CHECK IF DISTANCE BETWEEN TWO POINT IS MORE THAN 4 METERS */
    if (TinyGPSPlus::distanceBetween(floatLAT, floatLONG, previousLAT, previousLONG) > 4) {
      POINTdist = true;
      previousLONG = floatLONG;
      previousLAT = floatLAT;
    }
    else POINTdist = false;
    /* CREATING XML STRINGS */
    //sprintf(TrckDATA, "<trkpt lon=\"%c%02ld.%6ld\" lat=\"%c%02ld.%6ld\" ele=\"%d\">", LngSIGN, LONGint, LONGdec, LatSIGN, LATint, LATdec, (int)elev);
    //sprintf(TrckTIME, "<time>%d-%02d-%02dT%02d:%02d:%02dZ</time>", yr, mth, dy, hr, mn, sc);
    sprintf(TrckDATA, "%c%02ld.%6ld\" lat=\"%c%02ld.%6ld\" ele=\"%d\">", LngSIGN, LONGint, LONGdec, LatSIGN, LATint, LATdec, (int)elev);
    sprintf(TrckTIME, "%d-%02d-%02dT%02d:%02d:%02dZ</time>", yr, mth, dy, hr, mn, sc);
    /* CHECK IF ALL GPS DATA ARE VALID */
    if (gps.hdop.isValid() && (gps.hdop.hdop() <= 10) && gps.location.isValid() && gps.altitude.isValid()) {
      DEBUG_PRINTLN(TrckDATA);
      DEBUG_PRINTLN(TrckTIME);
      validDATA = true;
    }
    else validDATA = false;
    /* START LOGGING*/
    if (startLog == true) {
      GPSlog();
    }
    gpsSecond = gps.time.second();
  }
  if (validDATA == true) {
    blinkDATALED();
  }
  if (startLog == true) {
    blinkSDLED();
  }
}


void GPSlog() {

  if (openLog == true &&  validDATA == true) {

    /* LOOK FOR EXISTING FILE */
    while (SD.exists(trackName)) { // until name doesnt exist
      sprintf(trackName, "%i-track.gpx", i);
      i += 1;
    }

    /* INDEX FILE GPX */
    myTracks = SD.open(trackName, FILE_WRITE);
    if (myTracks) {
      myTracks.println(F("<?xml version=\"1.0\" encoding=\"ISO-8859-1\" standalone=\"no\" ?>")); //  In a C/C++ string, you can "escape" a special character by preceding it with a backslash \ eg. Serial.println("\""); // prints "
      myTracks.println(F("<gpx version=\"1.0\" creator=\"doppiozero\">"));
      myTracks.close();
    }
    // if the file isn't open, pop up an error:
    else {
      DEBUG_PRINTLN(F("err opn index"));
    }

    /* NEW TRACK */
    myTracks = SD.open(trackName, FILE_WRITE);
    if (myTracks) {
      //sprintf(TrckNAME, "<trk name = \"Track - %02d/%02d/%04d %02d:%02d:%02d\">", dy, mth, yr, hr, mn, sc);
      sprintf(TrckNAME, "%02d/%02d/%04d %02d:%02d:%02d\">", dy, mth, yr, hr, mn, sc);
      // sprintf(TrckNUM, "<number>-%d</number>", i - 1);
      myTracks.print(F("<trk name = \"Track - "));
      myTracks.println(TrckNAME);
      myTracks.print(F("<number>-"));
      myTracks.print(i - 1);
      myTracks.println(F("</number>"));
      myTracks.println(F("<trkseg>"));
      myTracks.close();
    }
    // if the file isn't open, pop up an error:
    else {
      DEBUG_PRINTLN(F("err opn newtrk"));
    }
    openLog = false; // create file only once, must stop logging to create another file
  }
  /* LOG SINGLE POINT*/
  if (closeLog == false && POINTdist == true  &&  validDATA == true) {
    myTracks = SD.open(trackName, FILE_WRITE);
    if (myTracks) {
      myTracks.print(F("<trkpt lon=\""));
      myTracks.println(TrckDATA);
      myTracks.print(F("<time>"));
      myTracks.println(TrckTIME);
      myTracks.println(F("</trkpt>"));
      myTracks.close();
    }
    else {
      DEBUG_PRINTLN(F("err log pnt"));
    }
  }
  if (closeLog) { // if true, program should close file
    /*CLOSE TRACK AND FILE */
    myTracks = SD.open(trackName, FILE_WRITE);
    if (myTracks) {
      myTracks.println(F("</trkseg>"));
      myTracks.println(F("</trk>"));
      myTracks.println(F("</gpx>"));
      myTracks.close();
      closeLog = false;
      startLog = false; // cannot enter in GPSlog()
    }
    else {
      DEBUG_PRINTLN(F("err cls file"));
    }
    /* END */
    DEBUG_PRINTLN(trackName);
    DEBUG_PRINTLN(F("next free log index = "));
    DEBUG_PRINTLN(i);
    DEBUG_PRINTLN(F("TEST COMPLETED"));
  }

}

void blinkDATALED() {

  if (millis() - previousMillisDATALED > 1000 && DATALEDstatus == 0) {
    digitalWrite(DATALED, HIGH);
    previousMillisDATALED = millis();
    DATALEDstatus = 1;
  }
  if (millis() - previousMillisDATALED > 50 && DATALEDstatus == 1) {
    digitalWrite(DATALED, LOW);
    DATALEDstatus = 0;
  }
}

void blinkSDLED() {

  if (millis() - previousMillisSDLED > 1000 && SDLEDstatus == 0) {
    digitalWrite(SDLED, HIGH);
    previousMillisSDLED = millis();
    SDLEDstatus = 1;
  }
  if (millis() - previousMillisSDLED > 50 && SDLEDstatus == 1) {
    digitalWrite(SDLED, LOW);
    SDLEDstatus = 0;
  }
}




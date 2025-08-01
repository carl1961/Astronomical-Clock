// this code is written by illusionmanager in the year 2024
// it is for the eclipse spinner (astronomical clock)
// software version 1.04

// you'll need to program the MYSSID and MYPASSKEY to match your network credentials
// as well as the longitude and lattitude and timezone
//#define MYSSID ""
//#define MYPASSKEY ""


// pick timezone from TZ.h (https://github.com/esp8266/Arduino/blob/master/cores/esp8266/TZ.h)
//#define TIMEZONE "CET-1CEST,M3.5.0,M10.5.0/3"
//#define TZ_America_Chicago	PSTR("CST6CDT,M3.2.0,M11.1.0")
#define TIMEZONE "CST6CDT,M3.2.0,M11.1.0"
//LONGITUDE -90.4345  LATTITUDE 30.5806 https://www.mooncalc.org/ https://www.moongiant.com/calendar/
#define LONGITUDE -90.43  //6.57
#define LATTITUDE 30.58   //53.22

// This must be my ugliest code I've ever written. Sorry for that.

// in the main loop there is commented out testroutine.
// if you enable that the clock should rotate and move everything at 90 degrees angle
// from eachother. Perhaps comment out the print command in the rotate routine as well and use
// #define NOWIFI so it doesn't waste time getting data from NASA.
// if it doesn't show the planets exactly at 90 degrees you need to calibrate the numbers stored in touchValues.
// (( however if they are aligned, but more in a 60 and 120 degree pattern, you have mounted the sunrise pointer
// or time wheel on the wrong site of the tabs on the sunrise wheel))
// Calibrate by first hitting the 'r' key to reset the clock
// enter the value close the one (but a little less) currently in touchValues[0].
// it should move mercury to exacly 0 degrees. But as you entered a slightly less value,
// rotate a bit more by entering -1 (it rotates 1 degree more). Note that when Mercury is at 0 degrees,
// the other planets are not because of the thickness of the tabs. When during the testrouting all the planets seem to
// be rotated by the same amount, this value for mercury is the one to change.
// When mercury is exactly at 0 degrees add the total rotation you did together and store that
// into touchValues[0] (should be a negative rotation!)
// Next is Venus, it needs to know how much to rotate in the oposite direction until it is just about to
// start rotating Venus. It also needs to compensate a little for the thickness. Those values are stored in
// offsetValues. So rotate a fraction more to set Venus in the 0 orientation and set that value in offsetValues, but
// for now, those values are based on the design, and should be almost perfect, so you can leave them alone, but you might need
// to adjust them.

//  -1075 puts sunset directly opposite sunrise; rotate a bit further to set the time difference between sunset and sunrise
// 880 puts sunrise at 00:00 ; rotate a bit further to set sunrise
// -697 brings moon set to 00:00; rotate a bit further to set moonset
//
// 360 starts pushing moon rise pointers. one is at 10 degrees (1h33') before moonset. pushing it further moves it back towards 00:00
// so move it back by the difference between moonrise and moonset - 10 degrees, keeping in mind the 00:00 crossing
// if it passes 0 the other moonrise pointer takes over at 24:00
// At this point the time shown is determined by the moonphase, which also sets the final value of orient

float touchValue[] = { -341, 4107, -3750, 3395, -3038, 2677, -2319, 1959, -1602, 1247.5, -1076, 883, -697.5, 361.5 };
float offsetValue[] = { -3.1, 2.2, -1.78, 1.3, -0.8, 0.7, -0.65, 1.5, 0.5, 3 };
// motor appears a bit stronger with more microsteps, so I use 32 instead of 8
#define MICRO_STEPS 32
// for some the 28BYJ-48 has ratio 64*403/405 = 63.6839506
// multiply that by 32 results in 2037.88642 steps to go around once.
// but mine apparently does need exactly 2048 steps to go around.
// It varies, so check your motor
// multiplied by the gear ratio in this clock 80/22 results in 7447.272727 full steps
// for the clock to go around once
// clock has ratio 80/22 = 3.636363
// 7447.27272727 steps to go around once
#define STEPS_PER_DEGREE 20.686868
#define DEGREE_PER_STEP 0.048339843


// should have been 180, but on my clock it is slightly more. Probably has to do with the thickness of the clockface
#define DEGREE_PER_HOUR (181.0 / 24)

// ZERO_OFFSET should be positive
#define ZERO_OFFSET (189.2f)
#define DIR_PIN 5
#define STEP_PIN 6
#define ENABLE_PIN 7
#define LED_PIN 8

// stop_pin connected to #9 as this has an internal pull-up resistor.
// also to be used for triggering boot mode.
// if you need to "press" the boot switch to load a program, just rotate the sunrise wheel
// so the magnet is exactly at the back and power on. If you want to run the program, make
// sure the magnet isn't exactly at the back during power on.
#define STOP_PIN 9

// you might need to reduce the MAX_SPEED if you motor cannot handle the speed.
#define MAX_SPEED 500
// even though there is no mechanical reason not to go faster, somehow the free spinning pointers and time scale
// do move when going faster.
#define PRECISE_SPEED 400
#define ACCELERATION 150

#define SERIAL_BAUD_RATE 115200
#define RUN_CURRENT_PERCENT 100

// set this close to 0 in production version
#define SHORT_DELAY 2500

//#define NOWIFI
#define PLANETSPINNER "       Eclipse Spinner\n\n       by illusionmanager\n (youtube.com/@illusionmanager)\n\n"

#include <Arduino.h>
#include <TMC2209.h>
#include <WiFiClientSecure.h>
//#include <ESPAsyncWebSrv.h>
#include <time.h>
#include <AccelStepper.h>
// AP
#include <WiFiManager.h>  // https://github.com/tzapu/WiFiManager

WiFiManager wifiManager;

// TFT 1.8 ILI9341

#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <Fonts/FreeMono9pt7b.h>
#include <Fonts/FreeMono12pt7b.h>

#define TFT_RST 0
#define TFT_DC 1
#define TFT_SCLK 2
#define TFT_CS 3
#define TFT_MOSI 4


Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);



AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

// UART connection to controller
HardwareSerial& serial_stream = Serial0;

// Instantiate TMC2209
TMC2209 stepper_driver;

// stores the absolute orientation of the mercury, which drives the rest
float orient = 0;

bool debug = false;
// for no really good reason all the relevant status info is stored in global variables.

// holds the orientation of each planet;
float planet[] = { 90, 180, 270, 90, 180, 270, 90, 180, 270 };
String planetName[] = { "Mercury", "Venus", "Earth", "Mars", "Jupiter", "Saturn", "Uranus", "Neptune" };
float sunrise, sunset, moonrise, moonset;
float moonphase, moonperifocus;  // perifocus is the position of the moon in its oribt measured for the ascending node
// next variables hold the current local time.
int year;
uint8_t month;
uint8_t day;
uint8_t hour;
uint8_t minute;
float currenttime;
// mark is set by the doTime routine and indicates orientation of the tab on the moonphase brake wheel
// ( to be avoided during normal clock operation )
float mark;
float timeOffset = 0.0;
float lattitude = LATTITUDE;
float longitude = fmod(LONGITUDE + 360, 360);
WiFiClientSecure client;

volatile bool stopped;  // used with interrupt routine to detect home position
bool initDoneToday = false;

void rotate(float deg) {
  // positive is counterclockwise, but the stepper library has that as negative
  if (debug) Serial.printf("rotating %4.2f\xC2\xB0\n", deg);
  stepper.move((-deg / DEGREE_PER_STEP) * MICRO_STEPS);
  stepper.runToPosition();
}

void rotateTo(int dir, float newOrient) {
  // rotates from the current orientation (stored in variable orient)
  // it assumes the pointer/planet is just about to move (should be touching)
  // to the neworientation, in the given direction (-1 or 1)
  newOrient = fmod(newOrient + 360, 360);
  if (dir > 0) {
    if (newOrient <= orient) {
      rotate(360 + newOrient - orient);
    } else {
      rotate(newOrient - orient);
    }
  } else {
    if (newOrient >= orient) {
      rotate(-360 + newOrient - orient);
    } else {
      rotate(newOrient - orient);
    }
  }
  orient = newOrient;
}

void adjustOrient(float delta) {
  orient = fmod(orient + 360 + delta, 360);
}

void getTime(void) {
  struct tm timeinfo;
  getLocalTime(&timeinfo);
  year = timeinfo.tm_year + 1900;
  month = timeinfo.tm_mon + 1;
  day = timeinfo.tm_mday;
  hour = timeinfo.tm_hour;
  minute = timeinfo.tm_min;
  currenttime = hour + minute / 60.0 + timeinfo.tm_sec / 3600.00;
  currenttime += timeOffset;
  if (currenttime >= 24) {
    day++;
    currenttime -= 24;
  }
  hour = (int)currenttime;
  minute = (int)((currenttime - hour) * 60);
}

// returns offset with GMT in minutes
int getTimeDifference(void) {
  struct tm gmtTimeInfo, localTimeInfo;
  time_t now;

  time(&now);                    // Get current time
  gmtime_r(&now, &gmtTimeInfo);  // Get GMT time
  int gmtmin = gmtTimeInfo.tm_hour * 60 + gmtTimeInfo.tm_min;
  localtime_r(&now, &localTimeInfo);  // Get local time
  int localmin = localTimeInfo.tm_hour * 60 + localTimeInfo.tm_min;
  if (gmtTimeInfo.tm_yday != localTimeInfo.tm_yday) {
    return (localmin - gmtmin + 24 * 60);
  } else {
    return (localmin - gmtmin);
  }
}

float getFloat(void) {
  // the client.parseFloat is buggy and cannot handle floats with lots of decimal places
  float X = client.parseInt();  // stops at decimal point
  char c = client.read();
  int fraction = 0;  // read 6 digits of the fractional part
  for (int i = 0; i < 6; i++) {
    c = client.read();
    fraction = fraction * 10 + c - '0';
  }
  client.parseInt();  // ignore the rest
  float exponent = client.parseFloat();
  if (X > 0) {
    X = (X + fraction * 0.000001) * pow(10, exponent);
  } else {
    X = (X - fraction * 0.000001) * pow(10, exponent);
  }
  return X;
}

float getOrientation(int id, int center, int lyear, uint8_t lmonth, uint8_t lday) {
  // Nasa id mercury = 199, venus = 299, earth = 399, etc.
  // moon= 301, center = 10 for sun.
  client.setInsecure();
  delay(100);
  if (client.connect("ssd.jpl.nasa.gov", 443)) {
    if (debug) Serial.printf("https://ssd.jpl.nasa.gov/api/horizons.api?format=text&COMMAND=%%27%d"
                             "%%27&OBJ_DATA=%%27NO%%27&MAKE_EPHEM=%%27YES%%27&EPHEM_TYPE=%%27VECTOR%%27&CENTER=%%27500@%d%%27&TLIST=%%27"
                             "%d-%d-%d 12:00%%27&QUANTITIES=%%271%%27 HTTP/1.0\n",
                             id, center, lyear, lmonth, lday);
    client.printf("GET https://ssd.jpl.nasa.gov/api/horizons.api?format=text&COMMAND=%%27%d"
                  "%%27&OBJ_DATA=%%27NO%%27&MAKE_EPHEM=%%27YES%%27&EPHEM_TYPE=%%27VECTOR%%27&CENTER=%%27500@%d%%27&TLIST=%%27"
                  "%d-%d-%d 12:00%%27&QUANTITIES=%%271%%27 HTTP/1.0\n",
                  id, center, lyear, lmonth, lday);
    client.println("Connection: close");
    client.println();
    // read all headers
    while (client.connected()) {
      String line = client.readStringUntil('\n');
      if (line == "\r") {  // Headers received
        break;
      }
    }
    // skip text until $$SOE is found
    while (client.connected() && !client.find("$$SOE"))
      ;
    if (!client.available()) {
      Serial.printf("!!! no data for %d !!!\n", id);
      tft.printf("!!! no data for %d !!!\n", id);
      return 0;
    }
    while (client.connected() && !client.find("X ="))
      ;
    float X = getFloat();
    while (client.connected() && !client.find("Y ="))
      ;
    float Y = getFloat();
    while (client.available()) {
      char c = client.read();
    }
    client.stop();
    return fmod(atan2(Y, -X) / 3.141592 * 180 + 360 + 90, 360);
  } else {
    Serial.println("Connection failed");
    tft.println("Connection failed");
    // rotate a bit to indicate the error
    rotate(-20);
    rotate(20);
    rotate(-20);
    rotate(20);
    rotate(-20);
    rotate(20);
    delay(3000);
    return 0;
  }
  Serial.println("what???");
  return 0;
}

float getMoonPhase(int lyear, uint8_t lmonth, uint8_t lday) {
  Serial.printf("getting moon phase... ");
  tft.printf("moon phase... ");
  float earth = getOrientation(399, 10, lyear, lmonth, lday);
  float moon = getOrientation(301, 399, lyear, lmonth, lday);
  float phase = fmod(moon - earth + 360 + 180, 360);
  Serial.printf("%0.2f\xC2\xB0\n", phase);
  tft.printf("%0.2f\xC2\xB0\n", phase);
  return phase;
}

float getMoonNode(int lyear, uint8_t lmonth, uint8_t lday) {
  Serial.print("getting moon's argument of perifocus and true anomaly... ");
  tft.print("get moon's anomaly... ");
  client.setInsecure();
  delay(100);
  if (client.connect("ssd.jpl.nasa.gov", 443)) {
    client.printf("GET https://ssd.jpl.nasa.gov/api/horizons.api?format=text&COMMAND=%%27%d"
                  "%%27&OBJ_DATA=%%27NO%%27&MAKE_EPHEM=%%27YES%%27&EPHEM_TYPE=%%27ELEMENTS%%27&CENTER=%%27500@399%%27&TLIST=%%27"
                  "%d-%d-%d 12:00%%27&QUANTITIES=%%271%%27 HTTP/1.0\n",
                  301, lyear, lmonth, lday);
    client.println("Connection: close");
    client.println();
    // read all headers
    while (client.connected()) {
      String line = client.readStringUntil('\n');
      if (line == "\r") {  // Headers received
        break;
      }
    }
    // skip text until $$SOE is found
    while (client.connected() && !client.find("$$SOE"))
      ;
    if (!client.available()) {
      Serial.println("!!! no data for the moon !!! ");
      tft.println("!!! no data for the moon !!! ");
      return 0;
    }
    while (client.connected() && !client.find("W ="))
      ;
    float W = getFloat();
    while (client.connected() && !client.find("TA="))
      ;
    float TA = getFloat();
    while (client.available()) {
      char c = client.read();
    }
    client.stop();
    W = fmod(W + TA, 360.0);
    Serial.printf("sum of those is %0.2f\xC2\xB0\n", W);
    tft.printf("%0.2f\xC2\xB0\n", W);
    return W;
  } else {
    Serial.println("Connection failed");
    tft.println("Connection failed");
    // rotate a bit to indicate the error
    rotate(-20);
    rotate(20);
    rotate(-20);
    rotate(20);
    rotate(-20);
    rotate(20);
    delay(3000);
    return 0;
  }
  Serial.println("what???");
  return 0;
}

void getPlanets(int lyear, uint8_t lmonth, uint8_t lday) {
  Serial.print("getting data from NASA....\n");
  tft.setFont(&FreeMono9pt7b);
  tft.print("getting data from NASA....\n");
  for (uint8_t i = 0; i < 8; i++) {
    // default value in case connection fails
    planet[i] = fmod(i * 90, 360);
    Serial.print(planetName[i] + "...");
    tft.print(planetName[i] + "...");
    planet[i] = getOrientation((i + 1) * 100 + 99, 10, lyear, lmonth, lday);

    Serial.printf(" %0.2f\xC2\xB0\n", planet[i]);
    tft.printf(" %0.2f\xC2\xB0\n", planet[i]);

    // for some reason the clock produces a mirror image, so lets switch the orientation as an easy fix
    planet[i] = fmod(360 - planet[i], 360);
  }
}

void getRiseSet(int id, int lyear, uint8_t lmonth, uint8_t lday, float& rise, float& set) {
  // id should be 10 (sun) or 301 (moon)

  client.setInsecure();
  delay(200);
  // some default values in case the connection fails
  rise = 06.00;
  set = 18.00;
  int diff = getTimeDifference();
  int diffhours = diff / 60;
  int diffminutes = abs(diff - diffhours * 60);
  Serial.printf("getting rise and set times...\n ");
  tft.fillScreen(ILI9341_BLACK);  // Fill screen with black
  tft.setFont(&FreeMono9pt7b);
  tft.setCursor(0, 10);
  tft.printf("getting rise and set times...\n ");
  if (client.connect("ssd.jpl.nasa.gov", 443)) {
    client.printf("GET https://ssd.jpl.nasa.gov/api/horizons.api?format=text&COMMAND=%%27%d"
                  "%%27&OBJ_DATA=%%27NO%%27&MAKE_EPHEM=%%27YES%%27&EPHEM_TYPE=%%27OBSERVER%%27&CENTER=%%27c@399%%27&COORD_TYPE=%%27GEODETIC%%27&SITE_COORD=%%27%0.2f,%0.2f,0%%27&R_T_S_ONLY=%%27YES%%27&START_TIME=%%27"
                  "%d-%d-%d 00:00%%27&STOP_TIME=%%27%d-%d-%d 23:59&TIME_ZONE=%%27%02d:%02d%%27&STEP_SIZE=%%273%%20min%%27&QUANTITIES=%%274%%27 HTTP/1.0\n",
                  id, longitude, lattitude, lyear, lmonth, lday, lyear, lmonth, lday, diffhours, diffminutes);
    client.println("Connection: close");
    client.println();
    // read all headers
    while (client.connected()) {
      String line = client.readStringUntil('\n');
      if (line == "\r") {  // Headers received
        break;
      }
    }
    // skip text until $$SOE is found
    while (client.connected() && !client.find("$$SOE"))
      ;
    if (!client.available()) {
      Serial.println("!!! no data for the Rise & set times !!! ");
      tft.println("!!! no data for the Rise & set times !!! ");
      return;
    }
    // it might happen that the moon/sun doesn't rise or set
    rise = -1;
    set = -1;
    int c = client.read();
    c = client.read();
    while (c == ' ' || c == 'b') {
      // skip the date
      for (int i = 0; i < 12; i++) {
        c = client.read();
      }
      int hour, min;
      hour = client.parseInt();
      c = client.read();  // colon between hour and minute
      min = client.parseInt();
      client.read();
      client.read();
      // next character could be 'r' for rise and 's' for set or something else
      c = client.read();
      if (c == 'r') {
        rise = hour + min / 60.0;
      }
      if (c == 's') {
        set = hour + min / 60.0;
      }
      client.readStringUntil('\n');  // skip until end of line
      c = client.read();
    }
    while (client.available()) {
      c = client.read();
    }
    client.stop();
    return;
  } else {
    Serial.println("Connection failed");
    tft.println("Connection failed");
    // rotate a bit to indicate the error
    rotate(-20);
    rotate(20);
    rotate(-20);
    rotate(20);
    rotate(-20);
    rotate(20);
    delay(3000);
    return;
  }
  Serial.println("what???");
  return;
}

// called by interrupt routing
void stop() {
  stopped = true;
}
void gohome() {
  // 13 is the worst case, during testing I set this to 4 and spin the planets counter clockwise by hand during the reset.
  const int Num_to_do = 13;
  Serial.println("going home");
  tft.setCursor(0, 50);
  tft.println("going home.........");
  stepper.setMaxSpeed(MAX_SPEED * MICRO_STEPS);
  stepper.setAcceleration(ACCELERATION * MICRO_STEPS);
  stopped = false;

  attachInterrupt(digitalPinToInterrupt(STOP_PIN), stop, RISING);
  // rotate a bit more than 360 degrees to make sure we catch the magnet
  // that causes the interrupt to happen
  stepper.move(400.0 / DEGREE_PER_STEP * MICRO_STEPS);
  stepper.run();
  while (stepper.isRunning() && !stopped) {
    stepper.run();
  }
  if (!stopped) {
    Serial.println("interrupt not working, magnet not seen?\n");
    tft.setCursor(0, 100);
    tft.println("interrupt not working,        magnet not seen?\n");
    tft.setFont(&FreeMono9pt7b);
  }
  // set the target to move so it stops at 00:00; should do at least 360 but
  // a true reset needs to grab all the planets first, which requires multiple turn
  stepper.move((-ZERO_OFFSET + Num_to_do * 360.0) / DEGREE_PER_STEP * MICRO_STEPS);
  Serial.println("rotating to 00:00");
  Serial.println("rotating clockwise 13 times!");
  tft.println("rotating to 00:00");
  tft.println("rotating CW 13 times!");
  stepper.runToPosition();
  Serial.println("home done");
  tft.println("home done");
  // current orient value is where mercury is when the clock is at 00:00
  // note that instead of moving to 00:00 we could have moved mercury directly into
  // place, but it looks nice as the beginning of the reset to move to 00:00 first
  // and it only saves a couple of seconds.
  orient = -touchValue[0];
}

void doPlanets() {
  // assumes a reset has been done with all the planets grabbed
  stepper.setMaxSpeed(MAX_SPEED * MICRO_STEPS);
  int dir = -1;
  for (int i = 0; i < 8; i++) {
    Serial.printf("spinning %s into place at %0.1f\xC2\xB0\n", planetName[i], planet[i]);
    tft.fillScreen(ILI9341_BLACK);  // Fill screen with black
    tft.setCursor(20, 100);
    tft.printf("Spinning %s to %0.1f\xC2\xB0\n", planetName[i], planet[i]);
    if (i != 0) {
      // mercury is already touching after the reset
      rotate(touchValue[i]);
    }
    delay(500);
    rotateTo(dir, planet[i]);
    adjustOrient(offsetValue[i]);
    dir = -dir;
  }
}

void doMoon() {
  Serial.printf("spinning moon's node pointers into place at %d\xC2\xB0\n", (int)moonperifocus);
  tft.fillScreen(ILI9341_BLACK);  // Fill screen with black
  tft.setCursor(0, 20);
  tft.printf("spin moon node pointers %d\xC2\xB0\n", (int)moonperifocus);
  rotate(touchValue[8]);
  rotateTo(-1, moonperifocus);
  adjustOrient(offsetValue[8]);
  Serial.printf("spinning moonphase to %d\xC2\xB0\n", (int)moonphase);
  tft.printf("spin moonphase to %d\xC2\xB0\n", (int)moonphase);
  rotate(touchValue[9]);  // moon phase
  rotateTo(1, moonphase);
  adjustOrient(offsetValue[9]);
}

// sets time of sun/moon rise/set and the clock
// assumes currenttime is set
void doTime(void) {
  Serial.printf("sunset %02d:%02d\n", (int)sunrise, (int)((sunset - (int)sunset) * 60));
  tft.printf("sunset %02d:%02d\n", (int)sunrise, (int)((sunset - (int)sunset) * 60));
  Serial.printf("spinning difference between sunrise and sunset %02d:%02d\n", (int)(sunset - sunrise), abs((int)(((sunset - sunrise) - (int)(sunset - sunrise)) * 60)));
  tft.setFont();
  tft.printf("spinning difference between sunrise-sunset %02d:%02d\n", (int)(sunset - sunrise), abs((int)(((sunset - sunrise) - (int)(sunset - sunrise)) * 60)));
  tft.setFont(&FreeMono9pt7b);
  tft.setCursor(0, 100);
  rotate(touchValue[10]);  // sunset
  //rotate so the distance between sunset and sunrise is correct; 24-sunset+sunrise
  stepper.setMaxSpeed(PRECISE_SPEED * MICRO_STEPS);
  stepper.setAcceleration(ACCELERATION * MICRO_STEPS * 0.1);

  rotate(-(24 - sunset + sunrise) * DEGREE_PER_HOUR + 2);
  Serial.printf("spinning sunrise %02d:%02d\n", (int)sunrise, (int)((sunrise - (int)sunrise) * 60));
  tft.printf("spin sunrise %02d:%02d\n", (int)sunrise, (int)((sunrise - (int)sunrise) * 60));

  rotate(touchValue[11]);  // sunrise
  rotate(sunrise * DEGREE_PER_HOUR);

  Serial.printf("spinning moonset %02d:%02d\n", (int)moonset, (int)((moonset - (int)moonset) * 60));
  tft.printf("spin moonset %02d:%02d\n", (int)moonset, (int)((moonset - (int)moonset) * 60));
  rotate(touchValue[12]);  //moon set
  rotate(-moonset * DEGREE_PER_HOUR);
  Serial.printf("spinning moonrise %02d:%02d\n", (int)moonrise, (int)((moonrise - (int)moonrise) * 60));
  tft.printf("spin moonrise %02d:%02d\n", (int)moonrise, (int)((moonrise - (int)moonrise) * 60));
  rotate(touchValue[13]);  // moon rise

  // there are two pointers
  // if moonrise < moonset  (moonset-moonrise) *degree_per_hour-10 (negative number
  // might push moonset, but this only happens if moonrise and moonset are within 1.5 hours of eachother)
  // else (24-moonrise+moonset)*degree_per_hour - 10
  if (moonrise < moonset) {
    rotate((moonset - moonrise) * DEGREE_PER_HOUR - 10);
  } else {
    rotate((moonset - moonrise + 24) * DEGREE_PER_HOUR - 10);
  }
  // time itself is a bit more difficult20
  getTime();
  if (moonrise > moonset) {
    mark = -orient / DEGREE_PER_HOUR + moonrise;
    if (debug) Serial.printf("A currenttime %0.2f mark %0.2f\n", currenttime, mark);
    if (currenttime < moonrise) {
      if (currenttime > mark) {
        rotate(-(currenttime - mark) * DEGREE_PER_HOUR + 0.0);
      } else {
        // -2 for correcting slack
        rotate(-(currenttime - mark) * DEGREE_PER_HOUR - 360 + 2.5);
      }
    } else {
      if (currenttime > mark) {
        // added -2 for slack correction
        rotate(-(currenttime - mark) * DEGREE_PER_HOUR - 2.5);
      } else {
        rotate(-(currenttime - mark) * DEGREE_PER_HOUR - 360);
      }
    }
  } else {
    mark = 24 - orient / DEGREE_PER_HOUR + moonrise;
    if (debug) Serial.printf("B currenttime %0.2f mark %0.2f\n", currenttime, mark);
    if (mark < -24) {
      mark = mark + 48;
      if (currenttime < mark) {
        rotate(-(currenttime - mark) * DEGREE_PER_HOUR - 360);
      } else {
        rotate(-(currenttime - mark) * DEGREE_PER_HOUR - 2);
      }
    } else {
      if (currenttime < mark) {
        rotate(-(currenttime - mark) * DEGREE_PER_HOUR - 360 - 2);
      } else {
        rotate(-(currenttime - mark) * DEGREE_PER_HOUR);
      }
    }
  }
}


// please ignore the routine, it was only used for testing
void timetestRoutine() {
  moonphase = 222.0;
  moonperifocus = 330;
  currenttime = 16.00;
  sunrise = 05.15;
  sunset = 22.25;
  moonrise = 18.75;
  moonset = 2.4;
  stepper.setMaxSpeed(MAX_SPEED * MICRO_STEPS);
  //rotate(touchValue[8]);  // node ascending node!
  //rotate(-259); // -169 sets perifocus to 180
  rotate(-319);            // perifocu 330
  orient = moonperifocus;  // 0
  adjustOrient(offsetValue[8]);

  rotate(touchValue[9]);  // moon phase
  rotateTo(1, moonphase);
  adjustOrient(offsetValue[9]);

  rotate(touchValue[10]);  // sunset
  //rotate so the distance between sunset and sunrise is correct; 24-sunset+sunrise
  Serial.printf("slow speed %d\n", MAX_SPEED);
  stepper.setMaxSpeed(PRECISE_SPEED * MICRO_STEPS);
  rotate(-(24 - sunset + sunrise) * DEGREE_PER_HOUR);

  rotate(touchValue[11]);  // sunrise
  rotate(sunrise * DEGREE_PER_HOUR);

  rotate(touchValue[12]);  //moon set
  rotate(-moonset * DEGREE_PER_HOUR);

  rotate(touchValue[13]);  // moon rise

  // there are two pointers
  // if moonrise < moonset  (moonset-moonrise) *degree_per_hour-10 (negative number
  // might push moonset, but this only happens if moonrise and moonset are within 1.5 hours of eachother)
  // else (24-moonrise+moonset)*degree_per_hour - 10
  if (moonrise < moonset) {
    rotate((moonset - moonrise) * DEGREE_PER_HOUR - 10);
  } else {
    rotate((moonset - moonrise + 24) * DEGREE_PER_HOUR - 10);
  }
  // time itself is a bit more difficult20
  float rot = 0;
  if (moonrise > moonset) {
    mark = -orient / DEGREE_PER_HOUR + moonrise;
    Serial.printf("mark %0.2f\n", mark);
    if (currenttime < moonrise) {
      if (currenttime > mark) {
        Serial.println("A");
        rot = -(currenttime - mark) * DEGREE_PER_HOUR + 3;
      } else {
        // -2 for correcting slack
        Serial.println("B");
        rot = -(currenttime - mark) * DEGREE_PER_HOUR - 360 - 2;
      }
    } else {
      if (currenttime > mark) {
        // added -2 for slack correction
        Serial.println("C");
        rot = -(currenttime - mark) * DEGREE_PER_HOUR - 2;
      } else {
        Serial.println("D");
        rot = -(currenttime - mark) * DEGREE_PER_HOUR - 360;
      }
    }

  } else {
    mark = 24 - orient / DEGREE_PER_HOUR + moonrise;
    if (mark < -24) {
      mark = mark + 48;
      Serial.printf("mark %0.2f\n", mark);
      if (currenttime < mark) {
        Serial.printf("case E \n");
        rot = -(currenttime - mark) * DEGREE_PER_HOUR - 360;
      } else {
        Serial.printf("case F\n");
        rot = -(currenttime - mark) * DEGREE_PER_HOUR - 2;
      }
    } else {
      if (currenttime < mark) {
        Serial.printf("case G \n");
        rot = -(currenttime - mark) * DEGREE_PER_HOUR - 360 - 2;
      } else {
        Serial.printf("case H\n");
        rot = -(currenttime - mark) * DEGREE_PER_HOUR;
      }
    }
  }
  Serial.printf("rotation is %0.2f\n", rot);
  //rotate(rot);
  Serial.printf("mark %0.2f %2d:%02d\n", mark, (int)mark, (int)(fabs(mark) - (int)(fabs(mark))) * 60);
}

void setup() {
  Serial.begin(SERIAL_BAUD_RATE);
  delay(400);
  //TFT
  tft.begin();  // Initialize ILI9341 chip
  tft.setFont();
  tft.fillScreen(ILI9341_BLACK);  // Fill screen with black
  tft.setRotation(1);
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(2);
  tft.setCursor(0, 20);
  tft.println("    Astronomical Clock");
  tft.setFont(&FreeMono9pt7b);
  tft.setTextSize(1);
  tft.setCursor(0, 60);
  tft.println("     By illusionmanager");

  pinMode(LED_PIN, OUTPUT);
  delay(400);
  pinMode(LED_PIN, OUTPUT);
#ifndef NOWIFI
  //AP

  wifiManager.setConfigPortalTimeout(3000);
  Serial.println("Calling autoConnect");
   tft.setCursor(0,100);
  tft.println("    Calling autoConnect");
 
  // bool ACsuccess = wifiManager.autoConnect("AstroClockAP", "12345678");
  bool ACsuccess = wifiManager.autoConnect("AstroClockAP");

  if (ACsuccess) {
    Serial.println("connected...yeey :)");
	 tft.setCursor(0,120);
     tft.println("       Connected!");
    digitalWrite(LED_PIN, HIGH);
  } else {
    Serial.print("Time out or EXIT pushed, so Setting AP (Access Point)… ");
    tft.setCursor(0,140);
    tft.println("Time out or EXIT pushed,");
    tft.println("so Setting AP (Access Point)… ");
  }
  /*
  int cnt=0;
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print("-");
    cnt++;
    delay(500);
    digitalWrite(LED_PIN,cnt%2 == 0?true:false);
    if (cnt %20 == 0) {
      WiFi.disconnect(true,  true);
      delay(200);
      WiFi.begin(MYSSID, MYPASSKEY);
      Serial.print("*");
    }
  }
*/
  setenv("TZ", TIMEZONE, 1);
  tzset();
  configTzTime(TIMEZONE, "pool.ntp.org");
  getTime();
#endif
  stepper_driver.setup(serial_stream, 38400, stepper_driver.SERIAL_ADDRESS_0);
  stepper_driver.enable();
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);
  pinMode(STOP_PIN, INPUT);
  digitalWrite(ENABLE_PIN, HIGH);
  delay(500);
  stepper_driver.setMicrostepsPerStep(MICRO_STEPS);
  stepper_driver.setRunCurrent(RUN_CURRENT_PERCENT);
  stepper_driver.useInternalSenseResistors();
  stepper_driver.enableStealthChop();
  stepper_driver.enableAutomaticCurrentScaling();
  stepper_driver.enableCoolStep();
  stepper_driver.enable();
  stepper_driver.setHoldDelay(6);  // don't want to keep to motor powered all day
  stepper_driver.setHoldCurrent(0);
  stepper_driver.setStandstillMode(stepper_driver.FREEWHEELING);
  stepper_driver.moveUsingStepDirInterface();
  digitalWrite(ENABLE_PIN, LOW);

  attachInterrupt(digitalPinToInterrupt(STOP_PIN), stop, RISING);
  stepper.setMaxSpeed(MAX_SPEED * MICRO_STEPS);
  stepper.setAcceleration(ACCELERATION * MICRO_STEPS);
  // Serial.begin(SERIAL_BAUD_RATE);
  // delay(400);
  // move a little to signal wake up
  rotate(5);
  rotate(-5);
  Serial.printf("\n\n\n\n\n\n\n\n");
  tft.setCursor(0, 70);
  tft.printf("\n\n\n\n\n\n\n\n");
  Serial.println("setup done");
  //tft.fillScreen(ILI9341_BLACK); // Fill screen with black
  //tft.setTextColor(ILI9341_WHITE);
  //tft.setTextSize(2);
  tft.setCursor(30, 180);
  tft.println("      setup done");
}

int sgn = 1;
int rot = 0;
float fraction = 0;
float fractionfactor = 0;
bool hasfraction = false;
float now = 0;
bool initNeeded = true;
float prevtime = 0;
bool doOffset = false;

void loop() {
  // try to do 8 full steps with the motor
  const float timestep = DEGREE_PER_STEP * 8 / DEGREE_PER_HOUR;  // almost 3 minutes
  float truestep;
  if (initNeeded == true) {
    initDoneToday = true;
    getTime();
    Serial.printf("Local time is %02d:%02d\n", hour, minute);
    tft.fillScreen(ILI9341_BLACK);  // Fill screen with black
    tft.setCursor(0, 10);
    tft.setFont(&FreeMono9pt7b);
    tft.printf("Local time is %02d:%02d\n", hour, minute);
    getPlanets(year, month, day);  // stores data in array Planets[]
    moonperifocus = getMoonNode(year, month, day);
    moonphase = getMoonPhase(year, month, day);
    getRiseSet(10, year, month, day, sunrise, sunset);
    Serial.printf("sunrise %02d:%02d sunset %02d:%02d\n", (int)sunrise, (int)((sunrise - (int)sunrise) * 60), (int)sunset, (int)((sunset - (int)sunset) * 60));
    tft.printf("sunrise %02d:%02d sunset %02d:%02d\n", (int)sunrise, (int)((sunrise - (int)sunrise) * 60), (int)sunset, (int)((sunset - (int)sunset) * 60));
    if (sunrise == -1) sunrise = 00.00;
    if (sunset == -1) sunset = 30;
    getRiseSet(301, year, month, day, moonrise, moonset);
    Serial.printf("moonrise %02d:%02d moonset %02d:%02d\n", (int)moonrise, (int)((moonrise - (int)moonrise) * 60), (int)moonset, (int)((moonset - (int)moonset) * 60));
    tft.setCursor(0, 25);
    tft.printf("moonrise %02d:%02d moonset %02d:%02d\n", (int)moonrise, (int)((moonrise - (int)moonrise) * 60), (int)moonset, (int)((moonset - (int)moonset) * 60));
    if (moonrise == -1) moonrise = 00.00;
    if (moonset == -1) moonset = -2.00;
    tft.setFont(&FreeMono9pt7b);
    gohome();
    doPlanets();
    doMoon();
    getTime();
    prevtime = currenttime;
    doTime();
    Serial.println("Ready.");
    tft.println("Ready.");
    //timetestRoutine();
    stepper.setMaxSpeed(PRECISE_SPEED * MICRO_STEPS);
    stepper.setAcceleration(ACCELERATION * MICRO_STEPS / 2);
    initNeeded = false;
    //now = currenttime;
  } else {

    getTime();
    truestep = currenttime - prevtime;
    if (truestep < 0) truestep += 24;

    if (truestep > timestep) {
      if (debug) Serial.printf("prevtime %0.4f timestep %0.4f truestep %0.4f currenttime %0.4f\n", prevtime, timestep, truestep, currenttime);
      float now;
      now = prevtime + truestep;
      if (now >= 24) {
        if (mark < 0 || mark >= 24) {
          if (debug) Serial.printf("rotA %0.2f\n", (24 - truestep) * DEGREE_PER_HOUR);
          rotate((24 - truestep) * DEGREE_PER_HOUR);
        } else {
          if (debug) Serial.printf("rotB %0.2f\n", -360 + (24 - truestep) * DEGREE_PER_HOUR);
          rotate(-360 + (24 - truestep) * DEGREE_PER_HOUR);
        }
      } else {
        if (now > mark && now - mark < truestep) {
          //rotate around mark
          if (debug) Serial.printf("rotC %0.2f\n", 360 - truestep * DEGREE_PER_HOUR);
          rotate(360 - truestep * DEGREE_PER_HOUR);
        } else {
          if (debug) Serial.printf("rotD %0.2f\n", -truestep * DEGREE_PER_HOUR);
          rotate(-truestep * DEGREE_PER_HOUR);
        }
        if (now < 03.00) {
          initDoneToday = false;
        } else if (!initDoneToday) {
          initNeeded = true;
        }
      }
      prevtime = currenttime;
    }
  }
  delay(20);


  // for debugging and testing purposes, while in the main loop you can enter a number
  // and it will rotate by that many degrees
  // 'r' does the reset procudere
  // 't' sets all the planets at 90 degree angles, rise and set indicators to 06:00 and 18:00, and time to 12:00 (reset needed first)
  // 'n' turns eclipse spinner back to normal operation (does a full reset)
  // 'f' sets the normal (fast) speed
  // 's' sets the slow (half) speed
  // 'p' prints relevant data
  // '=' followed by an offset, adds some time to the real time (should be less<24 hour)
  // 'd' toggle print additional debugging info

  char inChar;
  inChar = 0;

  if (Serial.available()) {
    inChar = (char)Serial.read();
  }
  switch (inChar) {
    case 'r':
      gohome();
      break;
    case 't':
      for (int i = 0; i < 8; i++) {
        planet[i] = fmod(i * 90 + 90.0, 360);
      }
      moonperifocus = 90;
      moonphase = 270;
      sunrise = 06.00;
      sunset = 18.00;
      moonrise = 06.00;
      moonset = 18.00;
      currenttime = 12.00;
      gohome();
      doPlanets();
      doMoon();
      prevtime = currenttime;
      doTime();
      Serial.println("done. press any key to continue");
      while (!Serial.available()) {
        delay(20);
      }
      inChar = (char)Serial.read();
      break;
    case 'n':
      initNeeded = true;
      break;
    case 'q':  // ignore this one
      timetestRoutine();
      break;
    case 'd':
      debug = !debug;
      break;
    case 'f':
      Serial.printf("fast speed %d\n", MAX_SPEED);
      stepper.setMaxSpeed(MAX_SPEED * MICRO_STEPS);
      break;
    case 's':
      Serial.printf("slow speed %d\n", MAX_SPEED / 2);
      stepper.setMaxSpeed(MAX_SPEED * MICRO_STEPS / 2);
      break;
    case 'p':
      Serial.printf("Orient is %0.1f\n", orient);
      Serial.printf("Currenttime is %02d:%02d (%0.2f)\n", (int)currenttime, (int)((currenttime - (int)currenttime) * 60), currenttime);
      Serial.printf("Mark is at %02d:%02d (%0.2f)\n", (int)mark, (int)abs((mark - (int)mark) * 60), mark);
      Serial.printf("Planets are at %0.0f\xC2\xB0,%0.0f\xC2\xB0,%0.0f\xC2\xB0,%0.0f\xC2\xB0,%0.0f\xC2\xB0,%0.0f\xC2\xB0,%0.0f\xC2\xB0,%0.0f\xC2\xB0\n", planet[0], planet[1],
                    planet[2], planet[3], planet[4], planet[5], planet[6], planet[7]);
      Serial.printf("Moon perifocus is at %0.2f\xC2\xB0 Moonphase is %0.2f\xC2\xB0\n", moonperifocus, moonphase);
      Serial.printf("sunrise %02d:%02d sunset %02d:%02d\n", (int)sunrise, (int)((sunrise - (int)sunrise) * 60), (int)sunset, (int)((sunset - (int)sunset) * 60));
      Serial.printf("moonrise %02d:%02d moonset %02d:%02d\n", (int)moonrise, (int)((moonrise - (int)moonrise) * 60), (int)moonset, (int)((moonset - (int)moonset) * 60));
      break;
    case '.':
    case ':':
      fractionfactor = 0.1;
      hasfraction = true;
      break;
    case '-':
      sgn = -1;
      break;
    case '1':
    case '2':
    case '3':
    case '4':
    case '5':
    case '6':
    case '7':
    case '8':
    case '9':
    case '0':
      if (hasfraction) {
        fraction = fraction + fractionfactor * (inChar - '0');
        fractionfactor *= 0.1;
      } else {
        rot = rot * 10 + (inChar - '0');
      }
      break;
    case '=':  // for my personal debugging only
      doOffset = true;
      Serial.printf("offset true\n");
      break;
    case '\n':
      if (doOffset) {
        timeOffset = rot + fraction;
        Serial.printf("timeoffset %0.2f\n", timeOffset);
        doOffset = false;
      } else if (rot + fraction > 0.001) {
        rotate(sgn * (rot + fraction));
      }
      sgn = 1;
      rot = 0;
      hasfraction = false;
      fraction = 0.0;
      break;
  }
}

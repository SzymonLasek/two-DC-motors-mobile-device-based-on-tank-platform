#include <SoftwareSerial.h>
#include <BlynkSimpleSerialBLE.h>
#include <Wire.h>
#include <MechaQMC5883.h>
MechaQMC5883 qmc;
#include <TinyGPS.h>

#define rxPin 8 //GPSplatform
#define txPin 9 //GPSplatform
SoftwareSerial ss = SoftwareSerial(rxPin, txPin);
TinyGPS gps;
SoftwareSerial SerialBLE(0, 1)
#define BLYNK_USE_DIRECT_CONNECT
#define BLYNK_PRINT Serial
#define DEC_ANGLE 0.0797f
#define compass_offset 0.0f
#define GPS_UPDATE_INTERVAL 1000
#define GPS_STREAM_TIMEOUT 18

#define LFP 2 //Left Forward Pin
#define LBP 4 //Left Backward Pin
#define LPP 3 //Left PWM Pin

#define RFP 7 //Right Forward Pin
#define RBP 5 //Right Backward Pin
#define RPP 6 // Right PWP Pin

;char auth[] = "d575b77c91604464ad51672c42025ba4";

int enabled = 0;
struct Loc {
  float lat;
  float lon;
};
bool feedgps() {
  while (ss.available()) {
    if(gps.encode(ss.read()))
  
    return true;
  }
  return false;
}

Loc checkGPS() {
  bool newdata = false;
  unsigned long start = millis();

  while (millis() - start < GPS_UPDATE_INTERVAL) {
    if (feedgps())
      newdata = true;
  }
  if (newdata) {
    return gpsdump(gps);
  }
  Loc platformLoc;
  platformLoc.lat = 0.0;
  platformLoc.lon = 0.0;
  return platformLoc;
}

Loc gpsdump(TinyGPS &gps) {
  float flat, flon;
  unsigned long age;

  gps.f_get_position(&flat, &flon, &age);
flat== TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6;
flon== TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6;
  Loc platformLoc;
  platformLoc.lat = flat;
  platformLoc.lon = flon;

  Serial.print(platformLoc.lat, 6); Serial.print(","); Serial.println(platformLoc.lon, 6);
  return platformLoc;
}

  BLYNK_WRITE(V0) {
    enabled = param.asInt();
   
  }
  BLYNK_WRITE(V1) {
    GpsParam gps(param);
    
    Serial.print(gps.getLat(), 6); Serial.print(","); Serial.println(gps.getLon(), 6);

    Loc phoneLoc;
    phoneLoc.lat = gps.getLat(), 6;
    phoneLoc.lon = gps.getLon(), 6;


    driveTo(phoneLoc, GPS_STREAM_TIMEOUT);
  }
#ifndef DEGTOGRAD
#define DEGTORAD 0.0174532925199432957f
#define RADTODEG 57.295779513082320876f
#endif
  float Bearing(struct Loc &a, struct Loc &b) {
    float y = sin(b.lon - a.lon) * cos(b.lat);
    float x = cos(a.lat) * sin(b.lat) - sin(a.lat) * cos(b.lat) * cos(b.lon - a.lon);
    return atan2(y, x) * RADTODEG;
  }

  float Distance(struct Loc &a, struct Loc &b) {
    const float R = 6371000; //km
    float p1 = a.lat * DEGTORAD;
    float p2 = b.lat * DEGTORAD;
    float dp = (b.lat - a.lat) * DEGTORAD;
    float dl = (b.lon - a.lon) * DEGTORAD;
    float x = sin(dp / 2) * sin(dp / 2) + cos(p1) * cos(p2) * sin(dl / 2) * sin(dl / 2);
    float y = 2 * atan2(sqrt(x), sqrt(1 - x));
    return R * y;
  }
  float gHeading() {
    int x;
    int y;
    int z;
    float azimuth;
   qmc.read(&x, &y, &z);
   azimuth = atan2(y, x);
   azimuth -= DEC_ANGLE;
    azimuth -= compass_offset;
   if(azimuth <0)
   azimuth +=2*PI;
   if(azimuth > 2*PI)
   azimuth -= 2*PI;
    float headingDegrees = azimuth * 180/M_PI;
    while (headingDegrees < -180) headingDegrees += 360;
    while (headingDegrees > 180) headingDegrees -= 360;
        return headingDegrees;
      }

void setSpeedR(int speeda) {
    digitalWrite(RFP, HIGH);
    digitalWrite(RBP, LOW);
    analogWrite(RPP, speeda);
  }
  void setSpeedL(int speeda) {
    digitalWrite(LFP, HIGH);
    digitalWrite(LBP, LOW);
    analogWrite(LPP, speeda);
  }
  void setSpeed(int speeda) {
    setSpeedR(speeda);
    setSpeedL(speeda);
  }

  void stopp() {
    
    digitalWrite(RFP, LOW);
    digitalWrite(RBP, LOW);
    digitalWrite(LFP, LOW);
    digitalWrite(LBP, LOW);
  }

  void drive(int distance, float turn) {
    int fullSpeed = 250;
    int stopSpeed = 0;
    int s = fullSpeed;
    if (distance < 8) {
      int wouldBeSpeed = s - stopSpeed;
      wouldBeSpeed *= distance / 8.0f;
      s = stopSpeed + wouldBeSpeed;
    }
    int autoThrottle = constrain(s, stopSpeed, fullSpeed);
    autoThrottle = 250;
    float t = turn;
    while (t < -180) t += 360;
    while (t >  180) t -= 360;
    Serial.print(t);
    Serial.print(turn);
    float t_modifier = (180.0 - abs(t)) / 180.0;
    float autoSteerR = 1;
    float autoSteerL = 1;
    if (t < 0) {
      autoSteerL = t_modifier;
    } else if (t > 0) {
      autoSteerR = t_modifier;
    }
    int speedR = (int) (((float) autoThrottle) * autoSteerR);
    int speedL = (int) (((float) autoThrottle) * autoSteerL);
    setSpeedR(speedR);
    setSpeedL(speedL);
  }
  void driveTo(struct Loc &locc, int timeout) {
    ss.listen();
    Loc platformLoc = checkGPS();
    SerialBLE.listen();
    if (platformLoc.lat != 0 && platformLoc.lon != 0 && enabled ==1) {
      float d = 0;
      do {
        ss.listen();
        platformLoc = checkGPS();
        SerialBLE.listen();
        d = Distance(platformLoc, locc);
        float t = Bearing(platformLoc, locc) - gHeading();
        
        drive(d, t);
        timeout -= 1;
      } while (d > 3.0 || enabled==0 || timeout > 0);
      stopp();
    }
  }
  

  void setup()
  {
    pinMode(rxPin, INPUT);
  pinMode(txPin, OUTPUT);
    pinMode(LFP, OUTPUT);
    pinMode(LBP, OUTPUT);
    pinMode(LPP, OUTPUT);
    pinMode(RFP, OUTPUT);
    pinMode(RBP, OUTPUT);
    pinMode(RPP, OUTPUT);
    Serial.begin(9600);
    ss.begin(9600);
    SerialBLE.begin(9600);
    Blynk.begin(SerialBLE, auth);
    Wire.begin();
    qmc.init();
  }

  void loop()
  {

    Blynk.run();
  }


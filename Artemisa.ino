// Pins
#define RX 8
#define TX 9
#define R_WHEEL_NEG 10
#define R_WHEEL_POS 11
#define L_WHEEL_NEG 12
#define L_WHEEL_POS 13
#define MIC A0

// Parameters
#define LAT 1
#define LNG 1
#define NAV_TOL 0.1
#define STEP_TIME 1000

#include <TinyGPS++.h>
#include <SoftwareSerial.h>

SoftwareSerial gpsSerial(RX, TX);
float lat0;
float lng0;
long lastAdjustment = 0;

void setup() {
  Serial.begin(9600);
  gpsSerial.begin(9600);
  pinMode(RX, INPUT);
  pinMode(TX, INPUT);
  pinMode(L_WHEEL_POS, OUTPUT);
  pinMode(L_WHEEL_NEG, OUTPUT);
  pinMode(R_WHEEL_POS, OUTPUT);
  pinMode(R_WHEEL_NEG, OUTPUT);

  // Get starting location
  while (true) {
    TinyGPSPlus gps;
    if (gpsSerial.available() > 0 && gps.encode(gpsSerial.read())) {
      lat0 = gps.location.lat();
      lng0 = gps.location.lng();
      break;
    }
  }
  navForwards();
  lastAdjustment = millis();
  while (true) {
    if (navigate()) { break; }
  }
}

void navForwards() {
  digitalWrite(L_WHEEL_POS, HIGH);
  digitalWrite(L_WHEEL_NEG, LOW);
  digitalWrite(R_WHEEL_POS, HIGH);
  digitalWrite(R_WHEEL_NEG, LOW);
}

void navStop() {
  digitalWrite(L_WHEEL_POS, LOW);
  digitalWrite(L_WHEEL_NEG, LOW);
  digitalWrite(R_WHEEL_POS, LOW);
  digitalWrite(R_WHEEL_NEG, LOW);
}

double getDirection(float dLat, float dLng) {
  double angle = atan(dLat/dLng);
  if (dLng < 0) { angle += PI/2; }
  if (angle < 0) { angle += 2*PI; }
  return angle;
}

boolean navigate() {
  if (millis() - lastAdjustment > STEP_TIME) {
    // Adjust direction
    TinyGPSPlus gps;
    if (gpsSerial.available() <= 0 || !gps.encode(gpsSerial.read())) {
      // GPS not available, stand still
      navStop();
      return false;
    }
    
    float lat1 = gps.location.lat();
    float lng1 = gps.location.lng();

    // Calculate required direction
    float dLat = LAT-lat1;
    float dLng = LNG-lng1;
    if (dLat < NAV_TOL && dLng < NAV_TOL) {
      // Arrived
      navStop();
      return true;
    }
    double dir = getDirection(dLat, dLng);

    // Calculate direction since last readjustment
    
    lat0 = lat1-lat0;
    lng0 = lng1-lng0;
    if (lat0 == 0 && lng0 == 0) {
      // Movement not detected, move forwards and see what changes
      navForwards();
      lastAdjustment = millis();
      return false;
    }
    dir = getDirection(lat0, lng0)-dir;
    if (dir > 0) {
      // Turn left
      digitalWrite(L_WHEEL_POS, LOW);
      digitalWrite(L_WHEEL_NEG, LOW);
      digitalWrite(R_WHEEL_POS, HIGH);
      digitalWrite(R_WHEEL_NEG, LOW);
    } else if (dir < 0) {
      // Turn right
      digitalWrite(L_WHEEL_POS, HIGH);
      digitalWrite(L_WHEEL_NEG, LOW);
      digitalWrite(R_WHEEL_POS, LOW);
      digitalWrite(R_WHEEL_NEG, LOW);
    } else {
      // Keep forward
      navForwards();
    }
    lat0 = lat1;
    lng0 = lng1;
    lastAdjustment = millis();
    return false;
  } else {
    navStop();
    return false;
  }
}

void loop() { }

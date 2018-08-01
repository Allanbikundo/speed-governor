#include <NewSoftSerial.h>
#include <TinyGPS.h>

#define NUM_OFF 2
#define DELAY 50
#define TEST_BUTTON 9
#define DEFAULT_SPEED_LIMIT 55
#define NSPEEDZONES 6

boolean debugMode = true;

boolean speeding = false;
int blue[4];
int red[4];
int headlight1;
int headlight2;

class Vertex {
public:
  Vertex(float llat, float llng) {
    lat = llat;
    lng = llng;
  }
  float lat;
  float lng;
};

class SpeedZone {
public:
  SpeedZone(int s) {
    speedLimit = s;
  }
  void setVertices(int n, Vertex *v) {
    nVertices = n;
    vertices = v;
  }
  int nVertices;
  Vertex *vertices;
  int speedLimit;
};


SpeedZone *speedZones[NSPEEDZONES];
TinyGPS gps;
NewSoftSerial nss(2, 3);
boolean buttonPressed = false;

void setup() {
  Serial.begin(115200);

  setupSpeedZones();

  blue[0] = 12;
  blue[1] = 13;
  blue[2] = 14;
  blue[3] = 15;
  red[0] = 16;
  red[1] = 17;
  red[2] = 18;
  red[3] = 19;
  headlight1 = 10;
  headlight2 = 11;

  for(int i=0;i<4;i++) {
    pinMode(blue[i], OUTPUT);
    pinMode(red[i], OUTPUT);
  }
  pinMode(headlight1, OUTPUT);
  pinMode(headlight2, OUTPUT);
  pinMode(TEST_BUTTON, INPUT);
  digitalWrite(TEST_BUTTON, HIGH);

  randomSeed(analogRead(0));

  // Allow EM-406a to power up
  delay(3000);

  // Establish serial connection to EM-406a
  nss.begin(4800);
}



void setupSpeedZones() {
  // Rockford Road
  speedZones[0] = &SpeedZone(45);
  speedZones[0]->setVertices(6, (Vertex[6]){
      Vertex(45.027162772967756, -93.48137855529785),
      Vertex(45.02946790848425, -93.4742546081543),
      Vertex(45.02955889877115, -93.46193790435791),
      Vertex(45.02861865883124, -93.46172332763672),
      Vertex(45.02861865883124, -93.47412586212158),
      Vertex(45.02649547957147, -93.48133563995361)});


  // Schmidt Lake Rd
  speedZones[1] = &SpeedZone(45);
  speedZones[1]->setVertices(10, (Vertex[10]){
      Vertex(45.044176126280206, -93.48219394683838),
      Vertex(45.04390322470628, -93.47322463989258),
      Vertex(45.043387740403595, -93.46974849700928),
      Vertex(45.0440548368525, -93.46498489379883),
      Vertex(45.0440548368525, -93.46185207366943),
      Vertex(45.04332709488612, -93.46185207366943),
      Vertex(45.0433574176529, -93.46580028533936),
      Vertex(45.04272063617576, -93.46983432769775),
      Vertex(45.043266449304376, -93.47434043884277),
      Vertex(45.04332709488612, -93.48215103149414)});


  // Vicksburg Lane
  speedZones[2] = &SpeedZone(50);
  speedZones[2]->setVertices(4, (Vertex[4]){
      Vertex(45.042993543391, -93.48219394683838),
      Vertex(45.04320580365836, -93.4812068939209),
      Vertex(45.02722343561804, -93.48116397857666),
      Vertex(45.02725376691909, -93.4823226928711)});
		

  // I-494
  speedZones[3] = &SpeedZone(65);
  speedZones[3]->setVertices(8, (Vertex[8]){
      Vertex(45.04453999302037, -93.45356941223145),
      Vertex(45.03950629768142, -93.45369815826416),
      Vertex(45.029255597252295, -93.45378398895264),
      Vertex(45.02594950646131, -93.45386981964111),
      Vertex(45.02616182995624, -93.4522819519043),
      Vertex(45.028921963725224, -93.45168113708496),
      Vertex(45.0390514238751, -93.4520673751831),
      Vertex(45.044267093182235, -93.45150947570801)});
		

  // Fernbrook Lane
  speedZones[4] = &SpeedZone(40);
  speedZones[4]->setVertices(4, (Vertex[4]){
      Vertex(45.04317548081123, -93.4621524810791),
      Vertex(45.04332709488612, -93.46099376678467),
      Vertex(45.029680218928654, -93.46065044403076),
      Vertex(45.02961955888205, -93.46189498901367)});

		
  // Bounding rectangle for residential areas
  // This should be defined last in the list because it's the "catch all" speed zone.
  speedZones[5] = &SpeedZone(25);
  speedZones[5]->setVertices(4, (Vertex[4]){
      Vertex(45.045783186920296, -93.48395347595215),
      Vertex(45.0456315793546, -93.44983577728271),
      Vertex(45.02585851043667, -93.45009326934814),
      Vertex(45.02594950646131, -93.48326683044434)});


  if (debugMode) {
    printSpeedZones();
  }

}

/*
 * This is the point-in-polygon algorithm adapted from
 * http://www.ecse.rpi.edu/Homepages/wrf/Research/Short_Notes/pnpoly.html
 */
boolean inSpeedZone(int speedZone, float lat, float lng) {
  SpeedZone *s = speedZones[speedZone];

  int i, j;
  boolean inside = false;
  for (i = 0, j = s->nVertices-1; i < s->nVertices; j = i++) {
    if ( ((s->vertices[i].lat > lat) != (s->vertices[j].lat > lat)) &&
         (lng < (s->vertices[j].lng - s->vertices[i].lng) * (lat - s->vertices[i].lat) / (s->vertices[j].lat - s->vertices[i].lat) + s->vertices[i].lng) )
       inside = !inside;
  }

  return inside;
}


boolean inSpeedZone(int speedZone) {
  float lat, lng;
  unsigned long  fix_age;

  // retrieves +/- lat/long in 100,000ths of a degree
  gps.f_get_position(&lat, &lng, &fix_age);

  return inSpeedZone(speedZone, lat, lng);
}

void loop() {

  if (readGPS()) {
    if (debugMode) {
      debug();
    }
    speeding = isSpeeding();
  }

  if (digitalRead(TEST_BUTTON) == LOW) {
    buttonPressed = true;
  } else {
    buttonPressed = false;
  }
  if ((speeding || buttonPressed) && (!(speeding && buttonPressed))) {
    policeLights();
  } else {
    allOff();
  }

}

bool readGPS() {
  while (nss.available()) {
    if (gps.encode(nss.read())) {
      return true;
    }
  }
  return false;
}

  
int getSpeedLimit() {
  boolean isInSpeedZone;

  for(int i=0;i<NSPEEDZONES;i++) {
    isInSpeedZone = inSpeedZone(i);
    if (isInSpeedZone) {
      return speedZones[i]->speedLimit;
    }
  }
  return DEFAULT_SPEED_LIMIT;
}

boolean isSpeeding() {
  int speed = (int)(gps.f_speed_mph() + 0.5);
  int speedLimit = getSpeedLimit();

  if (speed > speedLimit) {
    return true;
  }
  return false;
}

boolean policeLights() {

  // The white LEDs are so bright we actually turn them down quite a bit.
  // 0 is full brightness and 255 is off (because the cathode is connected to the PWM pin)
  analogWrite(headlight2, 200);
  analogWrite(headlight1, 200); 

  allOn();

  // Turn off some of the red and blue lights to give them a flashing effect.
  for(int i=0;i<NUM_OFF;i++) {
    digitalWrite(blue[random(4)], HIGH);
    digitalWrite(red[random(4)], HIGH);
  }
  delay(DELAY);

}

void allOn() {
  for(int i=0;i<4;i++) {
    digitalWrite(blue[i], LOW);
    digitalWrite(red[i], LOW);
  }
}

void allOff() {
  for(int i=0;i<4;i++) {
    digitalWrite(blue[i], HIGH);
    digitalWrite(red[i], HIGH);
  }
  digitalWrite(headlight1, HIGH);
  digitalWrite(headlight2, HIGH);
}

void printSpeedZones() {

  for(int i=0;i<NSPEEDZONES;i++) {
    SpeedZone *s = speedZones[i];
    Serial.println(s->speedLimit);
    for(int v=0;v<s->nVertices;v++) {
      Serial.print("(");
      Serial.print(s->vertices[v].lat);
      Serial.print(", ");
      Serial.print(s->vertices[v].lng);
      Serial.println(")");
    }
  }
}

void debug() {
  long lat, lon;
  unsigned long fix_age, time, date, speed, course;

  // retrieves +/- lat/long in 100000ths of a degree
  gps.get_position(&lat, &lon, &fix_age);

  Serial.println(getSpeedLimit());

  Serial.print("lat: ");
  Serial.print(lat);
  Serial.print("    lng: ");
  Serial.print(lon);
  Serial.print("    speed: ");
  Serial.println(gps.f_speed_mph());
}

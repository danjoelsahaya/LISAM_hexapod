#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <math.h>

/*
~front~
A (T1,E1,pwm0:4,2,0)    D (T2,E1,pwm0:5,3,1)
B (T2,E2,pwm1:2,1,0)    E (T1,E2,pwm0:11,12,13)
C (T1,E3,pwm1:10,12,14) F (T2,E3,pwm1:11,13,15)
~back~

4 points of leg cycle:
- Step High (<--Drag Back)      updateAngleXX(-strideLength/2.0+xCycloid(startMillisRatio*M_PI),0,0+zCycloid(startMillisRatio*M_PI),angle);//curX1+diffX instead of curX1 for leg raise using ellipse instead of cycloidal
- Drag Forward (<--Step High)   updateAngleXX(-strideLength/2.0+xCycloid(M_PI+(startMillisRatio*M_PI)),0,0+zCycloid(M_PI+(startMillisRatio*M_PI)),angle+degToRad*55.0);//curX1+diffX instead of curX1 for leg raise using ellipse instead of cycloidal
- Drag Middle (<--Drag Forward) updateAngleXX(+strideLength/2.0-strideLength/2.0*startMillisRatio,0,0,angle);
- Drag Back (<--Drag Middle)    updateAngleXX(-(float)(strideLength/2.0*startMillisRatio),0,0,angle);
*/

//constants
const float MIDDLE=0.0; //starting x coordinate
const float NORMAL=65*2.0; //starting y coordinate
const float DOWN=-165.0; //starting z coordinate
const float tibiaLen=141.341; //length of the tibia from pivot to pivot
const float femurLen=78.70; //length of the femur from pivot to pivot
const float coxaLen=59.58; //length of coxa from pivot to pivot
const float radToDeg=180.0/M_PI; //self explanatory, multiple to a radian to convert to degrees
const float degToRad=M_PI/180.0; //self explanatory, multiple to a degree to convert to radians
const float magStart=0.1;
const int SERVOMIN=150; // this is the 'minimum' pulse length count (out of 4096); originally 125
const int SERVOMAX=600; // this is the 'maximum' pulse length count (out of 4096); originally 625
const int USMIN=500; // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
const int USMAX=2500; // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
const int SERVO_FREQ=50; // Analog servos run at ~50 Hz updates
#define AC 4
#define AF 2
#define AT 0
#define BC 2
#define BF 1
#define BT 0
#define CC 10
#define CF 12
#define CT 14
#define DC 5
#define DF 3
#define DT 1
#define EC 11
#define EF 12
#define ET 13
#define FC 11
#define FF 13
#define FT 15

float strideLength=125.0;//mm of stride length per full walk cycle
float strideHeight=50.0;//mm of stride height per full walk cycle
float startHeight=125.0; //add to DOWN to be off position
float xOffsetFromBody=MIDDLE; //distance from COG to foot point in x-axis relative to foot point
float yOffsetFromBody=54.805*2+NORMAL; //distance from COG to foot point in y-axis relative to foot point
float zOffsetFromBody=DOWN; //distance from COG to foot point in z-axis relative to foot point

//inverse kinematics initial implementation
float legLen=NORMAL; //length of the leg away from the body; aka L1
float cAngle=atan2(MIDDLE,NORMAL); //calculate angle of coxa servo movement
float l2=sqrt(pow(DOWN,2)+pow(legLen-coxaLen,2));//diagonal that connects coxa-femur point to tip of tibia
float tAngle=acos(((tibiaLen*tibiaLen)+(femurLen*femurLen)-(l2*l2))/(2*tibiaLen*femurLen)); //calculate angle of tibia servo movement
float fAngle=acos(DOWN/l2)+acos(((femurLen*femurLen)+(l2*l2)-(tibiaLen*tibiaLen))/(2*femurLen*l2)); //calculate angle for femur movement

//timing declaration
unsigned long startMillis; 
unsigned long currentMillis;
float startMillisRatio;
long legCycleTime = 1000/((long)4);  //750/((long)4) as min; milliseconds taken for leg to complete one full walk cycle
long postCommandDelay=15;//15 as min; hard coded delay default delay time post single command; must using due to not knowing position of servo as moving; ideally should be 2.833... as one degree takes 2.8333mSec according to data sheet

//radio declaration & initialization
RF24 radio(9, 10);  // CE, CSN
const byte address[6] = "00001";
char receivedData[32] = "";
float angle=0.0;
float prevAng=angle;
float mag=1.0;
float swapGait=0;

//PCA9685 declaration
Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver(0x41);
Adafruit_PWMServoDriver pwm0 = Adafruit_PWMServoDriver(0x40);

void setup() {
  Serial.begin(9600); //used to check for prints and debug code
  //radio setup
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();
  //PCA setup
  pwm1.begin();
  pwm0.begin();
  pwm1.setOscillatorFrequency(27000000);
  pwm1.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
  pwm0.setOscillatorFrequency(27000000);
  pwm0.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
  //put hexapod to startup position and have it wait for radio to be enabled to run loop
  delay(1000);
  //updateAngle(0, 0, 0);//+startHeight
  //offOn();
}

void loop() {
  inverseKinematicsTest(); //use to test legs
  /*if (swapGait==1){
    tripod(); //standard tripod gait ACE-BDF
  } 
  else if (swapGait==2){
    tetrapod(); //standard tetrapod gait DC-AF-BE
  }
  else{
    ripple(); //standard ripple gait A-E-C-D-B-F
  }*/
}

void tetrapod(){
  //e1 drag middle --> drag back --> step high --> drag forward
  //e2 step high --> drag forward --> drag middle --> drag back 
  //e3 drag back --> step high --> drag forward --> drag middle 
  angleUpdate(); //check if user wants movement, 0 is base state, .01-6.28 is movement, 10.00 is reset to standby and will return as 0, 99.99 is off/on, freezing code till on again will return as 0
  mag=1.0;
  while (angle!=0){
    angle+=-M_PI/2.0; //compensate for controller & hexapod rotational matrix by setting forward on joystick be forward in movement 
    prevAng=angle;
    startMillis = millis();  //initial start time
    recalculateRatio(); //sets ratio to 0, therefore putting leg cycle at start
    mag=1.0;
    while ((currentMillis - startMillis) < legCycleTime){ //E1 drag mid, E2 step high, E3 drag back
      updateAngleE1(+strideLength/2.0-strideLength/2.0*startMillisRatio,0,0,angle);
      updateAngleE2(-strideLength/2.0+xCycloid(startMillisRatio*M_PI),0,0+zCycloid(startMillisRatio*M_PI),angle);//curX1+diffX instead of curX1 for leg raise using ellipse instead of cycloidal
      updateAngleE3(-(float)(strideLength/2.0*startMillisRatio),0,0,angle);
      delay(postCommandDelay); 
      recalculateRatio(); //progresses ratio by time passed in order to decide next step in walking trajectory
    }
    angleUpdate(); //check if angle changes
    if (angle==0) //break if user stops wanting movement 
      break;
    if (abs(prevAng+M_PI/2.0-angle)>M_PI/6.0)
      mag=magStart;
    angle+=-M_PI/2.0; //compensate for controller & hexapod rotational matrix by setting forward on joystick be forward in movement 
    prevAng=angle;
    startMillis = millis();  //initial start time
    recalculateRatio(); //sets ratio to 0, therefore putting leg cycle at start
    while ((currentMillis - startMillis) < legCycleTime){ //E1 drag back, E2 drag forward, E3 step high
      updateAngleE1(-(float)(strideLength/2.0*startMillisRatio),0,0,angle);
      updateAngleE2(-strideLength/2.0+xCycloid(M_PI+(startMillisRatio*M_PI)),0,0+zCycloid(M_PI+(startMillisRatio*M_PI)),angle);//curX1+diffX instead of curX1 for leg raise using ellipse instead of cycloidal
      updateAngleE3(-strideLength/2.0+xCycloid(startMillisRatio*M_PI),0,0+zCycloid(startMillisRatio*M_PI),angle);//curX1+diffX instead of curX1 for leg raise using ellipse instead of cycloidal
      delay(postCommandDelay); 
      updateMag();
      recalculateRatio(); //progresses ratio by time passed in order to decide next step in walking trajectory
    }
    angleUpdate(); //check if angle changes
    if (angle==0) //break if user stops wanting movement 
      break;
    if (abs(prevAng+M_PI/2.0-angle)>M_PI/6.0)
      mag=magStart;
    angle+=-M_PI/2.0; //compensate for controller & hexapod rotational matrix by setting forward on joystick be forward in movement 
    prevAng=angle;
    startMillis = millis();  //initial start time
    recalculateRatio(); //sets ratio to 0, therefore putting leg cycle at start
    while ((currentMillis - startMillis) < legCycleTime){ //E1 step high, E2 drag middle, E3 drag forward
      updateAngleE1(-strideLength/2.0+xCycloid(startMillisRatio*M_PI),0,0+zCycloid(startMillisRatio*M_PI),angle);
      updateAngleE2(+strideLength/2.0-strideLength/2.0*startMillisRatio,0,0,angle);
      updateAngleE3(-strideLength/2.0+xCycloid(M_PI+(startMillisRatio*M_PI)),0,0+zCycloid(M_PI+(startMillisRatio*M_PI)),angle);
      delay(postCommandDelay); 
      updateMag();
      recalculateRatio(); //progresses ratio by time passed in order to decide next step in walking trajectory
      
    }
    angleUpdate(); //check if angle changes
    if (angle==0) //break if user stops wanting movement 
      break;
    if (abs(prevAng+M_PI/2.0-angle)>M_PI/6.0){
      mag=magStart;
    }
    angle+=-M_PI/2.0; //compensate for controller & hexapod rotational matrix by setting forward on joystick be forward in movement 
    prevAng=angle;
    startMillis = millis();  //initial start time
    recalculateRatio(); //sets ratio to 0, therefore putting leg cycle at start
    while ((currentMillis - startMillis) < legCycleTime){ //E1 drag forward, E2 drag back, E3 drag middle
      updateAngleE1(-strideLength/2.0+xCycloid(M_PI+(startMillisRatio*M_PI)),0,0+zCycloid(M_PI+(startMillisRatio*M_PI)),angle);//curX1+diffX instead of curX1 for leg raise using ellipse instead of cycloidal
      updateAngleE2(-(float)(strideLength/2.0*startMillisRatio),0,0,angle);
      updateAngleE3(+strideLength/2.0-strideLength/2.0*startMillisRatio,0,0,angle);
      delay(postCommandDelay); 
      updateMag();
      recalculateRatio(); //progresses ratio by time passed in order to decide next step in walking trajectory
    }
    angleUpdate(); //check if angle changes
    if (angle==0) //break if user stops wanting movement 
      break;
    if (abs(prevAng+M_PI/2.0-angle)>M_PI/6.0){
      mag=magStart;
    }
  }
}

void ripple(){ //A --> E --> C --> D --> B --> F
  angleUpdate(); //check if user wants movement, 0 is base state, .01-6.28 is movement, 10.00 is reset to standby and will return as 0, 99.99 is off/on, freezing code till on again and will return as 0
  mag=1.0;
  while (angle!=0){
    angle+=-M_PI/2.0; //compensate for controller & hexapod rotational matrix by setting forward on joystick be forward in movement 
    prevAng=angle;
    startMillis = millis();  //initial start time
    recalculateRatio(); //sets ratio to 0, therefore putting leg cycle at start
    while ((currentMillis - startMillis) < legCycleTime/mag){ //A step high, F drag forward, B drag back 1/4, D drag back 2/4, C drag back 3/4, E drag back 4/4
      updateAngleA(-strideLength/2.0+xCycloid(startMillisRatio*M_PI),0,0+zCycloid(startMillisRatio*M_PI),-angle-degToRad*55.0);//curX1+diffX instead of curX1 for leg raise using ellipse instead of cycloidal
      updateAngleF(-strideLength/2.0+xCycloid(M_PI+(startMillisRatio*M_PI)),0,0+zCycloid(M_PI+(startMillisRatio*M_PI)),angle+degToRad*55.0);//curX1+diffX instead of curX1 for leg raise using ellipse instead of cycloidal
      updateAngleB(+strideLength/2.0-strideLength/4.0*startMillisRatio,0,0,-angle);
      updateAngleD(+strideLength/2.0-strideLength/4.0-strideLength/4.0*startMillisRatio,0,0,angle-degToRad*55.0);
      updateAngleC(+strideLength/2.0-strideLength*2.0/4.0-strideLength/4.0*startMillisRatio,0,0,-angle+degToRad*55.0);
      updateAngleE(+strideLength/2.0-strideLength*3.0/4.0-strideLength/4.0*startMillisRatio,0,0,angle);
      delay(postCommandDelay); 
      updateMag();
      recalculateRatio(); //progresses ratio by time passed in order to decide next step in walking trajectory
    }
    angleUpdate(); //check if angle changes
    if (angle==0) //break if user stops wanting movement 
      break;
    if (abs(prevAng+M_PI/2.0-angle)>M_PI/6.0)
      mag=magStart;
    angle+=-M_PI/2.0; //compensate for controller & hexapod rotational matrix by setting forward on joystick be forward in movement 
    prevAng=angle;
    startMillis = millis();  //initial start time
    recalculateRatio(); //sets ratio to 0, therefore putting leg cycle at start
    while ((currentMillis - startMillis) < legCycleTime/mag){ //E step high, A drag forward, F drag back 1/4, B drag back 2/4, D drag back 3/4, C drag back 4/4
      updateAngleE(-strideLength/2.0+xCycloid(startMillisRatio*M_PI),0,0+zCycloid(startMillisRatio*M_PI),angle);//curX1+diffX instead of curX1 for leg raise using ellipse instead of cycloidal
      updateAngleA(-strideLength/2.0+xCycloid(M_PI+(startMillisRatio*M_PI)),0,0+zCycloid(M_PI+(startMillisRatio*M_PI)),-angle-degToRad*55.0);//curX1+diffX instead of curX1 for leg raise using ellipse instead of cycloidal
      updateAngleF(+strideLength/2.0-strideLength/4.0*startMillisRatio,0,0,angle+degToRad*55.0);
      updateAngleB(+strideLength/2.0-strideLength/4.0-strideLength/4.0*startMillisRatio,0,0,-angle);
      updateAngleD(+strideLength/2.0-strideLength*2.0/4.0-strideLength/4.0*startMillisRatio,0,0,angle-degToRad*55.0);
      updateAngleC(+strideLength/2.0-strideLength*3.0/4.0-strideLength/4.0*startMillisRatio,0,0,-angle+degToRad*55.0);
      delay(postCommandDelay); 
      updateMag();
      recalculateRatio(); //progresses ratio by time passed in order to decide next step in walking trajectory
    }
    angleUpdate(); //check if angle changes
    if (angle==0) //break if user stops wanting movement 
      break;
    if (abs(prevAng+M_PI/2.0-angle)>M_PI/6.0)
      mag=magStart;
    angle+=-M_PI/2.0; //compensate for controller & hexapod rotational matrix by setting forward on joystick be forward in movement 
    prevAng=angle;
    startMillis = millis();  //initial start time
    recalculateRatio(); //sets ratio to 0, therefore putting leg cycle at start
    while ((currentMillis - startMillis) < legCycleTime/mag){ //C step high, E drag forward, A drag back 1/4, F drag back 2/4, B drag back 3/4, D drag back 4/4
      updateAngleC(-strideLength/2.0+xCycloid(startMillisRatio*M_PI),0,0+zCycloid(startMillisRatio*M_PI),-angle+degToRad*55.0);//curX1+diffX instead of curX1 for leg raise using ellipse instead of cycloidal
      updateAngleE(-strideLength/2.0+xCycloid(M_PI+(startMillisRatio*M_PI)),0,0+zCycloid(M_PI+(startMillisRatio*M_PI)),angle);//curX1+diffX instead of curX1 for leg raise using ellipse instead of cycloidal
      updateAngleA(+strideLength/2.0-strideLength/4.0*startMillisRatio,0,0,-angle-degToRad*55.0);
      updateAngleF(+strideLength/2.0-strideLength/4.0-strideLength/4.0*startMillisRatio,0,0,angle+degToRad*55.0);
      updateAngleB(+strideLength/2.0-strideLength*2.0/4.0-strideLength/4.0*startMillisRatio,0,0,-angle);
      updateAngleD(+strideLength/2.0-strideLength*3.0/4.0-strideLength/4.0*startMillisRatio,0,0,angle-degToRad*55.0);
      delay(postCommandDelay); 
      updateMag();
      recalculateRatio(); //progresses ratio by time passed in order to decide next step in walking trajectory
    }
    angleUpdate(); //check if angle changes
    if (angle==0) //break if user stops wanting movement 
      break;
    if (abs(prevAng+M_PI/2.0-angle)>M_PI/6.0)
      mag=magStart;
    angle+=-M_PI/2.0; //compensate for controller & hexapod rotational matrix by setting forward on joystick be forward in movement 
    prevAng=angle;
    startMillis = millis();  //initial start time
    recalculateRatio(); //sets ratio to 0, therefore putting leg cycle at start
    while ((currentMillis - startMillis) < legCycleTime/mag){ //D step high, C drag forward, E drag back 1/4, A drag back 2/4, F drag back 3/4, B drag back 4/4
      updateAngleD(-strideLength/2.0+xCycloid(startMillisRatio*M_PI),0,0+zCycloid(startMillisRatio*M_PI),angle-degToRad*55.0);//curX1+diffX instead of curX1 for leg raise using ellipse instead of cycloidal
      updateAngleC(-strideLength/2.0+xCycloid(M_PI+(startMillisRatio*M_PI)),0,0+zCycloid(M_PI+(startMillisRatio*M_PI)),-angle+degToRad*55.0);//curX1+diffX instead of curX1 for leg raise using ellipse instead of cycloidal
      updateAngleE(+strideLength/2.0-strideLength/4.0*startMillisRatio,0,0,angle);
      updateAngleA(+strideLength/2.0-strideLength/4.0-strideLength/4.0*startMillisRatio,0,0,-angle-degToRad*55.0);
      updateAngleF(+strideLength/2.0-strideLength*2.0/4.0-strideLength/4.0*startMillisRatio,0,0,angle+degToRad*55.0);
      updateAngleB(+strideLength/2.0-strideLength*3.0/4.0-strideLength/4.0*startMillisRatio,0,0,-angle);
      delay(postCommandDelay); 
      updateMag();
      recalculateRatio(); //progresses ratio by time passed in order to decide next step in walking trajectory
    }
    angleUpdate(); //check if angle changes
    if (angle==0) //break if user stops wanting movement 
      break;
    if (abs(prevAng+M_PI/2.0-angle)>M_PI/6.0)
      mag=magStart;
    angle+=-M_PI/2.0; //compensate for controller & hexapod rotational matrix by setting forward on joystick be forward in movement 
    prevAng=angle;    startMillis = millis();  //initial start time
    recalculateRatio(); //sets ratio to 0, therefore putting leg cycle at start
    while ((currentMillis - startMillis) < legCycleTime/mag){ //B step high, D drag forward, C drag back 1/4, E drag back 2/4, A drag back 3/4, F drag back 4/4
      updateAngleB(-strideLength/2.0+xCycloid(startMillisRatio*M_PI),0,0+zCycloid(startMillisRatio*M_PI),-angle);//curX1+diffX instead of curX1 for leg raise using ellipse instead of cycloidal
      updateAngleD(-strideLength/2.0+xCycloid(M_PI+(startMillisRatio*M_PI)),0,0+zCycloid(M_PI+(startMillisRatio*M_PI)),angle-degToRad*55.0);//curX1+diffX instead of curX1 for leg raise using ellipse instead of cycloidal
      updateAngleC(+strideLength/2.0-strideLength/4.0*startMillisRatio,0,0,-angle+degToRad*55.0);
      updateAngleE(+strideLength/2.0-strideLength/4.0-strideLength/4.0*startMillisRatio,0,0,angle);
      updateAngleA(+strideLength/2.0-strideLength*2.0/4.0-strideLength/4.0*startMillisRatio,0,0,-angle-degToRad*55.0);
      updateAngleF(+strideLength/2.0-strideLength*3.0/4.0-strideLength/4.0*startMillisRatio,0,0,angle+degToRad*55.0);
      delay(postCommandDelay); 
      updateMag();
      recalculateRatio(); //progresses ratio by time passed in order to decide next step in walking trajectory
    }
    angleUpdate(); //check if angle changes
    if (angle==0) //break if user stops wanting movement 
      break;
    if (abs(prevAng+M_PI/2.0-angle)>M_PI/6.0)
      mag=magStart;
    angle+=-M_PI/2.0; //compensate for controller & hexapod rotational matrix by setting forward on joystick be forward in movement 
    prevAng=angle;
    startMillis = millis();  //initial start time
    recalculateRatio(); //sets ratio to 0, therefore putting leg cycle at start
    while ((currentMillis - startMillis) < legCycleTime/mag){ //F step high, B drag forward, D drag back 1/4, C drag back 2/4, E drag back 3/4, A drag back 4/4
      updateAngleF(-strideLength/2.0+xCycloid(startMillisRatio*M_PI),0,0+zCycloid(startMillisRatio*M_PI),angle+degToRad*55.0);//curX1+diffX instead of curX1 for leg raise using ellipse instead of cycloidal
      updateAngleB(-strideLength/2.0+xCycloid(M_PI+(startMillisRatio*M_PI)),0,0+zCycloid(M_PI+(startMillisRatio*M_PI)),-angle);//curX1+diffX instead of curX1 for leg raise using ellipse instead of cycloidal
      updateAngleD(+strideLength/2.0-strideLength/4.0*startMillisRatio,0,0,angle-degToRad*55.0);
      updateAngleC(+strideLength/2.0-strideLength/4.0-strideLength/4.0*startMillisRatio,0,0,-angle+degToRad*55.0);
      updateAngleE(+strideLength/2.0-strideLength*2.0/4.0-strideLength/4.0*startMillisRatio,0,0,angle);
      updateAngleA(+strideLength/2.0-strideLength*3.0/4.0-strideLength/4.0*startMillisRatio,0,0,-angle-degToRad*55.0);
      delay(postCommandDelay); 
      updateMag();
      recalculateRatio(); //progresses ratio by time passed in order to decide next step in walking trajectory
    }
    angleUpdate(); //check if angle changes
    if (angle==0) //break if user stops wanting movement 
      break;
    if (abs(prevAng+M_PI/2.0-angle)>M_PI/6.0)
      mag=magStart;
  }
}

void tripod(){
  //t1 drag middle --> drag back    --> step high   --> drag forward --> drag middle
  //t2 step high   --> drag forward --> drag middle --> drag back    --> step high  
  angleUpdate(); //check if user wants movement, 0 is base state, .01-6.28 is movement, 10.00 is reset to standby and will return as 0, 99.99 is off/on and will freeze code till on again and will return as ang=0
  mag=1.0;
  while (angle!=0){
    angle+=-M_PI/2.0; //compensate for controller & hexapod rotational matrix by setting forward on joystick be forward in movement 
    prevAng=angle;
    startMillis = millis();  //initial start time
    recalculateRatio(); //sets ratio to 0, therefore putting leg cycle at start
    while ((currentMillis - startMillis) < legCycleTime){ //t1 drag back, t2 drag forward
      updateAngleT1(-(float)(strideLength/2.0*startMillisRatio),0,0,angle);
      updateAngleT2(-strideLength/2.0+xCycloid(M_PI+(startMillisRatio*M_PI)),0,0+zCycloid(M_PI+(startMillisRatio*M_PI)),angle); //curX2+diffX instead of curX2 for leg raise using ellipse instead of cycloidal
      delay(postCommandDelay); 
      updateMag();
      recalculateRatio(); //progresses ratio by time passed in order to decide next step in walking trajectory
    }
    angleUpdate(); //check if angle changes
    if (angle==0) //break if user stops wanting movement 
      break;
    if (abs(prevAng+M_PI/2.0-angle)>M_PI/6.0)
      mag=magStart;
    angle+=-M_PI/2.0; //compensate for controller & hexapod rotational matrix by setting forward on joystick be forward in movement 
    prevAng=angle;
    startMillis = millis();  //reset initial start time
    recalculateRatio(); //sets ratio to 0, therefore putting leg cycle at start
    while ((currentMillis - startMillis) < legCycleTime){ //t1 step high, t2 drag middle
      updateAngleT1(-strideLength/2.0+xCycloid(startMillisRatio*M_PI),0,0+zCycloid(startMillisRatio*M_PI),angle);//curX1+diffX instead of curX1 for leg raise using ellipse instead of cycloidal
      updateAngleT2(+strideLength/2.0-strideLength/2.0*startMillisRatio,0,0,angle);
      delay(postCommandDelay);
      updateMag();
      recalculateRatio(); //progresses ratio by time passed in order to decide next step in walking trajectory
    }
    angleUpdate(); //check if angle changes
    if (angle==0) //break if user stops wanting movement 
      break;
    if (abs(prevAng+M_PI/2.0-angle)>M_PI/6.0)
      mag=magStart;
    angle+=-M_PI/2.0; //compensate for controller & hexapod rotational matrix by setting forward on joystick be forward in movement 
    prevAng=angle;
    startMillis = millis();  //reset initial start time
    recalculateRatio(); //sets ratio to 0, therefore putting leg cycle at start
    while ((currentMillis - startMillis) < legCycleTime){ //t1 drag forward, t2 drag back
      updateAngleT1(-strideLength/2.0+xCycloid(M_PI+(startMillisRatio*M_PI)),0,0+zCycloid(M_PI+(startMillisRatio*M_PI)),angle);//curX1+diffX instead of curX1 for leg raise using ellipse instead of cycloidal
      updateAngleT2(-(float)(strideLength/2.0*startMillisRatio),0,0,angle);
      delay(postCommandDelay); 
      updateMag();
      recalculateRatio(); //progresses ratio by time passed in order to decide next step in walking trajectory
    }
    angleUpdate(); //check if angle changes
    if (angle==0) //break if user stops wanting movement 
      break;
    if (abs(prevAng+M_PI/2.0-angle)>M_PI/6.0)
      mag=magStart;
    angle+=-M_PI/2.0; //compensate for controller & hexapod rotational matrix by setting forward on joystick be forward in movement 
    prevAng=angle;
    startMillis = millis();  //reset initial start time
    recalculateRatio(); //sets ratio to 0, therefore putting leg cycle at start
    while ((currentMillis - startMillis) < legCycleTime){ //t1 drag middle, t2 step high
      updateAngleT1(+strideLength/2.0-strideLength/2.0*startMillisRatio,0,0,angle);
      updateAngleT2(-strideLength/2.0+xCycloid(startMillisRatio*M_PI),0,0+zCycloid(startMillisRatio*M_PI),angle); //curX2+diffX instead of curX2 for leg raise using ellipse instead of cycloidal
      delay(postCommandDelay);  
      updateMag();
      recalculateRatio(); //progresses ratio by time passed in order to decide next step in walking trajectory
    }
    angleUpdate(); //check if angle changes
    if (angle==0) //break if user stops wanting movement 
      break;
    if (abs(prevAng+M_PI/2.0-angle)>M_PI/6.0)
      mag=magStart;
  }
}

void inverseKinematicsTest(){
  postCommandDelay=15;
  float x=0;
  float y=0;
  float z=0;
  float zRot=0; //roll; rotation of leg test about z axis in RADIANS
  float yRot=0; //pitch; rotation of leg test about y axis in RADIANS
  float xRot=0; //yaw; rotation of leg test about x axis in RADIANS
  float zBR=0; //translation of the body about x axis in mm
  float yBR=0; //translation of the body about y axis in mm
  float xBR=0; //translation of the body about z axis in mm
  float repeat=1; //times each test repeats 
  for (int k=0;k<4;k++)
  {
    //updateAngleA(x-40,y,z,zRot,yRot,xRot,zBR,yBR,xBR);
    //updateAngle(x-40,y,z,zRot,yRot,xRot,zBR,yBR,xBR);
    for (int i=0; i<repeat;i++){
      for (x=-40; x<40; x+=4){
        updateAngleA(+x,y,z,zRot,yRot,xRot,zBR,yBR,xBR);
        //updateAngle(+x,y,z,zRot,yRot,xRot,zBR,yBR,xBR);
        delay(postCommandDelay);
      }
      for (x=40; x>-40; x-=4){
        updateAngleA(+x,y,z,zRot,yRot,xRot,zBR,yBR,xBR);
        //updateAngle(+x,y,z,zRot,yRot,xRot,zBR,yBR,xBR);
        delay(postCommandDelay);
      }
    }
    x=0;
    //updateAngleA(x,y-40,z,zRot,yRot,xRot,zBR,yBR,xBR);
    //updateAngle(x,y-40,z,zRot,yRot,xRot,zBR,yBR,xBR);
    for (int i=0; i<repeat;i++){
      for (y=0-40; y<0+40;y+=4){
        updateAngleA(x,y,z,zRot,yRot,xRot,zBR,yBR,xBR);
        //updateAngle(x,y,z,zRot,yRot,xRot,zBR,yBR,xBR);
        delay(postCommandDelay);
      }
      for (y=0+40; y>0-40;y-=4){
        updateAngleA(x,y,z,zRot,yRot,xRot,zBR,yBR,xBR);
        //updateAngle(x,y,z,zRot,yRot,xRot,zBR,yBR,xBR);
        delay(postCommandDelay);
      }
    }
    y=0;
    updateAngleA(x,y,z,zRot,yRot,xRot,zBR,yBR,xBR);
    //updateAngle(x,y,z,zRot,yRot,xRot,zBR,yBR,xBR);
    delay(postCommandDelay);
    for (int i=0; i<repeat;i++){
      for (z=0; z<0+60;z+=3){
        updateAngleA(x,y,z,zRot,yRot,xRot,zBR,yBR,xBR);
        //updateAngle(x,y,z,zRot,yRot,xRot,zBR,yBR,xBR);
        delay(postCommandDelay);
      }
      for (z=0+60; z>0;z-=3){
        updateAngleA(x,y,z,zRot,yRot,xRot,zBR,yBR,xBR);
        //updateAngle(x,y,z,zRot,yRot,xRot,zBR,yBR,xBR);
        delay(postCommandDelay);
      }
    }
    z=0;
  }
  //updateAngleA(0,0,0,zRot,yRot,xRot,zBR,yBR,xBR);
  delay(postCommandDelay*10);
  postCommandDelay=15;
}

void updateMag(){
  mag+=.1;
  if (mag>1)
    mag=1.0;
}

void angleUpdate(){//radio checker & updates angle to be returned to loop
  if (radio.available()) {  
    radio.read(&receivedData, sizeof(receivedData)); // Read the data and put it into character array
    angle = atoi(&receivedData[0]); // Convert the data from the character array (received X value) into integer -M_PI/2.0
    angle/=100;
    Serial.println(swapGait);
    delay(10);
    if (angle==10.00){//if Z button pressed, set hexapod to standby position and end walking
      standby();
      swapGait++;
      if (swapGait==3){
        swapGait=0;
      }
      angle=0;
      delay(100);
    }
    if (angle==99.99){//if C button pressed, run OnOff "interrupt" and end walking
      onOff();
      angle=0;
    }
  }
}

void updateAngleA(float x, float y, float z, float zR, float yR, float xR, float zBR, float yBR, float xBR){
  //Serial.print(x); Serial.print(" "); Serial.print(y); Serial.print(" "); Serial.print(z); Serial.println(" "); //pre-transformation coordinate
  xR-=degToRad*25.0;
  //y*=2;
  //float x1=(x+xOffsetFromBody)*(cos(yBR)*cos(zBR))+(y+yOffsetFromBody)*(sin(xBR)*sin(yBR)*cos(zBR)-cos(xBR)*sin(zBR))+(z+zOffsetFromBody)*(cos(xBR)*sin(yBR)*cos(zBR)+sin(xBR)*sin(zBR)); //rotation for body
  //float y1=(x+xOffsetFromBody)*(cos(yBR)*sin(zBR))+(y+yOffsetFromBody)*(sin(xBR)*sin(yBR)*sin(zBR)+cos(xBR)*cos(zBR))+(z+zOffsetFromBody)*(cos(xBR)*sin(yBR)*sin(zBR)-sin(xBR)*cos(zBR));
  //float z1=(x+xOffsetFromBody)*(-sin(yBR))+(y+yOffsetFromBody)*(sin(xBR)*cos(yBR))+(z+zOffsetFromBody)*(cos(xBR)*cos(yBR));
  float x1=x*(cos(yR)*cos(zR))+y*(sin(xR)*sin(yR)*cos(zR)-cos(xR)*sin(zR))+z*(cos(xR)*sin(yR)*cos(zR)+sin(xR)*sin(zR));//rotation for point in point 
  float y1=x*(cos(yR)*sin(zR))+y*(sin(xR)*sin(yR)*sin(zR)+cos(xR)*cos(zR))+z*(cos(xR)*sin(yR)*sin(zR)-sin(xR)*cos(zR));
  float z1=x*(-sin(yR))+y*(sin(xR)*cos(yR))+z*(cos(xR)*cos(yR));
  x=x1;
  y=y1;
  z=z1;
  x+=MIDDLE; //reverts x, y, z from (0,0,0) to MIDDLE, NORMAL, DOWN post-translation
  y+=NORMAL; 
  z+=DOWN;
  //Serial.print(x); Serial.print(" "); Serial.print(y); Serial.print(" "); Serial.print(z); Serial.println(" "); //post-transformation coordinate
  legLen=sqrt(pow(y,2)+pow(x,2));
  l2=sqrt(pow(z,2)+pow(legLen-coxaLen,2));
  cAngle=radToDeg*atan2(x,y)+110.0;
  fAngle=60.0+radToDeg*acos((femurLen*femurLen+l2*l2-tibiaLen*tibiaLen)/(2*femurLen*l2))+radToDeg*atan2(z,legLen);
  tAngle=150.0-radToDeg*acos((tibiaLen*tibiaLen+femurLen*femurLen-l2*l2)/(2*tibiaLen*femurLen)); 
  //Serial.print(cAngle); Serial.print(" "); Serial.print(fAngle); Serial.print(" "); Serial.print(tAngle); Serial.println(" "); //display angle movements
  angAC(cAngle);
  angAF(fAngle);
  angAT(tAngle);
}

void updateAngleB(float x, float y, float z, float zR, float yR, float xR, float zBR, float yBR, float xBR){
  //Serial.print(x); Serial.print(" "); Serial.print(y); Serial.print(" "); Serial.print(z); Serial.println(" "); //pre-transformation coordinate
  xR-=degToRad*25.0;
  //y*=2;
  //float x1=(x+xOffsetFromBody)*(cos(yBR)*cos(zBR))+(y+yOffsetFromBody)*(sin(xBR)*sin(yBR)*cos(zBR)-cos(xBR)*sin(zBR))+(z+zOffsetFromBody)*(cos(xBR)*sin(yBR)*cos(zBR)+sin(xBR)*sin(zBR)); //rotation for body
  //float y1=(x+xOffsetFromBody)*(cos(yBR)*sin(zBR))+(y+yOffsetFromBody)*(sin(xBR)*sin(yBR)*sin(zBR)+cos(xBR)*cos(zBR))+(z+zOffsetFromBody)*(cos(xBR)*sin(yBR)*sin(zBR)-sin(xBR)*cos(zBR));
  //float z1=(x+xOffsetFromBody)*(-sin(yBR))+(y+yOffsetFromBody)*(sin(xBR)*cos(yBR))+(z+zOffsetFromBody)*(cos(xBR)*cos(yBR));
  float x1=x*(cos(yR)*cos(zR))+y*(sin(xR)*sin(yR)*cos(zR)-cos(xR)*sin(zR))+z*(cos(xR)*sin(yR)*cos(zR)+sin(xR)*sin(zR));//rotation for point in point 
  float y1=x*(cos(yR)*sin(zR))+y*(sin(xR)*sin(yR)*sin(zR)+cos(xR)*cos(zR))+z*(cos(xR)*sin(yR)*sin(zR)-sin(xR)*cos(zR));
  float z1=x*(-sin(yR))+y*(sin(xR+degToRad*25.0)*cos(yR))+z*(cos(xR+degToRad*25.0)*cos(yR));
  x=x1;
  y=y1;
  z=z1;
  x+=MIDDLE; //reverts x, y, z from (0,0,0) to MIDDLE, NORMAL, DOWN post-translation
  y+=NORMAL; 
  z+=DOWN;
  //Serial.print(x); Serial.print(" "); Serial.print(y); Serial.print(" "); Serial.print(z); Serial.println(" "); //post-transformation coordinate
  legLen=sqrt(pow(y,2)+pow(x,2));
  l2=sqrt(pow(z,2)+pow(legLen-coxaLen,2));
  cAngle=radToDeg*atan2(x,y)+90.0;
  fAngle=60.0+radToDeg*acos((femurLen*femurLen+l2*l2-tibiaLen*tibiaLen)/(2*femurLen*l2))+radToDeg*atan2(z,legLen);
  tAngle=180.0-radToDeg*acos((tibiaLen*tibiaLen+femurLen*femurLen-l2*l2)/(2*tibiaLen*femurLen)); 
  //Serial.print(cAngle); Serial.print(" "); Serial.print(fAngle); Serial.print(" "); Serial.print(tAngle); Serial.println(" "); //display angle movements
  angBC(cAngle);
  angBF(fAngle);
  angBT(tAngle);
}

void updateAngleC(float x, float y, float z, float zR, float yR, float xR, float zBR, float yBR, float xBR){
  //Serial.print(x); Serial.print(" "); Serial.print(y); Serial.print(" "); Serial.print(z); Serial.println(" "); //pre-transformation coordinate
  xR-=degToRad*25.0;
  //y*=2;
  //float x1=(x+xOffsetFromBody)*(cos(yBR)*cos(zBR))+(y+yOffsetFromBody)*(sin(xBR)*sin(yBR)*cos(zBR)-cos(xBR)*sin(zBR))+(z+zOffsetFromBody)*(cos(xBR)*sin(yBR)*cos(zBR)+sin(xBR)*sin(zBR)); //rotation for body
  //float y1=(x+xOffsetFromBody)*(cos(yBR)*sin(zBR))+(y+yOffsetFromBody)*(sin(xBR)*sin(yBR)*sin(zBR)+cos(xBR)*cos(zBR))+(z+zOffsetFromBody)*(cos(xBR)*sin(yBR)*sin(zBR)-sin(xBR)*cos(zBR));
  //float z1=(x+xOffsetFromBody)*(-sin(yBR))+(y+yOffsetFromBody)*(sin(xBR)*cos(yBR))+(z+zOffsetFromBody)*(cos(xBR)*cos(yBR));
  float x1=x*(cos(yR)*cos(zR))+y*(sin(xR)*sin(yR)*cos(zR)-cos(xR)*sin(zR))+z*(cos(xR)*sin(yR)*cos(zR)+sin(xR)*sin(zR));//rotation for point in point 
  float y1=x*(cos(yR)*sin(zR))+y*(sin(xR)*sin(yR)*sin(zR)+cos(xR)*cos(zR))+z*(cos(xR)*sin(yR)*sin(zR)-sin(xR)*cos(zR));
  float z1=x*(-sin(yR))+y*(sin(xR+degToRad*25.0)*cos(yR))+z*(cos(xR+degToRad*25.0)*cos(yR));
  x=x1;
  y=y1;
  z=z1;
  x+=MIDDLE; //reverts x, y, z from (0,0,0) to MIDDLE, NORMAL, DOWN post-translation
  y+=NORMAL; 
  z+=DOWN;
  //Serial.print(x); Serial.print(" "); Serial.print(y); Serial.print(" "); Serial.print(z); Serial.println(" "); //post-transformation coordinate
  legLen=sqrt(pow(y,2)+pow(x,2));
  l2=sqrt(pow(z,2)+pow(legLen-coxaLen,2));
  cAngle=radToDeg*atan2(x,y)+67.5;
  fAngle=60.0+radToDeg*acos((femurLen*femurLen+l2*l2-tibiaLen*tibiaLen)/(2*femurLen*l2))+radToDeg*atan2(z,legLen);
  tAngle=180.0-radToDeg*acos((tibiaLen*tibiaLen+femurLen*femurLen-l2*l2)/(2*tibiaLen*femurLen));
  //Serial.print(cAngle); Serial.print(" "); Serial.print(fAngle); Serial.print(" "); Serial.print(tAngle); Serial.println(" "); //display angle movements
  angCC(cAngle);
  angCF(fAngle);
  angCT(tAngle);
}

void updateAngleD(float x, float y, float z, float zR, float yR, float xR, float zBR, float yBR, float xBR){
  //Serial.print(x); Serial.print(" "); Serial.print(y); Serial.print(" "); Serial.print(z); Serial.println(" "); //pre-transformation coordinate
  xR-=degToRad*25.0;
  //y*=2;
  //float x1=(x+xOffsetFromBody)*(cos(yBR)*cos(zBR))+(y+yOffsetFromBody)*(sin(xBR)*sin(yBR)*cos(zBR)-cos(xBR)*sin(zBR))+(z+zOffsetFromBody)*(cos(xBR)*sin(yBR)*cos(zBR)+sin(xBR)*sin(zBR)); //rotation for body
  //float y1=(x+xOffsetFromBody)*(cos(yBR)*sin(zBR))+(y+yOffsetFromBody)*(sin(xBR)*sin(yBR)*sin(zBR)+cos(xBR)*cos(zBR))+(z+zOffsetFromBody)*(cos(xBR)*sin(yBR)*sin(zBR)-sin(xBR)*cos(zBR));
  //float z1=(x+xOffsetFromBody)*(-sin(yBR))+(y+yOffsetFromBody)*(sin(xBR)*cos(yBR))+(z+zOffsetFromBody)*(cos(xBR)*cos(yBR));
  float x1=x*(cos(yR)*cos(zR))+y*(sin(xR)*sin(yR)*cos(zR)-cos(xR)*sin(zR))+z*(cos(xR)*sin(yR)*cos(zR)+sin(xR)*sin(zR));//rotation for point in point 
  float y1=x*(cos(yR)*sin(zR))+y*(sin(xR)*sin(yR)*sin(zR)+cos(xR)*cos(zR))+z*(cos(xR)*sin(yR)*sin(zR)-sin(xR)*cos(zR));
  float z1=x*(-sin(yR))+y*(sin(xR+degToRad*25.0)*cos(yR))+z*(cos(xR+degToRad*25.0)*cos(yR));
  x=x1;
  y=y1;
  z=z1;
  x+=MIDDLE; //reverts x, y, z from (0,0,0) to MIDDLE, NORMAL, DOWN post-translation
  y+=NORMAL; 
  z+=DOWN+5;
  //Serial.print(x); Serial.print(" "); Serial.print(y); Serial.print(" "); Serial.print(z); Serial.println(" "); //post-transformation coordinate
  legLen=sqrt(pow(y,2)+pow(x,2));
  l2=sqrt(pow(z,2)+pow(legLen-coxaLen,2));
  cAngle=radToDeg*atan2(x,y)+95.0;
  fAngle=60.0+radToDeg*acos((femurLen*femurLen+l2*l2-tibiaLen*tibiaLen)/(2*femurLen*l2))+radToDeg*atan2(z,legLen);
  tAngle=180.0-radToDeg*acos((tibiaLen*tibiaLen+femurLen*femurLen-l2*l2)/(2*tibiaLen*femurLen)); 
  //Serial.print(cAngle); Serial.print(" "); Serial.print(fAngle); Serial.print(" "); Serial.print(tAngle); Serial.println(" "); //display angle movements
  angDC(180.0-cAngle);
  angDF(fAngle);
  angDT(tAngle);
}

void updateAngleE(float x, float y, float z, float zR, float yR, float xR, float zBR, float yBR, float xBR){
  //Serial.print(x); Serial.print(" "); Serial.print(y); Serial.print(" "); Serial.print(z); Serial.println(" "); //pre-transformation coordinate
  xR-=degToRad*25.0;
  //y*=2;
  //float x1=(x+xOffsetFromBody)*(cos(yBR)*cos(zBR))+(y+yOffsetFromBody)*(sin(xBR)*sin(yBR)*cos(zBR)-cos(xBR)*sin(zBR))+(z+zOffsetFromBody)*(cos(xBR)*sin(yBR)*cos(zBR)+sin(xBR)*sin(zBR)); //rotation for body
  //float y1=(x+xOffsetFromBody)*(cos(yBR)*sin(zBR))+(y+yOffsetFromBody)*(sin(xBR)*sin(yBR)*sin(zBR)+cos(xBR)*cos(zBR))+(z+zOffsetFromBody)*(cos(xBR)*sin(yBR)*sin(zBR)-sin(xBR)*cos(zBR));
  //float z1=(x+xOffsetFromBody)*(-sin(yBR))+(y+yOffsetFromBody)*(sin(xBR)*cos(yBR))+(z+zOffsetFromBody)*(cos(xBR)*cos(yBR));
  float x1=x*(cos(yR)*cos(zR))+y*(sin(xR)*sin(yR)*cos(zR)-cos(xR)*sin(zR))+z*(cos(xR)*sin(yR)*cos(zR)+sin(xR)*sin(zR));//rotation for point in point 
  float y1=x*(cos(yR)*sin(zR))+y*(sin(xR)*sin(yR)*sin(zR)+cos(xR)*cos(zR))+z*(cos(xR)*sin(yR)*sin(zR)-sin(xR)*cos(zR));
  float z1=x*(-sin(yR))+y*(sin(xR+degToRad*25.0)*cos(yR))+z*(cos(xR+degToRad*25.0)*cos(yR));
  x=x1;
  y=y1;
  z=z1;
  x+=MIDDLE; //reverts x, y, z from (0,0,0) to MIDDLE, NORMAL, DOWN post-translation
  y+=NORMAL; 
  z+=DOWN;
  //Serial.print(x); Serial.print(" "); Serial.print(y); Serial.print(" "); Serial.print(z); Serial.println(" "); //post-transformation coordinate
  legLen=sqrt(pow(y,2)+pow(x,2));
  l2=sqrt(pow(z,2)+pow(legLen-coxaLen,2));
  cAngle=radToDeg*atan2(x,y)+85.0;
  fAngle=60.0+radToDeg*acos((femurLen*femurLen+l2*l2-tibiaLen*tibiaLen)/(2*femurLen*l2))+radToDeg*atan2(z,legLen); 
  tAngle=180.0-radToDeg*acos((tibiaLen*tibiaLen+femurLen*femurLen-l2*l2)/(2*tibiaLen*femurLen)); 
  //Serial.print(cAngle); Serial.print(" "); Serial.print(fAngle); Serial.print(" "); Serial.print(tAngle); Serial.println(" "); //display angle movements
  angEC(180.0-cAngle);
  angEF(fAngle);
  angET(tAngle);
}

void updateAngleF(float x, float y, float z, float zR, float yR, float xR, float zBR, float yBR, float xBR){
  //Serial.print(x); Serial.print(" "); Serial.print(y); Serial.print(" "); Serial.print(z); Serial.println(" "); //pre-transformation coordinate
  xR-=degToRad*25.0;
  //y*=2;
  //float x1=(x+xOffsetFromBody)*(cos(yBR)*cos(zBR))+(y+yOffsetFromBody)*(sin(xBR)*sin(yBR)*cos(zBR)-cos(xBR)*sin(zBR))+(z+zOffsetFromBody)*(cos(xBR)*sin(yBR)*cos(zBR)+sin(xBR)*sin(zBR)); //rotation for body
  //float y1=(x+xOffsetFromBody)*(cos(yBR)*sin(zBR))+(y+yOffsetFromBody)*(sin(xBR)*sin(yBR)*sin(zBR)+cos(xBR)*cos(zBR))+(z+zOffsetFromBody)*(cos(xBR)*sin(yBR)*sin(zBR)-sin(xBR)*cos(zBR));
  //float z1=(x+xOffsetFromBody)*(-sin(yBR))+(y+yOffsetFromBody)*(sin(xBR)*cos(yBR))+(z+zOffsetFromBody)*(cos(xBR)*cos(yBR));
  float x1=x*(cos(yR)*cos(zR))+y*(sin(xR)*sin(yR)*cos(zR)-cos(xR)*sin(zR))+z*(cos(xR)*sin(yR)*cos(zR)+sin(xR)*sin(zR));//rotation for point in point 
  float y1=x*(cos(yR)*sin(zR))+y*(sin(xR)*sin(yR)*sin(zR)+cos(xR)*cos(zR))+z*(cos(xR)*sin(yR)*sin(zR)-sin(xR)*cos(zR));
  float z1=x*(-sin(yR))+y*(sin(xR+degToRad*25.0)*cos(yR))+z*(cos(xR+degToRad*25.0)*cos(yR));
  x=x1;
  y=y1;
  z=z1;
  x+=MIDDLE; //reverts x, y, z from (0,0,0) to MIDDLE, NORMAL, DOWN post-translation
  y+=NORMAL; 
  z+=DOWN;
  //Serial.print(x); Serial.print(" "); Serial.print(y); Serial.print(" "); Serial.print(z); Serial.println(" "); //post-transformation coordinate
  legLen=sqrt(pow(y,2)+pow(x,2));
  l2=sqrt(pow(z,2)+pow(legLen-coxaLen,2));
  cAngle=radToDeg*atan2(x,y)+90.0;
  fAngle=60.0+radToDeg*acos((femurLen*femurLen+l2*l2-tibiaLen*tibiaLen)/(2*femurLen*l2))+radToDeg*atan2(z,legLen);
  tAngle=180.0-radToDeg*acos((tibiaLen*tibiaLen+femurLen*femurLen-l2*l2)/(2*tibiaLen*femurLen)); 
  //Serial.print(cAngle); Serial.print(" "); Serial.print(fAngle); Serial.print(" "); Serial.print(tAngle); Serial.println(" "); //display angle movements
  angFC(180.0-cAngle);
  angFF(fAngle);
  angFT(tAngle);
}

float xCycloid(float time){ //returns x position using cycloidal given time [0,2pi]
  while (time<0)
    time+=M_PI*2.0;
  while (time>2.0*M_PI)
    time-=2.0*M_PI;
  return (float)(strideLength/(2.0*M_PI))*(float)(time-sin(time));//uses a cyclodical equation to return an optimal walking movement X-coordinate based on given length (strideLength/(2pi)) and time 0<=t<=2pi spaced out using startMillisRatio
}

float zCycloid(float time){ //returns z position using cycloidal given time [0,2pi]
  while (time<0)
    time+=M_PI*2.0;
  while (time>2.0*M_PI)
    time-=2.0*M_PI;
  return (float)(strideHeight/2.0)*(float)(1.0-cos(time));//uses a cyclodical equation to return an optimal walking movement Y-coordinate based on given height (strideHeight/2) and time 0<=t<=2pi spaced out using startMillisRatio
}

void recalculateRatio(){
  currentMillis = millis();  //get the current "time" (actually the number of milliseconds since the program started)
  startMillisRatio = (float)(currentMillis - startMillis)/((float)(legCycleTime)/mag); //ratio of time passed to total time in this part of cycle
  if (startMillisRatio>1) //check and send the maximum ratio of 1 if time passed is higher than time needed
    startMillisRatio=1;
}

/*bool sensorRun(){ //detect and return if SFR sensor is being pressed aka foot is touching the ground
  analog_input = analogRead(A0);
  Serial.print(analog_input); Serial.print(" ");
  analog_input = analog_input/(1023.0);
  if (analog_input>0.5){
    pressing=true;
  }
  else{
    pressing=false;
  }
  Serial.print(analog_input); Serial.print(" "); Serial.println(pressing);
}*/

/*float stepHigh(float x){//uses a half ellipse of height strideHeight and length strideLength
  return sqrt(strideHeight*strideHeight*(1.0-((4.0*x*x)/(strideLength*strideLength))));
}*/

void onOff(){ //when radio is disabled, reset position to standby, go to startup position, and move to offOn()
  standby();
  delay(250);
  for (int i=0;i<startHeight;i+=2){ updateAngle(0, 0, 0+i); }
  offOn();
}

void offOn(){
  while (angle==99.99){ //while radio continues to be disabled, just keep checking if the state changes
    delay(10);
    if (radio.available()) {  
      radio.read(&receivedData, sizeof(receivedData)); // Read the data and put it into character array
      angle = atoi(&receivedData[0]); // Convert the data from the character array (received X value) into integer
      angle/=100; //decrypt int into float
      Serial.println(angle);
      delay(10);
    }
  }
  for (int i=startHeight;i>0;i-=2){ updateAngle(0, 0, 0+i); } //once radio is enabled, go from startup position to standby
}

void setServoPulse(uint8_t n, double pulse) {
  double pulselength;
  pulselength = 1000000;   // 1,000,000 us per second
  pulselength /= SERVO_FREQ;   // Analog servos run at ~60 Hz updates
  Serial.print(pulselength); Serial.println(" us per period");
  pulselength /= 4096;  // 12 bits of resolution
  Serial.print(pulselength); Serial.println(" us per bit");
  pulse *= 1000000;  // convert input seconds to us
  pulse /= pulselength;
  Serial.println(pulse);
  pwm1.setPWM(n, 0, pulse);
  pwm0.setPWM(n, 0, pulse);
}

void updateAngleT1(float x, float y, float z, float zR, float yR, float xR, float zBR, float yBR, float xBR){ //controls A, C, E legs as part of tripod gait
  updateAngleA(x, y, z,-zR-degToRad*55.0,yR, xR, zBR, yBR, xBR);
  updateAngleC(x, y, z,-zR+degToRad*55.0,yR, xR, zBR, yBR, xBR);
  updateAngleE(x, y, z,zR, yR, xR, zBR, yBR, xBR);
}

void updateAngleT2(float x, float y, float z, float zR, float yR, float xR, float zBR, float yBR, float xBR){ //controls B, D, F legs as part of tripod gait
  updateAngleB(x, y, z,-zR, yR, xR, zBR, yBR, xBR);
  updateAngleD(x, y, z,zR-degToRad*55.0, yR, xR, zBR, yBR, xBR);
  updateAngleF(x, y, z,zR+degToRad*55.0, yR, xR, zBR, yBR, xBR);
}

void updateAngleE1(float x, float y, float z, float zR, float yR, float xR, float zBR, float yBR, float xBR){ //controls A, C, E legs as part of tripod gait
  updateAngleC(x, y, z,-zR+degToRad*55.0,yR, xR, zBR, yBR, xBR);
  updateAngleD(x, y, z,zR-degToRad*55.0, yR, xR, zBR, yBR, xBR);
}

void updateAngleE2(float x, float y, float z, float zR, float yR, float xR, float zBR, float yBR, float xBR){ //controls B, D, F legs as part of tripod gait
  updateAngleA(x, y, z,-zR-degToRad*55.0,yR, xR, zBR, yBR, xBR);
  updateAngleF(x, y, z,zR+degToRad*55.0, yR, xR, zBR, yBR, xBR);
}

void updateAngleE3(float x, float y, float z, float zR, float yR, float xR, float zBR, float yBR, float xBR){ //controls B, D, F legs as part of tripod gait
  updateAngleB(x, y, z,-zR, yR, xR, zBR, yBR, xBR);
  updateAngleE(x, y, z,zR, yR, xR, zBR, yBR, xBR);
}

void standby(){ //base state when actively waiting for commands from radio
  updateAngle(0, 0, 0);
  delay(postCommandDelay);
}

//overloaded methods
void updateAngleT1(float x, float y, float z, float zR){ updateAngleT1(x, y, z, zR, 0, 0, 0, 0, 0); }
void updateAngleT1(float x, float y, float z){ updateAngleT1(x, y, z, 0, 0, 0, 0, 0, 0); }
void updateAngleT2(float x, float y, float z, float zR){ updateAngleT2(x, y, z, zR, 0, 0, 0, 0, 0); }
void updateAngleT2(float x, float y, float z){ updateAngleT2(x, y, z, 0, 0, 0, 0, 0, 0); }
void updateAngleE1(float x, float y, float z, float zR){ updateAngleE1(x, y, z, zR, 0, 0, 0, 0, 0); }
void updateAngleE1(float x, float y, float z){ updateAngleE1(x, y, z, 0, 0, 0, 0, 0, 0); }
void updateAngleE2(float x, float y, float z, float zR){ updateAngleE2(x, y, z, zR, 0, 0, 0, 0, 0); }
void updateAngleE2(float x, float y, float z){ updateAngleE2(x, y, z, 0, 0, 0, 0, 0, 0); }
void updateAngleE3(float x, float y, float z, float zR){ updateAngleE3(x, y, z, zR, 0, 0, 0, 0, 0); }
void updateAngleE3(float x, float y, float z){ updateAngleE3(x, y, z, 0, 0, 0, 0, 0, 0); }
void updateAngle(float x, float y, float z, float zR, float yR, float xR, float zBR, float yBR, float xBR){ updateAngleA(x,y,z,zR,yR,xR,zBR,yBR,xBR); updateAngleB(x,y,z,zR,yR,xR,zBR,yBR,xBR); updateAngleC(x,y,z,zR,yR,xR,zBR,yBR,xBR); updateAngleD(x,y,z,zR,yR,xR,zBR,yBR,xBR); updateAngleE(x,y,z,zR,yR,xR,zBR,yBR,xBR); updateAngleF(x,y,z,zR,yR,xR,zBR,yBR,xBR); }
void updateAngle(float x, float y, float z, float zR, float yR, float xR){ updateAngleA(x, y, z, zR, yR, xR, 0, 0, 0); updateAngleB(x, y, z, zR, yR, xR, 0, 0, 0); updateAngleC(x, y, z, zR, yR, xR, 0, 0, 0); updateAngleD(x, y, z, zR, yR, xR, 0, 0, 0); updateAngleE(x, y, z, zR, yR, xR, 0, 0, 0); updateAngleF(x, y, z, zR, yR, xR, 0, 0, 0); }
void updateAngle(float x, float y, float z, float zR){ updateAngleA(x, y, z, zR, 0, 0, 0, 0, 0); updateAngleB(x, y, z, zR, 0, 0, 0, 0, 0); updateAngleC(x, y, z, zR, 0, 0, 0, 0, 0); updateAngleD(x, y, z, zR, 0, 0, 0, 0, 0); updateAngleE(x, y, z, zR, 0, 0, 0, 0, 0); updateAngleF(x, y, z, zR, 0, 0, 0, 0, 0); }
void updateAngle(float x, float y, float z){ updateAngleA(x, y, z, 0, 0, 0, 0, 0, 0); updateAngleB(x, y, z, 0, 0, 0, 0, 0, 0); updateAngleC(x, y, z, 0, 0, 0, 0, 0, 0); updateAngleD(x, y, z, 0, 0, 0, 0, 0, 0); updateAngleE(x, y, z, 0, 0, 0, 0, 0, 0); updateAngleF(x, y, z, 0, 0, 0, 0, 0, 0); }
void updateAngleA(float x, float y, float z, float zR, float yR, float xR){ updateAngleA(x, y, z, zR, yR, xR, 0, 0, 0); }
void updateAngleA(float x, float y, float z, float zR){ updateAngleA(x, y, z, zR, 0, 0, 0, 0, 0); }
void updateAngleA(float x, float y, float z){ updateAngleA(x, y, z, 0, 0, 0, 0, 0, 0); }
void updateAngleB(float x, float y, float z, float zR, float yR, float xR){ updateAngleB(x, y, z, zR, yR, xR, 0, 0, 0); }
void updateAngleB(float x, float y, float z, float zR){ updateAngleB(x, y, z, zR, 0, 0, 0, 0, 0); }
void updateAngleB(float x, float y, float z){ updateAngleB(x, y, z, 0, 0, 0, 0, 0, 0); }
void updateAngleC(float x, float y, float z, float zR, float yR, float xR){ updateAngleC(x, y, z, zR, yR, xR, 0, 0, 0); }
void updateAngleC(float x, float y, float z, float zR){ updateAngleC(x, y, z, zR, 0, 0, 0, 0, 0); }
void updateAngleC(float x, float y, float z){ updateAngleC(x, y, z, 0, 0, 0, 0, 0, 0); }
void updateAngleD(float x, float y, float z, float zR, float yR, float xR){ updateAngleD(x, y, z, zR, yR, xR, 0, 0, 0); }
void updateAngleD(float x, float y, float z, float zR){ updateAngleD(x, y, z, zR, 0, 0, 0, 0, 0); }
void updateAngleD(float x, float y, float z){ updateAngleD(x, y, z, 0, 0, 0, 0, 0, 0); }
void updateAngleE(float x, float y, float z, float zR, float yR, float xR){ updateAngleE(x, y, z, zR, yR, xR, 0, 0, 0); }
void updateAngleE(float x, float y, float z, float zR){ updateAngleE(x, y, z, zR, 0, 0, 0, 0, 0); }
void updateAngleE(float x, float y, float z){ updateAngleE(x, y, z, 0, 0, 0, 0, 0, 0); }
void updateAngleF(float x, float y, float z, float zR, float yR, float xR){ updateAngleF(x, y, z, zR, yR, xR, 0, 0, 0); }
void updateAngleF(float x, float y, float z, float zR){ updateAngleF(x, y, z, zR, 0, 0, 0, 0, 0); }
void updateAngleF(float x, float y, float z){ updateAngleF(x, y, z, 0, 0, 0, 0, 0, 0); }

//servo angle input method
int angMax=130;
void angAC(int angle){ pwm0.setPWM(AC, 0, angletopulse(angle)); }
void angDC(int angle){ pwm0.setPWM(DC, 0, angletopulse(angle)); }
void angEC(int angle){ pwm0.setPWM(EC, 0, angletopulse(angle)); }
void angBC(int angle){ pwm1.setPWM(BC, 0, angletopulse(angle)); }
void angCC(int angle){ pwm1.setPWM(CC, 0, angletopulse(angle)); }
void angFC(int angle){ pwm1.setPWM(FC, 0, angletopulse(angle)); }
void angAT(int angle){ if (angle>angMax){ angle=angMax; } pwm0.setPWM(AT, 0, angletopulse(angle)); }
void angDT(int angle){ if (angle>angMax){ angle=angMax; } pwm0.setPWM(DT, 0, angletopulse(angle)); }
void angET(int angle){ if (angle>angMax){ angle=angMax; } pwm0.setPWM(ET, 0, angletopulse(angle)); }
void angBT(int angle){ if (angle>angMax){ angle=angMax; } pwm1.setPWM(BT, 0, angletopulse(angle)); }
void angCT(int angle){ if (angle>angMax){ angle=angMax; } pwm1.setPWM(CT, 0, angletopulse(angle)); }
void angFT(int angle){ if (angle>angMax){ angle=angMax; } pwm1.setPWM(FT, 0, angletopulse(angle)); }
void angAF(int angle){ if (angle>150){ angle=150; } pwm0.setPWM(AF, 0, angletopulse(angle)); }
void angDF(int angle){ if (angle>150){ angle=150; } pwm0.setPWM(DF, 0, angletopulse(angle)); }
void angEF(int angle){ if (angle>150){ angle=150; } pwm0.setPWM(EF, 0, angletopulse(angle)); }
void angBF(int angle){ if (angle>140){ angle=140; } pwm1.setPWM(BF, 0, angletopulse(angle)); }
void angCF(int angle){ if (angle>140){ angle=140; } pwm1.setPWM(CF, 0, angletopulse(angle)); }
void angFF(int angle){ if (angle>140){ angle=140; } pwm1.setPWM(FF, 0, angletopulse(angle)); }

int angletopulse(int ang){ //gets angle in degree and returns the pulse width; used in initiallizing each servo method
  int pulse = map(ang,0, 180, SERVOMIN,SERVOMAX);  // map angle of 0 to 180 to Servo min and Servo max
  //Serial.print("Angle: ");Serial.print(ang);
  //Serial.print(" pulse: ");Serial.println(pulse);
  return pulse;
}
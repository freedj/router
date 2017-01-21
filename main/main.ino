#include <PS2X_lib.h>
/*-----( Import needed libraries )-----*/
#include <Wire.h>  // Comes with Arduino IDE
// Get the LCD I2C Library here:
// https://bitbucket.org/fmalpartida/new-liquidcrystal/downloads
// Move any other LCD libraries to another folder or delete them
// See Library "Docs" folder for possible commands etc.
#include <LiquidCrystal_I2C.h>

#define StepperEnable 12

#define StepperXStep 7
#define StepperXDir 6

#define StepperYStep 9
#define StepperYDir 8

#define StepperZStep 11
#define StepperZDir 10

#define xy_button_rate  127
#define z_button_rate 2048
#define xmax 800
#define ymax 800
#define zmax 800
#define cutMax 128

#define STEPOVER 32

/*-----( Declare Constants )-----*/
/*-----( Declare objects )-----*/
// set the LCD address to 0x27 for a 16 chars 2 line display
// A FEW use address 0x3F
// Set the pins on the I2C chip used for LCD connections:
//                    addr, en,rw,rs,d4,d5,d6,d7,bl,blpol
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address




PS2X ps2x; // create ps2 controller class
int error = 0;
byte type = 0;

int tryRunStep = 0;
int i = 0;
int timer1_counter;

long targetx = 0;
long targety = 0;
long targetz = 0;

long xsteps = 0;

byte serialDebug = false;
bool steppersOn = false;
long debugTime = millis();

bool xStepping = false;
bool yStepping = false;
bool zStepping = false;
byte nextStep = 0;

bool mdi = false;





#include <AccelStepper.h>
#include "Axis.h"


AccelStepper stepper(AccelStepper::DRIVER, StepperXStep, StepperXDir); // Defaults to AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5
AccelStepper stepper2(AccelStepper::DRIVER, StepperYStep, StepperYDir); // Defaults to AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5
AccelStepper stepperz(AccelStepper::DRIVER, StepperZStep, StepperZDir); // Defaults to AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5

Axis xAxis(StepperXStep, StepperXDir);
Axis yAxis(StepperYStep, StepperYDir);
Axis zAxis(StepperZStep, StepperZDir);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("Enter an X to enable serial output.");

  lcd.begin(20, 4);

  pinMode(StepperEnable, OUTPUT);
  pinMode(13, OUTPUT);
  digitalWrite(StepperEnable, HIGH);

/**
 * setup interrupts
 */
  // initialize timer1 
  noInterrupts();           // disable all interrupts
  TCCR1A = 0;
  TCCR1B = 0;

  // Set timer1_counter to the correct value for our interrupt interval
  //timer1_counter = 65500;   // preload timer 65536-16MHz/256/100Hz
  //timer1_counter = 64286;   // preload timer 65536-16MHz/256/50Hz
  //timer1_counter = 34286;   // preload timer 65536-16MHz/256/2Hz

  //timer1_counter = 65390; // 400 steps /sec
  timer1_counter = 65525;
  TCNT1 = timer1_counter;   // preload timer
  TCCR1B |= (1 << CS12);    // 256 prescaler 
  TIMSK1 |= (1 << TOIE1);   // enable timer overflow interrupt



  
  interrupts();             // enable all interrupts

/** 
 *  end interrupt
 */


  stepper.setMaxSpeed(xmax);
  stepper.setAcceleration(300);
  stepper.setSpeed(xmax);


  stepper2.setMaxSpeed(ymax);
  stepper2.setAcceleration(300);
  stepper2.setSpeed(ymax);


  stepperz.setMaxSpeed(zmax);
  stepperz.setAcceleration(2048);
  stepperz.setSpeed(zmax);
  zAxis.rapidSpeed = zmax;
  zAxis.stepper.setAcceleration(2048);

  error = ps2x.config_gamepad(5, 4, 3, 2, true, true); //setup pins and settings:  GamePad(clock, command, attention, data, Pressures?, Rumble?) check for error

  currentPos();
}


/**
    set the state of the stepper flag
*/
void stepperState (bool state) {
  if (state == steppersOn) return;

  digitalWrite (StepperEnable, state);
  digitalWrite(13, state);
  steppersOn = state;
}

void moveToOrigin() {
  Serial.print ("Moving to origin... ");

  currentPos();
  stepperState(true);

  xAxis.goToOrigin();
  //stepper.runToNewPosition(0);
  currentPos();

  yAxis.goToOrigin();
  //stepper2.runToNewPosition(0);
  currentPos();

  Serial.println("At Origin!");
}

void moveToTarget() {
  Serial.print("Moving To Target... ");
  stepperState(true);
  //stepper2.runToNewPosition(targety);
  //stepper.runToNewPosition(targetx);
  xAxis.goToTarget();
  yAxis.goToTarget();
  Serial.println("At Target!");
}

void resolve_box () {
  Serial.println("Resolving box...");

  if (0 > xAxis.target) {
    Serial.println("Swapping X");
    xAxis.stepper.setCurrentPosition(0);
    xAxis.target = -xAxis.target;

    stepper.setCurrentPosition(0);
    targetx = xAxis.target;
  }

  if (0 > yAxis.target) {
    Serial.println("Swapping Y");
    yAxis.stepper.setCurrentPosition(0);
    yAxis.target = -yAxis.target;

    stepper2.setCurrentPosition(0);
    targety = yAxis.target;
  }

  Serial.println("Box Resolved");
  currentPos();

}

void fast_surface () {
  Serial.println("\n\nFast surfacing...");
  resolve_box();
  moveToOrigin();
//  long zplane = stepperz.currentPosition();
  long zplane = zAxis.stepper.currentPosition();
  long start = millis();
  int curtime = 0;
  float perc = 0;
  int est = 0;

  long smallx = 0;
  long smally = 0;
  long bigx = targetx;
  long bigy = targety;


  stepperState(true);
  while ((bigx > smallx) && (bigy > smally)) {
    curtime = (millis() - start) / 1000;
    Serial.print("bigx="); Serial.print(bigx);
    Serial.print(", smallx="); Serial.print(smallx);
    Serial.print(", bigy="); Serial.print(bigy);
    Serial.print(", smally="); Serial.print(smally);
    Serial.print(", runtime="); Serial.println(curtime);

    smallx += xAxis.stepover;
    xAxis.cut(smallx);
    while (xAxis.isRunning()) {
      // cut x
    }
    //stepper.runToNewPosition(smallx);
    currentPos();

    bigy -= yAxis.stepover;
    yAxis.cut(bigy);
    while (yAxis.isRunning()) {
      // cut y
    }
    //stepper2.runToNewPosition(bigy);
    currentPos();

    bigx -= xAxis.stepover;
    xAxis.cut(bigx);
    while (xAxis.isRunning()) {
      // cut x
    }
    //stepper.runToNewPosition(bigx);
    currentPos();

    smally += yAxis.stepover;
    yAxis.cut(smally);
    while (yAxis.isRunning()) {
      // cut y
    }
    //stepper2.runToNewPosition(smally);
    currentPos();
  }
  curtime = (millis() - start) / 1000;
  Serial.println("Raising cutter");
  zAxis.rapid(zplane + zAxis.buttonRate);
  while (zAxis.isRunning()) {
    // raise the z
  }
//  stepperz.runToNewPosition(zplane + z_button_rate);
  moveToOrigin();
  Serial.println("Lowering cutter to cut plain");
  zAxis.rapid(zplane);
  while (zAxis.isRunning()) {
    // lower z
  }
  //stepperz.runToNewPosition(zplane);
  currentPos();
}


/*
   surface using one axis
*/
void surface () {
  resolve_box();
  Serial.print("targetx: "); Serial.print(xAxis.target); Serial.print(", targety:"); Serial.println(yAxis.target);
  int curtime = 0;
  float perc = 0;
  int est = 0;
  stepperState (true);
  long currenty = 0;
  long currentx = 0;
  moveToOrigin();

  showStatus(currentx, xAxis.target, est);
  currentx = xAxis.target;
  xAxis.goToTarget();
//  stepper.runToNewPosition(targetx);
  long start = millis();
  
  while (currentx > 0) {
    currentx = currentx - xAxis.stepover;
    if (currentx < 0) currentx = 0;
    Serial.print ("Cutting on x: ");
    Serial.print (currentx);
    //stepper.setMaxSpeed(cutMax);
    //stepper2.setMaxSpeed(cutMax);
    xAxis.cut(currentx);
    while (xAxis.isRunning()) {
      // moving
    }
    //stepper.runToNewPosition(currentx);

    yAxis.cut(yAxis.target);
    while (yAxis.isRunning()) {
      // long cut
    }
    //stepper2.runToNewPosition(targety);

    //stepper.setMaxSpeed(xmax);
    //stepper2.setMaxSpeed(ymax);
    xAxis.rapid(currentx + (xAxis.stepover *.25));
    yAxis.rapid(0);

    while (xAxis.isRunning() || yAxis.isRunning()) {
      // rapiding back
    }
    //stepper.runToNewPosition(currentx + (STEPOVER * .25));
    //stepper2.runToNewPosition(0);
    perc = 1 - ((float)currentx / (float)xAxis.target);
    Serial.print("  Percent: ");
    Serial.print(perc);
    curtime = (millis() - start) / 1000;
    Serial.print(", Elapsed time: ");
    Serial.print(curtime);
    Serial.print(", Estimated time: ");
    est = (float)curtime / (float)perc;
    Serial.println(est);
    showStatus(currentx, xAxis.target, est - curtime);
  }
  moveToOrigin();
  currentPos();
  Serial.println("Surface Complete!");

}

float stepsToInch (long steps) {
  int stepsPerRev = 200;
  float pulleyDiameter = 0.5;
  float stepsPerInch = 1 * (pulleyDiameter * 3.1415) / stepsPerRev;
  return steps / stepsPerInch;
}

void currentPos () {
  int line = 2;
  lcd.setCursor(0, line);
  lcd.print ("C X:=     , Y:=     ");
  lcd.setCursor(5, line);
  lcd.print(stepsToInch(xAxis.stepper.currentPosition()));
  lcd.setCursor(15, line);
  lcd.print(stepsToInch(yAxis.stepper.currentPosition()));
  if (serialDebug) {
    
    Serial.print("xloc:");
    Serial.print(xAxis.stepper.targetPosition());
    Serial.print(", current: ");
    Serial.print(xAxis.stepper.currentPosition());
    Serial.print(",Max: ");
    Serial.print(xAxis.stepper.maxSpeed());
    Serial.print(", yloc: ");
    Serial.print(yAxis.stepper.targetPosition());
    Serial.print(", current: ");
    Serial.print(yAxis.stepper.currentPosition());
    Serial.print(", zloc: ");
    Serial.print(zAxis.stepper.targetPosition());
    Serial.print(", current: ");
    Serial.print(zAxis.stepper.currentPosition());
    Serial.print(", update time: ");
    Serial.print(millis() - debugTime);
    Serial.print(", tryrun: ");
    Serial.print(tryRunStep);
    Serial.print (",Xsteps since last: ");
    Serial.print (xAxis.stepper.currentPosition() - xsteps);
    Serial.print (", X/Sec: ");
    Serial.println ((xAxis.stepper.currentPosition() - xsteps) * (1000 / (millis() - debugTime)));
    xsteps = xAxis.stepper.currentPosition();

    debugTime = millis();
    tryRunStep = 0;

  }
}

void showStatus (long current, long target, int est) {
  int min = est / 60;
  int sec = est % 60;
  int line = 3;
  lcd.setCursor(0, line);
  lcd.print ("@          %        ");
  lcd.setCursor(1, line);
  lcd.print(stepsToInch(current));

  lcd.setCursor(9, line);
  int perc = (float)current / (float)target * 100;
  lcd.print(perc);
  lcd.setCursor(15, line);
  lcd.print(min);
  lcd.setCursor(18, line);
  lcd.print(sec);
  currentPos();
}

void setOrigin() {
  Serial.println("Setting Origin");
  xAxis.stepper.setCurrentPosition(0);
  yAxis.stepper.setCurrentPosition(0);
//  stepper.setCurrentPosition(0);
//  stepper2.setCurrentPosition(0);
  currentPos();
}

void setTarget () {
  Serial.println("Setting Target");
  xAxis.target = xAxis.stepper.currentPosition();
  yAxis.target = yAxis.stepper.currentPosition();


//  targetx = stepper.currentPosition();
//  targety = stepper2.currentPosition();
  targetx = xAxis.target;
  targety = yAxis.target;
}

int get_jog_rate (int rate) {
  if (ps2x.Button(PSB_L1)) rate = rate * 2;
  if (ps2x.Button(PSB_R1)) rate = rate * 2;
  if (ps2x.Button(PSB_L2)) rate = rate * 2;
  if (ps2x.Button(PSB_R2)) rate = rate * 2;
  return rate;
}


void jogMode () {
  Serial.println("Entering Jog Mode");
  stepperState(true);
  delay(500);

  int jog_rate_x = xmax/16;
  int jog_rate_y = ymax/16;
  int jog_rate_z = zmax/16;

  int a = 0;
  do {
    if (a % 100 == 0) {
      ps2x.read_gamepad(false, 0);
      if (ps2x.Button(PSB_PAD_LEFT)) {
        xAxis.stepper.setMaxSpeed(get_jog_rate(jog_rate_x));
        xAxis.stepper.moveTo(xAxis.stepper.currentPosition() - 10000);
      }
      else if (ps2x.Button(PSB_PAD_RIGHT)) {
        xAxis.stepper.setMaxSpeed(get_jog_rate(jog_rate_x));
        xAxis.stepper.moveTo(stepper.currentPosition() + 10000);
      }
      else {
        xAxis.stepper.stop();
      }

      if (ps2x.Button(PSB_PAD_UP)) {
        yAxis.stepper.setMaxSpeed(get_jog_rate(jog_rate_y));
        yAxis.stepper.moveTo(stepper.currentPosition() + 10000);
      }
      else if (ps2x.Button(PSB_PAD_DOWN)) {
        yAxis.stepper.setMaxSpeed(get_jog_rate(jog_rate_y));
        yAxis.stepper.moveTo(yAxis.stepper.currentPosition() - 10000);
      }
      else {
        yAxis.stepper.stop();
      }

      if (ps2x.Button(PSB_TRIANGLE)) {
        zAxis.stepper.setMaxSpeed(get_jog_rate(jog_rate_z));
        zAxis.stepper.moveTo(zAxis.stepper.currentPosition() + 10000);
      }
      else if (ps2x.Button(PSB_CROSS)) {
        zAxis.stepper.setMaxSpeed(get_jog_rate(jog_rate_z));
        zAxis.stepper.moveTo(zAxis.stepper.currentPosition() - 10000);
      }
      else {
        zAxis.stepper.stop();

      }
    }

  } while (!ps2x.Button(PSB_CIRCLE));

  xAxis.stepper.stop(); stepper.setMaxSpeed(xmax);
  yAxis.stepper.stop(); stepper2.setMaxSpeed(ymax);
  zAxis.stepper.stop(); stepperz.setMaxSpeed(zmax);
  Serial.println ("Exiting Jog Mode");
  delay(500);
}


ISR(TIMER1_OVF_vect)        // interrupt service routine 
{
  TCNT1 = timer1_counter;
  interrupts();
  runSteps();
}

/**
 * run stepper motors
 */
void runSteps () {
  tryRunStep++;
  if (!zStepping && zAxis.isRunning() && (nextStep == 0)) {
    zStepping = true;
    zAxis.stepper.run();
    zStepping = false;
  }
  if (!yStepping && yAxis.isRunning() && (nextStep == 1)) {
    yStepping = true;
    yAxis.stepper.run();
    yStepping = false;
  }
  if (!xStepping && xAxis.stepper.distanceToGo() && (nextStep == 2)) {
    xStepping = true;
    xAxis.stepper.run();
    xStepping = false;
  }
  nextStep++;
  if (nextStep > 2) nextStep = 0;
}

void loop() {
  // put your main code here, to run repeatedly:
  
  if (i % 5000 == 0) {
    if (steppersOn && serialDebug) {
      currentPos();
    }
    i = 0;

    ps2x.read_gamepad(false, 0);          //read controller and set large motor to spin at 'vibrate' speed

    if (ps2x.Button(PSB_START)) {                 //will be TRUE as long as button is pressed
      setTarget();
      surface();
    }
    if (ps2x.Button(PSB_SELECT)) {
      setOrigin();
    }
    if (ps2x.Button(PSB_SQUARE)) {
      setTarget();
      fast_surface();
    }

    if (ps2x.Button(PSB_PAD_UP)) {
      yAxis.rapid(yAxis.stepper.currentPosition() + yAxis.buttonRate);
      //stepper2.moveTo(stepper2.currentPosition() + xy_button_rate);
    } else if (ps2x.Button(PSB_PAD_DOWN)) {
      yAxis.rapid(yAxis.stepper.currentPosition() - yAxis.buttonRate);
      //stepper2.moveTo(stepper2.currentPosition() - xy_button_rate);
    } else if (stepper2.isRunning() && !mdi) {
      yAxis.stop();
      //stepper2.stop();
    }

    
    if (ps2x.Button(PSB_PAD_RIGHT)) {
      xAxis.rapid(xAxis.stepper.currentPosition() + xAxis.buttonRate);
      //stepper.moveTo(stepper.currentPosition() + xy_button_rate);
    } else if (ps2x.Button(PSB_PAD_LEFT)) {
      xAxis.rapid(xAxis.stepper.currentPosition() - xAxis.buttonRate);
      //stepper.moveTo(stepper.currentPosition() - xy_button_rate);
    } else if (stepper.isRunning() && !mdi) {
      xAxis.stop();
      //stepper.stop();
    }


    if (ps2x.Button(PSB_TRIANGLE)) {
      zAxis.rapid(zAxis.stepper.currentPosition() + zAxis.buttonRate);
      //stepperz.moveTo(stepperz.currentPosition() + z_button_rate);
    } else if (ps2x.Button(PSB_CROSS)) {
      zAxis.rapid(zAxis.stepper.currentPosition() - zAxis.buttonRate);
      //stepperz.moveTo(stepperz.currentPosition() - z_button_rate);
    } else if (stepperz.isRunning() && !mdi) {
      zAxis.stop();
      //stepperz.stop();
    }

    if (ps2x.Button(PSB_CIRCLE)) {
      jogMode();
    }

    if (Serial.available()) {
      byte incomingByte = Serial.read();
      if (incomingByte == 'x') {
        serialDebug = true;
        Serial.println ("Serial debugging enabled");
      }

      if (incomingByte == 'X') {
        serialDebug = false;
        Serial.println("Serial Debugging disabled");
      }

      if (incomingByte == 'r') {
        Serial.println("Beginning Fast Surface...");
        fast_surface();
      }

      if (incomingByte == 'o') {
        setOrigin();
      }

      if (incomingByte == 's') {
        surface();
      }

      if (incomingByte == 't') {
        setTarget();
      }

      if (incomingByte == 'g') {
        Serial.print ("Rapid Axis: ");
        char axis = Serial.read();
        Serial.print (axis);
        Serial.print("  Steps: ");
        int steps = Serial.parseInt();
        Serial.println(steps);
        switch (axis) {
          case 'x': xAxis.rapid(steps); break;
          case 'X': xAxis.rapid(steps * 127); break;
          case 'y': yAxis.rapid(steps); break;
          case 'Y': yAxis.rapid(steps * 127); break;
          case 'z': zAxis.rapid(steps); break;
          case 'Z': zAxis.rapid(steps * 2048); break;
        }
        mdi = true;
      }
    }

    if (!xAxis.isRunning() && !yAxis.isRunning() && !zAxis.isRunning()) {
      if (steppersOn) {
        Serial.print ("Steppers Disabled...");
        currentPos();
        stepperState(false);
        mdi = false;
      }
    }
    else {
      if (!steppersOn) {
        Serial.println ("Steppers Enabled!");
        stepperState(true);

      }
    }
  }
  i++;
}

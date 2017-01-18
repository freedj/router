#include <PS2X_lib.h>
#include <DueTimer.h>
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


#define PS2_DAT        53  //14    
#define PS2_CMD        51  //15
#define PS2_SEL        49  //16
#define PS2_CLK        47  //17


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

int i = 0;
int timer1_counter;

int buttonrate = 127;
int x_button_rate = 127;
int y_button_rate = 127;

int z_button_rate = 2048;
long targetx = 0;
long targety = 0;
long targetz = 0;

long xsteps = 0;

long stepover = 64;
long stepback = 12;
byte serialDebug = false;
bool steppersOn = false;
long debugTime = millis();

int xmax = 4800;
int ymax = 4800;
int zmax = 4800;

bool xStepping = false;
bool yStepping = false;
bool zStepping = false;
byte nextStep = 0;





#include <AccelStepper.h>

AccelStepper stepper(AccelStepper::DRIVER, StepperXStep, StepperXDir); // Defaults to AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5
AccelStepper stepper2(AccelStepper::DRIVER, StepperYStep, StepperYDir); // Defaults to AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5
AccelStepper stepperz(AccelStepper::DRIVER, StepperZStep, StepperZDir); // Defaults to AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("Enter an X to enable serial output.");

  lcd.begin(20, 4);

  pinMode(StepperEnable, OUTPUT);
  pinMode(13, OUTPUT);
  digitalWrite(StepperEnable, HIGH);

  Timer3.attachInterrupt(runSteps);
  Timer3.start(10);

  stepper.setMaxSpeed(xmax);
  stepper.setAcceleration(3000);
  stepper.setSpeed(xmax);


  stepper2.setMaxSpeed(ymax);
  stepper2.setAcceleration(1500);
  stepper2.setSpeed(ymax);


  stepperz.setMaxSpeed(zmax);
  stepperz.setAcceleration(8000);
  stepperz.setSpeed(zmax);

  //error = ps2x.config_gamepad(5, 4, 3, 2, true, true); //setup pins and settings:  GamePad(clock, command, attention, data, Pressures?, Rumble?) check for error
ps2x.config_gamepad(true,false);
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
  /*
  if (steppersOn) {
    interrupts();
   } else {
    noInterrupts();
   }
   */
}

void moveToOrigin() {
  Serial.print ("Moving to origin... ");

  currentPos();
  stepperState(true);

  stepper.runToNewPosition(0);
  currentPos();

  stepper2.runToNewPosition(0);
  currentPos();

  Serial.println("At Origin!");
}

void moveToTarget() {
  Serial.print("Moving To Target... ");
  stepperState(true);
  stepper2.runToNewPosition(targety);
  stepper.runToNewPosition(targetx);
  Serial.println("At Target!");
}

void resolve_box () {
  Serial.println("Resolving box...");

  if (0 > targetx) {
    Serial.println("Swapping X");
    stepper.setCurrentPosition(0);
    targetx = -targetx;
  }

  if (0 > targety) {
    Serial.println("Swapping Y");
    stepper2.setCurrentPosition(0);
    targety = -targety;
  }

  Serial.println("Box Resolved");
  currentPos();

}

void fast_surface () {
  Serial.println("\n\nFast surfacing...");
  resolve_box();
  moveToOrigin();
  long zplane = stepperz.currentPosition();
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

    smallx += stepover;
    stepper.runToNewPosition(smallx);
    currentPos();

    bigy -= stepover;
    stepper2.runToNewPosition(bigy);
    currentPos();

    bigx -= stepover;
    stepper.runToNewPosition(bigx);
    currentPos();

    smally += stepover;
    stepper2.runToNewPosition(smally);
    currentPos();
  }
  curtime = (millis() - start) / 1000;
  Serial.println("Raising cutter");
  stepperz.runToNewPosition(zplane + z_button_rate);
  moveToOrigin();
  Serial.println("Lowering cutter to cut plain");
  stepperz.runToNewPosition(zplane);
  currentPos();
}


/*
   surface using one axis
*/
void surface () {
  resolve_box();
  Serial.print("targetx: "); Serial.print(targetx); Serial.print(", targety:"); Serial.println(targety);
  long start = millis();
  int curtime = 0;
  float perc = 0;
  int est = 0;
  stepperState (true);
  long currenty = 0;
  moveToOrigin();
  showStatus(currenty, est);

  while (currenty < targety) {
    currenty = currenty + stepover;
    if (currenty > targety) currenty = targety;
    Serial.print ("Cutting on y: ");
    Serial.print (currenty);
    stepper2.runToNewPosition(currenty);
    stepper.runToNewPosition(targetx);
    stepper2.runToNewPosition(currenty - (stepover * .25));
    stepper.runToNewPosition(0);
    perc = ((float)currenty / (float)targety);
    Serial.print("  Percent: ");
    Serial.print(perc);
    curtime = (millis() - start) / 1000;
    Serial.print(", Elapsed time: ");
    Serial.print(curtime);
    Serial.print(", Estimated time: ");
    est = (float)curtime / (float)perc;
    Serial.println(est);
    showStatus(currenty, est - curtime);



  }

  moveToOrigin();
  currentPos();
  Serial.println("Surface Complete!");

}

float stepsToInch (long steps) {
  return steps / 127;
}

void currentPos () {

  int line = 2;
  lcd.setCursor(0, line);
  lcd.print ("C X:=     , Y:=     ");
  lcd.setCursor(5, line);
  lcd.print(stepsToInch(stepper.currentPosition()));
  lcd.setCursor(15, line);
  lcd.print(stepsToInch(stepper2.currentPosition()));
  if (serialDebug) {
    
    Serial.print("xloc:");
    Serial.print(stepper.targetPosition());
    Serial.print(", current: ");
    Serial.print(stepper.currentPosition());
    Serial.print(", yloc: ");
    Serial.print(stepper2.targetPosition());
    Serial.print(", current: ");
    Serial.print(stepper2.currentPosition());
    Serial.print(", zloc: ");
    Serial.print(stepperz.targetPosition());
    Serial.print(", current: ");
    Serial.print(stepperz.currentPosition());
    Serial.print(", update time: ");
    Serial.print(millis() - debugTime);
    Serial.print (",Xsteps since last: ");
    Serial.print (stepper.currentPosition() - xsteps);
    Serial.print (", X/Sec: ");
    Serial.println ((stepper.currentPosition() - xsteps) * (1000 / (millis() - debugTime)));
    xsteps = stepper.currentPosition();

    debugTime = millis();

  }
}

void showStatus (long currenty, int est) {
  int min = est / 60;
  int sec = est % 60;
  int line = 3;
  lcd.setCursor(0, line);
  lcd.print ("@          %        ");
  lcd.setCursor(1, line);
  lcd.print(stepsToInch(stepper2.currentPosition()));

  lcd.setCursor(9, line);
  int perc = (float)currenty / (float)targety * 100;
  lcd.print(perc);
  lcd.setCursor(15, line);
  lcd.print(min);
  lcd.setCursor(18, line);
  lcd.print(sec);
  currentPos();


}

void setOrigin() {
  Serial.println("Setting Origin");
  stepper.setCurrentPosition(0);
  stepper2.setCurrentPosition(0);
  currentPos();

}

void setTarget () {
  Serial.println("Setting Target");
  targetx = stepper.currentPosition();
  targety = stepper2.currentPosition();
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
        stepper.setMaxSpeed(get_jog_rate(jog_rate_x));
        stepper.moveTo(stepper.currentPosition() - 10000);
      }
      else if (ps2x.Button(PSB_PAD_RIGHT)) {
        stepper.setMaxSpeed(get_jog_rate(jog_rate_x));
        stepper.moveTo(stepper.currentPosition() + 10000);
      }
      else {
        stepper.stop();
      }

      if (ps2x.Button(PSB_PAD_UP)) {
        stepper2.setMaxSpeed(get_jog_rate(jog_rate_y));
        stepper2.moveTo(stepper2.currentPosition() + 10000);
      }
      else if (ps2x.Button(PSB_PAD_DOWN)) {
        stepper2.setMaxSpeed(get_jog_rate(jog_rate_y));
        stepper2.moveTo(stepper2.currentPosition() - 10000);
      }
      else {
        stepper2.stop();
      }

      if (ps2x.Button(PSB_TRIANGLE)) {
        stepperz.setMaxSpeed(get_jog_rate(jog_rate_z));
        stepperz.moveTo(stepperz.currentPosition() + 10000);
      }
      else if (ps2x.Button(PSB_CROSS)) {
        stepperz.setMaxSpeed(get_jog_rate(jog_rate_z));
        stepperz.moveTo(stepperz.currentPosition() - 10000);
      }
      else {
        stepperz.stop();

      }
    }

  } while (!ps2x.Button(PSB_CIRCLE));
  stepper.stop(); stepper.setMaxSpeed(xmax);
  stepper2.stop(); stepper2.setMaxSpeed(ymax);
  stepperz.stop(); stepperz.setMaxSpeed(zmax);
  Serial.println ("Exiting Jog Mode");
  delay(500);
}


/**
 * run stepper motors
 */
void runSteps () {
  if (!zStepping && stepperz.distanceToGo() && (nextStep == 0)) {
    zStepping = true;
    stepperz.run();
    zStepping = false;
  }
  if (!yStepping && stepper2.distanceToGo() && (nextStep == 1)) {
    yStepping = true;
    stepper2.run();
    yStepping = false;
  }
  if (!xStepping && stepper.distanceToGo() && (nextStep == 2)) {
    xStepping = true;
    stepper.run();
    xStepping = false;
  }
  nextStep++;
  if (nextStep > 2) nextStep = 0;
}

void loop() {
  // put your main code here, to run repeatedly:
  
//  runSteps();
  
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
      stepper2.moveTo(stepper2.targetPosition() + y_button_rate);
      if (stepper2.targetPosition() > stepper2.currentPosition() + 500) stepper2.moveTo(stepper2.targetPosition() + 500);
    }
    if (ps2x.Button(PSB_PAD_DOWN)) {
      stepper2.moveTo(stepper2.targetPosition() - y_button_rate);
      if (stepper2.targetPosition() < stepper2.currentPosition() - 500) stepper2.moveTo(stepper2.targetPosition() - 500);
    }

    
    if (ps2x.Button(PSB_PAD_RIGHT)) {
      stepper.moveTo(stepper.targetPosition() + x_button_rate);
            if (stepper.targetPosition() > stepper.currentPosition() + 500) stepper.moveTo(stepper.targetPosition() + 500);

    }

    if (ps2x.Button(PSB_PAD_LEFT)) {
      stepper.moveTo(stepper.targetPosition() - x_button_rate);
      if (stepper.targetPosition() < stepper.currentPosition() + 500) stepper.moveTo(stepper.targetPosition() - 500);

    }


    if (ps2x.Button(PSB_TRIANGLE)) {
      stepperz.moveTo(stepperz.targetPosition() + z_button_rate);
      if (stepperz.targetPosition() > stepperz.currentPosition() + 500) stepperz.moveTo(stepperz.targetPosition() + 500);

    }

    if (ps2x.Button(PSB_CROSS)) {
      stepperz.moveTo(stepperz.targetPosition() - z_button_rate);
      if (stepperz.targetPosition() < stepperz.currentPosition() + 500) stepperz.moveTo(stepperz.targetPosition() - 500);

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
          case 'x': stepper.moveTo(steps); break;
          case 'X': stepper.moveTo(steps * 127); break;
          case 'y': stepper2.moveTo(steps); break;
          case 'Y': stepper2.moveTo(steps * 127); break;
          case 'z': stepperz.moveTo(steps); break;
          case 'Z': stepperz.moveTo(steps * 2048); break;
        }
      }
    }

    if (!stepper.distanceToGo() && !stepper2.distanceToGo() && !stepperz.distanceToGo()) {
      if (steppersOn) {
        Serial.print ("Steppers Disabled...");
        currentPos();
        stepperState(false);
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

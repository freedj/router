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

int buttonrate = 127;
int z_button_rate = 2048;
long targetx = 0;
long targety = 0;
long targetz = 0;

long stepover = 64;
long stepback = 12;
byte serialDebug = false;
long debugTime = millis();

int xmax = 1000;
int ymax = 1000;
int zmax = 1000;



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
  digitalWrite(StepperEnable, HIGH);

  stepper.setMaxSpeed(1000);
  stepper.setAcceleration(3000);
  stepper.setSpeed(1000);


  stepper2.setMaxSpeed(1000);
  stepper2.setAcceleration(1500);
  stepper2.setSpeed(1000);


  stepperz.setMaxSpeed(4000);
  stepperz.setAcceleration(8000);
  stepperz.setSpeed(1000);

  error = ps2x.config_gamepad(5, 4, 3, 2, true, true); //setup pins and settings:  GamePad(clock, command, attention, data, Pressures?, Rumble?) check for error

  currentPos();
}

void moveToOrigin() {
  Serial.print ("Moving to origin... ");

  currentPos();
  digitalWrite(StepperEnable, HIGH);

  stepper.runToNewPosition(0);
  currentPos();

  stepper2.runToNewPosition(0);
  currentPos();

  Serial.println("At Origin!");
}

void moveToTarget() {
  Serial.print("Moving To Target... ");
  digitalWrite(StepperEnable, HIGH);
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


  digitalWrite(StepperEnable, HIGH);
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
  digitalWrite(StepperEnable, HIGH);
  long currenty = 0;
  moveToOrigin();
  //delay(1000 * 5);
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
  digitalWrite(StepperEnable, LOW);


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
    Serial.println(millis() - debugTime);
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

long jog_stop (long current_step, long stop_step, int increment) {
  int increments = stop_step / increment;
  if ((current_step < stop_step) && (stop_step % increment != 0)) {
    increments++;
  }

  return increment * increments;
}

void jogMode () {
  Serial.println("Entering Jog Mode");
  digitalWrite(StepperEnable, true);
  delay(500);

  int jog_rate_x = 30;
  int jog_rate_y = 30;
  int jog_rate_z = 300;

  int a = 0;
  do {
    if (a % 100 == 0) {
      ps2x.read_gamepad(false, 0);
      if (ps2x.Button(PSB_PAD_LEFT)) {
        stepper.setMaxSpeed(get_jog_rate(jog_rate_x));
        stepper.moveTo(stepper.currentPosition() + 10000);
      }
      else if (ps2x.Button(PSB_PAD_RIGHT)) {
        stepper.setMaxSpeed(get_jog_rate(jog_rate_x));
        stepper.moveTo(stepper.currentPosition() - 10000);
      }
      else {
        stepper.stop();
        stepper.moveTo(jog_stop(stepper.currentPosition(), stepper.targetPosition(), 16));
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
        stepper2.moveTo(jog_stop(stepper2.currentPosition(), stepper2.targetPosition(), 16));
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
        stepperz.moveTo(jog_stop(stepperz.currentPosition(), stepperz.targetPosition(), 205));

      }
    }
    if (stepper.distanceToGo()) stepper.run();
    if (stepper2.distanceToGo()) stepper2.run();
    if (stepperz.distanceToGo()) stepperz.run();


  } while (!ps2x.Button(PSB_CIRCLE));
  stepper.stop(); stepper.setMaxSpeed(xmax);
  stepper2.stop(); stepper2.setMaxSpeed(ymax);
  stepperz.stop(); stepperz.setMaxSpeed(zmax);
  digitalWrite(StepperEnable, false);
  Serial.println ("Exiting Jog Mode");
  delay(500);
}

void loop() {
  // put your main code here, to run repeatedly:

  if (stepperz.distanceToGo()) {
    stepperz.run();
  }
  if (stepper2.distanceToGo()) {
    stepper2.run();
  }
  if (stepper.distanceToGo()) {
    stepper.run();
  }

  if (i % 1000 == 0) {
    if (digitalRead(StepperEnable) && serialDebug) {
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
      digitalWrite(StepperEnable, HIGH);
      stepper2.moveTo(stepper2.targetPosition() - buttonrate);
    }
    if (ps2x.Button(PSB_PAD_RIGHT)) {
      digitalWrite(StepperEnable, HIGH);
      stepper.moveTo(stepper.targetPosition() - buttonrate);
    }

    if (ps2x.Button(PSB_PAD_LEFT)) {
      digitalWrite(StepperEnable, HIGH);
      stepper.moveTo(stepper.targetPosition() + buttonrate);
    }

    if (ps2x.Button(PSB_PAD_DOWN)) {
      digitalWrite(StepperEnable, HIGH);
      stepper2.moveTo(stepper2.targetPosition() + buttonrate);
    }

    if (ps2x.Button(PSB_TRIANGLE)) {
      digitalWrite(StepperEnable, HIGH);
      stepperz.moveTo(stepperz.targetPosition() + z_button_rate);
    }

    if (ps2x.Button(PSB_CROSS)) {
      digitalWrite(StepperEnable, HIGH);
      stepperz.moveTo(stepperz.targetPosition() - z_button_rate);
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
      if (digitalRead(StepperEnable)) {
        Serial.print ("Steppers Disabled...");
        currentPos();
        digitalWrite(StepperEnable, LOW);
      }
    }
    else {
      if (!digitalRead(StepperEnable)) {
        Serial.println ("Steppers Enabled!");
        digitalWrite(StepperEnable, HIGH);

      }
    }
  }
  i++;


}

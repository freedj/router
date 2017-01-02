#include <PS2X_lib.h>
/*-----( Import needed libraries )-----*/
#include <Wire.h>  // Comes with Arduino IDE
// Get the LCD I2C Library here: 
// https://bitbucket.org/fmalpartida/new-liquidcrystal/downloads
// Move any other LCD libraries to another folder or delete them
// See Library "Docs" folder for possible commands etc.
#include <LiquidCrystal_I2C.h>

#define StepperEnable 12

#define StepperXStep 6
#define StepperXDir 7 

#define StepperYStep 8 
#define StepperYDir 9

#define StepperZStep 10
#define StepperZDir 11


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
byte vibrate = 0;

long xloc = 0;
long yloc = 0;
int i = 0;

float yscale = 0.2;
int buttonrate = 127;
byte runstepper = 0;
long targetx = 0;
long targety = 0;

long stepover = 800;
long stepback = 100;



#include <AccelStepper.h>

AccelStepper stepper(AccelStepper::FULL2WIRE, StepperXStep, StepperXDir); // Defaults to AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5
AccelStepper stepper2(AccelStepper::FULL2WIRE, StepperYStep, StepperYDir); // Defaults to AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5




void setup() {
  // put your setup code here, to run once:
  Serial.begin(57600);

  lcd.begin(20,4);

  pinMode(StepperEnable, OUTPUT);
  digitalWrite(StepperEnable, HIGH);
  stepper.setMaxSpeed(4000);
  stepper.setAcceleration(3000);
  stepper.setSpeed(1000);
  stepper.moveTo(100);


  stepper2.setMaxSpeed(4000);
  stepper2.setAcceleration(1500);
  stepper2.setSpeed(1000);


  error = ps2x.config_gamepad(5, 4, 3, 2, true, true); //setup pins and settings:  GamePad(clock, command, attention, data, Pressures?, Rumble?) check for error
  currentPos();
}

void moveToOrigin() {
  Serial.print ("Moving to origin... ");
  digitalWrite(StepperEnable, HIGH);
  stepper.runToNewPosition(0);
  stepper2.runToNewPosition(0);
  Serial.println("At Origin!");
  xloc = 0;
  yloc = 0;
}

void moveToTarget() {
  Serial.print("Moving To Target... ");
  digitalWrite(StepperEnable, HIGH);
  stepper2.runToNewPosition(targety);
  stepper.runToNewPosition(targetx);
  Serial.println("At Target!");
}

void surface () {
  long start = millis();
  int curtime = 0;
  float perc = 0;
  int est = 0;
  digitalWrite(StepperEnable, HIGH);
  long currenty = 0;
  moveToOrigin();
  delay(1000*5);
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
    showStatus(currenty,est - curtime);
    


  }

  moveToOrigin();
  currentPos();
  digitalWrite(StepperEnable, LOW);


}

float stepsToInch (long steps) {
  return steps * 0.000984;
}

void currentPos () {
  int line = 2;
  lcd.setCursor(0,line);
  lcd.print ("C X:=     , Y:=     ");
  lcd.setCursor(5,line);
  lcd.print(stepsToInch(stepper.currentPosition()));
  lcd.setCursor(15,line);
  lcd.print(stepsToInch(stepper2.currentPosition()));
}

void showStatus (long currenty,int est) {
  int min = est / 60;
  int sec = est % 60;
  int line=3;
  lcd.setCursor(0,line);
  lcd.print ("@          %        ");
  lcd.setCursor(1,line);
  lcd.print(stepsToInch(stepper2.currentPosition()));

  lcd.setCursor(9,line);
  int perc = (float)currenty / (float)targety *100;
  lcd.print(perc);
  lcd.setCursor(15,line);
  lcd.print(min);
  lcd.setCursor(18,line);
  lcd.print(sec);

  
}

void loop() {
  // put your main code here, to run repeatedly:




  if (runstepper) {
    stepper.moveTo(xloc);
    stepper.run();
  } else {
    stepper2.moveTo(yloc);
    stepper2.run();
  }
  if (i % 1000 == 0) {
    if (digitalRead(StepperEnable)) {
      Serial.print("Running stepper(");
      Serial.print(runstepper);
      Serial.print("), xloc:");
      Serial.print(xloc);
      Serial.print(", current: ");
      Serial.print(stepper.currentPosition());
      Serial.print(", yloc: ");
      Serial.print(yloc);
      Serial.print(", current: ");
      Serial.println(stepper2.currentPosition());


    }
    i = 0;

    ps2x.read_gamepad(false, vibrate);          //read controller and set large motor to spin at 'vibrate' speed

    if (ps2x.Button(PSB_START)) {                 //will be TRUE as long as button is pressed
      Serial.println("Start is being held");
      targetx = stepper.currentPosition();
      targety = stepper2.currentPosition();

      // moveToOrigin();
      //moveToTarget();


      surface();
    }
    if (ps2x.Button(PSB_SELECT)) {
      Serial.println("Select is being held");
      stepper.setCurrentPosition(0);
      stepper2.setCurrentPosition(0);
      xloc = 0;
      yloc = 0;
      currentPos();
    }


    if (ps2x.Button(PSB_PAD_UP)) {        //will be TRUE as long as button is pressed
      yloc = yloc - buttonrate;
      digitalWrite(StepperEnable, HIGH);
      runstepper = 0;
    }
    if (ps2x.Button(PSB_PAD_RIGHT)) {
      digitalWrite(StepperEnable, HIGH);
      xloc = xloc - buttonrate;
      runstepper = 1;
    }
    if (ps2x.Button(PSB_PAD_LEFT)) {
      digitalWrite(StepperEnable, HIGH);
      xloc = xloc + buttonrate;
      runstepper = 1;
    }
    if (ps2x.Button(PSB_PAD_DOWN)) {
      digitalWrite(StepperEnable, HIGH);
      yloc = yloc + buttonrate;
      runstepper = 0;
    }

    if ((stepper.currentPosition() == xloc) && (stepper2.currentPosition() == yloc)) {
      if (digitalRead(StepperEnable)) {
        Serial.print ("Steppers Disabled...");
              currentPos();
      }
      digitalWrite(StepperEnable, LOW);
    }
    else {
      if (!digitalRead(StepperEnable)) {
        Serial.println ("Steppers Enabled!");

      }
      digitalWrite(StepperEnable, HIGH);
    }
  }
  i++;


}

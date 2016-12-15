#include <PS2X_lib.h>

PS2X ps2x; // create ps2 controller class
int error = 0;
byte type = 0;
byte vibrate = 0;

int xloc = 0;
int i = 0;

#include <AccelStepper.h>

AccelStepper stepper(AccelStepper::FULL2WIRE,12,11); // Defaults to AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5
AccelStepper stepper2(AccelStepper::FULL2WIRE,9,8); // Defaults to AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5




void setup() {
  // put your setup code here, to run once:
  Serial.begin(57600);

  pinMode(10, OUTPUT);
  digitalWrite(10,HIGH);
  stepper.setMaxSpeed(3000);
  stepper.setAcceleration(100);
  stepper.setSpeed(3000);
  stepper.moveTo(100);
  

  stepper2.setMaxSpeed(2000);
  stepper2.setAcceleration(1000);
  stepper2.setSpeed(1000);
  
  
  error = ps2x.config_gamepad(2,4,3,5, true, true);   //setup pins and settings:  GamePad(clock, command, attention, data, Pressures?, Rumble?) check for error

}

void loop() {
  // put your main code here, to run repeatedly:
 
  
    
  
 stepper.moveTo(xloc);    
 stepper.run();

 if (i % 100 == 0) {
Serial.print("Running steppers: ");
 Serial.print(xloc);
 Serial.print(" ");
 Serial.print(stepper.currentPosition());
 Serial.print(" ");
 
 Serial.print(stepper.targetPosition());
 Serial.print(" ");
 Serial.println(stepper.distanceToGo());
 i = 0;

   ps2x.read_gamepad(false, vibrate);          //read controller and set large motor to spin at 'vibrate' speed
    
    if(ps2x.Button(PSB_START)){                   //will be TRUE as long as button is pressed
         Serial.println("Start is being held");
         stepper.setCurrentPosition(0);
         stepper2.setCurrentPosition(0);
    }
    if(ps2x.Button(PSB_SELECT))
         Serial.println("Select is being held");
         
         
     if(ps2x.Button(PSB_PAD_UP)) {         //will be TRUE as long as button is pressed
      xloc = xloc + ps2x.Analog(PSAB_PAD_UP);
       Serial.print("Up held this hard: ");
       Serial.println(ps2x.Analog(PSAB_PAD_UP), DEC);
       stepper.moveTo(xloc);
      }
      if(ps2x.Button(PSB_PAD_RIGHT)){
       Serial.print("Right held this hard: ");
        Serial.println(ps2x.Analog(PSAB_PAD_RIGHT), DEC);
        xloc = xloc + 100;
      }
      if(ps2x.Button(PSB_PAD_LEFT)){
       Serial.print("LEFT held this hard: ");
        Serial.println(ps2x.Analog(PSAB_PAD_LEFT), DEC);
        xloc = xloc - 100;
      }
      if(ps2x.Button(PSB_PAD_DOWN)){
        xloc = xloc - ps2x.Analog(PSAB_PAD_DOWN);
       Serial.print("DOWN held this hard: ");
       Serial.println(ps2x.Analog(PSAB_PAD_DOWN), DEC);
       stepper.moveTo(xloc);
       Serial.println(xloc);
      }   
 
 
 }
 i++;

 
}

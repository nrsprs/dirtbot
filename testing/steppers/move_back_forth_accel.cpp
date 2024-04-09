
#include <AccelStepper.h>
#include <Arduino.h>

// Define a stepper and the pins it will use
AccelStepper stepper(AccelStepper::DRIVER, 9, 8);       // stp, dir

int pos = 200;

void setup()
{ 
  stepper.setMaxSpeed(2000);
  stepper.setSpeed(800);
  stepper.setAcceleration(1000);
}

void loop()
{
  if (stepper.distanceToGo() == 0)
  {
    delay(500);
    pos = -pos;
    stepper.moveTo(pos);
  }
  stepper.run();
}



    /* Working stepper code: will run to +1600 steps then to -1600 steps continually.
    Run stepper +/- 1600 
    There are 400 steps per revolution when the switches are in the state: 000111 with the top speed of 1000. 
    */

    // volatile int pos = 200;
    // stepper0.setSpeed(1000);
    // Serial.println("Speed: " + String(stepper0.speed()));
    // stepper0.setMaxSpeed(2000);
    // stepper0.setAcceleration(4000);
    // lcd.clear();

    // while (true) {
    //     // Get encoder status:
    //     // encXPos = encoderSteps(encX, encXPos);

    //     // Check stepper distance:
    //     if (stepper0.distanceToGo() == 0) {
    //         delay(500); // ms
    //         pos = -pos;
    //         stepper0.moveTo(pos);
    //     }
    //     lcd.setCursor(0,0);
    //     stepper0.setSpeed(1000);
    //     lcd.print("SPEED: " + String(stepper0.speed()));
    //     volatile bool isrunning = stepper0.run();
    //     // volatile bool ls1Status = debounce(limitSwitch1);
    //     // Serial.println("Limit Switch 1 Status: " + String(ls1Status));
    //     lcd.setCursor(0,1);
    //     lcd.print(" MTR: " + String(isrunning));
    // }
    
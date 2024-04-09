/*
DirtBot main demo sequence

Built for use on Arduino AVR Architecture using a MEGA 2560 board

Developed for Professors Caldwell, Tawaf, and Zoyhofski
Class of 2024, RMET
Author: Nick Spears
*/

#include <Arduino.h>
#include <AccelStepper.h>
#include <LiquidCrystal.h> 
#include <Wire.h>
#include <elapsedMillis.h>
#include <Encoder.h>
#include <InputDebounce.h>
#include <Dictionary.h>
#include <Vector.h>

#define elif else if


void setup()                     // init 
{  
    // Begin serial:
    Serial.begin(250000);
    Serial.println("Serial Begun.");

    // Assign Pin Modes: 
    pinMode(LED_BUILTIN, OUTPUT);       // Built-in LED
    // Establish LCD:
    pinMode(A14,OUTPUT);
    pinMode(A13,OUTPUT);
    pinMode(A4,OUTPUT);
    pinMode(A0,OUTPUT);
    pinMode(A2,OUTPUT);
    pinMode(A1,OUTPUT);
    digitalWrite(A14,LOW); 
    digitalWrite(A13,HIGH); 
    digitalWrite(A4,LOW); 
    digitalWrite(A0,LOW);
    digitalWrite(A2,LOW);
    digitalWrite(A1,HIGH);
}


AccelStepper initStepper(int indx) {
    // Motor Connections for constant current, step/direction bipolar motor driver
    typedef struct {
        const int stepPin;
        const int dirPin;
        } pinDictionary;

        pinDictionary PinDict[] = {
        {22, 23},                  // Stepper 0 : X-Axis motor pins
        {24, 25},                  // Stepper 1: Y-Axis motor pins
        {26, 27},                  // Stepper 2: Reservoir metering motor pins
        {28, 29}                   // Stepper 3: Auger motor pins
        };
    AccelStepper stepper(AccelStepper::DRIVER, PinDict[indx].stepPin, PinDict[indx].dirPin);
    Serial.println(String("Index: ") + String(indx));
    Serial.println(String("DirPin: ") + String(PinDict[indx].dirPin) + String(" StepPin: ") + String(PinDict[indx].stepPin));
    // for Bipolar, constant current, step/direction driver
    return stepper;
}


/* ### Debounce A Digital Input
Takes a InputDebounce object and returns 1 or 0 depending on status. 
Meant to be called multiple times, saves on interrupt pins. 
### Returns :: Bool
*/
bool debounce(InputDebounce& switch_object) {
    // Reset vars for switches:
    static unsigned int LS1_Count = 0;
    static unsigned int LS1_OnTimeLast = 0;
    bool result = 0;

    unsigned long now = millis();
    unsigned int buttonTest_OnTime = switch_object.process(now);

    // Handle input button
    if (buttonTest_OnTime) {
        // Save current on-time (button pressed) for release
        LS1_OnTimeLast = buttonTest_OnTime;
        // Check for state change
        unsigned int count = switch_object.getStatePressedCount();
        if (LS1_Count != count) {
            LS1_Count = count;
            // Handle pressed state
            digitalWrite(LED_BUILTIN, HIGH);                 // Turn on built_in LED
            result = 1;
            Serial.println("HIGH");
        }
        else {
            // Handle still pressed state
            Serial.println("HIGH still pressed (" + String(buttonTest_OnTime) + "ms)");
        }
    }
    else {
        if (LS1_OnTimeLast) {
            // Handle released state
            digitalWrite(LED_BUILTIN, LOW);                 // Turn off built_in LED
            result = 0;
            Serial.print("LOW (last on-time: HIGH (" + String(LS1_OnTimeLast) + ")");
            LS1_OnTimeLast = 0;                             // Reset 
        }
    }
    return result; 
}


/* Calculates whether encoder has stepped. 
Encoder position calculation is 4x PPR, so a 20 PPR encoder will have 80 steps.

Params :: encoder object, previous encoder position

Returns :: updated encoder step position*/
long encoderSteps(Encoder& encoder, long prevPos) {
    volatile long newPos = encoder.read();
    if (newPos != prevPos) {
        prevPos = newPos;
    }
    return newPos;
}


// Roundup double d
double round(double d)
{return floor(d + 0.5);}


/*
Motor Direction Map: 
auger: CW 
hopper: CCW
x-axis: CW to home
y-axis: CCW to home
*/


/*
Closed loop single axis control scheme: 

init ::
create objects for steppers, limit switches and encoders

call axis, set home:
double homeXAxis(stepperX, encX, LSX) {
    determine direction stepper needs to rotate to get close to home (pos/neg; cc/ccw)
    set stepper to half speed 
    drive stepper towards LS
    check encoder position
    check LS
    if LS is triggered:
        stop stepper
        reset encoder position to 0
        roll off in ~+80 steps opposite direction for double-tap
        roll back ~-80 steps, slowly
        if LS is triggered:
            stop stepper
            reset encoder position to 0
            roll off by ~+40 steps to release LS
    return stepper currentPosition, encoderPos
} */
long homeAxis(AccelStepper& stepper, Encoder& encoder, InputDebounce& switchObj, int rotDir) {
    // Determine rotation direction for home | CW is pos, CCW is neg
    const float homeVel = 300.0;                                                    // TESTING: update speed
    volatile long encoderPos = 0;
    stepper.setAcceleration(homeVel*rotDir/2);
    stepper.setMaxSpeed(homeVel*rotDir*1.2);
    stepper.setSpeed(homeVel*rotDir);
    
    Serial.println("Running stepper...");
    // Move in direction of home:
    while (debounce(switchObj) != 1) {           // Testing:: check if switch is active high or low in circuit, result bit still is 1..
        stepper.runSpeed();
        encoderPos = encoderSteps(encoder, encoderPos);
    }
    Serial.println("Switch triggered");
    // Switch has been hit, reset positions:
    encoderPos = 0;
    stepper.setCurrentPosition(0);
    // Move off by 80 steps, return and wait for switch to be hit:
    stepper.moveTo(-80*rotDir);
    delay(200);
    stepper.setSpeed(homeVel*rotDir/3);
    while ((stepper.distanceToGo() != 0) && (debounce(switchObj) != 1)) {stepper.run();}
    // Send back to home:
    stepper.moveTo(100*rotDir);
    delay(200);
    while ((stepper.distanceToGo() != 0) && (debounce(switchObj) != 1)) {stepper.run();}
    // Has hit home again, move off 60 steps and return encoder position:
    stepper.moveTo(60*rotDir);
    encoderPos = 0;
    while (stepper.distanceToGo() != 0) {
        stepper.run();
        encoderPos = encoderSteps(encoder, encoderPos);
    }    
    Serial.println("Ending function, stepper pos: " + String(stepper.currentPosition()) + " encoder pos: " + encoderPos);
    return stepper.currentPosition(), encoderPos;
}


/*
call axis, move a distance, direction and size should be calculated before calling:
bool moveXAxis(stepperX, encX, LSX, position) {
    ## Angular to linear: https://pressbooks.bccampus.ca/humanbiomechanics/chapter/6-1-rotation-angle-and-angular-velocity-2/
    ## wtf is a microstep: https://www.linearmotiontips.com/microstepping-basics/#:~:text=Microstepping%20control%20divides%20each%20full,or%2051%2C200%20microsteps%20per%20revolution.

    # linear distance = radius * angle(rads)
    
    wheel radius = 13.97    # mm 
    driver pulse per rev = 400 
    1 rev = 2 pi # rads
    
    ## what angle do I need to spin to? 
    angle_rads = position (->mm) / 13.97 mm
    # Convert from rads to revolutions:
    angle_revs = angle_rads / 2*pi
    distance_in_pulses = angle_revs * 400 # ppr
    
    # Calculate encoder pulse position:

    while encoder position has not reached goal ()
    ## Move motor n pulses:

    ## Check encoder position after every pulse moved, make sure encoder position is <= the
} */
float moveAxis(AccelStepper& stepper, Encoder& encoder, InputDebounce& switchObj, double linearDist) {
    // Convert linear distance to rotations:
    const float wheel_radius = 13.97;    // mm
    const int driver_ppr = 400;           // set on stepper controller
    const float rev_rads = 6.283185307179586476925286766559;
    volatile long encoderPos = 0;
    int rotDir = 0;
    const int encoder_ppr = 20*4;           // 20 ppr encoder, res of 4/ppr using encoder.h
    float result = 0;

    // Calculate angle needed to be reached: 
    float angle_rads = linearDist/wheel_radius;
    //Convert from rads to revs:
    float angle_revs = angle_rads/rev_rads;
    // Calculate pulse distance:
    float distance_pulses = angle_revs * driver_ppr;
    // Calculate encoder pulse position required: 
    long finalEncPos = int(round(angle_revs)) * encoder_ppr;

    if (linearDist <= 0) {          // Positive distance corresponds to CW direction
        rotDir = -1;}           // Set CCW
    else {rotDir = 1;}          // Set CW

    // Call stepper and set params:
    const float vel = 500.0;
    stepper.setAcceleration(vel*rotDir/2);
    stepper.setMaxSpeed(vel*1.2*rotDir); 
    stepper.setSpeed(vel);
    stepper.move(distance_pulses);

    Serial.println("Moving axis to: " + String(distance_pulses));

    // Move to angular position:
    while ((result == 0) && (debounce(switchObj) != 1) ) {
        // Check encoder pos: 
        if (rotDir == 1) {       // Check if axis should be moving positively:
            if ((stepper.distanceToGo() > 0) && (encoderPos < finalEncPos)) {
                stepper.run();
                encoderPos = encoderSteps(encoder, encoderPos);
            }
        }
        if (rotDir == -1) {     // Check if the axis should be moving negatively:
            if ((stepper.distanceToGo() < 0) && (encoderPos > finalEncPos)) {
                stepper.run();
                encoderPos = encoderSteps(encoder, encoderPos);
            }
        }
    }
    // Moved to position, calculate error:
    int err_ppr = finalEncPos - encoderPos;
    float err_revs = err_ppr / encoder_ppr;
    float err_rads = err_revs * rev_rads; 
    result = err_rads * wheel_radius;
    String result_txt = String(result);
    Serial.println("Error in mm: " + result_txt);

    return result;
}


/*
Run the hopper once to the number of revolutions needed to dispense the whole volume of soil. 
#### Params :: AccelStepper stepper, Encoder encoder

#### Returns :: None
*/
void runHopper(AccelStepper& stepper, Encoder& encoder) {
    const int encoder_ppr = 20*4;           // 20 ppr encoder, res of 4/ppr using encoder.h
    const int driver_ppr = 400;             // set on stepper controller
    const float num_revs = 1/5;
    long steps = driver_ppr * num_revs;
    long finalEncPos = encoder_ppr * num_revs;
    long encPos = 0;
    
    // Call stepper and set params:
    const float vel = 500.0;
    stepper.setAcceleration(-vel/2);        // Hopper runs CCW
    stepper.setMaxSpeed(-vel*1.2); 
    stepper.setSpeed(vel);
    stepper.setCurrentPosition(0);
    stepper.moveTo(steps);

    Serial.println("Moving hopper by: " + String(steps) + " steps.");

    while ((stepper.distanceToGo() != 0) && (encPos < finalEncPos)) {
        stepper.run();
        encPos = encoderSteps(encoder, encPos);
    }
}


/*
Run the auger once to half the number of revolutions needed to dispense the whole volume of soil. 
#### Params :: AccelStepper stepper, Encoder encoder

#### Returns :: None
*/
void runAuger(AccelStepper& stepper, Encoder& encoder) {
    const int encoder_ppr = 20*4;           // 20 ppr encoder, res of 4/ppr using encoder.h
    const int driver_ppr = 400;             // set on stepper controller
    const float num_revs = 2.5;                                                //## Testing:: need to validate this nunmber.                       
    float steps = driver_ppr * num_revs;
    float finalEncPos = encoder_ppr * num_revs;
    long encPos = 0;
    
    // Call stepper and set params:
    const float vel = 1000.0;
    stepper.setAcceleration(vel/2);        // Auger runs CW
    stepper.setMaxSpeed(vel*1.2); 
    stepper.setSpeed(vel);
    stepper.setCurrentPosition(0);
    stepper.moveTo(steps);

    Serial.println("Moving hopper by: " + String(steps) + " steps.");

    while ((stepper.distanceToGo() != 0) && (encPos < finalEncPos)) {
        stepper.run();
        encPos = encoderSteps(encoder, encPos);
    }
}


void sensorDemo(LiquidCrystal& lcd, Encoder& encX, InputDebounce& limitSwitch1) {
    // Demoing Sensors:
    lcd.clear();
    int encXPos = -99;
    while (true) {
        encXPos = encoderSteps(encX, encXPos);
        lcd.setCursor(0, 0);
        lcd.print("ENC: " + String(encXPos));

        // Check LS Status:
        bool ls1Status = debounce(limitSwitch1);
        lcd.setCursor(0,1);
        lcd.print("LS: " + String(ls1Status));
        if (ls1Status == 1) {Serial.println("LS STATUS: "+String(ls1Status)); delay(100);}
    }
}



void directControlDemo(AccelStepper& stepper, LiquidCrystal& lcd, Encoder& encX) {
    // Set baud rate to 250000 for accurate stepping calls. 
    // Direct encoder to motor position control test: 
    volatile int pos = 0;
    volatile int encXPos = -99;
    stepper.setSpeed(1000);
    stepper.setMaxSpeed(2000);
    stepper.setAcceleration(800);

    while (true) {
        // Get encoder status:
        encXPos = encoderSteps(encX, encXPos);
        // If encoder is in negative space, move backwards. 
        if (pos < 0) {stepper.moveTo(-pos);}
        else {stepper.moveTo(pos);}
        pos = encXPos * 20;

        // Set and display LCD:
        lcd.setCursor(0,0);
        lcd.print("ENC: " + String(encXPos));
        lcd.setCursor(0,1);
        lcd.print("STP: " + String(pos));
        lcd.setCursor(8,0);
        lcd.print("DTG: " + String(stepper.distanceToGo()));
        Serial.println("STP Distance To Go: " + String(stepper.distanceToGo()));
        Serial.println("ENC Pos: " + String(pos));
        Serial.println("Stepper Speed: " + String(stepper.speed()));       // Returns most recent speed in steps/s
        if (stepper.distanceToGo() != 0) {
            stepper.run();
        }
    }
}



/*
Interface for receiving the number of plugs in the X and Y directions & blocks until user is ready. 

Returns :: <Vector> (int: number of x trays, number of y trays)
*/
Vector<int> userInput(LiquidCrystal& lcd, Encoder& encoder, InputDebounce& pushButton) {
    // User Input & Start Screen:
    lcd.clear();
    int storage_array[2];
    Vector<int> processParams(storage_array);
    volatile int encPos = -99;
    volatile bool start_cmd = 0;
    volatile bool dirt_filled = 0;
    volatile bool tray_size_x = 0;
    int x_tray_cnt;
    volatile bool tray_size_y = 0;
    int y_tray_cnt;
    volatile bool start_confirmation = 0;
    int _testing = 0;
    
    while (start_cmd == 0) {
        // Q: Dirt filled state:
        volatile bool pbStatus;
        volatile int prevEncPos = 0;
        while (dirt_filled == 0) {
            volatile bool dirt_fill_yes = 0;
            encPos = 0;
            lcd.setCursor(0, 0);
            lcd.print("IS DIRT FILLED? ");
            encPos = encoderSteps(encoder, prevEncPos);
            
            if (encPos <= 0) {       // "NO" state
                lcd.setCursor(0,1);
                lcd.print("  [] NO    YES  ");
            }
            if (encPos > 0) {      // "YES" state
                lcd.setCursor(0,1);
                lcd.print("     NO [] YES  ");
                dirt_fill_yes = 1;
                pbStatus = debounce(pushButton);
            }

            if (_testing >= 1) {
            pbStatus = 1;
            dirt_fill_yes = 1;
            }
            if ((pbStatus == 1) && (dirt_fill_yes == 1)) {        // ON Pressed state
                lcd.clear();
                lcd.setCursor(4,0);
                lcd.print("RECEIVED:");
                lcd.setCursor(7,1);
                lcd.print("YES");
                dirt_filled = 1;            // End prompt
                delay (2000);
                prevEncPos = 0;
                lcd.clear();
                encPos = 0;
            }
        }
    while (tray_size_x == 0) {
        // Q: TRAY X SIZE? 
        lcd.setCursor(0, 0);
        lcd.print("  TRAY X SIZE:    ");
        encPos = encoderSteps(encoder, encPos);
        
        if (encPos <= 0) {       // Num trays is less than 0, just set to 0..
            x_tray_cnt = 0;
            encPos = 0;
            prevEncPos = 0;
            lcd.setCursor(0,1);
            lcd.print("    [" + String(x_tray_cnt) + "] PLUGS    ");
        }
        if (encPos > 0) {                                   //  Increment counter for number of trays: 
            x_tray_cnt = encPos/4;                          // 1 Increment is 4 ticks 
            if (x_tray_cnt >= 4) {x_tray_cnt = 4;}
            lcd.setCursor(0,1);
            lcd.print("    [" + String(x_tray_cnt) + "] PLUGS    ");
            pbStatus = debounce(pushButton);
        }

        if (_testing >= 2) {
            pbStatus = 1;
            x_tray_cnt = 2;
        }
        if ((pbStatus == 1) && (x_tray_cnt >= 0)) {        // ON Pressed state
            lcd.clear();
            lcd.setCursor(4,0);
            lcd.print("RECEIVED:");
            lcd.setCursor(4,1);
            lcd.print("[" + String(x_tray_cnt) + "] PLUGS    ");
            tray_size_x = 1;                                // End prompt
            processParams.push_back(x_tray_cnt);
            delay (2000);
            lcd.clear();
            encPos = 0;
        }
    }
    encPos = 0;
    while (tray_size_y == 0) {
        // Q: TRAY Y SIZE? 
        lcd.setCursor(0, 0);
        lcd.print("  TRAY Y SIZE:    ");
        encPos = encoderSteps(encoder, encPos);
        
        if (encPos <= 0) {       // Num trays is less than 0, just set to 0..
            y_tray_cnt = 0;
            encPos = 0;
            prevEncPos = 0;
            lcd.setCursor(0,1);
            lcd.print("    [" + String(y_tray_cnt) + "] PLUGS    ");
        }
        if (encPos > 0) {                                   //  Increment counter for number of trays: 
            y_tray_cnt = encPos/4;                          // 1 Increment is 4 ticks 
            if (y_tray_cnt >= 3) {y_tray_cnt = 3;}         // Keep the tray count maxxed at 4
            lcd.setCursor(0,1);
            lcd.print("    [" + String(y_tray_cnt) + "] PLUGS    ");
            pbStatus = debounce(pushButton);
        }

        if (_testing >= 3) {
            pbStatus = 1;
            y_tray_cnt = 3;
        }
        if ((pbStatus == 1) && (y_tray_cnt >= 0)) {         // ON Pressed state
            lcd.clear();
            lcd.setCursor(4,0);
            lcd.print("RECEIVED:");
            lcd.setCursor(4,1);
            lcd.print("[" + String(y_tray_cnt) + "] PLUGS    ");
            tray_size_y = 1;                                // End prompt
            processParams.push_back(y_tray_cnt);            // Insert y count to return vector
            delay (2000);
            lcd.clear();
            encPos = 0;
            prevEncPos = 0;
        }
    }
    while (start_confirmation == 0) {
        volatile bool start_confirmed = 0;
        encPos = 0;
        lcd.setCursor(0, 0);
        lcd.print("START DIRTBOT? ");
        encPos = encoderSteps(encoder, prevEncPos);
        
        if (encPos <= 0) {       // "NO" state
            lcd.setCursor(0,1);
            lcd.print("  [] NO    YES  ");
        }
        if (encPos > 0) {      // "YES" state
            lcd.setCursor(0,1);
            lcd.print("     NO [] YES  ");
            start_confirmed = 1;
            pbStatus = debounce(pushButton);
        }

        if (_testing >= 4) {
            pbStatus = 1;
            start_confirmed = 1;
        }
        if ((pbStatus == 1) && (start_confirmed == 1)) {        // ON Pressed state
            lcd.clear();
            lcd.setCursor(4,0);
            lcd.print("RECEIVED:");
            lcd.setCursor(7,1);
            lcd.print("YES");
            start_confirmation = 1;            // End prompt
            start_cmd = 1;
            delay (2000);
            prevEncPos = 0;
            lcd.clear();
            encPos = 0;
        }
    } 
    }
    return processParams;
}


void loop() {                     // main 
    // Initalize stepper motor objects:
    AccelStepper stepper0 = initStepper(0);                                                         // X-Axis motor, pins 22, 23
    AccelStepper stepper1 = initStepper(1);                                                         // Y-Axis motor, pins 24, 25
    AccelStepper stepper2 = initStepper(2);                                                         // Hopper motor, pins 26, 27
    AccelStepper stepper3 = initStepper(3);                                                         // Auger motor, pins 28, 29


    // Initalize limit switches:
    #define BUTTON_DB_DELAY 100     // ms
    // Create input DB object:
    static InputDebounce limitSwitch1;      // Pin 4
    static InputDebounce limitSwitch2;      // Pin 5
    static InputDebounce userPushButton;    // Pin 11
    // Setup debounced pull-down pin:
    limitSwitch1.setup(4, BUTTON_DB_DELAY, InputDebounce::PIM_EXT_PULL_DOWN_RES);          // Pin 4
    limitSwitch2.setup(5, BUTTON_DB_DELAY, InputDebounce::PIM_EXT_PULL_DOWN_RES);          // Pin 5
    userPushButton.setup(11, BUTTON_DB_DELAY, InputDebounce::PIM_INT_PULL_UP_RES);       // Pin 11


    // Initalize encoder objects:
    static Encoder encX(20, 21);           // Pins 20 & 21 are for X-Axis encoder (intr 1 & 0)
    static Encoder encY(18, 19);           // Pins 18 & 19 are for Y-Axis encoder (intr 2 & 3)
    static Encoder enc3(2, 3);             // Pins 2 & 3 are for the hopper encoder (intr 4 & 5)
    static Encoder enc4(6, 7);             // Pins 6 & 7 are for the auger encoder (no intr)
    static Encoder userEncKnob(12, 13);    // Pins 12 & 13 are for the user input encoder (no intr)
    volatile long encXPos = -99, encYPos = -99, enc3Pos = -99, enc4Pos = -99;
    

    // Initalize LCD:
    const int rs = A3, en = A5, d4 = A9, d5 = A10, d6 = A11, d7 = A12;
    LiquidCrystal lcd(rs, en, d4, d5, d6, d7);
    lcd.begin(16, 2);


    // Sensor demo to make sure that your sensors actually work:
    // sensorDemo(lcd, userEncKnob, userPushButton);

    // Direct encoder to motor position control test: 
    // directControlDemo(stepper0, lcd, encX);

    /*
    Guidelines for calling stepper.run():
     * * Minimize math, LCD, Serial, and encoder calls while in loop that calls stepper.run()
     * * Maximize speed of stepper by maintaining at least as many calls as the pwm rate
     * * Increase baud rate to 250000 to reduce lost steps
    */


    // Start Animation: 
    bool startup_animation = 0;
    if (startup_animation == 1) {
        lcd.setCursor(0,0);
        lcd.print("DIRTBOT  BOOTING");
        lcd.setCursor(0,1);
        for (int _i = 0; _i <= 16; ++_i) {
            lcd.print(".");
            lcd.setCursor(_i,1);
            digitalWrite(LED_BUILTIN, HIGH);
            delay(200);
            digitalWrite(LED_BUILTIN, LOW);
            delay(50);
        }
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print(" DIRTBOT  READY");
        lcd.setCursor(0,1);
        delay(2000);
        lcd.clear();
    
        // Get user input and number of plugs in the X and Y direction:
        Vector<int> processParams = userInput(lcd, userEncKnob, userPushButton);
        Serial.println("User Output X: " + String(processParams[0]));
        Serial.println("User Output Y: " + String(processParams[1]));
        delay(3000);    
    }


    // runAuger() test:
    runAuger(stepper0, encX);


    // Home Axis Test:
    // long home_pos_stepper = 1.2;
    // long home_pos_enc = 0.5;
    // // Control Loop Demo:
    // lcd.setCursor(0,0);
    // lcd.print("HOMING X-AXIS...");
    // //          |0123456789ABCDEF|

    // static_cast<int>(home_pos_stepper);
    // static_cast<int>(home_pos_enc);
    // lcd.setCursor(0,1);
    // lcd.print("STP: " + String(home_pos_stepper) + " ENC: " + String(home_pos_enc));

    // // Call homeAxis:
    // home_pos_stepper, home_pos_enc = homeAxis(stepper0, encX, limitSwitch1, 1);      // Testing home on X-axis, move CW to get to home.
    // Serial.print("Finished Home Sequence for X Axis...");
    // static_cast<int>(home_pos_stepper);
    // static_cast<int>(home_pos_enc);
    // lcd.clear();
    // lcd.setCursor(0,0);
    // lcd.print("Homing Done...");
    // lcd.setCursor(0,1);
    // lcd.print("STP: " + String(home_pos_stepper) + " ENC: " + String(home_pos_enc));

    // delay(5000);
    // Serial.print("DONE");
    // while (1) {}
    exit (0);
}
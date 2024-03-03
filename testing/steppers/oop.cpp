/*
Framework for calling stepper libs singularly. Will this work? Maybe! 
*/

#include <AccelStepper.h>

class StepperController {
public:
    // Singleton instance
    static StepperController& getInstance() {
        static StepperController instance; // Guaranteed to be destroyed and instantiated on first use.
        return instance;
    }

    // Initialize the stepper
    void setupStepper() {
        stepper.setMaxSpeed(100000);
        stepper.setSpeed(90000);
    }

    // Run the stepper
    void runStepper() {
        stepper.runSpeed();
    }

private:
    // Private constructor for Singleton
    StepperController() {}

    // Private copy constructor and assignment operator to prevent copying
    StepperController(const StepperController&) = delete;
    StepperController& operator=(const StepperController&) = delete;

    // Stepper object
    AccelStepper stepper = AccelStepper(AccelStepper::DRIVER, 9, 8);
};

void setup() {
    Serial.begin(115200);
    StepperController::getInstance().setupStepper();
}

void loop() {
    StepperController::getInstance().runStepper();

}


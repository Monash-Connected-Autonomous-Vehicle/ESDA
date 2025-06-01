/* Serial port baud rate */
#define BAUDRATE 115200

// Output from arduino to safety light
#define SL_RELAY_PIN_OUTPUT 11
// Input from remote receiver to arduino
#define AM_RELAY_PIN_INPUT 12

#define PWM_PERIOD_MS 500
#define PWM_HALF_PERIOD_MS PWM_PERIOD_MS/2

#define AM_ACTIVE_STATE LOW

#define DEBOUNCE_MS 250

int autonomousMode;
uint32_t pwmCounter;
uint32_t debounceCounter;

/* Setup function--runs once at startup. */
void setup() {
  Serial.begin(BAUDRATE);

    autonomousMode = 0;
    pwmCounter = 0;
    debounceCounter = 0;

  // Set up pin modes
  pinMode(SL_RELAY_PIN_OUTPUT, OUTPUT);
  pinMode(AM_RELAY_PIN_INPUT, INPUT);
}

void loop() {
  int am_relay_state = (digitalRead(AM_RELAY_PIN_INPUT) == AM_ACTIVE_STATE);
  // If the state of the autonomous mode doesn't match the current latch state (0+1=1 or 1+0=1)
  if (
    am_relay_state == HIGH
  ) {
    if (debounceCounter > DEBOUNCE_MS) {
        // Flip the latch
        autonomousMode = !autonomousMode;
        // Reset the pwm counter
        pwmCounter = 0;
        // Set safety light state just in case
        // We can do this because in autonomous mode the first half of the blink period is high (light off)
        digitalWrite(SL_RELAY_PIN_OUTPUT, autonomousMode ? HIGH : LOW);
        // Send the message
        Serial.write(autonomousMode == 1 ? '1' : '0');
    }
    // Reset the debounce counter
    debounceCounter = 0;
  }

  // Reload pwm counter at overflow
  if (pwmCounter >= PWM_PERIOD_MS) {
    pwmCounter = 0;
  }
  // Operate PWM when autonomous mode active
   if (autonomousMode == 1) {
    if (pwmCounter == 0) {
        // Safety light off
        digitalWrite(SL_RELAY_PIN_OUTPUT, HIGH);
    }
    if (pwmCounter == PWM_HALF_PERIOD_MS) {
         // Safety light on
        digitalWrite(SL_RELAY_PIN_OUTPUT, LOW);
    }
  }

  // Sleep for 1ms then increment the counter before repeating
  delay(1);
  pwmCounter++;
  debounceCounter++;
}
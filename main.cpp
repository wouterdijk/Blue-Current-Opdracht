#include <Arduino.h>
#include "Adafruit_NeoPixel.h"
#include "vars.h"

// Pin definitions
#define measPin (1)
#define buzzerPin (14)
#define powerLostPin (6)

// State definitions
enum ChargingState {
    STATE_A,  // EV not connected (12V)
    STATE_B,  // EV connected, not ready (9V)
    STATE_C,  // EV connected, ready, no ventilation (6V)
    STATE_UNKNOWN
};

// Voltage thresholds after voltage divider (assuming 5V max)
#define VOLTAGE_A 5.0 // 100% of 5V after voltage devider (12V) 
#define VOLTAGE_B 3.75 // 75% of 5V after voltage divider (9V)
#define VOLTAGE_C 2.5 // 50% of 5V after voltage divider (6V)

// Timer configurations
#define MEASUREMENT_INTERVAL 1000  // Check state every 1kHz = 1000Hz
#define BUZZER_DURATION 500      // Buzzer duration 500ms

// Variables
ChargingState currentState = STATE_UNKNOWN;
ChargingState previousState = STATE_UNKNOWN;
unsigned long buzzerStartTime = 0;
bool buzzerActive = false;
bool ledsEnabled = false;
unsigned long ledOffTime = 10000; // LED off time 10 sec default = 10000 milliseconds
unsigned long lastStateChangeTime = 0;

// Regular timer interrupt variables
unsigned int prescaler = 64;
unsigned int measurementFrequency = MEASUREMENT_INTERVAL;


// Turns of LED's when power is lost
void powerLossISR() {
    leds_off();
    ledsEnabled = false;
}

// Determine the charging state
ChargingState determineState(float voltage) {
    if (abs(voltage - VOLTAGE_A) < 0.25) return STATE_A;
    if (abs(voltage - VOLTAGE_B) < 0.25) return STATE_B;
    if (abs(voltage - VOLTAGE_C) < 0.25) return STATE_C;
    return STATE_UNKNOWN;
}

// Handle LED states
void handleStateLeds(ChargingState state) {
    switch (state) {
        case STATE_A:
            setLed(101);
            setLed(200);
            setLed(300);
            break;
            
        case STATE_B:
            setLed(102);
            setLed(200);
            setLed(300);
            break;
            
        case STATE_C:
            setLed(104);
            setLed(200);
            setLed(300);
            break;
            
        default:
            break;
    }
    UpdateLeds();
}

// Function to set LED off time
void setLedOffTime(unsigned long seconds) {
    ledOffTime = seconds * 1000;
}

// Setup function
void setup() {
    // Initialize pins
    pinMode(measPin, INPUT);
    pinMode(buzzerPin, OUTPUT);
    pinMode(powerLostPin, INPUT_PULLUP);
    
    // Initialize Neo Pixel LED strip
    initNeoPixel();
    
    // Setup interrupt
    attachInterrupt(digitalPinToInterrupt(powerLostPin), powerLossISR, FALLING);
    
    // Configure Timer1 for state checking
    cli();  // Disable interrupts
    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1 = 0;
    OCR1A = (F_CPU / (prescaler * measurementFrequency)) - 1; // Timer compare value
    TCCR1B |= (1 << WGM12);  // CTC mode
    TCCR1B |= (1 << CS11) | (1 << CS10);  // Prescaler = 64
    TIMSK1 |= (1 << OCIE1A); // Enable timer compare interrupt
    sei();  // Enable interrupts
}

// Loop function
void loop() {
    // Handle buzzer timer
    if(buzzerActive && (millis() - buzzerStartTime >= BUZZER_DURATION)) {
        digitalWrite(buzzerPin, LOW);
        buzzerActive = false;
    }
    
    // Handle LED off time
    if(ledsEnabled && ledOffTime > 0 && (millis() - lastStateChangeTime >= ledOffTime)) {
        leds_off();
        ledsEnabled = false;
    }
}

// Timer1 Compare Interrupt Service Routine
ISR(TIMER1_COMPA_vect) {
    // Read analog value and convert to voltage, 1023 is the max reading from analogRead()
    float voltage = analogRead(measPin) * (5.0 / 1023.0);
    
    // Determine new state
    ChargingState newState = determineState(voltage);
    
    // Handle state change
    if(newState != currentState && newState != STATE_UNKNOWN){
        previousState = currentState;
        currentState = newState;
        
        // Update LED's
        handleStateLeds(currentState);
        
        // Activate buzzer
        digitalWrite(buzzerPin, HIGH);
        buzzerActive = true;
        buzzerStartTime = millis();
        
        // Update last state change time
        lastStateChangeTime = millis();
        ledsEnabled = true;
    }
}
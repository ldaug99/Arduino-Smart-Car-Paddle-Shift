//************************************************************************************
// Paddle shoft transmitter
//************************************************************************************
// Library inclution.
//************************************************************************************
#include <Arduino.h>
#include "RH_ASK.h"
#include "AES.h"
#include <avr/sleep.h>
#include <avr/power.h>
//************************************************************************************
// Message types and syntax.
//************************************************************************************
//const static char tVolume = 'v';
//const static char tGear = 'g';
//const static char tMedia = 'm';
//const static char aUp = 'u';
//const static char aDown = 'd';
//const static char aToggle = 't';
/* The first part of the message will be to characters representing the action, defined below:
 * Volume: "vu" - Volume up, "vd" - Volume down, "vt" - Volume mute toggle.
 * Gear: "gu" - Gear up, "gd" - Gear down, "gt" - Gear auto mode toggle.
 * Media: "mu" - Media forward, "md" - Media back, "mt" - Media pause toggle.
 * This is followed by a uint64_t time value, represented as 8-bytes. 
 * The entire message will thus be 10 bytes.
 * Message char to uint16_t table
   * Char -   Decimal  -  Hexadecimal -       Binary         - uint16_t value definition
*//* "vu" - 118 && 117 - 0x76 && 0x75 - 01110110 && 01110101 -*/ const static uint16_t vu = 30325;
  /* "vd" - 118 && 100 - 0x76 && 0x64 - 01110110 && 01100100 -*/ const static uint16_t vd = 30308;
  /* "vt" - 118 && 116 - 0x76 && 0x74 - 01110110 && 01110100 -*/ const static uint16_t vt = 30324;
  /* "gu" - 103 && 117 - 0x67 && 0x75 - 01100111 && 01110101 -*/ const static uint16_t gu = 26485;
  /* "gd" - 103 && 100 - 0x67 && 0x64 - 01100111 && 01100100 -*/ const static uint16_t gd = 26468;
  /* "gt" - 103 && 116 - 0x67 && 0x74 - 01100111 && 01110100 -*/ const static uint16_t gt = 26484;
  /* "mu" - 109 && 117 - 0x6d && 0x75 - 01101101 && 01110101 -*/ const static uint16_t mu = 28021;
  /* "md" - 109 && 100 - 0x6d && 0x64 - 01101101 && 01100100 -*/ const static uint16_t md = 28004;
  /* "mt" - 109 && 116 - 0x6d && 0x74 - 01101101 && 01110100  -*/const static uint16_t mt = 28020;
//************************************************************************************
// Definitions and pinout for interrupt and buttons.
//************************************************************************************
#define buttonScanDelay 100 // Milliseconds before cheking each button for press (Allows multi button press detect)
#define buttonHoldDelay 2000 // Milliseconds to hold button to continuously perform action.
#define messageLength 16 // Length of message. (Has to be 16 for AES)
#define numUint64ToByte 8 // Number of bytes per uint64_t value.
#define volUpPin 3 // On position 0 of array, with value 1.
#define volDownPin 4 // On position 1 of array, with value 2.
#define gearUpPin 5 // On position 2 of array, with value 4.
#define gearDownPin 6 // On position 3 of array, with value 8.
#define forwardPin 7 // On position 4 of array, with value 16.
#define backPin 8 // On position 5 of array, with value 32.
const static uint8_t interruptPin = 2;
const static uint8_t buttonPins[] = {volUpPin, volDownPin, gearUpPin, gearDownPin, forwardPin, backPin};
uint8_t pushedButtonValue = 0;
volatile bool handleInterrupt = false;
//************************************************************************************
// Enctyption defintions.
//************************************************************************************
byte key[16] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
                0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F}; // 16 byte key
AES128 aes;
//************************************************************************************
// Radio defintions.
//************************************************************************************
#define transmitPin 12 // Radio transmit pin.
RH_ASK driver(2000, 11, transmitPin, 13); // Speed, recieve pin, transmit pin, push to talk pin.
//************************************************************************************
// Functions.
//************************************************************************************
// Set button pins to interrupt mode, where a button push will trigger interrupt.
void setInterruptMode() {
    pinMode(interruptPin, INPUT_PULLUP);
    for (uint8_t i = 0; i < sizeof(buttonPins); i++) {
        digitalWrite(buttonPins[i], LOW);
        pinMode(buttonPins[i], OUTPUT);
    }
}
// Set button pins to push detect mode, to detect which button(s) are pressed.
void setPushMode() {
    digitalWrite(interruptPin, LOW);
    pinMode(interruptPin, OUTPUT);
    for (uint8_t i = 0; i < sizeof(buttonPins); i++) {
        pinMode(buttonPins[i], INPUT_PULLUP);
    }
}
// Wait for release of all buttons.
void waitForRelease() {
    bool pressed;
    do {
        pressed = false;
        for (uint8_t i = 0; i < sizeof(buttonPins); i++) {
            if (!digitalRead(buttonPins[i])) {
                pressed = true;
            }
        }
        //delay(20);
    } while (pressed);
}
// Button push interrupt handler, check each button pin for connection to GND.
void buttonInterrupt() {
    sleep_disable(); // Disable sleep mode.
    detachInterrupt(digitalPinToInterrupt(interruptPin)); // Deattatch interrupt to pin.
    handleInterrupt = true;
}
// Scan buttons for press.
void scanButtons() {
    setPushMode();
    delay(buttonScanDelay); // Delay for a time, to allow detecting multiple buttons pressed.
    for (uint8_t i = 0; i < sizeof(buttonPins); i++) {
        if (!digitalRead(buttonPins[i])) {
            pushedButtonValue += pow(2,i);
        }
    }
}
// Button action switch.
void buttonAction() {
    switch (pushedButtonValue) {
        case 1: // Volume up pressed.
            //continuousPess(vu, volUpPin); // ((tVolume << 8) + aUp)
            transmitMessage(vu);
        break;
        case 2: // Volume down pressed.
            //continuousPess(vd, volDownPin); // ((tVolume << 8) + aDown)
            transmitMessage(vd);
        break;
        case 3: // Volume up and down pressed.
            transmitMessage(vt); // ((tVolume << 8) + aToggle)
        break;
        case 4: // Gear up pressed.
            transmitMessage(gu); // ((tGear << 8) + aUp)
        break;
        case 8: // Gear down pressed.
            transmitMessage(gd); // ((tGear << 8) + aDown)
        break;
        case 12: // Gear up and down pressed.
            transmitMessage(gt); // ((tGear << 8) + aToggle)
        break;
        case 16: // Forward pressed.
            transmitMessage(mu); // ((tVolume << 8) + aUp)
        break;
        case 32: // Back pressed.
            transmitMessage(md); // ((tVolume << 8) + aDown)
        break;
        case 48: // Forward and back pressed.
            transmitMessage(mt); // ((tVolume << 8) + aToggle)
        break;
        default:

        break;
    }
    pushedButtonValue = 0;
}
// Continuous press action
void continuousPess(uint16_t action, uint8_t pin) {
    uint64_t transmitTime = 0; // Time of transmit.
    do {
        delay(buttonHoldDelay);
        transmitMessage(action); // Send action.
        transmitTime = millis(); // Log new transmit time.
    } while (!digitalRead(pin)); // Check if pin is still pressed.
}
// Transmit action and time information converted to a char array.
void transmitMessage(uint16_t action) {
    byte message[messageLength]; // Plaintext message array.
    message[1] = (byte) action; // Cast low byte of action to second position of message.
    message[0] = (byte) (action >> 8); // Cast high byte of action to first position of message.
    uint64_t time = millis(); // get current time.
    for (uint8_t i = 0; i < numUint64ToByte; i++) { // Bit shift uint64_t value as chars in message.
        message[1 + numUint64ToByte - i] = (byte) (time >> (8 * i)); 
    }
    for (uint8_t i = (numUint64ToByte + 2); i <= messageLength; i++) {
        message[i] = (byte) 0; // Unused part of message buffer.
    }
    aes.encryptBlock(&message[0], &message[0]); // Encrypt message.
    driver.send(&message[0], messageLength); // Send message.
    driver.waitPacketSent(); // Wait until message is sendt.
}
// Put the Arduino to sleep.
void enableDeepSleep() {
    sleep_enable(); // Enable sleep mode. Must be valled before setting interrupt!
    setInterruptMode(); // Set pins back to interrupt mode.
    attachInterrupt(digitalPinToInterrupt(interruptPin), buttonInterrupt, LOW); // Attatch interrupt to pin.
    // In all but IDLE mode, only LOW interrupt mode can be used!
    set_sleep_mode(SLEEP_MODE_PWR_DOWN); // Set the mode of sleep.
    cli(); // Clear global interrupt mask.
    sleep_bod_disable(); // Disable brown-out detection.
    sei(); // Set global interrupt mask.
    sleep_cpu(); // Put Arduino to sleep.
    sleep_disable(); // Make sure sleep is disabled!
}
// Setup.
//************************************************************************************
void setup() {
    power_adc_disable(); // Disable ADC to save power.
    power_spi_disable(); // Dsiable SPI to save power.
    driver.init(); // Start radio driver.
    aes.setKey(&key[0], sizeof(key)); // Set encryption key.
}
// Main loop.
//************************************************************************************
void loop() {
    if (handleInterrupt) { // Handle button press.
        handleInterrupt = false;
        scanButtons(); // Test each button for press.
        buttonAction(); // Process button press(es).
        waitForRelease(); // Make sure all buttons are released.
    }
    enableDeepSleep(); // Sleep Arduino.
}
// End
//************************************************************************************
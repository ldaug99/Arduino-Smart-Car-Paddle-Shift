//************************************************************************************
// Paddle shoft transmitter
//************************************************************************************
// Library inclution.
//************************************************************************************
#include <Arduino.h>
#include "ChaCha.h"
#include "RH_ASK.h"
#include <avr/sleep.h>
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
#define messageLength 10 // Length of message.
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
ChaCha chacha;
struct encryption {
    byte key[16] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};
    uint8_t rounds = 8;
    byte iv[8] = {101,102,103,104,105,106,107,108};
    byte counter[8] = {109, 110, 111, 112, 113, 114, 115, 116};
} const static cipher;


//byte key[16] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
//                0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F}; // 16 byte key
//AESTiny128 aes;

//************************************************************************************
// Process time analyser - Debug.
//************************************************************************************
//#define enableAnalyser true // Debug.
//#define analysePin 13 // Debug.

//************************************************************************************
// Radio defintions.
//************************************************************************************
#define transmitPin 13
RH_ASK driver(2000, 10, transmitPin, 11); // Speed, recieve pin, transmit pin, push to talk pin.
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
        delay(20);
    } while (pressed);
}
// Button push interrupt handler, check each button pin for connection to GND.
void buttonInterrupt() {
    detachInterrupt(digitalPinToInterrupt(interruptPin)); // Deattatch interrupt to pin.
    sleep_disable(); // Disable sleep mode.
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
    uint8_t message[messageLength]; // Plaintext message array.
    //char ciphertext[messageLength]; // Encrypted message array.
    message[1] = (uint8_t) action; // Cast low byte of action to second position of message.
    message[0] = (uint8_t) (action >> 8); // Cast high byte of action to first position of message.
    uint64_t time = millis(); // get current time.
    for (uint8_t i = 0; i < numUint64ToByte; i++) { // Bit shift uint64_t value as chars in message.
        message[1 + numUint64ToByte - i] = (uint8_t) (time >> (8 * i)); 
    }
    chacha.encrypt(message, message, messageLength); // Encrypt message.
    chacha.setCounter(cipher.counter, sizeof(cipher.counter)); // Set counter.
    //aes.encryptBlock(ciphertext, plaintext);
    //driver.send((uint8_t)ciphertext[0], messageLength); // Send message.
    driver.send(&message[0], messageLength); // Send message.
    driver.waitPacketSent(); // Wait until message is sendt.
}
// Process time analyser - Debug.
//void processTimer(uint8_t state) { // Debug.
//    if (state == 0) { // Done processing - Debug.
//        digitalWrite(analysePin, LOW); // Debug.
//    } else if (state == 1) { // Debug.
//        digitalWrite(analysePin, HIGH); // Start processing - Debug.
//    } // Debug.
//} // Debug.
// Setup.
//************************************************************************************
void setup() {
    //pinMode(analysePin, OUTPUT); // Debug.
    driver.init(); // Start radio driver.
    //driver.setModeIdle(); // Set the transmitter to idle mode.
    //aes.setKey(key, aes.keySize()); // Set encryption key.
    chacha.setNumRounds(cipher.rounds); // Set number of rounds.
    chacha.setKey(cipher.key, sizeof(cipher.key)); // Set key.
    chacha.setIV(cipher.iv, sizeof(cipher.iv)); // Set initialization vector.
    chacha.setCounter(cipher.counter, sizeof(cipher.counter)); // Set number of rounds.
}
// Main loop.
//************************************************************************************
void loop() {
    if (handleInterrupt) {
        handleInterrupt = false;
        scanButtons();
        //driver.setModeTx(); // Set transmitter to transmit mode.
        buttonAction();
        //driver.setModeIdle(); // Set the transmitter to idle mode.
        //processTimer(0); // Debug.
        waitForRelease();
    }
    setInterruptMode(); // Set pins back to interrupt mode.
    attachInterrupt(digitalPinToInterrupt(interruptPin), buttonInterrupt, FALLING); // Attatch interrupt to pin.
    sleep_enable(); // Enable sleep mode.
    set_sleep_mode(SLEEP_MODE_PWR_DOWN); // Set the mode of sleep.
    sleep_cpu(); // Put Arduino to sleep.
}
// End
//************************************************************************************
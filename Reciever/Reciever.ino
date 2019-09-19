//************************************************************************************
// Paddle shoft reciever
//************************************************************************************
// Library inclution.
//************************************************************************************
#include <Arduino.h>
#include "ChaCha.h"
#include "RH_ASK.h"
//************************************************************************************
// Message types and syntax.
//************************************************************************************
const static char tVolume = 'v';
const static char tGear = 'g';
const static char tMedia = 'm';
const static char aUp = 'u';
const static char aDown = 'd';
const static char aToggle = 't';
#define ATarraySize 3
const static char typeArray[ATarraySize] = {tVolume, tGear, tMedia};
const static char actionArray[ATarraySize] = {aUp, aDown, aToggle};
/* The first part of the message will be to characters representing the action, defined below:
 * Volume: "vu" - Volume up, "vd" - Volume down, "vt" - Volume mute toggle.
 * Gear: "gu" - Gear up, "gd" - Gear down, "gt" - Gear auto mode toggle.
 * Media: "mu" - Media forward, "md" - Media back, "mt" - Media pause toggle.
 * This is followed by a uint64_t time value, represented as 8-bytes. 
 * The entire message will thus be 10 bytes.
 * Message char to uint16_t table
   * Char -   Decimal  -  Hexadecimal -       Binary         - uint16_t value definition
*//* "vu" - 118 && 117 - 0x76 && 0x75 - 01110110 && 01110101 -*/ //const static uint16_t vu = 30325;
  /* "vd" - 118 && 100 - 0x76 && 0x64 - 01110110 && 01100100 -*/ //const static uint16_t vd = 30308;
  /* "vt" - 118 && 116 - 0x76 && 0x74 - 01110110 && 01110100 -*/ //const static uint16_t vt = 30324;
  /* "gu" - 103 && 117 - 0x67 && 0x75 - 01100111 && 01110101 -*/ //const static uint16_t gu = 26485;
  /* "gd" - 103 && 100 - 0x67 && 0x64 - 01100111 && 01100100 -*/ //const static uint16_t gd = 26468;
  /* "gt" - 103 && 116 - 0x67 && 0x74 - 01100111 && 01110100 -*/ //const static uint16_t gt = 26484;
  /* "mu" - 109 && 117 - 0x6d && 0x75 - 01101101 && 01110101 -*/ //const static uint16_t mu = 28021;
  /* "md" - 109 && 117 - 0x6d && 0x64 - 01101101 && 01100100 -*/ //const static uint16_t md = 28004;
  /* "mt" - 109 && 117 - 0x6d && 0x74 - 01101101 && 01110100  -*///const static uint16_t mt = 28020;
//************************************************************************************
// Definitions and pinout for interrupt and buttons.
//************************************************************************************
#define messageLength 10 // Length of message.
#define numUint64ToByte 8 // Number of bytes per uint64_t value.
#define pulseDuration 100 // Number of milliseconds to hold pin high.
#define typeNum 3 // Number of types.
#define actionNum 3 // Number of actions.
#define volUpPin 2 // On position 0 of array
#define volDownPin 3 // On position 1 of array
#define volToggPin 4 // On position 2 of array
#define gearUpPin 5 // On position 3 of array
#define gearDownPin 6 // On position 4 of array
#define gearToggPin 7 // On position 5 of array
#define forwardPin 8 // On position 6 of array
#define backPin 9 // On position 7 of array
#define togglePin 10 // On position 8 of array
const static uint8_t functionPins[typeNum][actionNum] = {
    {volUpPin, volDownPin, volToggPin},
    {gearUpPin, gearDownPin, gearToggPin},
    {forwardPin, backPin, togglePin}
}; // Function pin array, arranged by type and ction.
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
//AES128 aes;

//************************************************************************************
// Process time analyser - Debug.
//************************************************************************************
#define enableAnalyser true // Debug.
#define analysePin 13 // Debug.

//************************************************************************************
// Radio defintions.
//************************************************************************************
#define recievePin 13 // Radio recieve pin.
uint64_t prevMessageTime = 0; // Time of last data.
#define timeResetThreshold 0xFA // High byte time threshold for timer reset. 
RH_ASK driver(2000, recievePin, 11, 12); // Speed, recieve pin, transmit pin, push to talk pin.
//************************************************************************************
// Functions.
//************************************************************************************
// Set function pins to output.
void setOutput() {
    for (uint8_t i = 0; i < typeNum; i++) {
        for (uint8_t k = 0; k < actionNum; k++) {
            digitalWrite(functionPins[i][k], LOW);
            pinMode(functionPins[i][k], OUTPUT);
        }
    }
}
// Get message from radio library.
void getMessage() { // Check if a message has been recieved.
    uint8_t buf[RH_ASK_MAX_MESSAGE_LEN];
    uint8_t buflen;
    if (driver.recv(buf, &buflen)) { // Message with a good checksum received.
        driver.printBuffer("Got:", buf, buflen); // Debug.
        if (buflen == messageLength) { // Message of 10 bytes recieved as expected.
            uint8_t message[messageLength]; // Encrypted message array.
            for (uint8_t i = 0; i < messageLength; i++) { // Save message to ciphertext array.
                message[i] = buf[i];
            }
            chacha.decrypt(message, message, messageLength); // Decrypt message.
            chacha.setCounter(cipher.counter, sizeof(cipher.counter)); // Set counter.
            Serial.print("Decrypted message is: ");
            for (uint8_t i = 0; i < messageLength; i++) { // Print message.
                Serial.print(message[i], DEC);
                Serial.print(" ");
            }
            Serial.println("");
            handleMessage(&message[0]);
            Serial.println("Done processing message");
        }
    }
}
// Check if valid message and valid time.
void handleMessage(char *address) {
    uint8_t type = getMessageCommand(address, &typeArray[0]);
    uint8_t action = getMessageCommand(address + 1, &actionArray[0]);
    if (type != 3 && action != 3) {
        Serial.println("Valid message."); // Debug.
        if (confirmTime(address + 2)) {
            Serial.println("Valid time."); // Debug.
            pulsePin(functionPins[type][action]); // Pulse the pin.
        } else { // Debug.
            Serial.println("Invalid time."); // Debug.
        } // Debug.
    } else { // Debug.
        Serial.println("Invalid command."); // Debug.
    } // Debug.
}
// Get message type or action, depended on passed array.
uint8_t getMessageCommand(char *address, char *array) {
    for (uint8_t i = 0; i < ATarraySize; i++) { // For each element in passed action or type array.
        if (*address == *array) { // Check if message command is equal to passed array commands.
            return i;
        }
        array++;
    }
    return 3;
}
// Confirm timing of new message.
bool confirmTime(char *address) {
    bool confirmed = true;
    uint64_t messageTime = 0;
    if (((uint8_t)(prevMessageTime >> 56)) > timeResetThreshold) { // Check if close to overflow.
        prevMessageTime = 0;
    }
    for (int8_t i = 0; i < numUint64ToByte; i++) {
        messageTime += (uint8_t) *(address + i); // Add address value to message time, cast as uint.
        messageTime = messageTime << (56 - (8 * i)); // Shift left to match time value. First addres is high byte.
    }
    if (messageTime < prevMessageTime) {
        confirmed = false;
    }
    return confirmed;
}
// Puls the given pin to set relay on then off.
void pulsePin(uint8_t pin) {
    Serial.print("Setting pin: ");
    Serial.println(pin);
    digitalWrite(pin, HIGH);
    delay(pulseDuration);
    digitalWrite(pin, LOW);
}
// Setup.
//************************************************************************************
void setup() {
    setOutput(); // Set function pins to output pins.
    Serial.begin(115200);  // Debug.
    driver.init(); // Start radio driver.
    //driver.setModeRx(); // Set driver to recieve. Not really nessesary.
    //aes.setKey(key, aes.keySize()); // Set encryption key.
    chacha.setNumRounds(cipher.rounds); // Set number of rounds.
    chacha.setKey(cipher.key, sizeof(cipher.key)); // Set key.
    chacha.setIV(cipher.iv, sizeof(cipher.iv)); // Set initialization vector.
    chacha.setCounter(cipher.counter, sizeof(cipher.counter)); // Set number of rounds.
}
// Main loop.
//************************************************************************************
void loop() {
    getMessage(); // Check if any message is avaliable.
}
// End
//************************************************************************************
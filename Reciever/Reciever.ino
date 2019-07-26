#include <Arduino.h>
#include <AES.h>
#include <RH_ASK.h>

struct encryption {
    // 32 byte key
    byte key[32] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
                    0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F,
                    0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17};
    // 16 byte plaintext
    char plaintext[16];
    // 16 byte ciphertext
    char ciphertext[16];
} *encrypt;

AES256 aes256;
RH_ASK driver(2000, 3, 2, 5); // Speed, recieve pin, transmit pin, push to talk pin

void setup() {
    Serial.begin(115200);	  // Debugging only
    aes256.setKey(encrypt->key, aes256.keySize()); 
    if (!driver.init()) {
        Serial.println("init failed");
    } else {
        Serial.println("Ready for transmission");
    }
}

void loop() {
    uint8_t buf[RH_ASK_MAX_MESSAGE_LEN];
    uint8_t buflen = sizeof(buf);

    if (driver.recv(buf, &buflen)) { // Non-blocking
	    int i;
        // Message with a good checksum received, dump it.
        driver.printBuffer("Got:", buf, buflen);
        if (buflen >= 16) {
            for (uint8_t i = 0; i < buflen; i++) {
                encrypt->ciphertext[i] = (char)buf[i];
            }  
            aes256.decryptBlock(encrypt->plaintext, encrypt->ciphertext);
        } else {
            Serial.println("Buffer to large");
        }
    }
}
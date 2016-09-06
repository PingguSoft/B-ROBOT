#include <Arduino.h>
#include <stdarg.h>
#include "common.h"
#include "utils.h"

void setup()
{
    Serial.begin(115200);
    Serial.println("ESP8266 loop");
}

void loop()
{
    while(Serial.available()) {
        Serial.write(Serial.read());
    }
}

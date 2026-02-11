#include "HX711.h"

// HX711 wiring
static const byte HX_DOUT = 3; // DT
static const byte HX_SCK = 2;  // SCK

HX711 scale;

// Put your calibration factor here after you calibrate
// (it will likely be negative for some wiring setups; that's fine)
float CAL_FACTOR = -7050.0;

void setup()
{
    Serial.begin(115200);
    scale.begin(HX_DOUT, HX_SCK);

    // Optional: give HX711 time to settle
    delay(500);

    Serial.println("Taring...");
    scale.tare(); // zero out
    Serial.println("Ready.");
    Serial.println("Commands: t=tare, c=calibrate prompt");
}

void loop()
{
    // Handle simple commands from Serial Monitor
    if (Serial.available())
    {
        char cmd = (char)Serial.read();
        if (cmd == 't')
        {
            scale.tare();
            Serial.println("OK TARED");
        }
    }

    // Read weight
    if (scale.is_ready())
    {
        // average a few samples for stability
        long reading = scale.read_average(10);

        // If calibrated, use get_units; otherwise just send raw
        scale.set_scale(CAL_FACTOR);
        float units = scale.get_units(5);

        // Send both raw + units, easy to parse
        Serial.print("raw=");
        Serial.print(reading);
        Serial.print(", weight=");
        Serial.println(units, 3);
    }
    else
    {
        Serial.println("HX711 not ready");
    }

    delay(200);
}

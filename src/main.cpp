#include <Controllino.h>
#include <ArduinoLog.h>

#include "arduino_platform.h"
#include "boiler.h"

namespace {
    EthernetClient ethClient;
    ArduinoMqtt mqtt("192.168.20.36", 1883, &ethClient);
    ArduinoEeprom eeprom;
    ArduinoBoardIO io;
    byte mac[] = {
        0xDE, 0xAD, 0xBE, 0x12, 0x33, 0x55
    };
    // FIXME: adding bindings should not request temp sensors / map pins
    // Boiler boiler(&mqtt, &eeprom, &io);
    Boiler* boiler;
};

#ifdef __arm__
// should use uinstd.h to define sbrk but Due causes a conflict
extern "C" char* sbrk(int incr);
#else  // __ARM__
extern char *__brkval;
#endif  // __arm__

// https://learn.adafruit.com/memories-of-an-arduino/measuring-free-memory
int freeMemory() {
    char top;
#ifdef __arm__
    return &top - reinterpret_cast<char*>(sbrk(0));
#elif defined(CORE_TEENSY) || (ARDUINO > 103 && ARDUINO != 151)
    return &top - __brkval;
#else  // __arm__
    return __brkval ? &top - __brkval : &top - __malloc_heap_start;
#endif  // __arm__
}

void setup()
{
    Serial.begin(115200);
    Log.begin(LOG_LEVEL_VERBOSE, &Serial);
    Serial.println("boiler: starting");
    if (!Ethernet.begin(mac))
        Log.error("ethernet failed");
    else {
        // FIXME: use log
        Serial.print("local ip: ");
        Serial.println(Ethernet.localIP());
    }
    io.setPinMapping(Boiler::TemperaturePin, 20); // CONTROLLINO_PIN_HEADER_SDA
    io.setPinMapping(Boiler::ValveXPin, CONTROLLINO_A3);
    io.setPinMapping(Boiler::ValveYPin, CONTROLLINO_D0);
    io.setPinMapping(Boiler::PressurePin, CONTROLLINO_A1);
    io.setPinMapping(Boiler::FeedValvePin, CONTROLLINO_R10);
    io.setPinMapping(Boiler::RadiatorValve1Pin, CONTROLLINO_D12);
    io.setPinMapping(Boiler::RadiatorValve2Pin, CONTROLLINO_D13);
    io.setPinMapping(Boiler::RadiatorValve3Pin, CONTROLLINO_D14);
    io.setPinMapping(Boiler::RadiatorValve4Pin, CONTROLLINO_D15);
    io.setPinMapping(Boiler::RadiatorValve5Pin, CONTROLLINO_D16);
    io.setPinMapping(Boiler::RadiatorValve6Pin, CONTROLLINO_D17);
    io.setPinMapping(Boiler::RadiatorValve7Pin, CONTROLLINO_D18);
    io.setPinMapping(Boiler::BoilerCirculationRelay, CONTROLLINO_D19);
    io.setPinMapping(Boiler::RadiatorCirculationRelay, CONTROLLINO_D20);
    boiler = new Boiler(&mqtt, &eeprom, &io);
    boiler->setup();
}

void loop()
{
    static unsigned long lastFreeRamPrint = 0;
    unsigned long ms = millis();
    if ((ms > 0 && lastFreeRamPrint == 0) || ms - lastFreeRamPrint >= 10000) {
        lastFreeRamPrint = ms;
        Log.trace("FREE RAM: %d bytes\n", freeMemory());
    }
    mqtt.loop();
    boiler->loop();
}

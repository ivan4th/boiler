#ifndef __ARDUINO_PLATFORM_H__
#define __ARDUINO_PLATFORM_H__

#include <Arduino.h>
#include <Ethernet.h>
#include <PubSubClient.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#include "interfaces.h"

class ArduinoMqtt: public Mqtt {
public:
    ArduinoMqtt(const char* brokerIP, uint16_t port, EthernetClient* ethClient);
    void loop();
    bool connected();
    void setHandler(MqttHandler* handler);
    void publish(const char* topic, const char* payload, bool retain);
    virtual void subscribe(const char* topic);
private:
    static void callback(char* topic, byte* payload, unsigned int length);

    PubSubClient _client;
    MqttHandler* _handler = 0;
    static ArduinoMqtt* _instance;
    long lastReconnectAttempt = 0;
};

class ArduinoEeprom: public Eeprom {
public:
    void writeBytes(const uint8_t* data, int offset, int count);
    void readBytes(uint8_t *data, int offset, int count);
};

class ArduinoTempSensors: public TempSensors {
public:
    ArduinoTempSensors(int pin);
    void request();
    bool available();
    double read(uint8_t* address);
    unsigned long tempWaitMs();
private:
    OneWire _oneWire;
    DallasTemperature _sensors;
};

class ArduinoBoardIO: public BoardIO {
public:
    ArduinoBoardIO();
    void setPinMapping(int id, int physicalPin);
    unsigned long millis();
    void setPinMode(int id, PinMode mode);
    PinValue digitalRead(int id);
    void digitalWrite(int id, PinValue value);
    int analogRead(int id);
    void analogWrite(int id, int value);
    TempSensors* getTempSensors(int id);
private:
    static const int NumPins = 32;
    int _pinMappings[NumPins];
    int _tempPinId;
    TempSensors* _tempSensors;
};

#endif

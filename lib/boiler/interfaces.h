#ifndef __INTERFACES_H__
#define __INTERFACES_H__

#include <stdint.h>

class MqttHandler {
public:
    virtual ~MqttHandler() {}
    virtual void onConnect() = 0;
    virtual void onMessage(const char* topic, const char* payload) = 0;
};

class Mqtt {
public:
    virtual ~Mqtt() {}
    virtual bool connected() = 0;
    virtual void setHandler(MqttHandler* handler) = 0;
    virtual void publish(const char* topic, const char* payload, bool retain) = 0;
    virtual void subscribe(const char* topic) = 0;
};

class Eeprom {
public:
    virtual ~Eeprom() {}
    virtual void writeBytes(const uint8_t* data, int offset, int count) = 0;
    virtual void readBytes(uint8_t *data, int offset, int count) = 0;
};

class TempSensors {
public:
    static const int AddressSize = 8;
    ~TempSensors() {}
    virtual void request() = 0;
    virtual bool available() = 0;
    virtual double read(uint8_t* address) = 0;
    virtual unsigned long tempWaitMs() = 0;
};

class TimeSource {
public:
    virtual ~TimeSource() {}
    virtual unsigned long millis() = 0;
};

class BoardIO: public TimeSource {
public:
    enum PinMode { Input, Output };
    enum PinValue { Low, High };
    static const int MaxPWM = 255;
    virtual void setPinMode(int id, PinMode mode) = 0;
    virtual PinValue digitalRead(int id) = 0;
    virtual void digitalWrite(int id, PinValue value) = 0;
    virtual int analogRead(int id) = 0;
    virtual void analogWrite(int id, int value) = 0;
    virtual TempSensors* getTempSensors(int id) = 0;
};

#endif

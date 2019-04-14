#ifndef __FAKE_H__
#define __FAKE_H__

#include "boiler.h"

#define REC_END (const char*)42

class Recorder {
public:
    Recorder(): _nrec(0) {}
    ~Recorder();
    void record(const char* fmt, ...);
    void verify(const char* firstRec, ...);

private:
    static const int maxRec = 128;
    int _nrec;
    const char* recs[maxRec];
};

class FakeMqtt: public Mqtt {
public:
    FakeMqtt(Recorder *rec);
    ~FakeMqtt();
    void connect();
    bool connected();
    void setHandler(MqttHandler* handler);
    void publish(const char* topic, const char* payload, bool retain);
    void subscribe(const char* topic);

private:
    Recorder* _rec;
    static const int maxSubs = 64;
    int _nsubs;
    const char* subs[maxSubs];
    MqttHandler* _handler;
    bool _connected;
};

class FakeEeprom: public Eeprom {
public:
    FakeEeprom();
    void writeBytes(const uint8_t* data, int offset, int count);
    void readBytes(uint8_t *data, int offset, int count);
private:
    static const int size = 1024;
    uint8_t _buf[size];
};

class FakeTempSensors: public TempSensors {
public:
    FakeTempSensors(Recorder* rec);
    ~FakeTempSensors();
    void request();
    bool available();
    unsigned long tempWaitMs();
    double read(uint8_t* address);
    void set(uint8_t* address, double value);
    void makeAvailable();
private:
    struct TempValue {
        uint8_t address[AddressSize];
        double value;
        TempValue* next;
    };
    TempValue* findTempValue(uint8_t *address);

    Recorder* _rec;
    TempValue* _vals;
    bool _requested, _ready;
};

class FakeBoardIO: public BoardIO {
public:
    FakeBoardIO(Recorder* rec);
    ~FakeBoardIO();
    unsigned long millis();
    void setPinMode(int id, PinMode mode);
    PinValue digitalRead(int id);
    void digitalWrite(int id, PinValue value);
    int analogRead(int id);
    void analogWrite(int id, int value);
    void setDigitalPinValue(int id, PinValue value);
    void setAnalogPinValue(int id, int value);
    TempSensors* getTempSensors(int id);
    void setTemperature(int id, uint8_t* address, double value);
    void makeTempsAvailable();
    void elapse(unsigned long interval);
private:
    struct Pin {
        Pin(int _id, PinMode _mode = Output, PinValue _value = Low):
            id(_id), mode(_mode), value(_value), analogValue(0), tempSensors(0), next(0) {}
        int id;
        PinMode mode;
        PinValue value;
        int analogValue;
        FakeTempSensors* tempSensors;
        Pin* next;
    };
    Pin* findPin(int id);
    Pin* ensurePin(int id);

    Recorder* _rec;
    Pin* _pins;
    unsigned long _millis;
};

#endif

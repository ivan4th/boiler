#include <unity.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include <stdio.h>

#include "debug.h"
#include "fake.h"

Recorder::~Recorder()
{
    for (int i = 0; i < _nrec; ++i)
        free((void *)recs[i]);
}

void Recorder::record(const char* format, ...)
{
    va_list args;
    va_start (args, format);
    char buf[512];
    vsnprintf(buf, 512, format, args);
    va_end(args);
    printf("REC: %s\n", buf);
    recs[_nrec++] = strdup(buf);
}

void Recorder::verify(const char* firstRec, ...)
{
    char errBuf[512];
    va_list ap;
    va_start(ap, firstRec);
    int n = 0;
    for (const char* r = firstRec; r != REC_END; r = va_arg(ap, const char*), n++) {
        if (n >= _nrec) {
            snprintf(errBuf, 512, "Missing item '%s'", r);
            TEST_FAIL_MESSAGE(errBuf);
        } else if (strcmp(r, recs[n])) {
            snprintf(errBuf, 512, "Mismatch: expected '%s' != '%s'", r, recs[n]);
            TEST_FAIL_MESSAGE(errBuf);
        }
    }
    va_end(ap);
    while (n < _nrec) {
        snprintf(errBuf, 512, "Excess item '%s'", recs[n++]);
        TEST_FAIL_MESSAGE(errBuf);
    }
    for (int i = 0; i < _nrec; ++i)
        free((void *)recs[i]);
    _nrec = 0;
}

FakeMqtt::FakeMqtt(Recorder* rec): _rec(rec), _nsubs(0), _handler(0), _connected(false) {}

FakeMqtt::~FakeMqtt()
{
    for (int i = 0; i < _nsubs; ++i)
        free((void *)subs[i]);
}

void FakeMqtt::connect()
{
    printf("mqtt connect\n");
    _connected = true;
    if (_handler)
        _handler->onConnect();
}

bool FakeMqtt::connected()
{
    return _connected;
}

void FakeMqtt::setHandler(MqttHandler* handler)
{
    _handler = handler;
}

void FakeMqtt::publish(const char* topic, const char* payload, bool retain)
{
    if (!_handler)
        return;
    _rec->record("%s%s: %s", retain ? "+" : "", topic, payload);
    for (int i = 0; i < _nsubs; ++i) {
        if (!strcmp(topic, subs[i]))
            _handler->onMessage(topic, payload);
    }
}

void FakeMqtt::subscribe(const char* topic)
{
    if (_nsubs == maxSubs)
        return;
    subs[_nsubs++] = strdup(topic);
    printf("subscribe: %s\n", topic);
}

FakeEeprom::FakeEeprom()
{
    memset(_buf, 0, sizeof(_buf));
}

void FakeEeprom::writeBytes(const uint8_t* data, int offset, int count)
{
    TEST_ASSERT(offset + count < size);
    memcpy(_buf + offset, data, count);
}

void FakeEeprom::readBytes(uint8_t *data, int offset, int count)
{
    TEST_ASSERT(offset + count < size);
    memcpy(data, _buf + offset, count);
}

FakeTempSensors::FakeTempSensors(Recorder* rec): _rec(rec), _vals(0), _ready(false) {}

FakeTempSensors::~FakeTempSensors()
{
    for (TempValue *next, *t = _vals; t; t = next) {
        next = t->next;
        delete t;
    }
}

void FakeTempSensors::request()
{
    DEBUG_LOG("request()");
    _requested = true;
    _ready = false;
    _rec->record("request");
}

bool FakeTempSensors::available()
{
    return _ready;
}

unsigned long FakeTempSensors::tempWaitMs()
{
    return 750;
}

void FakeTempSensors::makeAvailable()
{
    DEBUG_LOG("makeAvailable()");
    if (!_requested)
        TEST_FAIL_MESSAGE("temps not requested");
    _requested = false;
    _ready = true;
}

double FakeTempSensors::read(uint8_t *address)
{
    TempValue* t = findTempValue(address);
    if (!t)
        TEST_FAIL_MESSAGE("temp sensor not found");
    return t->value;
}

void FakeTempSensors::set(uint8_t* address, double value)
{
    TempValue* t = findTempValue(address);
    if (!t) {
        t = new TempValue();
        memcpy(t->address, address, AddressSize);
        t->next = _vals;
        _vals = t;
        DEBUG_LOG("set(): add temp value: %02x%02x%02x%02x%02x%02x%02x%02x",
                  address[0], address[1], address[2], address[3],
                  address[4], address[5], address[6], address[7]);
    }
    t->value = value;
}

FakeTempSensors::TempValue* FakeTempSensors::findTempValue(uint8_t *address) {
    DEBUG_LOG("findTempValue(): %02x%02x%02x%02x%02x%02x%02x%02x",
              address[0], address[1], address[2], address[3],
              address[4], address[5], address[6], address[7]);
    for (TempValue* t = _vals; t; t = t->next) {
        if (!memcmp(t->address, address, AddressSize))
            return t;
    }

    DEBUG_LOG("findTempValue(): NOT FOUND: %02x%02x%02x%02x%02x%02x%02x%02x",
              address[0], address[1], address[2], address[3],
              address[4], address[5], address[6], address[7]);
    return 0;
}

FakeBoardIO::FakeBoardIO(Recorder* rec): _rec(rec), _pins(0), _millis(0) {}

FakeBoardIO::~FakeBoardIO()
{
    for (Pin *next, *p = _pins; p; p = next) {
        next = p->next;
        if (p->tempSensors)
            delete p->tempSensors;
        delete p;
    }
}

unsigned long FakeBoardIO::millis()
{
    return _millis;
}

void FakeBoardIO::setPinMode(int id, BoardIO::PinMode mode)
{
    if (mode != BoardIO::Input && mode != BoardIO::Output)
        TEST_FAIL_MESSAGE("bad pin mode");
    _rec->record("Pin %d mode %s", id, mode == BoardIO::Input ? "Input" : "Output");
    ensurePin(id)->mode = mode;
}

BoardIO::PinValue FakeBoardIO::digitalRead(int id)
{
    Pin* p = findPin(id);
    if (!p)
        TEST_FAIL_MESSAGE("pin not found");
    return p->value;
}

void FakeBoardIO::digitalWrite(int id, PinValue value)
{
    _rec->record("digitalWrite: %d <- %s", id,
                 value == BoardIO::Low ? "Low" :
                 value == BoardIO::High ? "High" :
                 "<bad value>");
    setDigitalPinValue(id, value);
}

int FakeBoardIO::analogRead(int id)
{
    Pin* p = findPin(id);
    if (!p)
        TEST_FAIL_MESSAGE("pin not found");
    return p->analogValue;
}

void FakeBoardIO::analogWrite(int id, int value)
{
    _rec->record("analogWrite: %d <- %d", id, value);
    setAnalogPinValue(id, value);
}

void FakeBoardIO::setDigitalPinValue(int id, PinValue value)
{
    if (value != BoardIO::Low && value != BoardIO::High)
        TEST_FAIL_MESSAGE("bad digital pin value");
    Pin* p = findPin(id);
    if (!p)
        TEST_FAIL_MESSAGE("pin not found");
    p->value = value;
}

void FakeBoardIO::setAnalogPinValue(int id, int value)
{
    Pin* p = findPin(id);
    if (!p)
        TEST_FAIL_MESSAGE("pin not found");
    p->analogValue = value;
}

TempSensors* FakeBoardIO::getTempSensors(int id)
{
    Pin* p = findPin(id);
    if (!p) {
        p = ensurePin(id);
        p->tempSensors = new FakeTempSensors(_rec);
    } else if (!p->tempSensors)
        TEST_FAIL_MESSAGE("getTempSensors(): pin already in use");
    return p->tempSensors;
}

void FakeBoardIO::setTemperature(int id, uint8_t* address, double value)
{
    Pin* p = findPin(id);
    if (!p)
        TEST_FAIL_MESSAGE("pin not found");
    if (!p->tempSensors)
        TEST_FAIL_MESSAGE("setTemperature(): pin is not associated with TempSensors");
    p->tempSensors->set(address, value);
}

void FakeBoardIO::makeTempsAvailable()
{
    for (Pin* p = _pins; p; p = p->next) {
        if (p->tempSensors)
            p->tempSensors->makeAvailable();
    }
}

void FakeBoardIO::elapse(unsigned long interval)
{
    _millis += interval;
    _rec->record("elapse: %d", interval);
}

FakeBoardIO::Pin* FakeBoardIO::findPin(int id)
{
    for (Pin* p = _pins; p; p = p->next) {
        if (p->id == id)
            return p;
    }
    return 0;
}

FakeBoardIO::Pin* FakeBoardIO::ensurePin(int id)
{
    Pin *p = findPin(id);
    if (!p) {
        p = new Pin(id);
        p->next = _pins;
        _pins = p;
    }
    return p;
}

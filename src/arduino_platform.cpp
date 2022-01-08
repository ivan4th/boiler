#include <assert.h>
#include <avr/io.h>
#include <EEPROM.h>
#include <ArduinoLog.h>
#include <max6675.h>

#include "arduino_platform.h"

namespace {
    const unsigned int MaxPayloadLength = 32;
    const int mqttReconnectInterval = 5000;

    // TODO: REMOVE THIS!!!
    const int thermoSCK = CONTROLLINO_D6;
    const int thermoCS = CONTROLLINO_D8;
    const int thermoSO = CONTROLLINO_D10;
    MAX6675 thermocouple(thermoSCK, thermoCS, thermoSO);
};

ArduinoMqtt* ArduinoMqtt::_instance;

// XXX: pass brokerIP!!!
ArduinoMqtt::ArduinoMqtt(const char* brokerIP, uint16_t port, EthernetClient* ethClient):
    _client(IPAddress(192, 168, 20, 61), port, callback, *ethClient)
{
    _instance = this;
}

void ArduinoMqtt::loop()
{
    if (connected()) {
        _client.loop();
        return;
    }

    long now = millis();
    if (now - lastReconnectAttempt > mqttReconnectInterval) {
        lastReconnectAttempt = now;
        if (_client.connect("boilerClient")) {
            Log.notice(F("client connected\n"));
            lastReconnectAttempt = 0;
            if (_handler)
                _handler->onConnect();
        } else
            Log.error(F("MQTT connection failed\n"));
    }
}

bool ArduinoMqtt::connected()
{
    return _client.connected();
}

void ArduinoMqtt::setHandler(MqttHandler* handler)
{
    _handler = handler;
}

void ArduinoMqtt::publish(const char* topic, const char* payload, bool retain)
{
    Log.trace(F("publish: %s %s%s\n"), topic, payload, retain ? " r" : "");
    _client.publish(topic, payload, retain);
}

void ArduinoMqtt::subscribe(const char* topic)
{
    Log.trace(F("subscribe: %s\n"), topic);
    _client.subscribe(topic);
}

void ArduinoMqtt::callback(char* topic, byte* payload, unsigned int length)
{
    char buf[MaxPayloadLength];
    if (length >= MaxPayloadLength)
        length = MaxPayloadLength - 1;
    memcpy(buf, payload, length);
    buf[length] = 0;
    Log.trace(F("Got MQTT message: %s %s\n"), topic, buf);
    _instance->_handler->onMessage(topic, buf);
}

void ArduinoEeprom::writeBytes(const uint8_t* data, int offset, int count)
{
    Log.trace(F("EEPROM write %d bytes at %d\n"), offset, count);
    while (count--)
        EEPROM.update(offset++, *data++);
}

void ArduinoEeprom::readBytes(uint8_t *data, int offset, int count)
{
    Log.trace(F("EEPROM read %d bytes at %d\n"), offset, count);
    while (count--)
        *data++ = EEPROM.read(offset++);
}

ArduinoTempSensors::ArduinoTempSensors(int pin):
    _oneWire(pin), _sensors(&_oneWire)
{
    _sensors.setWaitForConversion(false);
    _sensors.begin();
    int n = _sensors.getDeviceCount();
    Log.trace(F("Got %d temp sensors\n"), n);
    for (int i = 0; i < n; i++) {
        DeviceAddress address;
        if (_sensors.getAddress(address, i)) {
            char addrStr[64];
            snprintf(addrStr, 64, "%02x%02x%02x%02x%02x%02x%02x%02x",
                     address[0], address[1], address[2], address[3],
                     address[4], address[5], address[6], address[7]);
            Log.trace(F("temp sensor %d: %s\n"), i, addrStr);
        } else
            Log.trace(F("temp sensor %d: error getting address\n"), i);
    }
}

void ArduinoTempSensors::request()
{
    Log.trace(F("request temperatures\n"));
    _sensors.requestTemperatures();
}

bool ArduinoTempSensors::available()
{
    return _sensors.isConversionComplete();
}

double ArduinoTempSensors::read(uint8_t* address)
{
    char addrStr[64];
    snprintf(addrStr, 64, "%02x%02x%02x%02x%02x%02x%02x%02x",
             address[0], address[1], address[2], address[3],
             address[4], address[5], address[6], address[7]);
    double r = _sensors.getTempC(address);
    Log.trace(F("read temperature @ %s = %D\n"), addrStr, r);
    return r;
}

unsigned long ArduinoTempSensors::tempWaitMs()
{
    // FIXME: support different bit resolutions, and, consequently,
    // shorter wait times
    return 750;
}

ArduinoBoardIO::ArduinoBoardIO(): _tempPinId(-1), _tempSensors(0)
{
    for (int i = 0; i < NumPins; ++i)
        _pinMappings[i] = -1;
}

unsigned long ArduinoBoardIO::millis()
{
    return ::millis();
}

void ArduinoBoardIO::setPinMapping(int id, int physicalPin)
{
    assert(id < NumPins);
    _pinMappings[id] = physicalPin;
    Log.trace(F("setPinMapping %d %d\n"), id, physicalPin);
}

void ArduinoBoardIO::setPinMode(int id, ArduinoBoardIO::PinMode mode)
{
    assert(id < NumPins && (mode == Input || mode == Output));
    int physicalPin = _pinMappings[id];
    assert(physicalPin >= 0);
    pinMode(physicalPin, mode == Input ? INPUT : OUTPUT);
    Log.trace(F("pinMode %d %s\n"), physicalPin, mode == Input ? "INPUT" : "OUTPUT");
}

ArduinoBoardIO::PinValue ArduinoBoardIO::digitalRead(int id)
{
    assert(id < NumPins);
    int physicalPin = _pinMappings[id];
    assert(physicalPin >= 0);
    return ::digitalRead(physicalPin) == HIGH ? High : Low;
}

void ArduinoBoardIO::digitalWrite(int id, PinValue value)
{
    assert(id < NumPins && (value == Low || value == High));
    int physicalPin = _pinMappings[id];
    assert(physicalPin >= 0);
    ::digitalWrite(physicalPin, value == Low ? LOW : HIGH);
    Log.trace(F("digitalWrite %d %s\n"), physicalPin, value == Low ? "LOW" : "HIGH");
}

int ArduinoBoardIO::analogRead(int id)
{
    assert(id < NumPins);
    int physicalPin = _pinMappings[id];
    assert(physicalPin >= 0);
    return ::analogRead(physicalPin);
}

void ArduinoBoardIO::analogWrite(int id, int value)
{
    assert(id < NumPins);
    int physicalPin = _pinMappings[id];
    assert(physicalPin >= 0);
    ::analogWrite(physicalPin, value);
    Log.trace(F("analogWrite %d %d\n"), physicalPin, value);
}

TempSensors* ArduinoBoardIO::getTempSensors(int id)
{
    // TODO: support more than one TempSensors instance
    assert(_tempPinId < 0 || id == _tempPinId);
    if (!_tempSensors) {
        assert(id < NumPins);
        int physicalPin = _pinMappings[id];
        assert(physicalPin >= 0);
        _tempSensors = new ArduinoTempSensors(physicalPin);
    }
    return _tempSensors;
}

double ArduinoBoardIO::getThermocoupleValue()
{
    return thermocouple.readCelsius();
}

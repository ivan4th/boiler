#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "cell.h"

Cell::Cell(const char* name):
    _next(0), _name(name), _writable(false), _persistent(false), _ready(true)
{
    for (int i = 0; i < maxObservers; ++i)
        _observers[i] = 0;
}

void Cell::notify()
{
    _ready = true;
    for (int i = 0; i < maxObservers; ++i) {
        if (_observers[i])
            _observers[i]->cellChanged(this);
    }
}

void Cell::observe(CellObserver* observer)
{
    for (int i = 0; i < maxObservers; ++i) {
        if (!_observers[i]) {
            _observers[i] = observer;
            break;
        }
    }
}

void Cell::setNotReady()
{
    _ready = false;
}

bool Cell::ready() const
{
    return _ready;
}

template<> void TypedCell<double>::toPayload(char* buf) const
{
#ifndef TEST_NATIVE
    dtostrf(_value, 1, 4, buf);
#else
    sprintf(buf, "%.4f", _value);
#endif
}

template<> void TypedCell<double>::fromPayload(const char* buf)
{
    _value = atof(buf);
}

template<> void TypedCell<bool>::toPayload(char* buf) const
{
    buf[0] = _value ? '1' : '0';
    buf[1] = 0;
}

template<> void TypedCell<bool>::fromPayload(const char* buf)
{
    _value = buf[0] == '1' && !buf[1];
}

CellSet::CellSet(Mqtt* mqtt, Eeprom* eeprom, const char* devName):
    _mqtt(mqtt), _storage(new CellStorage(eeprom)), _devName(devName),
    _firstCell(0), _lastCell(0)
{
    _mqtt->setHandler(this);
}

CellSet::~CellSet()
{
    for (Cell *next, *c = _firstCell; c; c = next) {
        next = c->next();
        delete c;
    }
    delete _storage;
}

void CellSet::addCell(Cell* cell)
{
    cell->observe(this);
    if (_firstCell) {
        _lastCell->setNext(cell);
        _lastCell = cell;
    } else
        _firstCell = _lastCell = cell;
}

void CellSet::onConnect()
{
    for (Cell* c = _firstCell; c; c = c->next()) {
        if (c->writable()) {
            char topic[128];
            snprintf(topic, 128, "/devices/%s/controls/%s/on", _devName, c->name());
            _mqtt->subscribe(topic);
        }
        publishCell(c);
    }
}

void CellSet::onMessage(const char* topic, const char* payload)
{
    if (memcmp(topic, "/devices/", 9))
        return;

    int l = strlen(_devName);
    if (strncmp(topic+9, _devName, l))
        return;

    const char *tail = topic + 9 + l;
    if (strncmp(tail, "/controls/", 10))
        return;

    tail += 10;
    int tailLen = strlen(tail);
    if (tailLen < 4 || strcmp(tail + tailLen - 3, "/on"))
        return;

    for (Cell* c = _firstCell; c; c = c->next()) {
        const char* name = c->name();
        int nameLen = strlen(name);
        if (nameLen == tailLen - 3 && !memcmp(tail, name, nameLen)) {
            c->fromPayload(payload);
            storeCell(c);
            c->notify(); // this publishes an MQTT message with the new value
            break;
        }
    }
}

void CellSet::cellChanged(Cell *cell)
{
    publishCell(cell);
}

void CellSet::publishCell(Cell *cell)
{
    if (!_mqtt->connected() || !cell->ready())
        return;

    char topic[128], payload[64];
    snprintf(topic, 128, "/devices/%s/controls/%s", _devName, cell->name());
    cell->toPayload(payload);
    _mqtt->publish(topic, payload, true);
}

void CellSet::storeCell(Cell *cell)
{
    // When the EEPROM has mismatching version or isn't initialized,
    // we must store all of the cell values
    bool canSkip = _storage->startWrite();
    for (Cell* c = _firstCell; c; c = c->next()) {
        if (!c->persistent())
            continue;
        if (canSkip && c != cell)
            _storage->skip(c->size());
        else {
            c->store(_storage);
            if (canSkip)
                break;
        }
    }
}

bool CellSet::load()
{
    if (!_storage->startRead())
        return false;
    for (Cell* c = _firstCell; c; c = c->next()) {
        if (!c->persistent())
            continue;
        c->load(_storage);
    }
    return true;
}

// the last 2 bytes of the signature represent the version
uint8_t CellStorage::signature[] = { 0xcd, 0x4c, 0x88, 0x0d, 0x93, 0xc4, 0x00, 0x00 };

CellStorage::CellStorage(Eeprom* eeprom): _eeprom(eeprom), _p(0), _signatureWritten(false)
{
    signature[signatureSize - 2] = (CELL_STORAGE_VERSION >> 8) & 255;
    signature[signatureSize - 1] = CELL_STORAGE_VERSION & 255;
}

bool CellStorage::startRead()
{
    uint8_t s[signatureSize];
    _eeprom->readBytes(s, 0, signatureSize);
    if (memcmp(s, signature, signatureSize))
        return false;
    _signatureWritten = true;
    _p = signatureSize;
    return true;
}

bool CellStorage::startWrite()
{
    _p = signatureSize;
    if (!_signatureWritten) {
        _eeprom->writeBytes(signature, 0, signatureSize);
        _signatureWritten = true;
        return false;
    }
    return true;
}

void CellStorage::readBytes(uint8_t* data, int count)
{
    _eeprom->readBytes(data, _p, count);
    _p += count;
}

void CellStorage::skip(int count)
{
    _p += count;
}

void CellStorage::writeBytes(const uint8_t* data, int count)
{
    _eeprom->writeBytes(data, _p, count);
    _p += count;
}

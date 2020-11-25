#ifndef __CELL_H__
#define __CELL_H__

#include <string.h>

#include "interfaces.h"

// FIXME: use md5 hash of persistent cell names + types instead
#ifndef CELL_STORAGE_VERSION
#define CELL_STORAGE_VERSION 6
#endif

class CellStorage {
public:
    CellStorage(Eeprom* eeprom);
    bool startRead();
    bool startWrite();
    void readBytes(uint8_t* data, int count);
    void skip(int count);
    void writeBytes(const uint8_t* data, int count);
private:
    Eeprom* _eeprom;
    int _p;
    static const int signatureSize = 8;
    static uint8_t signature[];
    bool _signatureWritten;
};

class Cell;

class CellObserver {
public:
    virtual ~CellObserver() {};
    virtual void cellChanged(Cell* cell) = 0;
};

class Cell {
public:
    Cell(const char* name);
    virtual ~Cell() {};
    Cell* next() const { return _next; }
    void setNext(Cell* next) { _next = next; }
    const char* name() const { return _name; }
    bool writable() const { return _writable; }
    bool persistent() const { return _persistent; }
    void observe(CellObserver* observer);
    virtual int size() const = 0;
    virtual void store(CellStorage* storage) const = 0;
    virtual void load(CellStorage* storage) = 0;
    virtual void toPayload(char* buf) const = 0;
    virtual void fromPayload(const char* buf) = 0;
    Cell* setWritable() { _writable = true; return this; }
    Cell* setPersistent() { _persistent = true; return this; }
    void notify();
    void setNotReady();
    bool ready() const;

private:
    static const int maxObservers = 8;
    Cell* _next;
    const char* _name;
    bool _writable, _persistent, _ready;
    CellObserver* _observers[maxObservers];
};

template<class T> class TypedCell: public Cell {
public:
    TypedCell(const char* name, double value): Cell(name), _value(value) {}
    T value() const { return _value; }
    void setValue(T v) {
        _value = v;
        notify();
    };
    int size() const { return sizeof(T); }
    void store(CellStorage* storage) const {
        storage->writeBytes((const uint8_t*)&_value, sizeof(_value));
    }
    void load(CellStorage* storage) {
        storage->readBytes((uint8_t*)&_value, sizeof(_value));
    }
    void toPayload(char* buf) const;
    void fromPayload(const char* buf);
    TypedCell* setWritable() { Cell::setWritable(); return this; }
    TypedCell* setPersistent() { Cell::setPersistent(); return this; }
private:
    T _value;
};

template<> void TypedCell<double>::toPayload(char* buf) const;
template<> void TypedCell<double>::fromPayload(const char* buf);
template<> void TypedCell<bool>::toPayload(char* buf) const;
template<> void TypedCell<bool>::fromPayload(const char* buf);

class CellSet: CellObserver, MqttHandler {
public:
    CellSet(Mqtt* mqtt, Eeprom* eeprom, const char* devName);
    ~CellSet();
    void addCell(Cell* cell);
    template<class T> TypedCell<T> *addCell(const char* name, T value = T()) {
        TypedCell<T> *c = new TypedCell<T>(name, value);
        addCell(c);
        return c;
    }
    void onConnect();
    void onMessage(const char* topic, const char* payload);
    void cellChanged(Cell* cell);
    bool load();

private:
    void publishCell(Cell* cell);
    void storeCell(Cell* cell);

    Mqtt* _mqtt;
    CellStorage* _storage;
    const char* _devName;
    Cell* _firstCell;
    Cell* _lastCell;
};

#endif

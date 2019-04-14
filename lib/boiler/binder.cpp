#include "binder.h"
#include "debug.h"

CellBinder::CellBinder(BoardIO* io, unsigned long tempPollIntervalMs, unsigned long pollIntervalMs):
    _io(io),
    _firstBinding(0),
    _nonTempsUpdated(false),
    _tempPollIntervalMs(tempPollIntervalMs),
    _pollIntervalMs(pollIntervalMs),
    _tempPollState(NotStarted),
    _tempSensors(0)
{
    _tempsRequestedAt = _nonTempsUpdatedAt = _io->millis();
}

CellBinder::~CellBinder()
{
    for (CellBinding *next, *b = _firstBinding; b; b = next) {
        next = b->next();
        delete b;
    }
}

void CellBinder::addDigitalInputBinding(int pinId, TypedCell<bool>* cell)
{
    addBinding(new DigitalInputBinding(_io, pinId, cell));
}

void CellBinder::addDigitalOutputBinding(int pinId, TypedCell<bool>* cell)
{
    addBinding(new DigitalOutputBinding(_io, pinId, cell));
}

void CellBinder::addAnalogInputBinding(int pinId, TypedCell<double>* cell, const ValueTransform& transform, int nAvg)
{
    addBinding(new AnalogInputBinding(_io, pinId, cell, transform, nAvg));
}

void CellBinder::addAnalogOutputBinding(int pinId, TypedCell<double>* cell, const ValueTransform& transform)
{
    addBinding(new AnalogOutputBinding(_io, pinId, cell, transform));
}

void CellBinder::addTemperatureBinding(int pinId, TypedCell<double> *cell, uint8_t* address, int nAvg)
{
    TempSensors* sensors = _io->getTempSensors(pinId);
    addBinding(new TemperatureBinding(sensors, cell, address, nAvg));
}

void CellBinder::addBinding(CellBinding* binding)
{
    // FIXME: support multiple TempSensors
    if (binding->tempSensors() && !_tempSensors)
        _tempSensors = binding->tempSensors();
    binding->setNext(_firstBinding);
    _firstBinding = binding;
}

void CellBinder::setup()
{
    for (CellBinding* b = _firstBinding; b; b = b->next())
        b->setup();
}

void CellBinder::updateCells()
{
    // DEBUG_LOG("updateCells()");
    // TODO: support multiple TempSensors
    bool requestTemps = false, updateTemps = false;
    unsigned long ms = _io->millis();
    if (_tempSensors) {
        switch (_tempPollState) {
        case NotStarted:
            DEBUG_LOG("updateCells(): NotStarted: requesting temps");
            requestTemps = true;
            break;
        case TempsRequested:
            if (_tempSensors->available()) {
                DEBUG_LOG("updateCells(): TempsRequested: temps available, updating");
                updateTemps = true;
                if ((ms - _tempsRequestedAt) >= _tempPollIntervalMs)
                    requestTemps = true;
                else
                    _tempPollState = Wait;
            } else if ((ms - _tempsRequestedAt) >= _tempSensors->tempWaitMs()) {
                DEBUG_LOG("updateCells(): TempsRequested: temp request timed out, re-requesting");
                requestTemps = true;
            } else
                DEBUG_LOG("updateCells(): TempsRequested: temps not available yet");
            break;
        case Wait:
            requestTemps = (ms - _tempsRequestedAt) >= _tempPollIntervalMs;
            DEBUG_LOG("updateCells(): Wait: ms %ld, _tempsRequestedAt %ld, interval %ld, requestTemps: %s", ms, _tempsRequestedAt, _tempPollIntervalMs, requestTemps ? "true" : "false");
            break;
        }
    } else
        DEBUG_LOG("updateCells(): no temp sensors");

    bool updateNonTemps = !_nonTempsUpdated || (ms - _nonTempsUpdatedAt) >= _pollIntervalMs;
    DEBUG_LOG("_nonTempsUpdatedAt %ld ms %ld pollIntervalMs %ld updateNonTemps %s", _nonTempsUpdatedAt, ms, _pollIntervalMs, updateNonTemps ? "true" : "false");
    if (updateNonTemps) {
        _nonTempsUpdated = true;
        _nonTempsUpdatedAt = ms;
    }

    for (CellBinding* b = _firstBinding; b; b = b->next()) {
        if (b->tempSensors()) {
            if (updateTemps)
                b->updateCellFromTarget();
            else
                b->setNotReady();
        } else if (updateNonTemps)
            b->updateCellFromTarget();
    }

    DEBUG_LOG("updateCells(): requestTemps: %s", requestTemps ? "true" : "false");
    if (requestTemps) {
        DEBUG_LOG("updateCells(): requesting temps");
        _tempPollState = TempsRequested;
        _tempsRequestedAt = ms;
        _tempSensors->request();
    }
}

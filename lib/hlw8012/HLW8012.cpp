/*
15mA or 0.5V for a 1Hz wave
For the suggested application in the datasheet a frequency of 1Hz corresponds to a ~12W active power, 10Hz to ~120W, 100Hz to ~1.2kW and so on.
HLW8012

Copyright (C) 2016-2023 by Xose Pérez <xose dot perez at gmail dot com>

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/

#include <Arduino.h>
#include "HLW8012.h"

void HLW8012::begin(
    unsigned char cf_pin,
    unsigned char cf1_pin,
    unsigned char sel_pin,
    unsigned char currentWhen,
    bool use_interrupts,
    unsigned long pulse_timeout)
{

    _cf_pin = cf_pin;
    _cf1_pin = cf1_pin;
    _sel_pin = sel_pin;
    _current_mode = currentWhen;
    _use_interrupts = use_interrupts;
    _pulse_timeout = pulse_timeout;

    pinMode(_cf_pin, INPUT_PULLUP);
    pinMode(_cf1_pin, INPUT_PULLUP);
    pinMode(_sel_pin, OUTPUT);

    _calculateDefaultMultipliers();

    _mode = _current_mode;
    digitalWrite(_sel_pin, _mode);
}

void HLW8012::setMode(hlw8012_mode_t mode)
{
    _mode = (mode == MODE_CURRENT) ? _current_mode : 1 - _current_mode;
    digitalWrite(_sel_pin, _mode);
    if (_use_interrupts)
    {
        _last_cf1_interrupt = _first_cf1_interrupt = micros();
    }
}

hlw8012_mode_t HLW8012::getMode()
{
    return (_mode == _current_mode) ? MODE_CURRENT : MODE_VOLTAGE;
}

hlw8012_mode_t HLW8012::toggleMode()
{
    hlw8012_mode_t new_mode = getMode() == MODE_CURRENT ? MODE_VOLTAGE : MODE_CURRENT;
    setMode(new_mode);
    return new_mode;
}

float HLW8012::getCurrent(unsigned long pulse_timeout)
{

    // Power measurements are more sensitive to switch offs,
    // so we first check if power is 0 to set _current to 0 too
    if (_power == 0)
    {
        // if (getActivePower() == 0) _current_pulse_width = 0;
        return 0;
    }
    else if (_use_interrupts)
    {
        _checkCF1Signal();
    }
    else if (_mode == _current_mode)
    {
        /* unsigned long pulse_width = 0;
         for (int i = 0; i < 4; i++)
         {
             pulse_width += pulseIn(_cf1_pin, HIGH, _pulse_timeout);
             if (pulse_width < 1 || _current_multiplier / pulse_width / 2 == 0)
             {
                 break;
             }
         }
         _current_pulse_width = pulse_width / 4;*/

        _current_pulse_width = pulseIn(_cf1_pin, HIGH, pulse_timeout);
        _current = (_current_pulse_width > 0) ? _current_multiplier / static_cast<float>(_current_pulse_width) / 2.0f : 0.0f;
    }

    return _current;
}

unsigned int HLW8012::getVoltage(unsigned long pulse_timeout)
{
    if (_use_interrupts)
    {
        _checkCF1Signal();
    }
    else if (_mode != _current_mode)
    {
        /*
        unsigned long pulse_width = 0;
        for (int i = 0; i < 10; i++)
        {
            pulse_width += pulseIn(_cf1_pin, HIGH, _pulse_timeout);
            if (pulse_width < 1 || _voltage_multiplier / pulse_width / 2 < 10)
            {
                break;
            }
        }
        _voltage_pulse_width = pulse_width / 10;
        */

        _voltage_pulse_width = pulseIn(_cf1_pin, HIGH, pulse_timeout);
        _voltage = (_voltage_pulse_width > 0) ? _voltage_multiplier / _voltage_pulse_width / 2 : 0;
    }

    return (_voltage > 10) ? _voltage : 0;
}

unsigned int HLW8012::getActivePower(unsigned long pulse_timeout)
{
    // if (_use_interrupts)
    // {
    //     _checkCFSignal();
    // }
    // else
    // {
    /*
    unsigned long pulse_width = 0;
    for (int i = 0; i < 4; i++)
    {
        pulse_width += pulseIn(_cf_pin, HIGH, _pulse_timeout);
        if (pulse_width < 1 || _power_multiplier / pulse_width / 2 < 1)
        {
            break;
        }
    }
    _power_pulse_width = pulse_width / 4;
    */
    _power_pulse_width = pulseIn(_cf_pin, HIGH, pulse_timeout);
    //}
    _power = (_power_pulse_width > 0) ? _power_multiplier / _power_pulse_width / 2 : 0;
    return _power;
}

unsigned int HLW8012::getApparentPower()
{
    float current = getCurrent(_pulse_timeout);
    unsigned int voltage = getVoltage(_pulse_timeout);
    return voltage * current;
}

unsigned int HLW8012::getReactivePower()
{
    float active = getActivePower(_pulse_timeout);
    float apparent = getApparentPower();
    if (apparent > active)
    {
        return sqrt(apparent * apparent - active * active);
    }
    else
    {
        return 0;
    }
}

float HLW8012::getPowerFactor()
{
    float active = getActivePower(_pulse_timeout);
    float apparent = getApparentPower();
    if (active > apparent)
        return 1.0f;
    if (apparent == 0)
        return 0.0f;
    return (float)active / apparent;
}

float HLW8012::getEnergy()
{

    // Counting pulses only works in IRQ mode
    if (!_use_interrupts)
        return 0;

    /*
    Pulse count is directly proportional to energy:
    P = m*f (m=power multiplier, f = Frequency)
    f = N/t (N=pulse count, t = time)
    E = P*t = m*N  (E=energy)
    */
    return _pulse_count * _power_multiplier / 1000000. / 2;
}

void HLW8012::resetEnergy()
{
    _pulse_count = 0;
}

void HLW8012::expectedCurrent(float value)
{
    if (static_cast<int>(_current) == 0)
        getCurrent(_pulse_timeout);
    if (static_cast<int>(_current) > 0)
        _current_multiplier *= (value / _current);
}

void HLW8012::expectedVoltage(float value)
{
    if (static_cast<int>(_voltage) == 0)
        getVoltage(_pulse_timeout);
    if (static_cast<int>(_voltage) > 0)
        _voltage_multiplier *= ((float)value / _voltage);
}

void HLW8012::expectedActivePower(float value)
{
    if (static_cast<int>(_power) == 0)
        getActivePower(_pulse_timeout);
    if (static_cast<int>(_power) > 0)
        _power_multiplier *= ((float)value / _power);
}

void HLW8012::resetMultipliers()
{
    _calculateDefaultMultipliers();
}

void HLW8012::setResistors(float current, float voltage_upstream, float voltage_downstream)
{
    if (voltage_downstream > 0)
    {
        _current_resistor = current;
        _voltage_resistor = (voltage_upstream + voltage_downstream) / voltage_downstream;
        _calculateDefaultMultipliers();
    }
}

void ICACHE_RAM_ATTR HLW8012::cf_interrupt()
{
    unsigned long now = micros();
    _power_pulse_width = now - _last_cf_interrupt;
    _last_cf_interrupt = now;
    _pulse_count++;
}

void ICACHE_RAM_ATTR HLW8012::cf1_interrupt()
{

    unsigned long now = micros();

    if ((now - _first_cf1_interrupt) > _pulse_timeout)
    {

        unsigned long pulse_width;

        if (_last_cf1_interrupt == _first_cf1_interrupt)
        {
            pulse_width = 0;
        }
        else
        {
            pulse_width = now - _last_cf1_interrupt;
        }

        if (_mode == _current_mode)
        {
            _current_pulse_width = pulse_width;
        }
        else
        {
            _voltage_pulse_width = pulse_width;
        }

        _mode = 1 - _mode;
        digitalWrite(_sel_pin, _mode);
        _first_cf1_interrupt = now;
    }

    _last_cf1_interrupt = now;
}

void HLW8012::_checkCFSignal()
{
    if ((micros() - _last_cf_interrupt) > _pulse_timeout)
        _power_pulse_width = 0;
}

void HLW8012::_checkCF1Signal()
{
    if ((micros() - _last_cf1_interrupt) > _pulse_timeout)
    {
        if (_mode == _current_mode)
        {
            _current_pulse_width = 0;
        }
        else
        {
            _voltage_pulse_width = 0;
        }
        toggleMode();
    }
}

// These are the multipliers for current, voltage and power as per datasheet
// These values divided by output period (in useconds) give the actual value
// For power a frequency of 1Hz means around 12W
// For current a frequency of 1Hz means around 15mA
// For voltage a frequency of 1Hz means around 0.5V
void HLW8012::_calculateDefaultMultipliers()
{
    _current_multiplier = (1000000.0 * 512 * V_REF / _current_resistor / 24.0 / F_OSC);
    _voltage_multiplier = (1000000.0 * 512 * V_REF * _voltage_resistor / 2.0 / F_OSC);
    _power_multiplier = (1000000.0 * 128 * V_REF * V_REF * _voltage_resistor / _current_resistor / 48.0 / F_OSC);
}

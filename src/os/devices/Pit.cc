/*
 * Copyright (C) 2018 Burak Akguel, Christian Gesse, Fabian Ruhland, Filip Krakowski, Michael Schoettner
 * Heinrich-Heine University
 *
 * This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any
 * later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>
 */

#include <kernel/Kernel.h>
#include <kernel/interrupts/IntDispatcher.h>
#include <kernel/interrupts/Pic.h>
#include <kernel/threads/NullYielder.h>
#include "kernel/threads/Scheduler.h"
#include "devices/Pit.h"

IOport control(0x43);

IOport data0(0x40);

Pit* Pit::instance = nullptr;

NullYielder nullYielder;

Pit::Pit(uint32_t us) {

    yieldable = &nullYielder;

    setInterval(us);
}

Pit *Pit::getInstance() {

    if(instance == nullptr) {

        instance = new Pit(DEFAULT_INTERVAL);
    }

    return instance;
}

void Pit::setInterval(uint32_t us) {

    timerInterval = us / 1000;

    uint32_t divisor = (us * 1000) / TIME_BASE;

    control.outb(0x36);

    data0.outb((uint8_t) (divisor & 0xff));

    data0.outb((uint8_t) (divisor >> 8));
}

uint32_t Pit::getInterval() {

    return timerInterval;
}

void Pit::plugin () {

    IntDispatcher::getInstance().assign(32, *this);

    Pic::getInstance()->allow(Pic::Interrupt::PIT);
}

void Pit::trigger () {

    ticks++;

    yieldable->yield();
}

uint32_t Pit::getNanos() {

    return ticks * timerInterval * 1000000;
}

uint32_t Pit::getMicros() {

    return ticks * timerInterval * 1000;
}

uint32_t Pit::getMillis() {

    return ticks * timerInterval;
}

uint32_t Pit::getSeconds() {

    return ticks * timerInterval / 1000;
}

uint32_t Pit::getMinutes() {

    return ticks * timerInterval / 1000 / 60;
}

uint32_t Pit::getHours() {

    return ticks * timerInterval / 1000 / 60 / 60;
}

uint32_t Pit::getDays() {

    return ticks * timerInterval / 1000 / 60 / 60 / 24;
}

uint32_t Pit::getYears() {

    return ticks * timerInterval / 1000 / 60 / 60 / 24 / 365;
}

void Pit::setYieldable(Yieldable *yieldable) {

    this->yieldable = yieldable;
}

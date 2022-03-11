/*
 * Copyright (C) 2018-2022 Heinrich-Heine-Universitaet Duesseldorf,
 * Institute of Computer Science, Department Operating Systems
 * Burak Akguel, Christian Gesse, Fabian Ruhland, Filip Krakowski, Michael Schoettner
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

#ifndef HHUOS_SCHEDULERCLEANER_H
#define HHUOS_SCHEDULERCLEANER_H

#include "Process.h"
#include "lib/util/memory/AtomicBitmap.h"

namespace Kernel {

class SchedulerCleaner : public Util::Async::Runnable {

public:
    /**
     * Default Constructor.
     */
    SchedulerCleaner();

    /**
     * Copy constructor.
     */
    SchedulerCleaner(const SchedulerCleaner &other) = delete;

    /**
     * Assignment operator.
     */
    SchedulerCleaner &operator=(const SchedulerCleaner &other) = delete;

    /**
     * Destructor.
     */
    ~SchedulerCleaner() override;

    void cleanup(Process *process);

    void cleanup(Thread *thread);

    void run() override;

private:

    void cleanupProcesses();

    void cleanupThreads();

    Process** processList;
    Thread** threadList;

    Util::Memory::AtomicBitmap processBitmap;
    Util::Memory::AtomicBitmap threadBitmap;

    static const constexpr uint32_t ARRAY_SIZE = 1024;
};

}

#endif
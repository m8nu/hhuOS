/*
 * Copyright (C) 2018-2021 Heinrich-Heine-Universitaet Duesseldorf,
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

#include <lib/util/memory/Address.h>
#include "File.h"

namespace Util::File::Elf {

File::File(uint32_t address) : address(address), fileHeader(*reinterpret_cast<Constants::FileHeader*>(address)) {
    if (!fileHeader.isValid()) {
        Util::Exception::throwException(Util::Exception::INVALID_ARGUMENT, "Elf: Invalid file!");
    }

    const auto &sectionHeaderStringHeader = *reinterpret_cast<Constants::SectionHeader*>(address + fileHeader.sectionHeader + fileHeader.sectionHeaderStringIndex * fileHeader.sectionHeaderEntrySize);
    sectionNames = reinterpret_cast<char *>(address + sectionHeaderStringHeader.offset);
    programHeaders = reinterpret_cast<Constants::ProgramHeader*>(address + fileHeader.programHeader);
    sectionHeaders = reinterpret_cast<Constants::SectionHeader*>(address + fileHeader.sectionHeader);
}

uint32_t File::getEndAddress() {
    uint32_t ret = 0;

    for(int i = 0; i < fileHeader.programHeaderEntries; i++) {
        auto header = programHeaders[i];

        if(header.type == Constants::ProgramHeaderType::LOAD) {
            const auto size = header.virtualAddress + header.memorySize;

            if(size > ret) {
                ret = size;
            }
        }
    }

    return ret;
}

void File::loadProgram() {
    for(int i = 0; i < fileHeader.programHeaderEntries; i++) {
        auto header = programHeaders[i];

        if(header.type == Constants::ProgramHeaderType::LOAD) {
            auto sourceAddress = Util::Memory::Address<uint32_t>(address + header.offset, header.fileSize);
            auto targetAddress = Util::Memory::Address<uint32_t>(header.virtualAddress, header.memorySize);

            targetAddress.copyRange(sourceAddress, header.fileSize);
        }
    }
}

}
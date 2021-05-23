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

#include <lib/interface.h>
#include "FileReader.h"

namespace Util::Stream {

FileReader::FileReader(const File::File &file) : fileDescriptor(openFile(file.getCanonicalPath())) {}

FileReader::FileReader(const Memory::String &path) : fileDescriptor(openFile(path)) {}

FileReader::~FileReader() {
    closeFile(fileDescriptor);
}

char FileReader::read() {
    char c = 0;
    int32_t count = read(&c, 0, 1);

    return count > 0 ? c : -1;
}

int32_t FileReader::read(char *targetBuffer, uint32_t length) {
    return read(targetBuffer, 0, length);
}

int32_t FileReader::read(char *targetBuffer, uint32_t offset, uint32_t length) {
    if (fileDescriptor < 0) {
        Util::Exception::throwException(Exception::ILLEGAL_STATE, "FileReader: Unable to open file!");
    }

    if (pos >= getFileLength(fileDescriptor)) {
        return -1;
    }

    uint32_t count = readFile(fileDescriptor, reinterpret_cast<uint8_t *>(targetBuffer + offset), pos, length);
    pos += count;

    return count > 0 ? count : -1;
}

Memory::String FileReader::read(uint32_t length) {
    char *tmpBuffer = new char[length + 1];
    int32_t count = read(tmpBuffer, length);

    Memory::String ret = count <= 0 ? Memory::String() : Memory::String(tmpBuffer);

    delete[] tmpBuffer;
    return ret;
}

}
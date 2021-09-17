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

#ifndef HHUOS_TERMINAL_H
#define HHUOS_TERMINAL_H

#include <cstdint>
#include <lib/util/stream/OutputStream.h>
#include <lib/util/memory/String.h>
#include "Color.h"
#include "Colors.h"

namespace Util::Graphic {

class Terminal : public Stream::OutputStream {

public:

    Terminal(uint16_t columns, uint16_t rows);

    Terminal(const Terminal &copy) = delete;

    Terminal &operator=(const Terminal &other) = delete;

    ~Terminal() override = default;

    void write(uint8_t c) override;

    void write(const uint8_t *sourceBuffer, uint32_t offset, uint32_t length) override;

    virtual void putChar(char c, const Color &foregroundColor, const Color &backgroundColor) = 0;

    virtual void clear(const Color &backgroundColor) = 0;

    virtual void setPosition(uint16_t column, uint16_t row) = 0;

    [[nodiscard]] uint16_t getColumns() const;

    [[nodiscard]] uint16_t getRows() const;

private:

    [[nodiscard]] static Graphic::Color getColor(uint8_t colorCode, const Util::Graphic::Color &defaultColor, const Data::Array<Memory::String> &codes, uint32_t &index);

    [[nodiscard]] static Graphic::Color parseComplexColor(const Data::Array<Memory::String> &codes, uint32_t &index);

    [[nodiscard]] static Graphic::Color parse256Color(const Data::Array<Memory::String> &codes, uint32_t &index);

    [[nodiscard]] static Graphic::Color parseTrueColor(const Data::Array<Memory::String> &codes, uint32_t &index);

    void parseGraphicRendition(uint8_t code);

    Memory::String currentEscapeCode;
    bool isEscapeActive = false;

    Util::Graphic::Color foregroundBaseColor = Graphic::Colors::WHITE;
    Util::Graphic::Color backgroundBaseColor = Graphic::Colors::BLACK;
    Util::Graphic::Color foregroundColor = Graphic::Colors::WHITE;
    Util::Graphic::Color backgroundColor = Graphic::Colors::BLACK;
    bool brightForeground = false;
    bool brightBackground = false;

    bool invert = false;
    bool bright = false;
    bool dim = false;

    const uint16_t columns;
    const uint16_t rows;

};

}

#endif

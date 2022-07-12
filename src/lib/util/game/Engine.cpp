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

#include "Engine.h"
#include "lib/util/time/Timestamp.h"
#include "lib/util/system/System.h"
#include "lib/util/graphic/StringDrawer.h"
#include "lib/util/graphic/Fonts.h"
#include "lib/util/graphic/Colors.h"

namespace Util::Game {

Engine::Engine(Game &game, Util::Graphic::LinearFrameBuffer &lfb, const uint8_t targetFrameRate) :
        game(game), lfb(lfb), targetFrameRate(targetFrameRate) {}

void Engine::run() {
    const auto delta = 1.0 / targetFrameRate;
    const auto deltaMilliseconds = static_cast<uint32_t>(delta * 1000);
    const auto stringDrawer = Graphic::StringDrawer(Graphic::PixelDrawer(lfb));
    uint32_t currentTime = Time::getSystemTime().toMilliseconds();
    uint32_t accumulated = 0;

    while (game.isRunning()) {
        uint32_t newTime = Time::getSystemTime().toMilliseconds();
        uint32_t frameTime = newTime - currentTime;
        if (frameTime == 0) {
            frameTime = 1;
        } else if (frameTime > 250) {
            frameTime = 250;
        }

        currentTime = newTime;
        accumulated += frameTime;

        while (accumulated >= deltaMilliseconds) {
            game.update(delta);
            game.applyChanges();
            accumulated -= deltaMilliseconds;
        }

        auto status = Memory::String::format("Resolution: %ux%u@%u, Objects: %u, Frame Time: %ums", lfb.getResolutionX(), lfb.getResolutionY(), lfb.getColorDepth(), game.getObjectCount(), frameTime);

        lfb.clear();
        game.draw(lfb);
        stringDrawer.drawString(Graphic::Fonts::TERMINAL_FONT_SMALL, 0, 0, static_cast<const char*>(status), Util::Graphic::Colors::WHITE, Util::Graphic::Colors::BLACK);
        lfb.flush();
    }
}

}
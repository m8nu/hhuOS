/*
 * Copyright (C) 2018-2023 Heinrich-Heine-Universitaet Duesseldorf,
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

#ifndef HHUOS_PLAYERMISSILE_H
#define HHUOS_PLAYERMISSILE_H

#include "lib/util/game/entity/Entity.h"
#include "lib/util/game/Sprite.h"
#include "Ship.h"

class PlayerMissile : public Util::Game::Entity {

public:
    /**
     * Constructor.
     */
    PlayerMissile(const Util::Math::Vector2D &position, Ship &ship);

    /**
     * Copy Constructor.
     */
    PlayerMissile(const PlayerMissile &other) = delete;

    /**
     * Assignment operator.
     */
    PlayerMissile &operator=(const PlayerMissile &other) = delete;

    /**
     * Destructor.
     */
    ~PlayerMissile() override = default;

    void initialize() override;

    void onUpdate(double delta) override;

    void onTranslationEvent(Util::Game::TranslationEvent &event) override;

    void onCollisionEvent(Util::Game::CollisionEvent &event) override;

    void draw(Util::Game::Graphics2D &graphics) override;

    static const constexpr uint32_t TAG = 1;
    static const constexpr double SIZE_X = 0.05;
    static const constexpr double SIZE_Y = 0.1083;

private:

    Util::Game::Sprite sprite;
    Ship &ship;
};

#endif

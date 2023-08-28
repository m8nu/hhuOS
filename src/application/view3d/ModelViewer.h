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

#ifndef HHUOS_MODELVIEWER_H
#define HHUOS_MODELVIEWER_H

#include "lib/util/game/3d/Scene.h"
#include "lib/util/game/KeyListener.h"
#include "lib/util/game/MouseListener.h"
#include "Model.h"

class ModelViewer : public Util::Game::D3::Scene, public Util::Game::KeyListener, public Util::Game::MouseListener {

public:
    /**
     * Constructor.
     */
    explicit ModelViewer(const Util::String &path);

    /**
     * Copy Constructor.
     */
    ModelViewer(const ModelViewer &other) = delete;

    /**
     * Assignment operator.
     */
    ModelViewer &operator=(const ModelViewer &other) = delete;

    /**
     * Destructor.
     */
    ~ModelViewer() override = default;

    void update(double delta) override;

    void keyPressed(Util::Io::Key key) override;

    void keyReleased(Util::Io::Key key) override;

    void buttonPressed(Util::Io::Mouse::Button key) override;

    void buttonReleased(Util::Io::Mouse::Button key) override;

    void mouseMoved(const Util::Math::Vector2D &relativeMovement) override;

    void mouseScrolled(Util::Io::Mouse::ScrollDirection direction) override;

private:

    Model *model;

    double zoom = 0;
    Util::Math::Vector3D rotation = Util::Math::Vector3D(0, 0, 0);
};


#endif

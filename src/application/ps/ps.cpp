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

#include <cstdint>
#include "lib/util/system/System.h"
#include "lib/util/stream/FileReader.h"

int32_t main(int32_t argc, char *argv[]) {
    auto processDirectory = Util::File::File("/process");

    Util::System::out << Util::Graphic::Ansi::FOREGROUND_BRIGHT_YELLOW << "PID\tName" << Util::Graphic::Ansi::FOREGROUND_DEFAULT << Util::Stream::PrintWriter::endl;
    for (const auto &child : processDirectory.getChildren()) {
        auto nameFile = Util::File::File(processDirectory.getCanonicalPath() + "/" + child + "/name");
        auto nameStream = Util::Stream::FileReader(nameFile);
        Util::System::out << child << "\t" << nameStream.read(nameFile.getLength() - 1) << Util::Stream::PrintWriter::endl;
    }

    Util::System::out << Util::Stream::PrintWriter::flush;
    return 0;
}
#pragma once

#include "cinder/qtime/QuickTime.h"
#include <string>
#include <vector>

#include "irrKlang/irrKlang.h"

namespace am // am -> asset manager
{
    std::shared_ptr<irrklang::ISound>& audio(const std::string& relativeName);

    ci::qtime::MovieGl& movieGl(const std::string& relativeName);

    ci::qtime::MovieSurface& movieSurface(const std::string& relativeName);
}

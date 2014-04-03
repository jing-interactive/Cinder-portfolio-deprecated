#pragma once

#include "cinder/gl/Texture.h"
#include "cinder/gl/Vbo.h"
#include "cinder/gl/GlslProg.h"
#include <string>
#include <vector>

namespace am // am -> asset manager
{
    ci::Surface& surface(const std::string& relativeName);

    ci::gl::Texture& texture(const std::string& relativeName);

    ci::TriMesh& triMesh(const std::string& relativeName);

    ci::gl::VboMesh& vboMesh(const std::string& relativeName);

    ci::gl::GlslProg& glslProg(const std::string& vsFileName, const std::string& fsFileName);

    std::string& str(const std::string& relativeName);

    std::vector<std::string> files(const std::string& relativeFolderName);
}

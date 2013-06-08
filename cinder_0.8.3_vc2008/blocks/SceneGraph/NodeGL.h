#pragma once

#include "cinder/gl/GlslProg.h"
#include "Node.h"

namespace ph { namespace nodes {

// Basic support for OpenGL nodes
typedef std::shared_ptr<class NodeGL> NodeGLRef;

class NodeGL :
    public Node
{
public:
    NodeGL(void){}
    virtual ~NodeGL(void){}	

    // shader support
    void	setShaderUniform( const std::string &name, int data ){ if(mShader) mShader.uniform(name, data); }
    void	setShaderUniform( const std::string &name, const ci::Vec2i &data ){ if(mShader) mShader.uniform(name, data); }
    void	setShaderUniform( const std::string &name, const int *data, int count ){ if(mShader) mShader.uniform(name, data, count); }	
    void	setShaderUniform( const std::string &name, const ci::Vec2i *data, int count ){ if(mShader) mShader.uniform(name, data, count); }	
    void	setShaderUniform( const std::string &name, float data ){ if(mShader) mShader.uniform(name, data); }
    void	setShaderUniform( const std::string &name, const ci::Vec2f &data ){ if(mShader) mShader.uniform(name, data); }
    void	setShaderUniform( const std::string &name, const ci::Vec3f &data ){ if(mShader) mShader.uniform(name, data); }
    void	setShaderUniform( const std::string &name, const ci::Vec4f &data ){ if(mShader) mShader.uniform(name, data); }
    void	setShaderUniform( const std::string &name, const ci::Color &data ){ if(mShader) mShader.uniform(name, data); }
    void	setShaderUniform( const std::string &name, const ci::ColorA &data ){ if(mShader) mShader.uniform(name, data); }
    void	setShaderUniform( const std::string &name, const ci::Matrix33f &data, bool transpose = false ){ if(mShader) mShader.uniform(name, data, transpose); }
    void	setShaderUniform( const std::string &name, const ci::Matrix44f &data, bool transpose = false ){ if(mShader) mShader.uniform(name, data, transpose); }
    void	setShaderUniform( const std::string &name, const float *data, int count ){ if(mShader) mShader.uniform(name, data, count); }
    void	setShaderUniform( const std::string &name, const ci::Vec2f *data, int count ){ if(mShader) mShader.uniform(name, data, count); }
    void	setShaderUniform( const std::string &name, const ci::Vec3f *data, int count ){ if(mShader) mShader.uniform(name, data, count); }
    void	setShaderUniform( const std::string &name, const ci::Vec4f *data, int count ){ if(mShader) mShader.uniform(name, data, count); }

    void	bindShader(){ if(mShader) mShader.bind(); }
    void	unbindShader(){ if(mShader) mShader.unbind(); }

    // stream support
    virtual inline std::string toString() const { return "NodeGL"; }
protected:
    ci::gl::GlslProg	mShader;
};

} }
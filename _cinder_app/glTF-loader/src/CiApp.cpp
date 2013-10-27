#include "cinder/app/AppBasic.h"
#include "cinder/ImageIo.h"
#include "cinder/Camera.h"
#include "cinder/Json.h"
#include "cinder/Text.h"

#include "cinder/gl/gl.h"
#include "cinder/gl/Texture.h"
#include "cinder/gl/Fbo.h"
#include "cinder/gl/GlslProg.h"
#include "cinder/gl/Vbo.h"
#include "cinder/params/Params.h"

#include "../../../_common/MiniConfig.h"

#include <boost/foreach.hpp>

using namespace ci;
using namespace ci::app;
using namespace std;

typedef function<void(const JsonTree&)> JsonHandler;
typedef pair<string, JsonHandler> NameHandlerPair;
typedef map<string, GLenum> NameEnumMap;

struct Hero
{
    struct Technique
    {
        bool blendEnable;
        bool cullFaceEnable;
        bool depthMask;
        bool depthTestEnable;
    };

    struct Material
    {
        const Technique* pTechnique;

        // vector of Texture ref
        // vector<KVPair> values;

        void execute()
        {
            // textures.bind()
            // set uniform parameters
            // glEnable / glDisable
        }
    };

    struct Index
    {
        gl::Vbo indexBuffer;

        static const int mode = GL_TRIANGLES;
        GLsizei count;
        GLenum  type;
        const GLvoid* indices;

        void execute()
        {
            indexBuffer.bind();
            glDrawElements(mode, count, type, indices);
            indexBuffer.unbind();
        }
    };

    struct MeshAttribute
    {
        gl::Vbo vextexBuffer;

        GLuint index;
        GLint size;
        GLenum type;
        GLboolean normalized;
        GLsizei stride;
        const GLvoid* pointer;

        void execute()
        {
            vextexBuffer.bind();
            glVertexAttribPointer(index, size, type, normalized, stride, pointer);
            vextexBuffer.unbind();
        }
    };
    map<string, DataSourceRef>              mBuffers;
    map<string, gl::Vbo>                    mBufferViews;
    map<string, gl::Texture::Format>        mSamplers;
    map<string, Surface>                    mImages;
    map<string, gl::Texture>                mTextures;
    map<string, Technique>                  mTechniques;
    map<string, Material>                   mMaterials;
    map<string, Index>                      mIndices;
    map<string, MeshAttribute>              mMeshAttributes;
};

struct CiApp : public AppBasic 
{
    void prepareSettings(Settings *settings)
    {
        readConfig();
        
        settings->setWindowPos(0, 0);
        settings->setWindowSize(WIN_WIDTH, WIN_HEIGHT);
    }

    void setup()
    {
        mParams = params::InterfaceGl("params", Vec2i(300, getConfigUIHeight()));
        setupConfigUI(&mParams);

        // parse dota2hero-gh-pages/heroes
        mHeroesPath = fs::path(HEROES_PATH);
        fs::directory_iterator end_iter;
        for (fs::directory_iterator dir_iter(mHeroesPath); dir_iter != end_iter; ++dir_iter)
        {
            if (fs::is_directory(*dir_iter))
            {
				mHeroNames.push_back((*dir_iter).path().filename().string());
            }
    	}

    	mCurrentHero = -1;
        mParams.removeParam("CURRENT_HERO");
    	mParams.addParam("CURRENT_HERO", mHeroNames, &CURRENT_HERO);

        mCurrentTexture = -1;

        // std::functios
#define BIND_PAIR(name, handler) do \
        {\
        JsonHandler childF = bind(&CiApp::handler, this, _1);\
        JsonHandler parentF = bind(&CiApp::handleChildren, this, _1, childF);\
        mCategories.push_back(make_pair(name, parentF));\
        } while (0);
        BIND_PAIR("buffers",        handleBuffer);
        BIND_PAIR("bufferViews",    handleBufferView);
        BIND_PAIR("images",         handleImage);
        BIND_PAIR("videos",         handleDefault);
        BIND_PAIR("samplers",       handleSampler);
        BIND_PAIR("textures",       handleTexture);
        BIND_PAIR("shaders",        handleDefault);
        BIND_PAIR("programs",       handleDefault);
        BIND_PAIR("techniques",     handleTechnique);
        BIND_PAIR("materials",      handleMaterial);
        BIND_PAIR("indices",        handleIndex);
        BIND_PAIR("attributes",     handleMeshAttribute);
        BIND_PAIR("meshes",         handleDefault);
        BIND_PAIR("cameras",        handleDefault);
        BIND_PAIR("lights",         handleDefault);
        BIND_PAIR("skins",          handleDefault);
        BIND_PAIR("nodes",          handleDefault);
        BIND_PAIR("scenes",         handleDefault);
        BIND_PAIR("animations",     handleDefault);
#undef BIND_PAIR  

#define ADD_ENUM(item) mGlNameMap[#item] = GL_##item
        // texture format
        ADD_ENUM(RGB);
        ADD_ENUM(RGBA);

        // texture target
        ADD_ENUM(TEXTURE_1D);
        ADD_ENUM(TEXTURE_2D);
        ADD_ENUM(TEXTURE_3D);

        // texture wrap mode
        ADD_ENUM(REPEAT);
        ADD_ENUM(CLAMP_TO_EDGE);

        // sampler filter
        ADD_ENUM(LINEAR);
        ADD_ENUM(LINEAR_MIPMAP_LINEAR);

        // buffer target
        ADD_ENUM(ARRAY_BUFFER);
        ADD_ENUM(ELEMENT_ARRAY_BUFFER);

        ADD_ENUM(UNSIGNED_SHORT);
#undef ADD_ENUM
    }

    void keyUp(KeyEvent event)
    {
        if (event.getCode() == KeyEvent::KEY_ESCAPE)
        {
            quit();
        }
    }

    void update()
    {
        if (mCurrentHero != CURRENT_HERO)
        {
            mCurrentHero = CURRENT_HERO;
            if (loadHero(mHeroNames[mCurrentHero]))
            {
                CURRENT_TEXTURE = 0;
                mParams.removeParam("CURRENT_TEXTURE");
                mParams.addParam("CURRENT_TEXTURE", mTextureNames, &CURRENT_TEXTURE);
            }
        }

        if (mCurrentTexture != CURRENT_TEXTURE)
        {
            mCurrentTexture = CURRENT_TEXTURE;
        }
    }

    void draw()
    {
        gl::clear(ColorA::black());
        gl::setMatricesWindow(getWindowSize());

        gl::draw(mHero.mTextures[mTextureNames[mCurrentTexture]], Vec2f(100, 100));

        mParams.draw();
    }

private:
    bool loadHero(const std::string& heroName)
    {
        fs::path heroRoot = mHeroesPath / heroName;

        fs::path meshPath = heroRoot / (heroName+".json");
        fs::path animPath = heroRoot / "animations.json";
        fs::path mtrlPath = heroRoot / "materials.json";

        if (!fs::exists(meshPath) ||
            !fs::exists(animPath) ||
            !fs::exists(mtrlPath))
            return false;

        JsonTree heroJsonRoot = JsonTree(loadFile(meshPath));

        mHero = Hero();
        mTextureNames.clear();

        // TODO: exception safe
        BOOST_FOREACH(const NameHandlerPair& aPair, mCategories)
        {
            string name = aPair.first;
            console() << "// " << name << endl;
            JsonHandler handler = aPair.second;

            if (heroJsonRoot.hasChild(name))
            {
               handler(heroJsonRoot.getChild(name)); 
            }
            console() << endl;
        }

        return true;
    }

    void handleDefault(const JsonTree& tree)
    {
    }

    void handleChildren(const JsonTree& tree, const JsonHandler& handler)
    {
        BOOST_FOREACH(const JsonTree& child, tree.getChildren())
        {
#ifdef _DEBUG
            console() << child.getKey() << ":" << child.getChildren().size() << endl;
#endif
            handler(child);
        }
    }

    //"abaddon.bin": {
    //    "byteLength": 1185936,
    //    "path": "abaddon.bin"
    //}
    void handleBuffer(const JsonTree& tree)
    {
        size_t byteLength = tree["byteLength"].getValue<size_t>();
        string path = tree["path"].getValue();
        DataSourceRef& dataSrc = loadFile(getAbsolutePath(path));
        if (dataSrc->getBuffer().getDataSize() != byteLength)
        {
            console() << tree.getKey() << ": byteLength mismatch." << endl;
        }

        mHero.mBuffers[tree.getKey()] = dataSrc;
    }

    //"bufferView_37": {
    //    "buffer": "abaddon.bin",
    //    "byteLength": 1147680,
    //    "byteOffset": 0,
    //    "target": "ARRAY_BUFFER"
    //},
    void handleBufferView(const JsonTree& tree)
    {
        gl::Vbo vbo(getGlEnum(tree, "target"));
        int path = tree["byteLength"].getValue<int>();
        size_t byteLength = tree["byteLength"].getValue<size_t>();
        size_t byteOffset = tree["byteOffset"].getValue<size_t>();

        string bufferName = tree["buffer"].getValue();
        DataSourceRef& dataSrc = mHero.mBuffers[bufferName];
        const Buffer& buffer = dataSrc->getBuffer();

        vbo.bufferData(byteLength, reinterpret_cast<const uint8_t*>(buffer.getData()) + byteOffset, GL_STATIC_DRAW);
        vbo.unbind();

        mHero.mBufferViews[tree.getKey()] = vbo;
    }

    //"image_0": {
    //    "path": "textures/abaddon_body_color.png"
    //},
    void handleImage(const JsonTree& tree)
    {
        fs::path imgPath = getAbsolutePath(tree["path"].getValue());

        mHero.mImages[tree.getKey()] = loadImageSafe(imgPath);
    }

    //"sampler_0": {
    //    "magFilter": "LINEAR",
    //    "minFilter": "LINEAR_MIPMAP_LINEAR",
    //    "wrapS": "REPEAT",
    //    "wrapT": "REPEAT"
    //},
    void handleSampler(const JsonTree& tree)
    {
        gl::Texture::Format format;
        format.setMagFilter(getGlEnum(tree, "magFilter"));
        format.setMinFilter(getGlEnum(tree, "minFilter"));
        format.setWrapS(getGlEnum(tree, "wrapS"));
        format.setWrapT(getGlEnum(tree, "wrapT"));
        format.enableMipmapping();

        mHero.mSamplers[tree.getKey()] = format;
    }

    //"Map #19": {
    //    "format": "RGBA",
    //    "internalFormat": "RGBA",
    //    "sampler": "sampler_0",
    //    "source": "image_0",
    //    "target": "TEXTURE_2D"
    //},
    void handleTexture(const JsonTree& tree)
    {
        const Surface& image = mHero.mImages[tree["source"].getValue()];

        gl::Texture::Format& format = mHero.mSamplers[tree["sampler"].getValue()];
        format.setTarget(getGlEnum(tree, "target"));
        format.setInternalFormat(getGlEnum(tree, "internalFormat"));

        mTextureNames.push_back(tree.getKey());
        mHero.mTextures[tree.getKey()] = gl::Texture(image, format);
    }

    //"technique_0": {
    //    "parameters": {},
    //    "pass": "defaultPass",
    //    "passes": {
    //        "defaultPass": {
    //            "instanceProgram": {
    //                "attributes": {},
    //                "program": "",
    //                "uniforms": {}
    //            },
    //            "states": {
    //                "blendEnable": false,
    //                "cullFaceEnable": false,
    //                "depthMask": true,
    //                "depthTestEnable": true
    //            }
    //        }
    //    }
    //},
    void handleTechnique(const JsonTree& tree)
    {
        Hero::Technique tech;
        string passName = tree["pass"].getValue();
        const JsonTree& defaultPassTree = tree["passes"][passName];
        tech.blendEnable = defaultPassTree["states"]["blendEnable"].getValue<bool>();
        tech.blendEnable = defaultPassTree["states"]["cullFaceEnable"].getValue<bool>();
        tech.blendEnable = defaultPassTree["states"]["depthMask"].getValue<bool>();
        tech.blendEnable = defaultPassTree["states"]["depthTestEnable"].getValue<bool>();

        mHero.mTechniques[tree.getKey()] = tech;
    }

    //"Material #151": {
    //    "instanceTechnique": {
    //        "technique": "technique_1",
    //        "values": [
    //        {
    //            "parameter": "ambient",
    //            "value": [
    //                0.5879999995231628,
    //                0.5879999995231628,
    //                0.5879999995231628
    //            ]
    //        },
    //        {
    //            "parameter": "shiness",
    //            "value": 2,
    //        },
    //    },
    //    "name": "Material #151"
    //},
    void handleMaterial(const JsonTree& tree)
    {
        string techniqueName = tree["instanceTechnique"]["technique"].getValue();
        const Hero::Technique& technique = mHero.mTechniques[techniqueName];
        Hero::Material material;
        material.pTechnique = &technique;

        // TODO: parse values

        mHero.mMaterials[tree.getKey()] = material;
    }

    //"indices_12": {
    //    "bufferView": "bufferView_38",
    //    "byteOffset": 13488,
    //    "count": 840,
    //    "type": "UNSIGNED_SHORT"
    //},
    void handleIndex(const JsonTree& tree)
    {
        Hero::Index index;
        index.indexBuffer = mHero.mBufferViews[tree["bufferView"].getValue()];
        index.count = tree["count"].getValue<GLsizei>();
        index.indices = reinterpret_cast<const GLvoid*>(tree["byteOffset"].getValue<int>());
        index.type = getGlEnum(tree, "type");

        mHero.mIndices[tree.getKey()] = index;
    }

    //"attribute_21": {
    //    "bufferView": "bufferView_37",
    //    "byteOffset": 755064,
    //    "byteStride": 8,
    //    "count": 8526,
    //    "max": [
    //        1,
    //        1,
    //        1
    //    ],
    //    "min": [
    //        0,
    //        0,
    //        0
    //    ],
    //    "type": "FLOAT_VEC2"
    //},
    void handleMeshAttribute(const JsonTree& tree)
    {
        Hero::MeshAttribute meshAttrb;
        meshAttrb.vextexBuffer = mHero.mBufferViews[tree["bufferView"].getValue()];
        //meshAttrb.index = ??
        meshAttrb.pointer = reinterpret_cast<const GLvoid*>(tree["byteOffset"].getValue<int>());
        meshAttrb.stride = tree["byteStride"].getValue<GLsizei>();

        // FLOAT|FLOAT_VEC2|FLOAT_VEC3|FLOAT_VEC4
        //meshAttrb.size = tree["byteStride"].getValue<GLsizei>();
        meshAttrb.normalized = tree.hasChild("normalized") ? tree["normalized"].getValue<bool>() : false;

        GLint size;
        GLenum type;
        GLboolean normalized;
        GLsizei stride;

        mHero.mMeshAttributes[tree.getKey()] = meshAttrb;
    }

private:

    fs::path getAbsolutePath(const string& relativePath)
    {
        return mHeroesPath / mHeroNames[mCurrentHero] / relativePath;
    }

    GLenum getGlEnum(const JsonTree& tree, const string& childKeyName)
    {
        const string& childKeyValue = tree[childKeyName].getValue();
        return getGlEnum(childKeyValue);
    }

    GLenum getGlEnum(const string& name)
    {
        NameEnumMap::const_iterator it = mGlNameMap.find(name);
        if (it != mGlNameMap.end())
        {
            return it->second;
        }
        
        console() << "Unsupported enum: " << name << endl;
        // report each unregistered enum for once
        mGlNameMap.insert(make_pair(name, GL_NONE));
        return GL_NONE;
    }

    Surface loadImageSafe(const fs::path& path)
    {
        Surface surf = loadImage(path);
        if (surf)
            return surf;

        TextLayout layout;

        layout.setFont(Font("Arial", 24));
        layout.setColor(Color(1, 1, 1));
        layout.addLine(path.string());
        return layout.render(true);
    }

private:
    fs::path                mHeroesPath;
    params::InterfaceGl     mParams;
    vector<NameHandlerPair> mCategories;
    NameEnumMap             mGlNameMap;

    Hero                    mHero;

    // params
    vector<string>          mHeroNames;
    int 		            mCurrentHero;
    vector<string>          mTextureNames;
    int                     mCurrentTexture;
};

CINDER_APP_BASIC(CiApp, RendererGl)

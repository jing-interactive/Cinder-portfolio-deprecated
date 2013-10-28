#include "cinder/app/AppBasic.h"
#include "cinder/ImageIo.h"
#include "cinder/Camera.h"
#include "cinder/Json.h"
#include "cinder/Text.h"
#include "cinder/Utilities.h"

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

        void draw()
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

        void draw()
        {
            indexBuffer.bind();
            glDrawElements(mode, count, type, indices);
            indexBuffer.unbind();
        }
    };

    struct MeshAttribute
    {
        gl::Vbo vextexBuffer;

        //GLuint index;
        GLint size;
        GLenum type;
        GLboolean normalized;
        GLsizei stride;
        const GLvoid* pointer;

        size_t count;   // The number of attributes referenced by this accessor

        //void draw()
        //{
        //    vextexBuffer.bind();
        //    glVertexAttribPointer(index, size, type, normalized, stride, pointer);
        //    vextexBuffer.unbind();
        //}
    };

    struct Node;

    struct Skin
    {
        float bindShapeMatrix[16];
        float inverseBindMatrices[16]; // TODO
        vector<Node*> pJoints;
        vector<Node*> pRoots;
    };

    struct Mesh
    {
        // TODO: array of struct?
        Index*      pIndices;
        Material*   pMaterial;
        map<string,  MeshAttribute*> semantics;
        Skin*       pSkin;
    };

    struct Node
    {
        vector<Node*> pChildren;
        float   matrix[16];         // It is directly usable by uniformMatrix4fv with transpose equal to false
        string  name;
        vector<Mesh*> pMeshes;

        void draw()
        {

        }
    };

    struct Scene
    {
        vector<Node*> pNodes;
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
    map<string, Mesh>                       mMeshes;
    map<string, Skin>                       mSkins;
    map<string, Node>                       mNodes;
    map<string, Scene>                      mScenes;
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

        mCurrentNode = -1;

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
        BIND_PAIR("meshes",         handleMesh);
        BIND_PAIR("cameras",        handleDefault);
        BIND_PAIR("lights",         handleDefault);
        BIND_PAIR("skins",          handleSkin);
        BIND_PAIR("nodes",          handleNode);
        BIND_PAIR("scenes",         handleScene);
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
                CURRENT_NODE = 0;
                mParams.removeParam("CURRENT_NODE");
                mParams.addParam("CURRENT_NODE", mNodeNames, &CURRENT_NODE);
            }
        }

        if (mCurrentNode != CURRENT_NODE)
        {
            mCurrentNode = CURRENT_NODE;
        }
    }

    void draw()
    {
        gl::clear(ColorA::black());
        gl::setMatricesWindow(getWindowSize());

        //gl::draw(mHero.mTextures[mNodeNames[mCurrentNode]], Vec2f(100, 100));

        mHero.mNodes[mNodeNames[mCurrentNode]].draw();

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
        mNodeNames.clear();

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
    // https://github.com/KhronosGroup/glTF/blob/master/specification/meshAttribute.schema.json
    void handleMeshAttribute(const JsonTree& tree)
    {
        Hero::MeshAttribute meshAttrib;
        meshAttrib.vextexBuffer = mHero.mBufferViews[tree["bufferView"].getValue()];

        // FLOAT|FLOAT_VEC2|FLOAT_VEC3|FLOAT_VEC4
        meshAttrib.type = getDataType(tree["type"].getValue());
        meshAttrib.size = getDataSize(tree["type"].getValue());
        meshAttrib.normalized = tree.hasChild("normalized") ? tree["normalized"].getValue<bool>() : false;
        meshAttrib.stride = tree["byteStride"].getValue<GLsizei>();
        meshAttrib.pointer = reinterpret_cast<const GLvoid*>(tree["byteOffset"].getValue<int>());

        meshAttrib.count = tree["count"].getValue<size_t>();

        mHero.mMeshAttributes[tree.getKey()] = meshAttrib;
    }

    //"abaddon_model_LOD0-mesh": {
    //    "name": "abaddon_model_LOD0-mesh",
    //    "primitives": [
    //        {
    //            "indices": "indices_6",
    //            "material": "Material #107",
    //            "semantics": {
    //                "JOINT": "attribute_4",
    //                "NORMAL": "attribute_2",
    //                "POSITION": "attribute_1",
    //                "TEXCOORD_0": "attribute_3",
    //                "WEIGHT": "attribute_5"
    //            },
    //            "skin": "skin_0"
    //        }
    //    ]
    //},
    // https://github.com/KhronosGroup/glTF/blob/master/specification/mesh.schema.json
    void handleMesh(const JsonTree& tree)
    {
        Hero::Mesh mesh;
        
        // TODO: supports multiple primitive?
        const JsonTree& firstPrimitive = tree["primitives"].getChild(0);
        mesh.pIndices = &mHero.mIndices[firstPrimitive["indices"].getValue()];
        mesh.pMaterial = &mHero.mMaterials[firstPrimitive["material"].getValue()];

        BOOST_FOREACH(const JsonTree& semantic, firstPrimitive["semantics"].getChildren())
        {
            mesh.semantics[semantic.getKey()] = &mHero.mMeshAttributes[semantic.getValue()];
        }

        mesh.pSkin = &mHero.mSkins[firstPrimitive["skin"].getValue()];

        mHero.mMeshes[tree.getKey()] = mesh;
    }

    //"skin_4": {
    //    "bindShapeMatrix": [],
    //    "inverseBindMatrices": {},
    //    "joints": [
    //        "clavicle_R",
    //        "clavicle_L"
    //    ],
    //    "roots": ["clavicle_R"]
    //},
    void handleSkin(const JsonTree& tree)
    {
        Hero::Skin skin;

        // TODO: supports more fields
        BOOST_FOREACH(const JsonTree& joint, tree["joints"].getChildren())
        {
            skin.pJoints.push_back(&mHero.mNodes[joint.getValue()]);
        }

        BOOST_FOREACH(const JsonTree& root, tree["roots"].getChildren())
        {
            skin.pRoots.push_back(&mHero.mNodes[root.getValue()]);
        }

        mHero.mSkins[tree.getKey()] = skin;
    }

    //"Cloth_R3C0": {
    //    "children": [
    //        "Cloth_R4C0"
    //    ],
    //    "matrix": [
    //        0.9979693889617922,
    //        -0.06097045540809615,
    //        -0.018415801227092805,
    //        0,
    //        0.05910318021817573,
    //        0.9942791120896233,
    //        -0.088971725890471,
    //        0,
    //        0.023735097457632096,
    //        0.08770264246253799,
    //        0.995863846984111,
    //        0,
    //        29.407180786132805,
    //        0,
    //        -0.000007629394538355427,
    //        1
    //    ],
    //    "name": "Cloth_R3C0"
    //},
    // https://github.com/KhronosGroup/glTF/blob/master/specification/node.schema.json
    void handleNode(const JsonTree& tree)
    {
        Hero::Node node;

        BOOST_FOREACH(const JsonTree& child, tree["children"].getChildren())
        {
            node.pChildren.push_back(&mHero.mNodes[child.getValue()]);
        }

        size_t i = 0;
        BOOST_FOREACH(const JsonTree& number, tree["matrix"].getChildren())
        {
            node.matrix[i] = number.getValue<float>();
            i++;
        }

        if (tree.hasChild("meshes"))
        {
            BOOST_FOREACH(const JsonTree& mesh, tree["meshes"].getChildren())
            {
                node.pMeshes.push_back(&mHero.mMeshes[mesh.getValue()]);
            }
        }

        node.name = tree.getKey();

        mHero.mNodes[tree.getKey()] = node;
    }

    //"scene_0": {
    //    "nodes": [
    //        "abaddon_model_LOD0",
    //        "cape_model_LOD0",
    //        "helmet_model_LOD0",
    //        "mount_model_LOD0",
    //        "shoulders_model_LOD0",
    //        "weapon_model_LOD0"
    //    ]
    //}
    void handleScene(const JsonTree& tree)
    {
        Hero::Scene scene;

        BOOST_FOREACH(const JsonTree& node, tree["nodes"].getChildren())
        {
            scene.pNodes.push_back(&mHero.mNodes[node.getValue()]);
            mNodeNames.push_back(node.getValue());
        }

        mHero.mScenes[tree.getKey()] = scene;
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

    // FLOAT_VEC2 -> GL_FLOAT
    static GLenum getDataType(const string& name)
    {
        // GL_BYTE, GL_UNSIGNED_BYTE, GL_SHORT, GL_UNSIGNED_SHORT, GL_FIXED, or GL_FLOAT
        // TODO:
        return GL_FLOAT;
    }

    // FLOAT_VEC2 -> 2
    static GLint getDataSize(const string& name)
    {
        if (name.find("4") != string::npos) return 4;
        if (name.find("3") != string::npos) return 3;
        if (name.find("2") != string::npos) return 2;
        return 1;
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
    vector<string>          mNodeNames;
    int                     mCurrentNode;
};

CINDER_APP_BASIC(CiApp, RendererGl)

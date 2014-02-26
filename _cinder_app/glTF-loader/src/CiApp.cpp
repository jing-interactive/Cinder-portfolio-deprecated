#include "cinder/app/AppBasic.h"
#include "cinder/ImageIo.h"
#include "cinder/Camera.h"
#include "cinder/Json.h"
#include "cinder/Text.h"
#include "cinder/Utilities.h"

#include "cinder/ip/Flip.h"

#include "cinder/gl/gl.h"
#include "cinder/gl/Texture.h"
#include "cinder/gl/Fbo.h"
#include "cinder/gl/GlslProg.h"
#include "cinder/gl/Vbo.h"
#include "cinder/params/Params.h"

#include "../../../_common/MiniConfig.h"
#include "../../../_common/GlslHotProg.h"

#include <boost/foreach.hpp>
#include <boost/tuple/tuple.hpp>

using namespace ci;
using namespace ci::app;
using namespace std;

const string kAllNodeName = "ALL";
const fs::directory_iterator kEndIt;

typedef function<void(const JsonTree&)> JsonHandler;
typedef pair<string, JsonHandler> NameHandlerPair;
typedef pair<string, gl::Texture> NameTexturePair;
typedef pair<string, vector<float>> NameValuePair;
typedef map<string, GLenum> NameEnumMap;

namespace
{
    GlslHotProg    gShader;
}

struct Hero
{
    struct Technique
    {
        string name;
        bool blendEnable;
        bool cullFaceEnable;
        bool depthMask;
        bool depthTestEnable;

        void preDraw() const
        {
            if (blendEnable)
            {
                gl::enableAlphaBlending();
            }
            else
            {
                gl::disableAlphaBlending();
            }

            if (cullFaceEnable)
            {
                glEnable(GL_CULL_FACE);
            }
            else
            {
                glDisable(GL_CULL_FACE);
            }

            gl::enableDepthWrite(depthMask);
            gl::enableDepthRead(depthTestEnable);
        }
    };

    struct Material
    {
        string name;
        const Technique* pTechnique;

        // vector of Texture ref
        vector<NameTexturePair> textures;
        vector<NameValuePair> uniforms;

        void preDraw()
        {
            GLuint texSlot = 0;
            BOOST_FOREACH(NameTexturePair& pair, textures)
            {
                pair.second.bind(texSlot);
                GLint loc = gShader.getProg().getUniformLocation(pair.first);
                glUniform1i(loc, texSlot);
                texSlot++;
            }

            BOOST_FOREACH(NameValuePair& pair, uniforms)
            {
                const vector<float>& values = pair.second;
                GLint loc = gShader.getProg().getUniformLocation(pair.first);
                if (loc == -1)
                    continue;

                // TODO: support int / ushort values
                switch (values.size())
                {
                case 4:
                    {
                        glUniform4f(loc, values[0], values[1], values[2], values[3]);
                    }break;
                case 3:
                    {
                        glUniform3f(loc, values[0], values[1], values[2]);
                    }break;
                case 2:
                    {
                        glUniform2f(loc, values[0], values[1]);
                    }break;
                case 1:
                    {
                        glUniform1f(loc, values[0]);
                    }break;
                default:
                    {
                        assert(0);
                    }
                }
            }

            pTechnique->preDraw();
        }

        void postDraw()
        {
            GLuint texSlot = 0;
            BOOST_FOREACH(NameTexturePair& pair, textures)
            {
                pair.second.unbind(texSlot);
                texSlot++;
            }
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

    struct Attribute
    {
        gl::Vbo vextexBuffer;

        //GLuint index;
        GLint size;
        GLenum type;
        GLboolean normalized;
        GLsizei stride;
        const GLvoid* pointer;

        size_t count;   // The number of attributes referenced by this accessor
        // not used in glVertexAttribPointer

        void preDraw(GLuint index)
        {
            if (index == -1)
                return;

            vextexBuffer.bind();
            glEnableVertexAttribArray(index);
            glVertexAttribPointer(index, size, type, normalized, stride, pointer);
        }

        void postDraw(GLuint index)
        {
            glDisableVertexAttribArray(index);
            vextexBuffer.unbind();
        }
    };

    struct Node;

    struct Skin
    {
        float bindShapeMatrix[16];
        float inverseBindMatrices[16]; // TODO
        vector<Node*> pJoints;
        vector<Node*> pRoots;

        void preDraw()
        {

        }
    };

    struct Mesh
    {
        string      name;
        struct Primitive
        {
            typedef pair<string, Attribute*> NameAttribPair;

            Index*      pIndexBuffer;
            Material*   pMaterial;
            vector<NameAttribPair> pVertexBuffers;
            Skin*       pSkin;

            void draw()
            {
                pMaterial->preDraw();
                BOOST_FOREACH(NameAttribPair& pair, pVertexBuffers)
                {
                    GLint loc = gShader.getProg().getAttribLocation(pair.first);
                    pair.second->preDraw(loc);
                }

                pSkin->preDraw();
                pIndexBuffer->draw();

                BOOST_FOREACH(NameAttribPair& pair, pVertexBuffers)
                {
                    GLint loc = gShader.getProg().getAttribLocation(pair.first);
                    pair.second->postDraw(loc);
                }
                pMaterial->postDraw();
            }
        };
        vector<Primitive> primitives;

        void draw()
        {
            BOOST_FOREACH(Primitive& prim, primitives)
            {
                prim.draw();
            }
        }
    };

    struct Node
    {
        vector<Node*> pChildren;
        Matrix44f matrix;
        string  name;
        vector<Mesh*> pMeshes;

        void draw()
        {
            BOOST_FOREACH(Mesh* pMesh, pMeshes)
            {
                pMesh->draw();
            }

            if (pChildren.empty())
                return;

            gl::pushModelView();
            glMultMatrixf(matrix.m);
            BOOST_FOREACH(Node* pChild, pChildren)
            {
                pChild->draw();
            }
            gl::popModelView();
        }
    };

    struct Scene
    {
        vector<Node*> pNodes;

        void draw()
        {
            BOOST_FOREACH(Node* pNode, pNodes)
            {
                pNode->draw();
            }
        }
    };

    struct JointInfo
    {
        string name;
        int parentId;
        Matrix44f invBindPose;
    };

    struct SkeltonInfo
    {
        vector<JointInfo> joints;
    };

    struct AnimTrack
    {
        struct JointPose
        {
            Vec3f pos;
            Vec3f rot;
            // Vec3f scale;
        };

        void interpolate(float timePos, const SkeltonInfo& skeleton, vector<Matrix44f>& localTransforms)
        {
            size_t index = timePos; // TODO
            index %= poseSamples.size();
            const vector<JointPose>& poses = poseSamples[index].jointPoses;

            for (size_t i=0; i<skeleton.joints.size(); i++)
            {
                localTransforms[i] = Matrix44f::createTranslation(poses[i].pos) * Matrix44f::createRotation(poses[i].rot);
            }
        }

        struct SkeletonPose
        {
            float time; // TODO: unused
            vector<JointPose> jointPoses;  
        };
        vector<SkeletonPose> poseSamples;

        void update(float timePos, const SkeltonInfo& skeleton, vector<Matrix44f>& boneMatrices)
        {
            boneMatrices.resize(skeleton.joints.size());

            vector<Matrix44f> toParentTransforms(skeleton.joints.size());

            // Interpolate all the bones of this clip at the given time instance.
            interpolate(timePos, skeleton, toParentTransforms);

            //
            // Traverse the hierarchy and transform all the bones to the root space.
            //
            vector<Matrix44f> toRootTransforms(skeleton.joints.size());

            // The root bone has index 0.  The root bone has no parent, so its toRootTransform
            // is just its local bone transform.
            toRootTransforms[0] = toParentTransforms[0];

            // Now find the toRootTransform of the children.
            for (size_t i = 1; i < skeleton.joints.size(); ++i)
            {
                int parentId = skeleton.joints[i].parentId;
                toRootTransforms[i] = toRootTransforms[parentId] * toParentTransforms[i];
            }

            // Post-multiply by the bone offset transform to get the final transform.
            for (size_t i = 0; i < skeleton.joints.size(); ++i)
            {
                boneMatrices[i] = toRootTransforms[i] * skeleton.joints[i].invBindPose;
            }
        }

        string name;
    };

    void preDraw()
    {
    }

    void draw()
    {
        typedef map<string, Scene> MapT;
        BOOST_FOREACH(MapT::value_type& pair, mScenes)
        {
            pair.second.draw();
        }
    }

    void postDraw()
    {
    }

    map<string, DataSourceRef>              mBuffers;
    map<string, gl::Vbo>                    mBufferViews;
    map<string, gl::Texture::Format>        mSamplers; // TODO: sampler object
    map<string, Surface>                    mImages;
    map<string, gl::Texture>                mTextures;
    map<string, Technique>                  mTechniques;
    map<string, Material>                   mMaterials;
    map<string, Index>                      mIndices;
    map<string, Attribute>                  mAttributes;
    map<string, Mesh>                       mMeshes;
    map<string, Skin>                       mSkins;
    map<string, Node>                       mNodes;
    map<string, Scene>                      mScenes;

    map<string, AnimTrack>                  mAnimTracks;
    SkeltonInfo                                 mSkeleton;
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
        gShader.load("dota2-hero.vs", "dota2-hero.fs");

        mParams = params::InterfaceGl("params", Vec2i(300, getConfigUIHeight()));
        setupConfigUI(&mParams);

        // parse dota2hero-gh-pages/heroes
        mHeroesFolder = fs::path(HEROES_PATH);
        if (!fs::exists(mHeroesFolder))
        {
            console() << mHeroesFolder << " doesn't exist" << endl;
            quit();
            return;
        }
        for (fs::directory_iterator it(mHeroesFolder); it != kEndIt; ++it)
        {
            if (fs::is_directory(*it))
            {
                mHeroNames.push_back((*it).path().filename().string());
            }
        }

        mParams.removeParam("CURRENT_HERO");
        mParams.addParam("CURRENT_HERO", mHeroNames, &CURRENT_HERO);

        mCurrentHero = -1;

        // functios
#define BIND_PAIR(name, handler) do \
        {\
        JsonHandler childF = bind(&CiApp::handler, this, std::_1);\
        JsonHandler parentF = bind(&CiApp::handleChildren, this, std::_1, childF);\
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
        BIND_PAIR("attributes",     handleAttribute);
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
            if (!loadHero(mHeroNames[mCurrentHero]))
            {
                quit();
                return;
            }
            mCurrentNode = -1;
            CURRENT_NODE = 0;
            mParams.removeParam("CURRENT_NODE");
            mParams.addParam("CURRENT_NODE", mNodeNames, &CURRENT_NODE);

            mCurrentAnim = -1;
            CURRENT_ANIM = 0;
            mParams.removeParam("CURRENT_ANIM");
            mParams.addParam("CURRENT_ANIM", mAnimNames, &CURRENT_ANIM);
        }

        if (mCurrentNode != CURRENT_NODE)
        {
            mCurrentNode = CURRENT_NODE;
        }

        if (mCurrentAnim != CURRENT_ANIM)
        {
            mCurrentAnim = CURRENT_ANIM;
            loadAnimTrack(mAnimNames[mCurrentAnim]);
        }
    }

    void draw()
    {
        gl::clear(ColorA::black());
        CameraPersp cam( getWindowWidth(), getWindowHeight(), 60);
        cam.lookAt(Vec3f(0, CAM_Y, CAM_Z), Vec3f(0, CAM_Y, 0));
        gl::setModelView(cam);

        //gl::rotate(ROTATION);
        
        gl::drawCoordinateFrame();

        drawHero();

        mParams.draw();
    }

private:

    void drawHero()
    {
        gl::color(Color::white());
        if (HERO_WIREFRAME)
        {
            gl::enableWireframe();
        }
        else
        {
            gl::disableWireframe();
        }

        mHero.preDraw();
        gShader.getProg().bind();

        vector<Matrix44f> boneMatrices; // MAXBONES = 128?
        mHero.mAnimTracks[mAnimNames[mCurrentAnim]].update(getElapsedSeconds() * FRAME_PER_SEC, mHero.mSkeleton, boneMatrices);
        gShader.getProg().uniform("uBoneMatrices", &boneMatrices[0], boneMatrices.size());

        if (mCurrentNode == 0)
        {
            mHero.draw();
        }
        else
        {
            mHero.mNodes[mNodeNames[mCurrentNode]].draw();
        }
        mHero.postDraw();
        gShader.getProg().unbind();
    }

    bool loadHero(const string& heroName)
    {
        fs::path heroRoot = mHeroesFolder / heroName;

        fs::path meshPath = heroRoot / (heroName+".json");
        fs::path mtrlPath = heroRoot / "materials.json";
        fs::path smdFolder = heroRoot / "smd";

        if (!fs::exists(meshPath) ||
            !fs::exists(mtrlPath) ||
            !fs::exists(smdFolder)
            )
        {
            console() << "Folder incomplete." << endl;
            return false;
        }

        // hero.json
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

        // animations.json
        mAnimNames.clear();

        for (fs::directory_iterator it(smdFolder); it != kEndIt; ++it)
        {
           mAnimNames.push_back((*it).path().filename().string());
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

        gl::Texture tex = gl::Texture(image, format);
        mHero.mTextures[tree.getKey()] = tex;
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
        tech.name = tree.getKey();

        string passName = tree["pass"].getValue();
        const JsonTree& defaultPassTree = tree["passes"][passName];
        tech.blendEnable = defaultPassTree["states"]["blendEnable"].getValue<bool>();
        tech.blendEnable = defaultPassTree["states"]["cullFaceEnable"].getValue<bool>();
        tech.blendEnable = defaultPassTree["states"]["depthMask"].getValue<bool>();
        tech.blendEnable = defaultPassTree["states"]["depthTestEnable"].getValue<bool>();

        mHero.mTechniques[tech.name] = tech;
    }

    //"Material #151": {
    //    "instanceTechnique": {
    //        "technique": "technique_1",
    //        "values": [
    //           {
    //               "parameter": "ambient",
    //               "value": [
    //                   0.5879999995231628,
    //                   0.5879999995231628,
    //                   0.5879999995231628
    //               ]
    //           },
    //           {
    //               "parameter": "diffuse",
    //               "value": "Map #27",
    //           },
    //        ]  
    //    },
    //    "name": "Material #151"
    //},
    void handleMaterial(const JsonTree& tree)
    {
        Hero::Material material;
        material.name = tree.getKey();

        string techniqueName = tree["instanceTechnique"]["technique"].getValue();
        const Hero::Technique& technique = mHero.mTechniques[techniqueName];
        material.pTechnique = &technique;

        BOOST_FOREACH(const JsonTree& value, tree["instanceTechnique"]["values"].getChildren())
        {
            string paramName = value["parameter"].getValue();

            string texName = value["value"].getValue();
            gl::Texture tex = mHero.mTextures[texName];
            if (tex)
            {
                material.textures.push_back(make_pair(paramName, tex));
                continue;
            }

            // TODO: support more types
            vector<float> floatArray;
            if (value["value"].getChildren().empty())
            {
                float floatValue = fromString<float>(value["value"].getValue());
                floatArray.push_back(floatValue);
            }
            else
            {
                BOOST_FOREACH(const JsonTree& floatItem, value["value"].getChildren())
                {
                    float floatValue = fromString<float>(floatItem.getValue());
                    floatArray.push_back(floatValue);
                }
            }
            material.uniforms.push_back(make_pair(paramName, floatArray));
        }

        mHero.mMaterials[material.name] = material;
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
    void handleAttribute(const JsonTree& tree)
    {
        Hero::Attribute meshAttrib;
        meshAttrib.vextexBuffer = mHero.mBufferViews[tree["bufferView"].getValue()];

        // FLOAT|FLOAT_VEC2|FLOAT_VEC3|FLOAT_VEC4
        meshAttrib.type = getDataType(tree["type"].getValue());
        meshAttrib.size = getDataSize(tree["type"].getValue());
        meshAttrib.normalized = tree.hasChild("normalized") ? tree["normalized"].getValue<bool>() : false;
        meshAttrib.stride = tree["byteStride"].getValue<GLsizei>();
        meshAttrib.pointer = reinterpret_cast<const GLvoid*>(tree["byteOffset"].getValue<int>());

        meshAttrib.count = tree["count"].getValue<size_t>();

        mHero.mAttributes[tree.getKey()] = meshAttrib;
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
        mesh.name = tree.getKey();

        BOOST_FOREACH(const JsonTree& primitive, tree["primitives"].getChildren())
        {
            Hero::Mesh::Primitive prim;
            prim.pIndexBuffer = &mHero.mIndices[primitive["indices"].getValue()];
            prim.pMaterial = &mHero.mMaterials[primitive["material"].getValue()];

            BOOST_FOREACH(const JsonTree& semantic, primitive["semantics"].getChildren())
            {
                prim.pVertexBuffers.push_back(make_pair(semantic.getKey(), &mHero.mAttributes[semantic.getValue()]));
            }

            prim.pSkin = &mHero.mSkins[primitive["skin"].getValue()];

            mesh.primitives.push_back(prim);
        }

        mHero.mMeshes[mesh.name] = mesh;
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
        node.name = tree.getKey();

        BOOST_FOREACH(const JsonTree& child, tree["children"].getChildren())
        {
            node.pChildren.push_back(&mHero.mNodes[child.getValue()]);
        }

        size_t i = 0;
        BOOST_FOREACH(const JsonTree& number, tree["matrix"].getChildren())
        {
            node.matrix.m[i] = number.getValue<float>();
            i++;
        }

        if (tree.hasChild("meshes"))
        {
            BOOST_FOREACH(const JsonTree& mesh, tree["meshes"].getChildren())
            {
                node.pMeshes.push_back(&mHero.mMeshes[mesh.getValue()]);
            }
        }

        mHero.mNodes[node.name] = node;
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

        mNodeNames.push_back(kAllNodeName);
        BOOST_FOREACH(const JsonTree& node, tree["nodes"].getChildren())
        {
            scene.pNodes.push_back(&mHero.mNodes[node.getValue()]);
            mNodeNames.push_back(node.getValue());
        }

        mHero.mScenes[tree.getKey()] = scene;
    }

    void loadAnimTrack(const string& name)
    {
        if (mHero.mAnimTracks.find(name) != mHero.mAnimTracks.end())
        {
            return;
        }

        mHero.mSkeleton.joints.clear();

        Hero::AnimTrack anim;
        anim.name = name;

        stringstream ss;
        ss << mHeroesFolder.generic_string() << "/" << mHeroNames[mCurrentHero] << "/smd/" << name;
        ifstream ifs(ss.str().c_str());

        string dummy;
        string line;

        getline(ifs, line); // version 1
        getline(ifs, line); // nodes
        getline(ifs, line); // 0 "root" -1
        while (line != "end")
        {
            // TODO: skeleton is unique for every hero, so only needs to be initialized once
            Hero::JointInfo joint;
            stringstream(line) >> dummy >> joint.name >> joint.parentId;
            mHero.mSkeleton.joints.push_back(joint);
            getline(ifs, line);
        }

        for (size_t i=0; i<mHero.mSkeleton.joints.size(); i++)
        {
            Hero::JointInfo& joint = mHero.mSkeleton.joints[i];
            const Hero::Node& node = mHero.mNodes[joint.name];
            Matrix44f inv = node.matrix.inverted();

            if (i == 0)
            {
                joint.invBindPose = inv;
            }
            else
            {
                Hero::JointInfo& parent = mHero.mSkeleton.joints[joint.parentId];
                joint.invBindPose = inv * parent.invBindPose;
            }
        }

        getline(ifs, line); // skeleton
        getline(ifs, line); // time 0
        Hero::AnimTrack::SkeletonPose skelPose;
        while (line != "end")
        {
            if (line.find("time") != string::npos)
            {
                if (!skelPose.jointPoses.empty())
                {
                    anim.poseSamples.push_back(skelPose);
                }
                skelPose = Hero::AnimTrack::SkeletonPose();
                stringstream(line) >> dummy >> skelPose.time;
            }
            else
            {
                Hero::AnimTrack::JointPose Pose;
                stringstream(line) >> dummy >> Pose.pos.x >> Pose.pos.y >> Pose.pos.z
                                    >> Pose.rot.x >> Pose.rot.y >> Pose.rot.z;
                skelPose.jointPoses.push_back(Pose);
            }

            getline(ifs, line);
        }

        anim.poseSamples.push_back(skelPose);

        mHero.mAnimTracks[name] = anim;
    }

private:

    fs::path getAbsolutePath(const string& relativePath) const
    {
        return mHeroesFolder / mHeroNames[mCurrentHero] / relativePath;
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

    static Surface loadImageSafe(const fs::path& path)
    {
        Surface surf = loadImage(path);
        if (surf)
        {
            ip::flipVertical(&surf); // flip vertically
            return surf;
        }

        TextLayout layout;

        layout.setFont(Font("Arial", 24));
        layout.setColor(Color(1, 1, 1));
        layout.addLine(path.string());
        return layout.render(true);
    }

private:
    fs::path                mHeroesFolder;
    params::InterfaceGl     mParams;
    vector<NameHandlerPair> mCategories;
    NameEnumMap             mGlNameMap;

    Hero                    mHero;

    // params
    vector<string>          mHeroNames;
    int 		            mCurrentHero;
    vector<string>          mNodeNames;
    int                     mCurrentNode;
    vector<string>          mAnimNames;
    int                     mCurrentAnim;
};

CINDER_APP_BASIC(CiApp, RendererGl)

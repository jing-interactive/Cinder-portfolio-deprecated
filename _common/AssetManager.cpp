#include "AssetManager.h"
#include "cinder/app/App.h"
#include "cinder/ImageIo.h"
#include "cinder/Function.h"
#include "cinder/Utilities.h"
#include <map>

using namespace std;
using namespace ci;
using namespace ci::app;

namespace
{
    template <typename T> 
    T& getAssetResource(const string& relativeName, function<T(const string&, const string&)> loadFunc, const string& relativeNameB = "")
    {
        typedef map<string, typename T> MapType;
        static MapType sMap;
        MapType::iterator it = sMap.find(relativeName);
        if (it != sMap.end())
        {
            return it->second;
        }
        console() << "Loading " << relativeName << endl;

        try
        {
            fs::path aPath = getAssetPath("") / relativeName;
            fs::path bPath = getAssetPath("") / relativeNameB;
            return sMap[relativeName] = loadFunc(aPath.string(), bPath.string());
        }
        catch (Exception& e)
        {
            console() << "getAssetResource: " << e.what() << endl;
            throw;
        }
    }
}

namespace am
{
    static Surface loadSurface(const string& absoluteName, const string&)
    {
        return loadImage(absoluteName);
    }

    Surface& surface(const string& relativeName)
    {
        return getAssetResource<Surface>(relativeName, loadSurface);
    }

    static gl::Texture loadTexture(const string& absoluteName, const string&)
    {
        return loadImage(absoluteName);
    }

    gl::Texture& texture(const string& relativeName)
    {
        return getAssetResource<gl::Texture>(relativeName, loadTexture);
    }

    static TriMesh loadTriMesh(const string& absoluteName, const string&)
    {
        TriMesh mesh;
        mesh.read(DataSourcePath::create(absoluteName));
        return mesh;
    }

    TriMesh& triMesh(const string& relativeName)
    {
        return getAssetResource<TriMesh>(relativeName, loadTriMesh);
    }

    static gl::VboMesh loadVboMesh(const string& absoluteName, const string&)
    {
        gl::VboMesh::Layout layout;
        gl::VboMesh mesh(loadTriMesh(absoluteName, ""), layout);
        return mesh;
    }

    gl::VboMesh& vboMesh(const string& relativeName)
    {
        return getAssetResource<gl::VboMesh>(relativeName, loadVboMesh);
    }

    static gl::GlslProg loadGlslProg(const string& vsAbsoluteName, const string& fsAbsoluteName)
    {
        return gl::GlslProg(DataSourcePath::create(vsAbsoluteName), DataSourcePath::create(fsAbsoluteName));
    }

    gl::GlslProg& glslProg(const string& vsFileName, const string& fsFileName)
    {
        return getAssetResource<gl::GlslProg>(vsFileName, loadGlslProg, fsFileName);
    }

    static string loadStr(const string& absoluteName, const string&)
    {
        return loadString(DataSourcePath::create(absoluteName));
    }

    string& str(const string& relativeName)
    {
        return getAssetResource<string>(relativeName, loadStr);
    }

    static vector<string> loadFiles(const string& absoluteFolderName, const string&)
    {
        vector<string> files;
        fs::directory_iterator kEnd;
        for (fs::directory_iterator it(absoluteFolderName); it != kEnd; ++it)
        {
            if (fs::is_regular_file(*it) && it->path().extension() != ".db" 
                && it->path().extension() != ".DS_Store")
            {
#ifdef _DEBUG
                //console() << it->path() << endl;
#endif
                files.push_back(it->path().generic_string());
            }
        }
        return files;
    }

    vector<string> files(const string& relativeFolderName)
    {
        return getAssetResource<vector<string>>(relativeFolderName, loadFiles);
    }
}

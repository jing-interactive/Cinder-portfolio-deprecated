#include "AssetManagerAV.h"
#include "cinder/app/App.h"
#include <map>

using namespace std;
using namespace ci;
using namespace ci::app;

#pragma comment(lib, "CVClient.lib")
#pragma comment(lib, "QTMLClient.lib")

#pragma comment(lib, "irrKlang.lib")

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
    static qtime::MovieGl loadMovieGl(const string& absoluteName, const string&)
    {
        return qtime::MovieGl(absoluteName);
    }

    qtime::MovieGl& movieGl(const std::string& relativeName)
    {
        return getAssetResource<qtime::MovieGl>(relativeName, loadMovieGl);
    }

    static qtime::MovieSurface loadMovieSurface(const string& absoluteName, const string&)
    {
        return qtime::MovieSurface(absoluteName);
    }

    qtime::MovieSurface& movieSurface(const std::string& relativeName)
    {
        return getAssetResource<qtime::MovieSurface>(relativeName, loadMovieSurface);
    }

    using namespace irrklang;

    shared_ptr<ISoundEngine> sSoundEngine;

    static shared_ptr<ISound> loadAudio(const string& absoluteName, const string&)
    {
        if (!sSoundEngine)
        {
            sSoundEngine = shared_ptr<ISoundEngine>(createIrrKlangDevice(), mem_fun(&ISoundEngine::drop));
        }
        ISound* snd = sSoundEngine->play2D(absoluteName.c_str(), false, true);
        if (snd == NULL)
        {
            console() << "Fails to play audio " << absoluteName << endl;
        }
        return shared_ptr<ISound>(snd, mem_fun(&ISound::drop));
    }

    shared_ptr<ISound>& audio(const std::string& relativeName)
    {
        return getAssetResource<shared_ptr<ISound>>(relativeName, loadAudio);
    }
}

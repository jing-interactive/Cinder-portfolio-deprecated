#include "cinder/Vector.h"
#include "cinder/Surface.h"
#include "cinder/Filesystem.h"
#include <vector>

using namespace std;

#define CINDER_APP_CONSOLE( APP, RENDERER )														\
	int main() {	\
	cinder::app::AppBasic::prepareLaunch();														\
	cinder::app::AppBasic *app = new APP;														\
	cinder::app::Renderer *ren = new RENDERER;													\
	cinder::app::AppBasic::executeLaunch( app, ren, #APP );										\
	cinder::app::AppBasic::cleanupLaunch();														\
	return 0;																					\
}

namespace cinder{

	Vec3f screenToWorld( const Vec2i &mouse, float z = 0 );

	Vec3f unproject( const Vec3f &pt );

	void writeImages(std::vector<Surface8u>& images, fs::path& familyName);

	void postImageToWeibo(std::string& fileName, std::string& text);

	void execute(string exeFile, string param);
}
#include "Functions.h"
#include <cinder/Utilities.h>
#include "Model.h"
#include "Scene.h"
#include "Content.h"

using namespace ci;

namespace ARContent{

 
	void FunctionShow::execute()
	{

	}

	void FunctionHide::execute()
	{

	}

	void FunctionGotoScene::execute()
	{
		if (parent != NULL)
		{
			Scene* scn = parent->parent;
			assert(scn != NULL);
			Content* ctt = scn->parent;
			assert(scn != NULL);
			ctt->setCurrentSceneByName(args[0]);
		}
	}

	void FunctionGotoLink::execute()
	{
		launchWebBrowser(Url(args[0]));
	}

	void FunctionPlayInAppVideo::execute()
	{

	}

	void FunctionBindAnimation::execute()
	{

	}

	void FunctionPlaySound::execute()
	{

	}

	void FunctionSetVar::execute()
	{

	}

	void FunctionShowOverlay::execute()
	{

	}

	void FunctionLoadCommands::execute()
	{

	}

	void FunctionLockLandscape::execute()
	{

	}
}
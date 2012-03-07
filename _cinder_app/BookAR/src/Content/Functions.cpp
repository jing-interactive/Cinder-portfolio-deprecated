#include "Functions.h"
#include <cinder/Utilities.h>


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
#ifndef UIELEMENT_MANAGER_H
#define UIELEMENT_MANAGER_H

#include <vector>
#include <cinder/Cinder.h>

class UIElementManager
{
	std::vector<std::shared_ptr<class UIElement>> elements;
};

#endif //UIELEMENT_MANAGER_H
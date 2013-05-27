#include <windows.h>
#include <xnamath.h>
#include "nsg/SceneNode.h"
#include <sstream>

namespace 
{
    struct Operations
    {
        static void multiplyTranslation(XMFLOAT4X4& mat, const XMFLOAT3& trans)
        {
            mat.setTranslate(trans);
        }

        static void multiplyRotation(XMFLOAT4X4& mat, const XMFLOAT4& quat)
        {
            mat.rotate(quat.getAxis(), quat.getAngle());
        }

        static void multiplyScale(XMFLOAT4X4& mat, const XMFLOAT4X4& scale)
        {
            mat.scale(scale);
        }
    };
    typedef ::nsg::SceneNode<XMFLOAT3, XMFLOAT4, XMFLOAT4X4, Operations> SceneNode;
}

int main()
{
    SceneNode::Ref root = SceneNode::create();
    root->setName("root");

    for (int i=0;i<10;i++)
    {
        SceneNode::Ref node = SceneNode::create(i);
        std::stringstream ss;
        ss << "child#" << i;
        node->setName(ss.str());
        root->addChild(node);
    }

    return 0;
}
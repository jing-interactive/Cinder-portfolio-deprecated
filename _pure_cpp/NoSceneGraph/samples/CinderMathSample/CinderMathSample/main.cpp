#include "cinder/Matrix.h"
#include "cinder/Quaternion.h"
#include "nsg/SceneNode.h"
#include <sstream>

namespace cinder{
struct Operations
{
    static void multiplyTranslation(Matrix44f& mat, const Vec3f& trans)
    {
        mat.setTranslate(trans);
    }

    static void multiplyRotation(Matrix44f& mat, const Quatf& quat)
    {
        mat.rotate(quat.getAxis(), quat.getAngle());
    }

    static void multiplyScale(Matrix44f& mat, const Vec3f& scale)
    {
        mat.scale(scale);
    }
};

typedef ::nsg::SceneNode<Vec3f, Quatf, Matrix44f, Operations> SceneNode;

}

using namespace ci;

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
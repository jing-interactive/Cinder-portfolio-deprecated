// Copyright (C) 2002-2012 Nikolaus Gebhardt
// This file is part of the "Irrlicht Engine".
// For conditions of distribution and use, see copyright notice in irrlicht.h

// (C) 2013 Vinjn Zhang

#ifndef __I_SCENE_NODE_H_INCLUDED__
#define __I_SCENE_NODE_H_INCLUDED__

#include <string>
#include <list>
#include <memory>

//MSVC++ 11.0 _MSC_VER = 1700 (Visual Studio 2012)
//MSVC++ 10.0 _MSC_VER = 1600 (Visual Studio 2010)
//MSVC++ 9.0  _MSC_VER = 1500 (Visual Studio 2008)
//MSVC++ 8.0  _MSC_VER = 1400 (Visual Studio 2005)
//MSVC++ 7.1  _MSC_VER = 1310 (Visual Studio 2003)
#if !defined(CINDER_CINDER) && defined( _MSC_VER ) && ( _MSC_VER >= 1500 )
namespace std {
    using std::tr1::shared_ptr;
    using std::tr1::weak_ptr;
    using std::tr1::enable_shared_from_this;
}
#endif

namespace nsg
{
	//! Scene node interface.
	/** A scene node is a node in the hierarchical scene graph. Every scene
	node may have children, which are also scene nodes. Children move
	relative to their parent's position. If the parent of a node is not
	visible, its children won't be visible either. In this way, it is for
	example easily possible to attach a light to a moving car, or to place
	a walking character on a moving platform on a moving ship.
	*/
    template <typename Float3Type, typename QuaternionType, typename Float4x4Type, typename Operations>
    class SceneNode : public std::enable_shared_from_this<SceneNode<Float3Type, QuaternionType, Float4x4Type, Operations>>
	{
	public:
        
        typedef std::shared_ptr<SceneNode>  Ref;
        typedef std::weak_ptr<SceneNode>    WeakRef;
        typedef std::list<Ref>              List;

    public:
		//! Constructor
        SceneNode()
        {
            mID = 0;
        }

		//! Destructor
		virtual ~SceneNode()
		{
			// delete all children
			removeChildren();
		}

    public:
        //! factory method
        static Ref create(int id=-1,
            const Float3Type& position = Float3Type(0,0,0),
            const QuaternionType& rotation = QuaternionType(0,0,0,0),
            const Float3Type& scale = Float3Type(1.0f, 1.0f, 1.0f))
        {
            SceneNode* rawPtr = new SceneNode();
            rawPtr->setID(id);
            rawPtr->setPosition(position);
            rawPtr->setRotation(rotation);
            rawPtr->setScale(scale);
            rawPtr->updateAbsolutePosition();

            return Ref(rawPtr);
        }

		//! OnAnimate() is called just before rendering the whole scene.
		/** Nodes may calculate or store animations here, and may do other useful things,
		depending on what they are. Also, OnAnimate() should be called for all
		child scene nodes here. This method will be called once per frame, independent
		of whether the scene node is visible or not.
		\param timeMs Current time in milliseconds. */
		void OnAnimate(unsigned int timeMs)
		{
			if (mIsVisible)
			{
				// update absolute position
				updateAbsolutePosition();

				// perform the post render process on all children
				List::iterator it = mChildren.begin();
				for (; it != mChildren.end(); ++it)
					(*it)->OnAnimate(timeMs);
			}
		}


		//! Renders the node.
        // todo: std::function based?
		virtual void render()
        {

        }


		//! Sets the name of the node.
		/** \param name New name of the scene node. */
		void setName(const std::string& name)
		{
			mName = name;
		}

#if 0
		//! Get the axis aligned, not transformed bounding box of this node.
		/** This means that if this node is an animated 3d character,
		moving in a room, the bounding box will always be around the
		origin. To get the box in real world coordinates, just
		transform it with the matrix you receive with
		getAbsoluteTransformation() or simply use
		getTransformedBoundingBox(), which does the same.
		\return The non-transformed bounding box. */
		virtual const core::aabbox3d<f32>& getBoundingBox() const = 0;


		//! Get the axis aligned, transformed and animated absolute bounding box of this node.
		/** \return The transformed bounding box. */
		const core::aabbox3d<f32> getTransformedBoundingBox() const
		{
			core::aabbox3d<f32> box = getBoundingBox();
			AbsoluteTransformation.transformBoxEx(box);
			return box;
		}
#endif

		//! Get the absolute transformation of the node. Is recalculated every OnAnimate()-call.
		/** NOTE: For speed reasons the absolute transformation is not 
		automatically recalculated on each change of the relative 
		transformation or by a transformation change of an parent. Instead the
		update usually happens once per frame in OnAnimate. You can enforce 
		an update with updateAbsolutePosition().
		\return The absolute transformation matrix. */
		const Float4x4Type& getAbsoluteTransformation() const
		{
			return mAbsoluteTransformation;
		}


		//! Returns the relative transformation of the scene node.
		/** The relative transformation is stored internally as 3
		vectors: translation, rotation and scale. To get the relative
		transformation matrix, it is calculated from these values.
		\return The relative transformation matrix. */
		Float4x4Type getRelativeTransformation() const
		{
			Float4x4Type mat;
            Operations::multiplyRotation(mat, mRelativeRotation);
            Operations::multiplyTranslation(mat, mRelativeTranslation);

			//if (RelativeScale != Float3Type(1.f,1.f,1.f))
			{
                Operations::multiplyScale(mat, mRelativeScale);
			}

			return mat;
		}


		//! Returns whether the node should be visible (if all of its parents are visible).
		/** This is only an option set by the user, but has nothing to
		do with geometry culling
		\return The requested visibility of the node, true means
		visible (if all parents are also visible). */
		bool isVisible() const
		{
			return mIsVisible;
		}

		//! Check whether the node is truly visible, taking into accounts its parents' visibility
		/** \return true if the node and all its parents are visible,
		false if this or any parent node is invisible. */
		bool isTrulyVisible() const
		{
			if(!mIsVisible)
				return false;

            Ref parent = getParent();

			if(!parent)
				return true;

			return parent->isTrulyVisible();
		}

		//! Sets if the node should be visible or not.
		/** All children of this node won't be visible either, when set
		to false. Invisible nodes are not valid candidates for selection by
		collision manager bounding box methods.
		\param isVisible If the node shall be visible. */
		void setVisible(bool isVisible)
		{
			mIsVisible = isVisible;
		}


		//! Get the id of the scene node.
		/** This id can be used to identify the node.
		\return The id. */
		int getID() const
		{
			return mID;
		}


		//! Sets the id of the scene node.
		/** This id can be used to identify the node.
		\param id The new id. */
		void setID(int id)
		{
			mID = id;
		}


		//! Adds a child to this scene node.
		/** If the scene node already has a parent it is first removed
		from the other parent.
		\param child A pointer to the new child. */
		void addChild(Ref child)
		{
			if (child && (child != shared_from_this()))
			{
				child->removeFromParent();
				mChildren.push_back(child);
                child->mParent = WeakRef(shared_from_this());
			}
		}


		//! Removes a child from this scene node.
		/** If found in the children list, the child pointer is also
		dropped and might be deleted if no other grab exists.
		\param child A pointer to the child which shall be removed.
		\return True if the child was removed, and false if not,
		e.g. because it couldn't be found in the children list. */
		bool removeChild(Ref child)
		{
			List::iterator it = mChildren.begin();
			for (; it != mChildren.end(); ++it)
				if ((*it) == child)
				{
					(*it)->mParent = WeakRef();
					mChildren.erase(it);
					return true;
				}

			return false;
		}


		//! Removes all children of this scene node
		/** The scene nodes found in the children list are also dropped
		and might be deleted if no other grab exists on them.
		*/
		void removeChildren()
		{
			List::iterator it = mChildren.begin();
			for (; it != mChildren.end(); ++it)
			{
				(*it)->mParent = WeakRef();
			}

			mChildren.clear();
		}


		//! Removes this scene node from the scene
		/** If no other grab exists for this node, it will be deleted.
		*/
		void removeFromParent()
		{
            Ref parent = getParent();
			if (parent)
				parent->removeChild(shared_from_this());
		}

		//! Gets the scale of the scene node relative to its parent.
		/** This is the scale of this node relative to its parent.
		If you want the absolute scale, use
		getAbsoluteTransformation().getScale()
		\return The scale of the scene node. */
		const Float3Type& getScale() const
		{
			return mRelativeScale;
		}


		//! Sets the relative scale of the scene node.
		/** \param scale New scale of the node, relative to its parent. */
		void setScale(const Float3Type& scale)
		{
			mRelativeScale = scale;
		}


		//! Gets the rotation of the node relative to its parent.
		/** Note that this is the relative rotation of the node.
		If you want the absolute rotation, use
		getAbsoluteTransformation().getRotation()
		\return Current relative rotation of the scene node. */
		const Float3Type& getRotation() const
		{
			return mRelativeRotation;
		}


		//! Sets the rotation of the node relative to its parent.
		/** This only modifies the relative rotation of the node.
		\param rotation New rotation of the node in degrees. */
		void setRotation(const QuaternionType& rotation)
		{
			mRelativeRotation = rotation;
		}


		//! Gets the position of the node relative to its parent.
		/** Note that the position is relative to the parent. If you want
		the position in world coordinates, use getAbsolutePosition() instead.
		\return The current position of the node relative to the parent. */
		const Float3Type& getPosition() const
		{
			return mRelativeTranslation;
		}


		//! Sets the position of the node relative to its parent.
		/** Note that the position is relative to the parent.
		\param newpos New relative position of the scene node. */
		void setPosition(const Float3Type& newpos)
		{
			mRelativeTranslation = newpos;
		}


		//! Gets the absolute position of the node in world coordinates.
		/** If you want the position of the node relative to its parent,
		use getPosition() instead.
		NOTE: For speed reasons the absolute position is not 
		automatically recalculated on each change of the relative 
		position or by a position change of an parent. Instead the 
		update usually happens once per frame in OnAnimate. You can enforce 
		an update with updateAbsolutePosition().
		\return The current absolute position of the scene node (updated on last call of updateAbsolutePosition). */
		Float3Type getAbsolutePosition() const
		{
			return mAbsoluteTransformation.getTranslation();
		}

		//! Sets if debug data like bounding boxes should be drawn.
		/** A bitwise OR of the types from @ref irr::E_DEBUG_SCENE_TYPE.
		Please note that not all scene nodes support all debug data types.
		\param state The debug data visibility state to be used. */
		void setDebugDataVisible(unsigned int state)
		{
			mDebugDataVisible = state;
		}

		//! Returns if debug data like bounding boxes are drawn.
		/** \return A bitwise OR of the debug data values from
		@ref irr::E_DEBUG_SCENE_TYPE that are currently visible. */
		unsigned int isDebugDataVisible() const
		{
			return mDebugDataVisible;
		}

		//! Sets if this scene node is a debug object.
		/** Debug objects have some special properties, for example they can be easily
		excluded from collision detection or from serialization, etc. */
		void setIsDebugObject(bool debugObject)
		{
			mIsDebugObject = debugObject;
		}


		//! Returns if this scene node is a debug object.
		/** Debug objects have some special properties, for example they can be easily
		excluded from collision detection or from serialization, etc.
		\return If this node is a debug object, true is returned. */
		bool isDebugObject() const
		{
			return mIsDebugObject;
		}


		//! Returns a const reference to the list of all children.
		/** \return The list of all children of this node. */
		const List& getChildren() const
		{
			return mChildren;
		}


		//! Changes the parent of the scene node.
		/** \param newParent The new parent to be used. */
		void setParent(Ref newParent)
		{
			removeFromParent();

			mParent = WeakRef(newParent);

			if (newParent)
				newParent->addChild(shared_from_this());
		}


		//! Updates the absolute position based on the relative and the parents position
		/** Note: This does not recursively update the parents absolute positions, so if you have a deeper
			hierarchy you might want to update the parents first.*/
		void updateAbsolutePosition()
		{
            Ref parent = getParent();
			if (parent)
			{
				mAbsoluteTransformation =
					parent->getAbsoluteTransformation() * getRelativeTransformation();
			}
			else
				mAbsoluteTransformation = getRelativeTransformation();
		}


		//! Returns the parent of this scene node
		/** \return A pointer to the parent. */
		Ref getParent() const
		{
			return mParent.lock();
		}


	protected:

		//! Name of the scene node.
		std::string mName;

		//! Absolute transformation of the node.
		Float4x4Type mAbsoluteTransformation;

		//! Relative translation of the scene node.
		Float3Type mRelativeTranslation;

		//! Relative rotation of the scene node.
		QuaternionType mRelativeRotation;

		//! Relative scale of the scene node.
		Float3Type mRelativeScale;

		//! Pointer to the parent
		WeakRef mParent;

		//! List of all children of this node
		List mChildren;

		//! ID of the node.
		int mID;

		//! Flag if debug data should be drawn, such as Bounding Boxes.
		unsigned int mDebugDataVisible;

		//! Is the node visible?
		bool mIsVisible;

		//! Is debug object?
		bool mIsDebugObject;
	};


} // end namespace nsg

#endif


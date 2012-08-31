#pragma once

#include <vector>
#include <memory>
#include <algorithm>
#include <functional>

namespace cinder { namespace dx9 {

struct IManagedObject
{
    IManagedObject(bool auto_delete):_auto_delete(auto_delete){}
    virtual HRESULT	OnResetDevice() = 0;
    virtual HRESULT	OnLostDevice() = 0;
    virtual void    Release() = 0;
protected:
    bool    _auto_delete;  //whether release the memory when delete this is called
    virtual void    DoRelease() = 0;
};

template <typename T> struct ManagedObject : IManagedObject
{
    T* _obj;
    ManagedObject(T* ptr, bool auto_delete):IManagedObject(auto_delete),_obj(ptr){}

    HRESULT	OnResetDevice()
    {
        HRESULT hr = S_OK;
        V(_obj->OnResetDevice());
        return hr;
    }

    HRESULT	OnLostDevice()
    {
        HRESULT hr = S_OK;
        V(_obj->OnLostDevice());
        return hr;
    }

    void Release()
    {
        DoRelease();
        if (_auto_delete)
        {
            delete _obj;
            _obj = NULL;
        }
    }

    void DoRelease()
    {
        if (_obj)
        {
            _obj->Release();
            _obj = NULL;
        }
    }
};

struct SAFE_DELETER
{
	template <typename T>
	void operator()(T*& p)
	{
		if( p)
		{
			delete p;
			p = NULL;
		}
	};
};

struct ManagedPool
{
    template <typename T> void insertObject(T* ptr, bool auto_delete = true)
    {
	    _objects.push_back(new ManagedObject<T>(ptr, auto_delete));
    }

    template <typename T> void insertObject(T& ref, bool auto_delete = false)
    {
	    _objects.push_back(new ManagedObject<T>(&ref, auto_delete));
    }

	HRESULT OnResetDevice()
    {
        std::for_each(_objects.begin(), _objects.end(), std::mem_fun(&IManagedObject::OnResetDevice));
        return S_OK;
    }

	HRESULT OnLostDevice()
    {
        std::for_each(_objects.begin(), _objects.end(), std::mem_fun(&IManagedObject::OnLostDevice));
        return S_OK;
    }

    ~ManagedPool()
    {
        clear();
    }

	void clear()
    {
	    std::for_each(_objects.begin(), _objects.end(), std::mem_fun(&IManagedObject::Release));
	    std::for_each(_objects.begin(), _objects.end(), SAFE_DELETER());
	    _objects.clear();
    }

	std::vector<IManagedObject*> _objects;
};

inline ManagedPool& getManagedPool()
{
    static ManagedPool _static;
    return _static;
}

}}// namespace cinder::dx9
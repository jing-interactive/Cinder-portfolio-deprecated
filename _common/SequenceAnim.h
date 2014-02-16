#pragma once

#include <vector>

template <typename T>
struct SequenceAnimT
{
    SequenceAnimT()
    {
        mIsOneshot = false;
        reset();
    }

    float index;
    std::vector<T> frames;
    std::string name;

    void setOneshot(bool flag)
    {
        mIsOneshot = flag;
    }

    void reset()
    {
        index = 0;
        mIsAlive = true;
    }

    void update(float speed)
    {
        if (!mIsAlive)
        {
            return;
        }

        index += speed;
        if (index >= frames.size())
        {
            index = 0;
            if (mIsOneshot)
            {
                mIsAlive = false;
            }
        }
    }

    bool isAlive() const
    {
        return mIsAlive;
    }

    const T& getFrame() const
    {
        return frames[static_cast<int>(index)];
    }

    const ci::gl::Texture& getTexture()
    {
        int id = static_cast<int>(index);

        if (!tex)
        {
            tex = ci::gl::Texture(frames[id]);
        }
        else
        {
            tex.update(frames[id], frames[id].getBounds());
        }

        return tex;
    }

private:
    ci::gl::Texture tex;
    bool            mIsOneshot;
    bool            mIsAlive;
};

typedef SequenceAnimT<ci::Channel> SequenceAnimGray;
typedef SequenceAnimT<ci::Surface> SequenceAnim;
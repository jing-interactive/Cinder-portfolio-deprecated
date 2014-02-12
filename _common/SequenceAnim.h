#pragma once

#include <vector>

template <typename T>
struct SequenceAnimT
{
    SequenceAnimT()
    {
        reset();
    }
    std::vector<T>  frames;
    std::string     name;
    float           index;

    void reset()
    {
        index = 0;
    }

    void update(float speed)
    {
        index += speed;
        if (index >= frames.size())
        {
            index = 0;
        }
    }

    const T& getFrame() const
    {
        return frames[static_cast<int>(index)];
    }

    const ci::gl::Texture& getTexture()
    {
        int id = static_cast<int>(index);

        if (!mTexture)
        {
            mTexture = ci::gl::Texture(frames[id]);
        }
        else
        {
            mTexture.update(frames[id], frames[id].getBounds());
        }

        return mTexture;
    }

private:
    ci::gl::Texture mTexture;
};

typedef SequenceAnimT<ci::Channel> SequenceAnimGray;
typedef SequenceAnimT<ci::Surface> SequenceAnim;
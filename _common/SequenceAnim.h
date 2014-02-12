#pragma once

#include <vector>

template <typename T>
struct SequenceAnimT
{
    SequenceAnimT()
    {
        reset();
    }
    float index;
    std::vector<T> frames;

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
};

typedef SequenceAnimT<ci::Channel> SequenceAnimGray;
typedef SequenceAnimT<ci::Surface> SequenceAnim;
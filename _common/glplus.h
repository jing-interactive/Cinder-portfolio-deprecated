#pragma once

namespace glplus {

#include <stdlib.h>
#include <stdio.h>
#include <vector>

// When rendering back and forth between two textures (ping-ponging), it is
// easiest to just call swapWith() after rendering.
struct Texture {
    unsigned int id;
    int width, height;

    Texture() : id(), width(), height() {}
    ~Texture() { glDeleteTextures(1, &id); }

    void bind(int unit = 0) const { glActiveTexture(GL_TEXTURE0 + unit); glBindTexture(GL_TEXTURE_2D, id); }
    void unbind(int unit = 0) const { glActiveTexture(GL_TEXTURE0 + unit); glBindTexture(GL_TEXTURE_2D, 0); }

    Texture &create(int w, int h, int internalFormat, int format, int type, int filter, int wrap, void *data = NULL) {
        width = w;
        height = h;
        if (!id) glGenTextures(1, &id);
        bind();
        glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, filter);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, filter);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, wrap);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, wrap);
        glTexImage2D(GL_TEXTURE_2D, 0, internalFormat, w, h, 0, format, type, data);
        unbind();
        return *this;
    }

    // Swap the members of this texture with the members of other.
    void swapWith(Texture &other) {
        std::swap(id, other.id);
        std::swap(width, other.width);
        std::swap(height, other.height);
    }
};

// A framebuffer object that can take color attachments. Draw calls between
// bind() and unbind() are drawn to the attached textures.
//
// Usage:
//
//     FBO fbo;
//
//     fbo.attachColor(texture).check().bind();
//     // draw stuff
//     fbo.unbind();
//
struct FBO {
    unsigned int id;
    unsigned int renderbuffer;
    bool autoDepth;
    bool resizeViewport;
    int newViewport[4], oldViewport[4];
    int renderbufferWidth, renderbufferHeight;
    std::vector<unsigned int> drawBuffers;

    FBO(bool autoDepth = true, bool resizeViewport = true) : id(), renderbuffer(), autoDepth(autoDepth),
        resizeViewport(resizeViewport), newViewport(), oldViewport(), renderbufferWidth(), renderbufferHeight() {}
    ~FBO() { glDeleteFramebuffers(1, &id); glDeleteRenderbuffers(1, &renderbuffer); }

    // Draw calls between these will be drawn to attachments. If resizeViewport
    // is true this will automatically resize the viewport to the size of the
    // last attached texture.
    void bind() {
        glBindFramebuffer(GL_FRAMEBUFFER, id);
        if (resizeViewport) {
            glGetIntegerv(GL_VIEWPORT, oldViewport);
            glViewport(newViewport[0], newViewport[1], newViewport[2], newViewport[3]);
        }
    }

    void unbind() {
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        if (resizeViewport) {
            glViewport(oldViewport[0], oldViewport[1], oldViewport[2], oldViewport[3]);
        }
    }

    // Draw to texture 2D in the indicated attachment location (or a 2D layer of
    // a 3D texture).
    FBO &attachColor(const Texture &texture, unsigned int attachment = 0) {
        newViewport[2] = texture.width;
        newViewport[3] = texture.height;
        if (!id) glGenFramebuffers(1, &id);
        bind();

        // Bind a 2D texture
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0 + attachment, GL_TEXTURE_2D, texture.id, 0);

        // Need to call glDrawBuffers() for OpenGL to draw to multiple attachments
        if (attachment >= drawBuffers.size()) drawBuffers.resize(attachment + 1, GL_NONE);
        drawBuffers[attachment] = GL_COLOR_ATTACHMENT0 + attachment;
        glDrawBuffers(drawBuffers.size(), &drawBuffers[0]);

        unbind();
        return *this;
    }

    // Stop drawing to the indicated color attachment
    FBO &detachColor(unsigned int attachment = 0) {
        bind();

        // Update the draw buffers
        if (attachment < drawBuffers.size()) {
            drawBuffers[attachment] = GL_NONE;
            glDrawBuffers(drawBuffers.size(), &drawBuffers[0]);
        }

        unbind();
        return *this;
    }

    // Call after all attachColor() calls, validates attachments.
    FBO &check() {
        bind();
        if (autoDepth) {
            if (renderbufferWidth != newViewport[2] || renderbufferHeight != newViewport[3]) {
                renderbufferWidth = newViewport[2];
                renderbufferHeight = newViewport[3];
                if (!renderbuffer) glGenRenderbuffers(1, &renderbuffer);
                glBindRenderbuffer(GL_RENDERBUFFER, renderbuffer);
                glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT32, renderbufferWidth, renderbufferHeight);
                glBindRenderbuffer(GL_RENDERBUFFER, 0);
            }
            glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, renderbuffer);
        }
        switch (glCheckFramebufferStatus(GL_FRAMEBUFFER)) {
        case GL_FRAMEBUFFER_COMPLETE: break;
        case GL_FRAMEBUFFER_UNDEFINED: printf("GL_FRAMEBUFFER_UNDEFINED\n"); exit(0);
        case GL_FRAMEBUFFER_UNSUPPORTED: printf("GL_FRAMEBUFFER_UNSUPPORTED\n"); exit(0);
        case GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT: printf("GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT\n"); exit(0);
        case GL_FRAMEBUFFER_INCOMPLETE_DRAW_BUFFER: printf("GL_FRAMEBUFFER_INCOMPLETE_DRAW_BUFFER\n"); exit(0);
        case GL_FRAMEBUFFER_INCOMPLETE_READ_BUFFER: printf("GL_FRAMEBUFFER_INCOMPLETE_READ_BUFFER\n"); exit(0);
        case GL_FRAMEBUFFER_INCOMPLETE_MULTISAMPLE: printf("GL_FRAMEBUFFER_INCOMPLETE_MULTISAMPLE\n"); exit(0);
        case GL_FRAMEBUFFER_INCOMPLETE_FORMATS_EXT: printf("GL_FRAMEBUFFER_INCOMPLETE_FORMATS_EXT\n"); exit(0);
        case GL_FRAMEBUFFER_INCOMPLETE_DIMENSIONS_EXT: printf("GL_FRAMEBUFFER_INCOMPLETE_DIMENSIONS_EXT\n"); exit(0);
        case GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT: printf("GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT\n"); exit(0);
        default: printf("Unknown glCheckFramebufferStatus error"); exit(0);
        }
        unbind();
        return *this;
    }
};

static void error(const char *type, const char *error, const char *source = NULL) {
#if 0
    printf("----- %s -----\n", type);
    printf("%s\n", error);
    if (source) {
        printf("----- source code -----\n");
        printf("%s\n", source);
    }
#else
    ci::app::console() << type << "\n" << error << std::endl;
    if (source) {
        ci::app::console() << source << std::endl;
    }
#endif
    exit(0);
}

// Use this macro to pass raw GLSL to Shader::shader()
#define RAW_STRING(x) #x

// Wraps a GLSL shader program and all attached shader stages. Meant to be used
// with the glsl() macro.
//
// Usage:
//
//     // Initialization
//     Shader shader;
//     shader.vertexShader(RAW_STRING(
//         attribute vec4 vertex;
//         void main() {
//             gl_Position = vertex;
//         }
//     )).fragmentShader(RAW_STRING(
//         void main() {
//             gl_FragColor = vec4(1.0, 0.0, 0.0, 1.0);
//         }
//     )).link();
//
//     // Rendering
//     shader.use();
//     // Draw stuff
//     shader.unuse();
//
struct Shader {
    unsigned int id;
    std::vector<unsigned int> stages;

    Shader() : id() {}
    ~Shader() {
        glDeleteProgram(id);
        for (size_t i = 0; i < stages.size(); i++) {
            glDeleteShader(stages[i]);
        }
    }

    Shader &shader(int type, const char *source) {
        // Compile shader
        unsigned int shader = glCreateShader(type);
        glShaderSource(shader, 1, &source, NULL);
        glCompileShader(shader);
        stages.push_back(shader);

        // Check for errors
        char buffer[512];
        int length;
        glGetShaderInfoLog(shader, sizeof(buffer), &length, buffer);
        if (length) error("compile error", buffer, source);

        // Allow chaining
        return *this;
    }

    Shader &vertexShader(const char *source) { return shader(GL_VERTEX_SHADER, source); }
    Shader &fragmentShader(const char *source) { return shader(GL_FRAGMENT_SHADER, source); }

    void link() {
        // Create and link program
        if (!id) id = glCreateProgram();
        for (size_t i = 0; i < stages.size(); i++) {
            glAttachShader(id, stages[i]);
        }
        glLinkProgram(id);

        // Check for errors
        char buffer[512];
        int length;
        glGetProgramInfoLog(id, sizeof(buffer), &length, buffer);
        if (length) error("link error", buffer);
    }

    void use() const { glUseProgram(id); }
    void unuse() const { glUseProgram(0); }

    unsigned int attribute(const char *name) const { return glGetAttribLocation(id, name); }
    unsigned int uniform(const char *name) const { return glGetUniformLocation(id, name); }

    void uniformInt(const char *name, int i) const { glUniform1i(uniform(name), i); }
    void uniformFloat(const char *name, float f) const { glUniform1f(uniform(name), f); }
    void uniform(const char *name, const ci::Vec2f &v) const { glUniform2fv(uniform(name), 1, &v.x); }
    void uniform(const char *name, const ci::Vec3f &v) const { glUniform3fv(uniform(name), 1, &v.x); }
    void uniform(const char *name, const ci::Vec4f &v) const { glUniform4fv(uniform(name), 1, &v.x); }
    void uniform(const char *name, const ci::Matrix44f &m) const { glUniformMatrix4fv(uniform(name), 1, GL_TRUE, m.m); }
};

// A vertex buffer containing a certain type. For example, the simplest vertex
// buffer would be Buffer<vec3> but more complex formats could use Buffer<Vertex>
// with struct Vertex { vec3 position; vec2 coord; }. Each Buffer is meant to be
// used with a VAO.
//
// Index buffers should be specified using an integer type directly, i.e.
// Buffer<unsigned short>, since that's what VAO expects.
//
// Usage:
//
//     Buffer<vec3> vertices;
//     vertices << vec3(1, 0, 0) << vec3(0, 1, 0) << vec3(0, 0, 1);
//     vertices.upload();
//
//     Buffer<char> indices;
//     indices << 0 << 1 << 2;
//     indices.upload(GL_ELEMENT_ARRAY_BUFFER);
//
template <typename T>
struct Buffer {
    std::vector<T> data;
    unsigned int id;
    int currentTarget;

    Buffer() : id(), currentTarget() {}
    ~Buffer() {
        glDeleteBuffers(1, &id);
    }

    void bind() const { glBindBuffer(currentTarget, id); }
    void unbind() const { glBindBuffer(currentTarget, 0); }

    void upload(int target = GL_ARRAY_BUFFER, int usage = GL_STATIC_DRAW) {
        if (!id) glGenBuffers(1, &id);
        currentTarget = target;
        bind();
        glBufferData(currentTarget, data.size() * sizeof(T), &data[0], usage);
        unbind();
    }

    unsigned int size() const { return data.size(); }
    Buffer<T> &operator << (const T &t) { data.push_back(t); return *this; }
};

// Convert a C++ type to an OpenGL type enum using TypeToOpenGL<T>::value
template <typename T> struct TypeToOpenGL {};
template <> struct TypeToOpenGL<bool> { enum { value = GL_BOOL }; };
template <> struct TypeToOpenGL<float> { enum { value = GL_FLOAT }; };
template <> struct TypeToOpenGL<double> { enum { value = GL_DOUBLE }; };
template <> struct TypeToOpenGL<int> { enum { value = GL_INT }; };
template <> struct TypeToOpenGL<char> { enum { value = GL_BYTE }; };
template <> struct TypeToOpenGL<short> { enum { value = GL_SHORT }; };
template <> struct TypeToOpenGL<unsigned int> { enum { value = GL_UNSIGNED_INT }; };
template <> struct TypeToOpenGL<unsigned char> { enum { value = GL_UNSIGNED_BYTE }; };
template <> struct TypeToOpenGL<unsigned short> { enum { value = GL_UNSIGNED_SHORT }; };

// Groups a Buffer for vertices with a set of attributes for drawing. Can
// optionally include a Buffer for indices too.
//
// Usage:
//
//     // Initialization
//     VAO vao;
//     vao.create(shader, vertices, indices).attribute<float>("vertices", 3).check();
//
//     // Rendering
//     vao.draw(GL_TRIANGLE_STRIP);
//
struct VAO {
    // A holder for a Buffer so we can query information (i.e. number of vertices
    // for drawing). Type erasure is used so the VAO class doesn't need to be
    // templated.
    struct BufferHolder {
        virtual ~BufferHolder() {}
        virtual int currentTarget() const = 0;
        virtual unsigned int size() const = 0;
    };
    template <typename T>
    struct BufferHolderImpl : BufferHolder {
        const Buffer<T> &buffer;
        BufferHolderImpl(const Buffer<T> &buffer) : buffer(buffer) {}
        int currentTarget() const { return buffer.currentTarget; }
        unsigned int size() const { return buffer.size(); }
    };

    // You should not need to access these
    unsigned int id;
    int stride, offset, indexType;
    const Shader *shader;
    const BufferHolder *vertices;
    const BufferHolder *indices;

    VAO() : id(), stride(), offset(), indexType(), shader(), vertices(), indices() {}
    ~VAO() { glDeleteVertexArrays(1, &id); delete vertices; delete indices; }

    // You should not need to bind a VAO directly
    void bind() const { glBindVertexArray(id); }
    void unbind() const { glBindVertexArray(0); }

    // Create a vertex array object referencing a shader and a vertex buffer.
    // The shader is used to query the location of attributes in attribute()
    // and the vertex buffer is used to determine the number of elements to
    // draw in draw() and drawInstanced().
    template <typename Vertex>
    VAO &create(const Shader &shader, const Buffer<Vertex> &vbo) {
        delete vertices;
        delete indices;

        this->shader = &shader;
        vertices = new BufferHolderImpl<Vertex>(vbo);
        indices = NULL;
        stride = sizeof(Vertex);
        indexType = GL_INVALID_ENUM;

        if (!id) glGenVertexArrays(1, &id);
        bind();
        vbo.bind();
        unbind();

        return *this;
    }

    // Create a vertex array object referencing a shader, a vertex buffer, and
    // an index buffer. The shader is used to query the location of attributes
    // in attribute() and the index buffer is used to determine the number of
    // elements to draw in draw() and drawInstanced().
    template <typename Vertex, typename Index>
    VAO &create(const Shader &shader, const Buffer<Vertex> &vbo, const Buffer<Index> &ibo) {
        delete vertices;
        delete indices;

        this->shader = &shader;
        vertices = new BufferHolderImpl<Vertex>(vbo);
        indices = new BufferHolderImpl<Index>(ibo);
        stride = sizeof(Vertex);
        indexType = TypeToOpenGL<Index>::value;

        if (!id) glGenVertexArrays(1, &id);
        bind();
        vbo.bind();
        ibo.bind();
        unbind();

        return *this;
    }

    // Define an attribute called name in the provided shader. This attribute
    // has count elements of type T. If normalized is true, integer types are
    // mapped from 0 to 2^n-1 (where n is the bit count) to values from 0 to 1.
    //
    // Attributes should be declared in the order they are declared in the
    // vertex struct (assuming interleaved data). Call check() after declaring
    // all attributes to make sure the vertex struct is packed.
    template <typename T>
    VAO &attribute(const char *name, int count, bool normalized = false) {
        int location = shader->attribute(name);
        bind();
        glEnableVertexAttribArray(location);
        glVertexAttribPointer(location, count, TypeToOpenGL<T>::value, normalized, stride, (char *)NULL + offset);
        unbind();
        offset += count * sizeof(T);
        return *this;
    }

    // Validate VBO modes and attribute byte sizes
    void check() {
        if (vertices && vertices->currentTarget() != GL_ARRAY_BUFFER) {
            printf("expected vertices to have GL_ARRAY_BUFFER, got 0x%04X\n", vertices->currentTarget());
            exit(0);
        }
        if (indices && indices->currentTarget() != GL_ELEMENT_ARRAY_BUFFER) {
            printf("expected indices to have GL_ELEMENT_ARRAY_BUFFER, got 0x%04X\n", indices->currentTarget());
            exit(0);
        }
        if (offset != stride) {
            printf("expected size of attributes (%d bytes) to add up to size of vertex (%d bytes)\n", offset, stride);
            exit(0);
        }
    }


    // Draw the attached VBOs
    void draw(int mode = GL_TRIANGLES) const {
        bind();
        if (indices) glDrawElements(mode, indices->size(), indexType, NULL);
        else glDrawArrays(mode, 0, vertices->size());
        unbind();
    }
};

}

#pragma once
#include "tgaimage.h"
#include "model.h"
#include <memory>
#include <initializer_list>
#include <utility>
#include <algorithm>
#include "Shader.h"

#define MY_PI 3.1415926

class rasterizer {
public:
    rasterizer(Model* model, float* buffer, TGAImage& framebuffer, TGAImage& Zbuffer)
        : model(model), buffer(buffer), framebuffer(framebuffer), Zbuffer(Zbuffer){
    }

    void draw();

    void set_model();

    void set_view(Eigen::Vector3f eye_pos);

    void set_projection(float eye_fov, float aspect_ratio, float zNear, float zFar);

    Eigen::Vector3f rasterizer::phone_texture(Eigen::Vector3f position, Eigen::Vector3f normal, triangle_world triangles, Eigen::Vector3f color);

    void MVP();

    Eigen::Matrix4f get_model() { return model_matrix; }

    Eigen::Matrix4f get_view() { return view_matrix; }

    Eigen::Matrix4f get_projection(){ return projection_matrix; }


private:

    void triangle(Vec3f a, Vec3f b, Vec3f c, Vec2f t0, Vec2f t1, Vec2f t2,float w0, float w1, float w2, TGAImage& texture, triangle_world triangles);
    std::tuple<float, float, float> computeBarycentric2D(float x, float y, Vec3f a, Vec3f b, Vec3f c);

    Eigen::Matrix4f model_matrix;
    Eigen::Matrix4f view_matrix;
    Eigen::Matrix4f projection_matrix;
    Eigen::Matrix4f mvp;

    Model* model;
    float* buffer;
    TGAImage& framebuffer;
    TGAImage& Zbuffer;

    int width = 1500;
    int height = 1500;
};

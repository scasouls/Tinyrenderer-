#pragma once
#include "tgaimage.h"
#include "model.h"
#include <memory>
#include <initializer_list>
#include <utility>
#include <algorithm>

#define MY_PI 3.1415926

class rasterizer {
public:
    rasterizer(Model* model, float* buffer, TGAImage& framebuffer, TGAImage& Zbuffer)
        : model(model), buffer(buffer), framebuffer(framebuffer), Zbuffer(Zbuffer){
    }

    void draw();

    Eigen::Matrix4f model_Matrix(float angle);

    Eigen::Matrix4f view_Matrix(Eigen::Vector3f eye_pos);

    Eigen::Matrix4f projection_Matrix(float eye_fov, float aspect_ratio, float zNear, float zFar);

    Eigen::Matrix4f MVP();


private:

    void triangle(Vec3f a, Vec3f b, Vec3f c, Vec2f t0, Vec2f t1, Vec2f t2, float w0, float w1, float w2, TGAImage& texture);
    std::tuple<float, float, float> computeBarycentric2D(float x, float y, Vec3f a, Vec3f b, Vec3f c);

    Model* model;
    float* buffer;
    TGAImage& framebuffer;
    TGAImage& Zbuffer;

    int width = 1000;
    int height = 1000;
};

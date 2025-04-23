#include "tgaimage.h"
#include "model.h"
#include <memory>
#include <initializer_list>
#include <utility>
#include <algorithm>

const TGAColor white = TGAColor(255, 255, 255, 255);
const TGAColor red = TGAColor(255, 0, 0, 255);
const TGAColor blue = TGAColor(0, 0, 255, 255);
const TGAColor green = TGAColor(0, 255, 0, 255);

std::unique_ptr<Model> model;//采用了智能指针替代了原先的new
float* buffer = nullptr;
const int width = 1000;
const int height = 1500;

//计算重心坐标
std::tuple<float, float, float> computeBarycentric2D(float x , float y, Vec3f a,Vec3f b , Vec3f c) {
    float c1 = ((b.y - c.y) * (x - c.x) + (c.x - b.x) * (y - c.y)) /
        ((b.y - c.y) * (a.x - c.x) + (c.x - b.x) * (a.y - c.y));
    float c2 = ((c.y - a.y) * (x - c.x) + (a.x - c.x) * (y - c.y)) /
        ((b.y - c.y) * (a.x - c.x) + (c.x - b.x) * (a.y - c.y));
    float c3 = 1.0f - c1 - c2;

    return { c1, c2, c3 };
}

//光栅化三角形
void triangle(Vec3f a, Vec3f b, Vec3f c,TGAImage& image,TGAColor Color,float* buffer) {
    int min_x = std::floor(std::min({ a.x,b.x,c.x }));
    int max_x = std::ceil(std::max({ a.x,b.x,c.x }));
    int min_y = std::floor(std::min({ a.y,b.y,c.y }));
    int max_y = std::ceil(std::max({ a.y,b.y,c.y }));


    for (int x = min_x; x < max_x; ++x) {
        for (int y = min_y; y < max_y; ++y) {
            auto [alpha, beta, gamma] = computeBarycentric2D(x+0.5, y+0.5, a, b, c);
            if (alpha < 0 || beta < 0 || gamma < 0) continue;
            float depth = alpha * a.z + beta * b.z + gamma * c.z;//深度插值
            int idx = y * width + x;
            if (depth > buffer[idx]) {
                buffer[idx] = depth;
                image.set(x, y, Color);
            }//更新深度缓存
        }
    }
}


void draw(const std::unique_ptr<Model>& model,TGAImage& framebuffer, TGAImage& Zbuffer,TGAColor Color,float* buffer) {
    for (int i = 0; i < model->nfaces(); ++i) {
        Vec3f v0, v1, v2;
        v0 = model->vert(model->face(i)[0]);
        v1 = model->vert(model->face(i)[1]);
        v2 = model->vert(model->face(i)[2]);
        
        //映射到屏幕空间
        v0.x = (v0.x + 1) * 0.5f * width;
        v0.y = (v0.y + 1) * 0.5f * height;
        v1.x = (v1.x + 1) * 0.5f * width;
        v1.y = (v1.y + 1) * 0.5f * height;
        v2.x = (v2.x + 1) * 0.5f * width;
        v2.y = (v2.y + 1) * 0.5f * height;

        triangle(v0, v1, v2, framebuffer, Color, buffer);
    }
    //生成灰度图
    for (int i = 0; i < width; i++) {
        for (int j = 0; j < height; j++) {
            float depth = buffer[i + j * width];//读取已经更新过的深度缓存
            unsigned char gray = static_cast<unsigned char>(std::clamp(depth * 255.0f, 0.0f, 255.0f));//限制范围
            Zbuffer.set(i, j, TGAColor(gray, 1));
        }
    }


    framebuffer.flip_vertically();
    framebuffer.write_tga_file("frame.tga");
    Zbuffer.flip_vertically();
    Zbuffer.write_tga_file("buffer.tga");
}


int main(int argc, char** argv) { 
    TGAImage framebuffer(width, height, TGAImage::RGB);
    TGAImage Zbuffer(width, height, TGAImage::GRAYSCALE);
    model = std::make_unique<Model>("../obj/delisha.obj");//采用blender转化的obj文件，顶点未归一化
    buffer = new float [width * height];

    for (int i = 0; i < width * height; i++) {
        buffer[i] = -std::numeric_limits<float>::infinity();//默认为无尽小数
    }

    model->normalized();

    draw(model, framebuffer,Zbuffer, white, buffer);
    
    delete[] buffer;

    return 0;
}

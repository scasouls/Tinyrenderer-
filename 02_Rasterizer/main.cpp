#include "tgaimage.h"
#include "model.h"
#include <memory>
#include <initializer_list>

const TGAColor white = TGAColor(255, 255, 255, 255);
const TGAColor red = TGAColor(255, 0, 0, 255);
const TGAColor blue = TGAColor(0, 0, 255, 255);
const TGAColor green = TGAColor(0, 255, 0, 255);

std::unique_ptr<Model> model;//采用了智能指针替代了原先的new
const int width = 1960;
const int height = 2180;

//画线算法
void line(int x0, int y0, int x1, int y1, TGAImage& image, TGAColor color) {
    bool steep = false;
    if (std::abs(x0 - x1) < std::abs(y0 - y1)) {
        std::swap(x0, y0);
        std::swap(x1, y1);
        steep = true;
    }
    if (x0 > x1) {
        std::swap(x0, x1);
        std::swap(y0, y1);
    }

    for (int x = x0; x <= x1; x++) {
        float t = (x - x0) / (float)(x1 - x0);
        int y = y0 * (1. - t) + y1 * t;
        if (steep) {
            image.set(y, x, color);
        }
        else {
            image.set(x, y, color);
        }
    }
}


double signed_triangle_area(int ax, int ay, int bx, int by, int cx, int cy) {
    return .5 * ((by - ay) * (bx + ax) + (cy - by) * (cx + bx) + (ay - cy) * (ax + cx));
}

void triangle(int ax, int ay, int bx, int by, int cx, int cy, TGAImage& framebuffer, TGAColor color) {
    int bbminx = std::min(std::min(ax, bx), cx);
    int bbminy = std::min(std::min(ay, by), cy);
    int bbmaxx = std::max(std::max(ax, bx), cx);
    int bbmaxy = std::max(std::max(ay, by), cy);
    double total_area = signed_triangle_area(ax, ay, bx, by, cx, cy);
    if (total_area < 1) return;

    for (int x = bbminx; x <= bbmaxx; x++) {
        for (int y = bbminy; y <= bbmaxy; y++) {
            double alpha = signed_triangle_area(x, y, bx, by, cx, cy) / total_area;
            double beta = signed_triangle_area(x, y, cx, cy, ax, ay) / total_area;
            double gamma = signed_triangle_area(x, y, ax, ay, bx, by) / total_area;
            const double eps = 1e-5;
            if (alpha < -eps || beta < -eps || gamma < -eps) continue;
            framebuffer.set(x, y, color);
        }
    }
}

int main(int argc, char** argv) {
    TGAImage image(width, height, TGAImage::RGB);
    model = std::make_unique<Model>("../obj/delisha.obj");//采用blender转化的obj文件，顶点未归一化

    float min_x = std::numeric_limits<float>::max(), max_x = std::numeric_limits<float>::lowest();
    float min_y = min_x, max_y = max_x;

    //得出顶点的包围盒范围
    for (int i = 0; i < model->nverts(); ++i) {
        Vec3f v = model->vert(i);
        min_x = std::min(min_x, v.x);
        max_x = std::max(max_x, v.x);
        min_y = std::min(min_y, v.y);
        max_y = std::max(max_y, v.y);
    }

    for (int i = 0; i < model->nfaces(); ++i) {
        std::vector<int> face = model->face(i);
            Vec3f v0 = model->vert(face[0]);
            Vec3f v1 = model->vert(face[1]);
            Vec3f v2 = model->vert(face[2]);
            //直接映射到屏幕范围
            int x0 = (v0.x - min_x) / (max_x - min_x) * (width - 1);
            int y0 = (v0.y - min_y) / (max_y - min_y) * (height - 1);
            int x1 = (v1.x - min_x) / (max_x - min_x) * (width - 1);
            int y1 = (v1.y - min_y) / (max_y - min_y) * (height - 1);
            int x2 = (v2.x - min_x) / (max_x - min_x) * (width - 1);
            int y2 = (v2.y - min_y) / (max_y - min_y) * (height - 1);
            triangle(x0, y0, x1, y1,x2,y2, image, white);
    }

    image.flip_vertically(); // i want to have the origin at the left bottom corner of the image
    image.write_tga_file("output.tga");
    return 0;
}

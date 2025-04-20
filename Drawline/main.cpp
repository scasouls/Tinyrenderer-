#include "tgaimage.h"
#include "model.h"
#include <memory>

const TGAColor white = TGAColor(255, 255, 255, 255);
const TGAColor red = TGAColor(255, 0, 0, 255);
const TGAColor blue = TGAColor(0, 0, 255, 255);
const TGAColor green = TGAColor(0, 255, 0, 255);

std::unique_ptr<Model> model;//采用了智能指针替代了原先的new
const int width = 1000;
const int height = 1500;

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
		for (int j = 0; j < 3; ++j) {

			Vec3f v0 = model->vert(face[j]);
			Vec3f v1 = model->vert(face[(j + 1) % 3]);
            //直接映射到屏幕范围
			int x0 = (v0.x - min_x) / (max_x - min_x) * (width - 1);
			int y0 = (v0.y - min_y) / (max_y - min_y) * (height - 1);
			int x1 = (v1.x - min_x) / (max_x - min_x) * (width - 1);
			int y1 = (v1.y - min_y) / (max_y - min_y) * (height - 1);
			line(x0, y0, x1, y1, image,red);

		}
	}

	image.flip_vertically(); // i want to have the origin at the left bottom corner of the image
	image.write_tga_file("output.tga");
	return 0;
}

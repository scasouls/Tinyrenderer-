#include "tgaimage.h"
#include "model.h"
#include "rasterizer.h"

const TGAColor white = TGAColor(255, 255, 255, 255);
const TGAColor red = TGAColor(255, 0, 0, 255);
const TGAColor blue = TGAColor(0, 0, 255, 255);
const TGAColor green = TGAColor(0, 255, 0, 255);

std::unique_ptr<Model> model;//����������ָ�������ԭ�ȵ�new
std::unique_ptr<float[]> buffer;
std::unique_ptr<rasterizer> Ras;

const int width = 1500;
const int height = 1500;


int main(int argc, char** argv) { 
    TGAImage framebuffer(width, height, TGAImage::RGB);
    TGAImage Zbuffer(width, height, TGAImage::GRAYSCALE);

    model = std::make_unique<Model>("../obj/youda.obj");//����blenderת����obj�ļ�������δ��һ��
    buffer = std::make_unique<float[]>(width * height);

    for (int i = 0; i < width * height; i++) {
        buffer[i] = std::numeric_limits<float>::infinity();
    }

    Ras = std::make_unique<rasterizer>(model.get(), buffer.get(), framebuffer, Zbuffer);
    Ras->draw();

    return 0;
}

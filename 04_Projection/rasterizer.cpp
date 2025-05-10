#include "rasterizer.h"

Eigen::Matrix4f rasterizer::model_Matrix(float angle) {
    Eigen::Matrix4f rotation = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f translate = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f scale;
    scale << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;

    return translate  * rotation * scale;
}

Eigen::Matrix4f rasterizer::view_Matrix(Eigen::Vector3f eye_pos) {
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0],
        0, 1, 0, -eye_pos[1],
        0, 0, 1, -eye_pos[2],
        0, 0, 0, 1;

    Eigen::Matrix4f rotate;
    rotate << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, -1, 0,
        0, 0, 0, 1;

    // 先旋转再平移
    view = rotate * translate;

    return view;
}

Eigen::Matrix4f rasterizer::projection_Matrix(float eye_fov, float aspect, float zNear, float zFar) {
    float angle = eye_fov * MY_PI / 180.0f;
    float t = tan(angle / 2.0f) * zNear;
    float r = t * aspect;
    float l = -r;
    float b = -t;

    // 透视投影矩阵的变换
    Eigen::Matrix4f persp2ortho;
    persp2ortho <<
        zNear, 0, 0, 0,
        0, zNear, 0, 0,
        0, 0, zNear + zFar, -zNear * zFar,
        0, 0, 1, 0;

    // 正射投影的缩放和平移
    Eigen::Matrix4f ortho_scale;
    ortho_scale <<
        2 / (r - l), 0, 0, 0,
        0, 2 / (t - b), 0, 0,
        0, 0, 2 / (zNear - zFar), 0,
        0, 0, 0, 1;

    Eigen::Matrix4f ortho_trans;
    ortho_trans <<
        1, 0, 0, -(r + l) / 2,
        0, 1, 0, -(t + b) / 2,
        0, 0, 1, -(zNear + zFar) / 2,
        0, 0, 0, 1;

    // 综合透视和正射投影
    Eigen::Matrix4f ortho = ortho_scale * ortho_trans;
    return ortho * persp2ortho;
}


Eigen::Matrix4f rasterizer::MVP() {
    Eigen::Vector3f eye_pos(0, 0.7f, 2.17f);
    float fov = 75.0f;
    float aspect = width / height;
    float zNear = 0.8f;
    float zFar = 3.0f;

    Eigen::Matrix4f M = model_Matrix(fov);
    Eigen::Matrix4f V = view_Matrix(eye_pos);
    Eigen::Matrix4f P = projection_Matrix(fov, aspect, zNear, zFar);
    return P * V * M;
}

//计算重心坐标
std::tuple<float, float, float> rasterizer::computeBarycentric2D(float x, float y, Vec3f a, Vec3f b, Vec3f c) {
    float c1 = ((b.y - c.y) * (x - c.x) + (c.x - b.x) * (y - c.y)) /
        ((b.y - c.y) * (a.x - c.x) + (c.x - b.x) * (a.y - c.y));
    float c2 = ((c.y - a.y) * (x - c.x) + (a.x - c.x) * (y - c.y)) /
        ((b.y - c.y) * (a.x - c.x) + (c.x - b.x) * (a.y - c.y));
    float c3 = 1.0f - c1 - c2;

    return { c1, c2, c3 };
}

//光栅化三角形
void rasterizer::triangle(Vec3f a, Vec3f b, Vec3f c, Vec2f t0, Vec2f t1, Vec2f t2, float w0, float w1, float w2, TGAImage& texture) {
    int min_x = std::floor(std::min({ a.x,b.x,c.x }));
    int max_x = std::ceil(std::max({ a.x,b.x,c.x }));
    int min_y = std::floor(std::min({ a.y,b.y,c.y }));
    int max_y = std::ceil(std::max({ a.y,b.y,c.y }));


    for (int x = min_x; x <= max_x; ++x) {
        for (int y = min_y; y <= max_y; ++y) {
            auto [alpha, beta, gamma] = computeBarycentric2D(x + 0.5f, y + 0.5f, a, b, c);
            if (alpha < 0 || beta < 0 || gamma < 0) continue;

            double w_reciprocal = 1.0f / (alpha * w0 + beta * w1 + gamma * w2);
            double z_interpolated = (alpha * a.z / w0 + beta * b.z / w1 + gamma * c.z / w2) * w_reciprocal;

            int idx = y * width + x;
            if (idx < 0 || idx >= width * height) continue;

            if (z_interpolated < buffer[idx]) {
                buffer[idx] = z_interpolated;

                //std::cout << z_interpolated << std::endl;                

                float u = (alpha * t0.u / w0 + beta * t1.u / w1 + gamma * t2.u / w2) * w_reciprocal;
                float v = (alpha * t0.v / w0 + beta * t1.v / w1 + gamma * t2.v / w2) * w_reciprocal;

                u = std::max(0.0f, std::min(u, 1.0f));
                v = std::max(0.0f, std::min(v, 1.0f));

                int tex_x = std::min(int(u * texture.get_width()), texture.get_width() - 1);
                int tex_y = std::min(int(v * texture.get_height()), texture.get_height() - 1);

                //std::cout << u<< v <<  std::endl;
                TGAColor color = texture.get(tex_x, tex_y);
                framebuffer.set(x, y, color);
            }
        }
    }
}

void rasterizer::draw() {
    TGAImage texture;
    texture.read_tga_file("../obj/youda.tga");

    Eigen::Matrix4f mvp = MVP();

    for (int i = 0; i < model->nfaces(); ++i) {
        // 顶点索引
        int idx0 = model->face(i)[0].x;
        int idx1 = model->face(i)[1].x;
        int idx2 = model->face(i)[2].x;

        Vec3f v0_obj = model->vert(idx0);
        Vec3f v1_obj = model->vert(idx1);
        Vec3f v2_obj = model->vert(idx2);

        // 原始纹理坐标
        Vec2f t0 = model->texture(model->face(i)[0].y);
        Vec2f t1 = model->texture(model->face(i)[1].y);
        Vec2f t2 = model->texture(model->face(i)[2].y);

        Eigen::Matrix4f screen;
        screen << width * 0.5f, 0, 0, width * 0.5f,
            0, height * 0.5f, 0, height * 0.5f,
            0, 0, 0.5f, 0.5f,
            0, 0, 0, 1;

        // 对每个顶点进行 MVP 变换 → NDC → 屏幕空间
        Eigen::Vector4f v0_clip = mvp * Eigen::Vector4f(v0_obj.x, v0_obj.y, v0_obj.z, 1.0f);
        Eigen::Vector4f v1_clip = mvp * Eigen::Vector4f(v1_obj.x, v1_obj.y, v1_obj.z, 1.0f);
        Eigen::Vector4f v2_clip = mvp * Eigen::Vector4f(v2_obj.x, v2_obj.y, v2_obj.z, 1.0f);

        float iw0 = 1.f / v0_clip.w();
        float iw1 = 1.f / v1_clip.w();
        float iw2 = 1.f / v2_clip.w();

        
        // NDC → 屏幕空间
        Eigen::Vector4f v0_screen_h = screen * (v0_clip / v0_clip.w());
        Eigen::Vector4f v1_screen_h = screen * (v1_clip / v1_clip.w());
        Eigen::Vector4f v2_screen_h = screen * (v2_clip / v2_clip.w());


        Vec3f v0_screen(v0_screen_h.x(), v0_screen_h.y(), v0_screen_h.z());
        Vec3f v1_screen(v1_screen_h.x(), v1_screen_h.y(), v1_screen_h.z());
        Vec3f v2_screen(v2_screen_h.x(), v2_screen_h.y(), v2_screen_h.z());

        // 对 UV 做透视矫正
        Vec2f t0_corr = t0 * iw0;
        Vec2f t1_corr = t1 * iw1;
        Vec2f t2_corr = t2 * iw2;

        triangle(v0_screen, v1_screen, v2_screen, t0_corr, t1_corr, t2_corr, iw0, iw1, iw2, texture);
    }
    //生成灰度图
    for (int i = 0; i < width; i++) {
        for (int j = 0; j < height; j++) {
            float depth = buffer[i + j * width];//读取已经更新过的深度缓存
            unsigned char gray = static_cast<unsigned char>(std::clamp((1.0f - depth) * 255.0f, 0.0f, 255.0f));//限制范围
            Zbuffer.set(i, j, TGAColor(gray, 1));
        }
    }

    framebuffer.flip_vertically();
    framebuffer.write_tga_file("frame.tga");
    Zbuffer.flip_vertically();
    Zbuffer.write_tga_file("buffer.tga");
}
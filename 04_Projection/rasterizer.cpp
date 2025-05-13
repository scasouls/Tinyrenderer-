#include "rasterizer.h"

Eigen::Matrix4f rasterizer::model_Matrix(float angle) {
    Eigen::Matrix4f rotation = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f translate = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f scale;
    scale << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0,1, 0,
        0, 0, 0, 1;

    //��y�������ת
    float a = 60.0 * MY_PI / 360.0f;
    Eigen::Matrix4f rota;
    rota << cos(a), 0, sin(a), 0,
        0, 1, 0, 0,
        -sin(a), 0, cos(a), 0,
        0, 0, 0, 1;

    return translate  * rota * scale;
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
        0, 0, 1, 0,
        0, 0, 0, 1;

    // ����ת��ƽ��
    view = translate * rotate;

    return view;
}

Eigen::Matrix4f rasterizer::projection_Matrix(float eye_fov, float aspect, float zNear, float zFar) {
    float angle = eye_fov * MY_PI / 180.0f;
    float t = tan(angle / 2.0f) * zNear;
    float r = t * aspect;

    //͸�Ӿ���
    Eigen::Matrix4f proj;
    proj <<
        zNear / r, 0, 0, 0,
        0, zNear / t, 0, 0,
        0, 0, -(zFar + zNear) / (zFar - zNear), -2 * zFar * zNear / (zFar - zNear),
        0, 0, -1, 0;
    

    //NDC ���� Screen
    Eigen::Matrix4f screen;
    screen << width * 0.5f, 0, 0, width * 0.5f,
        0, height * 0.5f, 0, height * 0.5f,
        0, 0, 0.5f, 0.5f,
        0, 0, 0, 1;

    // �ۺ�͸�Ӻ�����ͶӰ

    return screen * proj;
}


Eigen::Matrix4f rasterizer::MVP() {
    Eigen::Vector3f eye_pos(-0.5, 1, 1.5f);
    float fov = 90.0f;
    float aspect = width / height;
    float zNear = 1.0f;
    float zFar = 3.0f;

    Eigen::Matrix4f M = model_Matrix(fov);
    Eigen::Matrix4f V = view_Matrix(eye_pos);
    Eigen::Matrix4f P = projection_Matrix(fov, aspect, zNear, zFar);
    return P * V * M;
}

//������������
std::tuple<float, float, float> rasterizer::computeBarycentric2D(float x, float y, Vec3f a, Vec3f b, Vec3f c) {
    float c1 = ((b.y - c.y) * (x - c.x) + (c.x - b.x) * (y - c.y)) /
        ((b.y - c.y) * (a.x - c.x) + (c.x - b.x) * (a.y - c.y));
    float c2 = ((c.y - a.y) * (x - c.x) + (a.x - c.x) * (y - c.y)) /
        ((b.y - c.y) * (a.x - c.x) + (c.x - b.x) * (a.y - c.y));
    float c3 = 1.0f - c1 - c2;

    return { c1, c2, c3 };
}


//˫���Բ�ֵ
Eigen::Vector3f filtering(float u, float v, TGAImage& texture) {
    float u_img = u * (texture.get_width()-1);
    float v_img = v * (texture.get_height()-1);

    float lx = std::floor(u_img);
    float rx = std::ceil(u_img);
    float ty = std::ceil(v_img);
    float by = std::floor(v_img);

    auto lerp = [](float s, Eigen::Vector3f v0, Eigen::Vector3f v1) {
        return v0 + s * (v1 - v0);
        };

    auto get_color = [&](float u, float v) {
        auto color = texture.get(u, v);
        return Eigen::Vector3f(color.raw[0], color.raw[1], color.raw[2]);
        };
    
    Eigen::Vector3f u0 = lerp(u_img - lx, get_color(lx, ty), get_color(rx, ty));
    Eigen::Vector3f u1 = lerp(u_img - lx, get_color(lx, by), get_color(rx, by));

    Eigen::Vector3f r = lerp(v_img - by, u0, u1);
    return r;
}

//��դ��������
void rasterizer::triangle(Vec3f a, Vec3f b, Vec3f c, Vec2f t0, Vec2f t1, Vec2f t2, float w0, float w1, float w2, TGAImage& texture) {
    int min_x = std::floor(std::min({ a.x,b.x,c.x }));
    int max_x = std::ceil(std::max({ a.x,b.x,c.x }));
    int min_y = std::floor(std::min({ a.y,b.y,c.y }));
    int max_y = std::ceil(std::max({ a.y,b.y,c.y }));


    for (int x = min_x; x <= max_x; ++x) {
        for (int y = min_y; y <= max_y; ++y) {

            auto [alpha, beta, gamma] = computeBarycentric2D(x + 0.5f, y + 0.5f, a, b, c);
            if (alpha < 0 || beta < 0 || gamma < 0) continue;

            double z_interpolated = alpha * a.z + beta * b.z + gamma * c.z;

            int idx = y * width + x;
            if (idx < 0 || idx >= width * height) continue;

            if (z_interpolated < buffer[idx]) {
                buffer[idx] = z_interpolated;                

                //͸�ӽ���
                float u = (alpha * t0.u / w0 + beta * t1.u / w1 + gamma * t2.u / w2) /
                    (alpha / w0 + beta / w1 + gamma / w2);
                float v = (alpha * t0.v / w0 + beta * t1.v / w1 + gamma * t2.v / w2) /
                    (alpha / w0 + beta / w1 + gamma / w2);

                u = std::max(0.0f, std::min(u, 1.0f));
                v = std::max(0.0f, std::min(v, 1.0f));

                v = 1 - v;

                Eigen::Vector3f color_vec = filtering(u, v, texture);

                //color��ȡbgr����תΪrgb
                TGAColor color(color_vec.z(), color_vec.y(), color_vec.x(), 255);
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
        //����ģ������
        Vec3f v0 = model->vert(model->face(i)[0].x);
        Vec3f v1 = model->vert(model->face(i)[1].x);
        Vec3f v2 = model->vert(model->face(i)[2].x);



        // ԭʼ��������
        Vec2f t0 = model->texture(model->face(i)[0].y);
        Vec2f t1 = model->texture(model->face(i)[1].y);
        Vec2f t2 = model->texture(model->face(i)[2].y);

        //Ӧ��mvp�任
        Eigen::Vector4f v0_screen = mvp * Eigen::Vector4f(v0.x, v0.y, v0.z, 1.0f);
        Eigen::Vector4f v1_screen = mvp * Eigen::Vector4f(v1.x, v1.y, v1.z, 1.0f);
        Eigen::Vector4f v2_screen = mvp * Eigen::Vector4f(v2.x, v2.y, v2.z, 1.0f);

        //��ȡwֵ
        float w0, w1, w2;
        w0 = v0_screen.w();
        w1 = v1_screen.w();
        w2 = v2_screen.w();

        //��λ�����
        Vec3f v0_homo(v0_screen.x() / w0, v0_screen.y() / w0, v0_screen.z() / w0);
        Vec3f v1_homo(v1_screen.x() / w1, v1_screen.y() / w1, v1_screen.z() / w1);
        Vec3f v2_homo(v2_screen.x() / w2, v2_screen.y() / w2, v2_screen.z() / w2);

        triangle(v0_homo, v1_homo, v2_homo, t0, t1, t2, w0, w1, w2, texture);
    }

    //��ѡ������ģ����ǿ�����Ч����
    //float min_depth = -2;
    //float max_depth = 0.8;

        for (int i = 0; i < width; i++) {
            for (int j = 0; j < height; j++) {
                float depth = buffer[i + j * width];

                // ӳ�䵽 [0, 1]
                //float norm_depth = (depth - min_depth) / (max_depth - min_depth);

                depth = 1.0f - depth;

                unsigned char gray = static_cast<unsigned char>(std::clamp(depth * 255.0f, 0.0f, 255.0f));
                Zbuffer.set(i, j, TGAColor(gray, 1));
            }
        }

    framebuffer.flip_vertically();
    framebuffer.write_tga_file("frame.tga");
    Zbuffer.flip_vertically();
    Zbuffer.write_tga_file("buffer.tga");
}
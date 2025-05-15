#include "rasterizer.h"

void rasterizer::set_model() {
    Eigen::Matrix4f rotation = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f translate = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f scale;
    scale << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0,1, 0,
        0, 0, 0, 1;

    //按y轴进行旋转
    float a = 60.0 * MY_PI / 360.0f;
    Eigen::Matrix4f rota;
    rota << cos(a), 0, sin(a), 0,
        0, 1, 0, 0,
        -sin(a), 0, cos(a), 0,
        0, 0, 0, 1;

    model_matrix =  translate  * rota * scale;
}

void rasterizer::set_view(Eigen::Vector3f eye_pos) {
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

    // 先旋转再平移
    view = translate * rotate;

    view_matrix = view;
}

void rasterizer::set_projection(float eye_fov, float aspect, float zNear, float zFar) {
    float angle = eye_fov * MY_PI / 180.0f;
    float t = tan(angle / 2.0f) * zNear;
    float r = t * aspect;

    //透视矩阵
    Eigen::Matrix4f proj;
    proj <<
        zNear / r, 0, 0, 0,
        0, zNear / t, 0, 0,
        0, 0, -(zFar + zNear) / (zFar - zNear), -2 * zFar * zNear / (zFar - zNear),
        0, 0, -1, 0;
    

    //NDC —— Screen
    Eigen::Matrix4f screen;
    screen << width * 0.5f, 0, 0, width * 0.5f,
        0, height * 0.5f, 0, height * 0.5f,
        0, 0, 0.5f, 0.5f,
        0, 0, 0, 1;

    // 综合透视和正射投影

    projection_matrix = screen * proj;
}


void rasterizer::MVP() {
    Eigen::Vector3f eye_pos(-0.5, 1, 1.5f);
    float fov = 90.0f;
    float aspect = width / height;
    float zNear = 1.0f;
    float zFar = 3.0f;

    set_model();
    set_view(eye_pos);
    set_projection(fov, aspect, zNear, zFar);

    mvp = projection_matrix * view_matrix * model_matrix;
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


//双线性插值
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
    Eigen::Vector3f result = Eigen::Vector3f(r[2], r[1], r[0]);
    return result;
}

struct Light {
    Eigen::Vector3f position;
    Eigen::Vector3f intensity;
};

Eigen::Vector3f rasterizer::phone_texture(Eigen::Vector3f position, Eigen::Vector3f normal,triangle_world triangles, Eigen::Vector3f color) {
    normal.normalize();

    Eigen::Vector3f ka = Eigen::Vector3f(0.018f, 0.018f, 0.018f);
    Eigen::Vector3f kd = color / 255.0f;
    Eigen::Vector3f ks = Eigen::Vector3f(0.5, 0.5, 0.5);
     
    auto l1 = Light{ {1.5f, 1.5f, 3.f}, {100.0f, 100.0f, 100.0f} };
    auto l2 = Light{ {-3.f, 3.f, -2.f}, {30.0f, 30.0f, 30.0f} };
    auto l3 = Light{{0.0f, 4.0f, 0.0f},{10.0f, 10.0f, 10.0f}
    };

    std::vector<Light> lights = { l1,l2 ,l3};
    Eigen::Vector3f amb_light_intensity{ 3, 3, 3 };
    Eigen::Vector3f eye_pos(-0.5f, 1.0f, 1.5f);
    float p = 10; // 高光系数

    Eigen::Vector3f result_color = Eigen::Vector3f::Zero();

    for (auto& light : lights) {
        Eigen::Vector3f light_dir = light.position - position;
        float r2 = light_dir.dot(light_dir);
        light_dir.normalize();

        Eigen::Vector3f view_dir = (eye_pos - position).normalized();
        Eigen::Vector3f halfway = (view_dir + light_dir).normalized();
        
        // diffuse
        float diff = std::max(0.0f, normal.dot(light_dir));
        // specular
        float spec = std::pow(std::max(0.0f, normal.dot(halfway)), p);

        Eigen::Vector3f La = ka.cwiseProduct(amb_light_intensity);
        Eigen::Vector3f Ld = kd.cwiseProduct(light.intensity / r2) * diff;
        Eigen::Vector3f Ls = ks.cwiseProduct(light.intensity / r2) * spec;

        result_color +=  Ls + Ld + La;
    }

    result_color = result_color * 255.f;
    for (int i = 0; i < 3; ++i)
        result_color[i] = std::clamp(result_color[i], 0.0f, 255.0f);


    return result_color;
}
//光栅化三角形
void rasterizer::triangle(Vec3f a, Vec3f b, Vec3f c,
    Vec2f t0, Vec2f t1, Vec2f t2,
    float w0, float w1, float w2, TGAImage& texture, triangle_world triangles) {
    int min_x = std::floor(std::min({ a.x,b.x,c.x }));
    int max_x = std::ceil(std::max({ a.x,b.x,c.x }));
    int min_y = std::floor(std::min({ a.y,b.y,c.y }));
    int max_y = std::ceil(std::max({ a.y,b.y,c.y }));


    for (int x = min_x; x <= max_x; ++x) {
        for (int y = min_y; y <= max_y; ++y) {

            auto [alpha, beta, gamma] = computeBarycentric2D(x + 0.5f, y + 0.5f, a, b, c);
            if (alpha < 0 || beta < 0 || gamma < 0) continue;

            double z_interpolated = alpha * a.z + beta * b.z + gamma * c.z;

            Eigen::Vector3f position =
                alpha * Eigen::Vector3f(triangles.v_world[0].head<3>()) +
                beta * Eigen::Vector3f(triangles.v_world[1].head<3>()) +
                gamma * Eigen::Vector3f(triangles.v_world[2].head<3>());

            Eigen::Vector3f normal =
                alpha * triangles.n_world[0] +
                beta * triangles.n_world[1] +
                gamma * triangles.n_world[2];

            normal *= (1.0 / (alpha / w0 + beta / w1 + gamma / w2));

            int idx = y * width + x;
            if (idx < 0 || idx >= width * height) continue;

            if (z_interpolated < buffer[idx]) {
                buffer[idx] = z_interpolated;                

                //透视矫正
                float u = (alpha * t0.u / w0 + beta * t1.u / w1 + gamma * t2.u / w2) /
                    (alpha / w0 + beta / w1 + gamma / w2);
                float v = (alpha * t0.v / w0 + beta * t1.v / w1 + gamma * t2.v / w2) /
                    (alpha / w0 + beta / w1 + gamma / w2);

                u = std::max(0.0f, std::min(u, 1.0f));
                v = std::max(0.0f, std::min(v, 1.0f));

                v = 1 - v;

                Eigen::Vector3f color_vec = filtering(u, v, texture);

                Eigen::Vector3f phong_color = phone_texture(position,normal,triangles,color_vec);

                TGAColor color(phong_color.x(), phong_color.y(), phong_color.z(), 255);
                framebuffer.set(x, y, color);
            }
        }
    }
}

void rasterizer::draw() {
    TGAImage texture;
    texture.read_tga_file("../obj/youda.tga");

    MVP();

    triangle_world triangles;

    Eigen::Matrix4f inv = model_matrix.inverse().transpose();

    for (int i = 0; i < model->nfaces(); ++i) {
        //顶点模型坐标
        Vec3f v0 = model->vert(model->face(i)[0].x);
        Vec3f v1 = model->vert(model->face(i)[1].x);
        Vec3f v2 = model->vert(model->face(i)[2].x);

        triangles.v_world.push_back(model_matrix * Eigen::Vector4f(v0.x, v0.y, v0.z, 1.0f));
        triangles.v_world.push_back(model_matrix * Eigen::Vector4f(v1.x, v1.y, v1.z, 1.0f));
        triangles.v_world.push_back(model_matrix * Eigen::Vector4f(v2.x, v2.y, v2.z, 1.0f));

        // 原始纹理坐标
        Vec2f t0 = model->texture(model->face(i)[0].y);
        Vec2f t1 = model->texture(model->face(i)[1].y);
        Vec2f t2 = model->texture(model->face(i)[2].y);

        Vec3f n0 = model->normal(model->face(i)[0].z);
        Vec3f n1 = model->normal(model->face(i)[1].z);
        Vec3f n2 = model->normal(model->face(i)[2].z);

        Eigen::Vector4f n0_vm = inv * Eigen::Vector4f(n0.x, n0.y, n0.z,0.0f);
        Eigen::Vector4f n1_vm = inv * Eigen::Vector4f(n1.x, n1.y, n1.z,0.0f);
        Eigen::Vector4f n2_vm = inv * Eigen::Vector4f(n2.x, n2.y, n2.z,0.0f);

        //应用mvp变换
        Eigen::Vector4f v0_screen = mvp * Eigen::Vector4f(v0.x, v0.y, v0.z, 1.0f);
        Eigen::Vector4f v1_screen = mvp * Eigen::Vector4f(v1.x, v1.y, v1.z, 1.0f);
        Eigen::Vector4f v2_screen = mvp * Eigen::Vector4f(v2.x, v2.y, v2.z, 1.0f);

        //获取w值
        float w0, w1, w2;
        w0 = v0_screen.w();
        w1 = v1_screen.w();
        w2 = v2_screen.w();

        Eigen::Vector3f n0_eigen = Eigen::Vector3f(n0_vm.x() / w0, n0_vm.y() / w0, n0_vm.z() / w0);
        Eigen::Vector3f n1_eigen = Eigen::Vector3f(n0_vm.x() / w1, n0_vm.y() / w1, n0_vm.z() / w1);
        Eigen::Vector3f n2_eigen = Eigen::Vector3f(n0_vm.x() / w2, n0_vm.y() / w2, n0_vm.z() / w2);

        triangles.n_world.push_back(n0_eigen);
        triangles.n_world.push_back(n1_eigen);
        triangles.n_world.push_back(n2_eigen);

        //齐次化处理
        Vec3f v0_homo(v0_screen.x() / w0, v0_screen.y() / w0, v0_screen.z() / w0);
        Vec3f v1_homo(v1_screen.x() / w1, v1_screen.y() / w1, v1_screen.z() / w1);
        Vec3f v2_homo(v2_screen.x() / w2, v2_screen.y() / w2, v2_screen.z() / w2);

        triangle(v0_homo, v1_homo, v2_homo, t0, t1, t2,w0, w1, w2, texture,triangles);
    }

        for (int i = 0; i < width; i++) {
            for (int j = 0; j < height; j++) {
                float depth = buffer[i + j * width];

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
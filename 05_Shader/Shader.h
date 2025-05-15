#pragma once
#include "rasterizer.h"

struct ShaderPayload {
    Eigen::Vector3f view_pos;   // 顶点坐标（世界空间）
    Eigen::Vector3f normal;     // 世界空间法线
    Eigen::Vector3f color;      // 纹理/顶点颜色
    Eigen::Vector2f tex_coords; // 纹理坐标
    TGAImage* texture = nullptr;
};

struct triangle_world {
    std::vector<Eigen::Vector4f> v_world;
    std::vector<Eigen::Vector3f> n_world;
};

class Shader {
public:
    // 光照参数配置
    Eigen::Vector3f light_dir;             // 光源方向（世界空间）
    Eigen::Vector3f light_intensity;       // 光强度
    Eigen::Vector3f amb_light_intensity;   // 环境光

    Eigen::Vector3f ka = Eigen::Vector3f(0.023f, 0.023f, 0.023f);
    Eigen::Vector3f kd = Eigen::Vector3f(1.0f, 1.0f, 1.0f);
    Eigen::Vector3f ks = Eigen::Vector3f(0.5f, 0.5f, 0.5f);
    float shininess = 90.0f;

    // 构造
    Shader() {}

    // 着色函数，输入 Payload，返回颜色
    Eigen::Vector3f phong(const ShaderPayload& payload) const {
        Eigen::Vector3f n = payload.normal.normalized();
        Eigen::Vector3f l = light_dir.normalized();
        Eigen::Vector3f v = (payload.view_pos).normalized();
        Eigen::Vector3f h = (v + l).normalized();

        float distance2 = (light_dir - payload.view_pos).squaredNorm();
        Eigen::Vector3f ambient = ka.cwiseProduct(amb_light_intensity);

        float n_dot_l = std::max(0.0f, n.dot(l));
        Eigen::Vector3f diffuse = kd.cwiseProduct(light_intensity / distance2) * n_dot_l;

        float n_dot_h = std::max(0.0f, n.dot(h));
        Eigen::Vector3f specular = ks.cwiseProduct(light_intensity / distance2) * std::pow(n_dot_h, shininess);

        Eigen::Vector3f result = ambient + diffuse + specular;

        // 与纹理色混合（可选）
        if (payload.texture) {
            auto color = payload.texture->get(
                std::clamp(static_cast<int>(payload.tex_coords.x() * payload.texture->get_width()), 0, payload.texture->get_width() - 1),
                std::clamp(static_cast<int>((1 - payload.tex_coords.y()) * payload.texture->get_height()), 0, payload.texture->get_height() - 1)
            );
            Eigen::Vector3f tex_color(color.raw[2], color.raw[1], color.raw[0]); // BGR
            result = result.cwiseProduct(tex_color / 255.0f);
        }

        return result;
    }
};
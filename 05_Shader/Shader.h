#pragma once
#include "rasterizer.h"

struct ShaderPayload {
    Eigen::Vector3f view_pos;   // �������꣨����ռ䣩
    Eigen::Vector3f normal;     // ����ռ䷨��
    Eigen::Vector3f color;      // ����/������ɫ
    Eigen::Vector2f tex_coords; // ��������
    TGAImage* texture = nullptr;
};

struct triangle_world {
    std::vector<Eigen::Vector4f> v_world;
    std::vector<Eigen::Vector3f> n_world;
};

class Shader {
public:
    // ���ղ�������
    Eigen::Vector3f light_dir;             // ��Դ��������ռ䣩
    Eigen::Vector3f light_intensity;       // ��ǿ��
    Eigen::Vector3f amb_light_intensity;   // ������

    Eigen::Vector3f ka = Eigen::Vector3f(0.023f, 0.023f, 0.023f);
    Eigen::Vector3f kd = Eigen::Vector3f(1.0f, 1.0f, 1.0f);
    Eigen::Vector3f ks = Eigen::Vector3f(0.5f, 0.5f, 0.5f);
    float shininess = 90.0f;

    // ����
    Shader() {}

    // ��ɫ���������� Payload��������ɫ
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

        // ������ɫ��ϣ���ѡ��
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
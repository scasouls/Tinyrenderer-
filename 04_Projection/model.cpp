#include <string>
#include <fstream>
#include <sstream>
#include "model.h"

Model::Model(const char* filename) {
	std::fstream in;
	in.open(filename, std::ios::in);
	if (!in.is_open()){
		throw std::runtime_error("��obj�ļ�ʧ��");
	}
	std::string line;
	while (std::getline(in, line)) {
		if (line.empty()) continue;//��������
		std::string type;
		std::stringstream iss(line);
		iss >> type;//�����λ�ַ��ж϶�ȡ������
		if (type == "v") {
			Vec3f vert_val;
			for (int i = 0; i < 3; ++i) iss >> vert_val.raw[i];
			verts.push_back(vert_val);
		}
		else if (type == "vn") {
			Vec3f normal_val;
			for (int i = 0; i < 3; ++i) iss >> normal_val.raw[i];
			normals.push_back(normal_val);
		}
		else if (type == "vt") {
			Vec2f texture_val;
			for (int i = 0; i < 2; ++i) iss >> texture_val.raw[i];
			textures.push_back(texture_val);
		}
		else if (type == "f") {
			std::vector<Vec3f> face_val;
			char slash;
			int v, uv, nor;
			while (iss >> v >> slash >> uv >> slash >> nor) {
				v--;
				uv--;
				nor--;
				Vec3f index(v, uv, nor);

				face_val.push_back(index);//������1��ʼ���淶Ϊ����Ĵ��㿪ʼ
			}
			faces.push_back(face_val);
		}
	}
}

Model::~Model() {}


int Model::nverts() const{
	return (int)verts.size();
}

int Model::nfaces() const  {
	return (int)faces.size();
}

Vec3f Model::normal(int i) const {
	return normals[i];
}

Vec2f Model::texture(int i) const {
	return textures[i];
}

Vec3f Model::vert(int i)const {
	return verts[i];
}

std::vector<Vec3f> Model::face(int idx) const{
	return faces[idx];
}
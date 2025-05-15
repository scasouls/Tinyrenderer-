#include <string>
#include <fstream>
#include <sstream>
#include "model.h"

Model::Model(const char* filename) {
	std::fstream in;
	in.open(filename, std::ios::in);
	if (!in.is_open()){
		throw std::runtime_error("打开obj文件失败");
	}
	std::string line;
	while (std::getline(in, line)) {
		if (line.empty()) continue;//空行跳过
		std::string type;
		std::stringstream iss(line);
		iss >> type;//检测首位字符判断读取的数据
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

				face_val.push_back(index);//索引从1开始，规范为数组的从零开始
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
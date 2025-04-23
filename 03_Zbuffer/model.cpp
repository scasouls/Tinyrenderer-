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
		else if (type == "f") {
			std::vector<int> face_val;
			char slash;
			int index, uv, nor;
			while (iss >> index >> slash >> uv >> slash >> nor) {
				index--;
				face_val.push_back(index);//索引从1开始，规范为数组的从零开始
			}
			faces.push_back(face_val);
		}
	}
}

Model::~Model() {}

void Model::normalized() {
	float min_x = std::numeric_limits<float>::max(), max_x = std::numeric_limits<float>::lowest();
	float min_y = min_x, max_y = max_x;
	float min_z = min_x, max_z = max_x;

	//得出顶点的包围盒范围
	for (int i = 0; i < verts.size(); ++i) {
		Vec3f v = verts[i];
		min_x = std::min(min_x, v.x);
		max_x = std::max(max_x, v.x);
		min_y = std::min(min_y, v.y);
		max_y = std::max(max_y, v.y);
		min_z = std::min(min_z, v.z);
		max_z = std::max(max_z, v.z);
	}

	//将x，y归一化到-1~1，将z归一化到0~1
	for (int i = 0; i < verts.size(); ++i) {
		Vec3f v = verts[i];
		verts[i].x = (v.x - min_x) / (max_x - min_x) * 2. - 1.;
		verts[i].y = (v.y - min_y) / (max_y - min_y) * 2. - 1.;
		verts[i].z = (v.z - min_z) / (max_z - min_z);
	}

	return;
}

int Model::nverts() const{
	return (int)verts.size();
}

int Model::nfaces() const  {
	return (int)faces.size();
}

Vec3f Model::vert(int i)const {
	return verts[i];
}

std::vector<int> Model::face(int idx) const{
	return faces[idx];
}
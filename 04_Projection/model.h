#pragma once
#include <vector>
#include "geometry.h"
#include <Eigen/Eigen>

class Model {
private:
	std::vector<Vec3f> verts;//顶点
	std::vector<Vec3f> normals;
	std::vector<Vec2f> textures;
	std::vector<std::vector<Vec3f>> faces;//面

public:
	Model(const char* filename);

	~Model();

	//获得顶点和面的数量
	int nverts() const ;
	int nfaces() const;

	//获得数据
	Vec3f vert(int i) const;
	Vec3f normal(int i) const;
	Vec2f texture(int i) const;

	std::vector<Vec3f> face(int idx)const;

};


#pragma once
#include <vector>
#include "geometry.h"

class Model {
private:
	std::vector<Vec3f> verts;//顶点
	std::vector<std::vector<int>> faces;//面

public:
	Model(const char* filename);

	~Model();

	//获得顶点和面的数量
	int nverts() const ;
	int nfaces() const;

	//获得数据
	Vec3f vert(int i) const;

	std::vector<int> face(int idx)const;


};


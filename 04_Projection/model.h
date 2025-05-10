#pragma once
#include <vector>
#include "geometry.h"
#include <Eigen/Eigen>

class Model {
private:
	std::vector<Vec3f> verts;//����
	std::vector<Vec3f> normals;
	std::vector<Vec2f> textures;
	std::vector<std::vector<Vec3f>> faces;//��

public:
	Model(const char* filename);

	~Model();

	//��ö�����������
	int nverts() const ;
	int nfaces() const;

	//�������
	Vec3f vert(int i) const;
	Vec3f normal(int i) const;
	Vec2f texture(int i) const;

	std::vector<Vec3f> face(int idx)const;

};


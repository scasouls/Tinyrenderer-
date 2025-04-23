#pragma once
#include <vector>
#include "geometry.h"

class Model {
private:
	std::vector<Vec3f> verts;//����
	std::vector<std::vector<int>> faces;//��

public:
	Model(const char* filename);

	~Model();

	//��ö�����������
	int nverts() const ;
	int nfaces() const;

	//�������
	Vec3f vert(int i) const;
	
	void normalized();//�Զ������ݽ��й�һ������

	std::vector<int> face(int idx)const;




};


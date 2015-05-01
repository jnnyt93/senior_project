#pragma once
#include "glm.hpp"
#include <vector>
#include <string>
#include <iostream>
#include <fstream>

using namespace std;

class ObjLoader
{
public:
	ObjLoader(void);
	~ObjLoader(void);
	bool loadOBJ(const char * path);

	vector<glm::vec3> vertices_;        // vertex buffer
	vector<glm::vec3> normals_;         // normal buffer
	vector<glm::vec3> colors_;          // color buffer
	vector<unsigned int> indices_;      // index buffer
};


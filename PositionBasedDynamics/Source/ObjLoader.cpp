#include "ObjLoader.h"


ObjLoader::ObjLoader(void)
{
}


ObjLoader::~ObjLoader(void)
{
}

bool ObjLoader::loadOBJ(const char * path) {

	vector<unsigned int> tindices_, temp_indices, normal_indices;
	vector<glm::vec3> temp_vertices, out_vertices, temp_normals, out_normals;

	glm::vec3 white = glm::vec3(1.f, 1.f, 1.f);
	unsigned int numTriangles = 0;
	ifstream myfile (path);
	string word;

	if (myfile.is_open())
	{
		while ( myfile >> word )
		{
			if (word.compare("v") == 0)	{
				myfile >> word;
				float x = stof(word);
				myfile >> word;
				float y = stof(word);
				myfile >> word;
				float z = stof(word);
				temp_vertices.push_back(glm::vec3(x,y,z));
			}
			if (word.compare("vn") == 0) {
				myfile >> word;
				float x = stof(word);
				myfile >> word;
				float y = stof(word);
				myfile >> word;
				float z = stof(word);
				temp_normals.push_back(glm::vec3(x,y,z));
			}
			if (word.compare("f") == 0) {
				numTriangles++;

				myfile >> word;
				std::size_t slash = word.find("/");
				std::size_t slash2 = word.rfind("/");
				string cut = word.substr(0,slash);
				temp_indices.push_back(stoi(cut)-1);
				if (slash != slash2) {
					string cut2 = word.substr(slash2+1);
					normal_indices.push_back(stoi(cut2)-1);
				}

				myfile >> word;
				slash = word.find("/");
				slash2 = word.rfind("/");
				cut = word.substr(0,slash);
				temp_indices.push_back(stoi(cut)-1);
				if (slash != slash2) {
					string cut2 = word.substr(slash2+1);
					normal_indices.push_back(stoi(cut2)-1);
				}

				myfile >> word;
				slash = word.find("/");
				slash2 = word.rfind("/");
				cut = word.substr(0,slash);
				temp_indices.push_back(stoi(cut)-1);
				if (slash != slash2) {
					string cut2 = word.substr(slash2+1);
					normal_indices.push_back(stoi(cut2)-1);
				}

			}
		}
		myfile.close();
	} else {
		printf("Failed to open .obj file!\n");
	}

	for(unsigned int i = 0; i<temp_indices.size(); i++ ){
		unsigned int vertexIndex = temp_indices[i];
		glm::vec3 vertex = temp_vertices[ vertexIndex ];
		vertices_.push_back(vertex);
		colors_.push_back(white);
		indices_.push_back(i);
	}

	// Set up the normals
	if (normal_indices.size() > 0) {
		for(unsigned int i = 0; i < normal_indices.size(); i++) {
			unsigned int normalIndex = normal_indices[i];
			glm::vec3 normal = temp_normals[normalIndex];
			normals_.push_back(normal);
		}
	}
	return 0;	
}
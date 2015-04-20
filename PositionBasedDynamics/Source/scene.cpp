// Cloth Simulation using Position Based Dynamics
// Courtesy of Aline Normoyle
// Copyright 2013 Xing Du

#include "scene.h"
#include <cassert>

//----------Scene Class----------//
Scene::Scene(const char* file_name)
{
    load_from_file(file_name);
}

Scene::~Scene()
{
    for(std::vector<Primitive*>::iterator iter = m_primitives.begin(); iter != m_primitives.end(); ++iter)
    {
        delete (*iter);
    }
    m_primitives.clear();
}

void Scene::load_from_file(const char* file_name)
{
    TiXmlDocument xml_file(file_name);
    if(xml_file.LoadFile())
    {
        XMLSceneVisitor visitor(this);
        xml_file.Accept(&visitor);
    }
}

void Scene::draw(const VBO& vbos)
{
    for(std::vector<Primitive*>::iterator iter = m_primitives.begin(); iter != m_primitives.end(); ++iter)
    {
        (*iter)->draw(vbos);
    }
}

void Scene::insert_primitive(Primitive* const new_primitive)
{
    m_primitives.push_back(new_primitive);
}

bool Scene::line_intersection(const glm::vec3& p1, const glm::vec3& p2, float threshold, glm::vec3& intersect, glm::vec3& normal) const
{// assume no intersection between primitives. i.e. a line intersects at most one primitive.
    for(std::vector<Primitive*>::const_iterator iter = m_primitives.begin(); iter != m_primitives.end(); ++iter)
    {
        if((*iter)->line_intersection(p1, p2, threshold, intersect, normal))
            return true;
    }
    return false;
}

//----------Plane Class----------//
void Scene::Plane::init_visualization()
{
    m_positions.clear();
    m_colors.clear();
    m_normals.clear();
    m_indices.clear();

    glm::vec3 mat_color(0.6f);

	m_positions.push_back(glm::vec3(20,0,20));
	m_positions.push_back(glm::vec3(20,0,-20));
	m_positions.push_back(glm::vec3(-20,0,20));
	m_positions.push_back(glm::vec3(-20,0,-20));

	for (unsigned int i = 0; i < m_positions.size(); i++) {
		m_normals.push_back(m_normal);
		m_colors.push_back(mat_color);
	}
	m_indices.push_back(0);
	m_indices.push_back(1);
	m_indices.push_back(2);
	m_indices.push_back(1);
	m_indices.push_back(2);
	m_indices.push_back(3);
}

void Scene::Plane::draw(const VBO& vbos) const
{
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    // position
    glBindBuffer(GL_ARRAY_BUFFER, vbos.m_vbo);
    glBufferData(GL_ARRAY_BUFFER, 3 * m_positions.size() * sizeof(float), &m_positions[0], GL_STREAM_DRAW);

    // color
    glBindBuffer(GL_ARRAY_BUFFER, vbos.m_cbo);
    glBufferData(GL_ARRAY_BUFFER, 3 * m_colors.size() * sizeof(float), &m_colors[0], GL_STREAM_DRAW);

    // normal
    glBindBuffer(GL_ARRAY_BUFFER, vbos.m_nbo);
    glBufferData(GL_ARRAY_BUFFER, 3 * m_normals.size() * sizeof(float), &m_normals[0], GL_STREAM_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vbos.m_ibo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, m_indices.size() * sizeof(unsigned short), &m_indices[0], GL_STATIC_DRAW);

    glEnableVertexAttribArray(0);
    glEnableVertexAttribArray(1);
    glEnableVertexAttribArray(2);

    glBindBuffer(GL_ARRAY_BUFFER, vbos.m_vbo);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);

    glBindBuffer(GL_ARRAY_BUFFER, vbos.m_cbo);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, 0);

    glBindBuffer(GL_ARRAY_BUFFER, vbos.m_nbo);
    glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 0, 0);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vbos.m_ibo);
    glDrawElements(GL_TRIANGLES, m_indices.size(), GL_UNSIGNED_SHORT, 0);//GL_UNSIGNED_INT

    glDisableVertexAttribArray(0);
    glDisableVertexAttribArray(1);
    glDisableVertexAttribArray(2);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}

bool Scene::Plane::line_intersection(const glm::vec3& p1, const glm::vec3& p2, float threshold, glm::vec3& intersect, glm::vec3& normal) const
{
    float v1, v2;
    v1 = glm::dot(p1, m_normal) - m_value;
    v2 = glm::dot(p2, m_normal) - m_value;
    if(v2 < threshold)
    {
        normal = m_normal;
        if(v1 >= threshold)
        {// continuous collision handling.
            intersect = ((v1 - threshold) * p2 - (v2 - threshold) * p1) / (v1 - v2);
        }
        else
        {// static collision handling.
            intersect = p2 - (v2 - threshold) * normal;
        }
        return true;
    }
    else
        return false;
}

//----------Box Class----------//
void Scene::Box::init_visualization()
{
    m_positions.clear();
    m_colors.clear();
    m_normals.clear();
    m_indices.clear();

    glm::vec3 mat_color(0.8f);
	unsigned int num_faces = 4;
	unsigned int num_verts_per_face = 4;
	unsigned int num_verts_per_tri = 3;
	unsigned int num_tris_per_face = 2;

	dim = glm::vec3(2,2,6);
	center = glm::vec3(0,0,0);

	// bottom
	m_positions.push_back(glm::vec3(+dim.x,0,+dim.z));
	m_positions.push_back(glm::vec3(+dim.x,0,-dim.z));
	m_positions.push_back(glm::vec3(-dim.x,0,+dim.z));
	m_positions.push_back(glm::vec3(-dim.x,0,-dim.z));

	// right
	m_positions.push_back(glm::vec3(+dim.x,+dim.y,-dim.z));
	m_positions.push_back(glm::vec3(+dim.x,0,-dim.z));
	m_positions.push_back(glm::vec3(-dim.x,+dim.y,-dim.z));
	m_positions.push_back(glm::vec3(-dim.x,0,-dim.z));

	// left
	m_positions.push_back(glm::vec3(+dim.x,+dim.y,+dim.z));
	m_positions.push_back(glm::vec3(+dim.x,0,+dim.z));
	m_positions.push_back(glm::vec3(-dim.x,+dim.y,+dim.z));
	m_positions.push_back(glm::vec3(-dim.x,0,+dim.z));

	// back
	m_positions.push_back(glm::vec3(-dim.x,+dim.y,+dim.z));
	m_positions.push_back(glm::vec3(-dim.x,+dim.y,-dim.z));
	m_positions.push_back(glm::vec3(-dim.x,0,+dim.z));
	m_positions.push_back(glm::vec3(-dim.x,0,-dim.z));

	// front
	//m_positions.push_back(glm::vec3(dim.x,+dim.y,+dim.z));
	//m_positions.push_back(glm::vec3(dim.x,+dim.y,-dim.z));
	//m_positions.push_back(glm::vec3(dim.x,0,+dim.z));
	//m_positions.push_back(glm::vec3(dim.x,0,-dim.z));

	std::vector<glm::vec3> temp_normals;
	temp_normals.push_back(glm::vec3(0,1,0));
	temp_normals.push_back(glm::vec3(0,0,1));
	temp_normals.push_back(glm::vec3(0,0,-1));
	temp_normals.push_back(glm::vec3(1,0,0));
	//temp_normals.push_back(glm::vec3(-1,0,0));

	for (int i = 0; i < num_faces; i++) {
		for (int j = 0; j < num_verts_per_face; j++) {
			m_normals.push_back(temp_normals[i]);
			m_colors.push_back(mat_color);
		}
	}

	unsigned int index = 0;
	for (int k = 0; k < num_faces; k++) {
		for (int i = 0; i < num_tris_per_face; i++) {
			for (int j = 0; j < num_verts_per_tri; j++) {
				m_indices.push_back(i+j+index);
			}
		}
		index = index + 4;
	}

}

void Scene::Box::draw(const VBO& vbos) const
{
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    // position
    glBindBuffer(GL_ARRAY_BUFFER, vbos.m_vbo);
    glBufferData(GL_ARRAY_BUFFER, 3 * m_positions.size() * sizeof(float), &m_positions[0], GL_STREAM_DRAW);

    // color
    glBindBuffer(GL_ARRAY_BUFFER, vbos.m_cbo);
    glBufferData(GL_ARRAY_BUFFER, 3 * m_colors.size() * sizeof(float), &m_colors[0], GL_STREAM_DRAW);

    // normal
    glBindBuffer(GL_ARRAY_BUFFER, vbos.m_nbo);
    glBufferData(GL_ARRAY_BUFFER, 3 * m_normals.size() * sizeof(float), &m_normals[0], GL_STREAM_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vbos.m_ibo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, m_indices.size() * sizeof(unsigned short), &m_indices[0], GL_STATIC_DRAW);

    glEnableVertexAttribArray(0);
    glEnableVertexAttribArray(1);
    glEnableVertexAttribArray(2);

    glBindBuffer(GL_ARRAY_BUFFER, vbos.m_vbo);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);

    glBindBuffer(GL_ARRAY_BUFFER, vbos.m_cbo);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, 0);

    glBindBuffer(GL_ARRAY_BUFFER, vbos.m_nbo);
    glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 0, 0);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vbos.m_ibo);
    glDrawElements(GL_TRIANGLES, m_indices.size(), GL_UNSIGNED_SHORT, 0);//GL_UNSIGNED_INT

    glDisableVertexAttribArray(0);
    glDisableVertexAttribArray(1);
    glDisableVertexAttribArray(2);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}

bool Scene::Box::point_intersection(glm::vec3 p) {
	
	return false;
}


//----------Sphere Class----------//
void Scene::Sphere::init_visualization()
{
    m_positions.clear();
    m_colors.clear();
    m_normals.clear();
    m_indices.clear();

    glm::vec3 mat_color(0.6f);
    unsigned int slice = 24, stack = 10;

    glm::vec3 tnormal(0.0f, 1.0f, 0.0f), tpos;
	tpos = m_center + m_radius * tnormal;

    m_positions.push_back(tpos);
    m_normals.push_back(tnormal);
    m_colors.push_back(mat_color);

	float theta_z, theta_y, sin_z;
    float delta_y = 360.0f / slice, delta_z = 180.0f / stack;
	//loop over the sphere
	for(theta_z = delta_z; theta_z < 179.99f; theta_z += delta_z)
	{
		for(theta_y = 0.0f; theta_y < 359.99f; theta_y += delta_y)
		{
			sin_z = sin(glm::radians(theta_z));
			
            tnormal.x = sin_z * cos(glm::radians(theta_y));
			tnormal.y = cos(glm::radians(theta_z));
			tnormal.z = -sin_z * sin(glm::radians(theta_y));

			tpos = m_center + m_radius * tnormal;

            m_positions.push_back(tpos);
            m_normals.push_back(tnormal);
            m_colors.push_back(mat_color);
		}
	}
	tnormal = glm::vec3(0.0f, -1.0f, 0.0f);
    tpos = m_center + m_radius * tnormal;

    m_positions.push_back(tpos);
    m_normals.push_back(tnormal);
    m_colors.push_back(mat_color);

	//indices
	unsigned int j = 0, k = 0;
	for(j = 0; j < slice - 1; ++j)
	{
		m_indices.push_back(0);
		m_indices.push_back(j + 1);
		m_indices.push_back(j + 2);
	}
	m_indices.push_back(0);
	m_indices.push_back(slice);
	m_indices.push_back(1);

	for(j = 0; j < stack - 2; ++j)
	{
		for(k = 1 + slice * j; k < slice * (j + 1); ++k)
		{
			m_indices.push_back(k);
			m_indices.push_back(k + slice);
			m_indices.push_back(k + slice + 1);

			m_indices.push_back(k);
			m_indices.push_back(k + slice + 1);
			m_indices.push_back(k + 1);
		}
		m_indices.push_back(k);
		m_indices.push_back(k + slice);
		m_indices.push_back(k + 1);

		m_indices.push_back(k);
		m_indices.push_back(k + 1);
		m_indices.push_back(k + 1 - slice);
	}

    unsigned int bottom_id = (stack - 1) * slice + 1;
    unsigned int offset = bottom_id - slice;
	for(j = 0; j < slice - 1; ++j)
	{
		m_indices.push_back(j + offset);
		m_indices.push_back(bottom_id);
		m_indices.push_back(j + offset + 1);
	}
	m_indices.push_back(bottom_id - 1);
	m_indices.push_back(bottom_id);
	m_indices.push_back(offset);

	if(m_indices.size() != 6 * (stack - 1) * slice)
		printf("indices number not correct!\n");
}

void Scene::Sphere::draw(const VBO& vbos) const
{
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);                                                                                               
    // position
    glBindBuffer(GL_ARRAY_BUFFER, vbos.m_vbo);
    glBufferData(GL_ARRAY_BUFFER, 3 * m_positions.size() * sizeof(float), &m_positions[0], GL_STREAM_DRAW);

    // color
    glBindBuffer(GL_ARRAY_BUFFER, vbos.m_cbo);
    glBufferData(GL_ARRAY_BUFFER, 3 * m_colors.size() * sizeof(float), &m_colors[0], GL_STREAM_DRAW);

    // normal
    glBindBuffer(GL_ARRAY_BUFFER, vbos.m_nbo);
    glBufferData(GL_ARRAY_BUFFER, 3 * m_normals.size() * sizeof(float), &m_normals[0], GL_STREAM_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vbos.m_ibo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, m_indices.size() * sizeof(unsigned short), &m_indices[0], GL_STATIC_DRAW);

    glEnableVertexAttribArray(0);
    glEnableVertexAttribArray(1);
    glEnableVertexAttribArray(2);

    glBindBuffer(GL_ARRAY_BUFFER, vbos.m_vbo);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);

    glBindBuffer(GL_ARRAY_BUFFER, vbos.m_cbo);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, 0);

    glBindBuffer(GL_ARRAY_BUFFER, vbos.m_nbo);
    glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 0, 0);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vbos.m_ibo);
    glDrawElements(GL_TRIANGLES, m_indices.size(), GL_UNSIGNED_SHORT, 0);//GL_UNSIGNED_INT

    glDisableVertexAttribArray(0);
    glDisableVertexAttribArray(1);
    glDisableVertexAttribArray(2);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}

bool Scene::Sphere::line_intersection(const glm::vec3& p1, const glm::vec3& p2, float threshold, glm::vec3& intersect, glm::vec3& normal) const
{// TODO: implement line-sphere intersection. you can refer to line-plane intersection.

	float v1, v2;// v1 v2 are distance to sphere for p1 and p2.
    glm::vec3 center = m_center;
	float radius = m_radius;

	v1 = glm::length(p1 - center) - radius;
    v2 = glm::length(p2 - center) - radius;

    if(v2 < threshold)
    {
        if(v1 >= threshold)
        {// continuous collision handling.
			float r = radius + threshold;
			glm::vec3 l = glm::normalize(p2-p1);
			glm::vec3 OMinusC = p1-center;
			float loc = glm::dot(l, OMinusC);
			float d = -loc - glm::sqrt(loc*loc - glm::dot(OMinusC, OMinusC) + r*r);
			intersect = l*d + p1;
            normal = glm::normalize(intersect-center);
        }
        else
        {// static collision handling.
			normal = glm::normalize(p2-center);            
			intersect = p2 - (v2-threshold)*normal;
        }
        return true;
    }
    else
        return false;
}

//----------Scene Visitor Class----------//
XMLSceneVisitor::XMLSceneVisitor(Scene* const scene) : 
    TiXmlVisitor(),
    m_scene(scene),
    m_current(NULL)
{
    ;
}

XMLSceneVisitor::XMLSceneVisitor(const XMLSceneVisitor& other) : 
    TiXmlVisitor(other),
    m_scene(other.m_scene),
    m_current(other.m_current)
{
    ;
}

XMLSceneVisitor::~XMLSceneVisitor()
{
    ;
}

bool XMLSceneVisitor::VisitEnter(const TiXmlElement& element, const TiXmlAttribute* attribute)
{
    if(element.ValueStr() == "scene")
    {
        return (element.Parent() == element.GetDocument());
    }
    else if (element.ValueStr() == "materials")
    {
        return true;
    }
    else if (element.ValueStr() == "primitives")
    {
        return (element.Parent()->ValueStr() == "scene");
    }
    else if (element.ValueStr() == "plane")
    {
        if(element.Parent()->ValueStr() != "primitives")
            return false;
        assert(m_current == NULL);

        double nx(0.0f), ny(1.0f), nz(0.0f), value(0.0f);

        element.Attribute("nx", &nx);
        element.Attribute("ny", &ny);
        element.Attribute("nz", &nz);
        element.Attribute("value", &value);

        glm::vec3 normal(nx, ny, nz);
        normal = glm::normalize(normal);

        m_current = new Scene::Plane(normal, value);
        return true;
    }
	else if (element.ValueStr() == "box")
    {
        if(element.Parent()->ValueStr() != "primitives")
            return false;
        assert(m_current == NULL);

        m_current = new Scene::Box();
        return true;
    }
    else if (element.ValueStr() == "sphere")
    {
        if(element.Parent()->ValueStr() != "primitives")
            return false;
        assert(m_current == NULL);

        double cx(0.0f), cy(1.0f), cz(0.0f), radius(0.0f);

        element.Attribute("cx", &cx);
        element.Attribute("cy", &cy);
        element.Attribute("cz", &cz);
        element.Attribute("radius", &radius);

        glm::vec3 center(cx, cy, cz);
        m_current = new Scene::Sphere(center, radius);
        return true;
    }
    else
        return false;
}

bool XMLSceneVisitor::VisitExit( const TiXmlElement& element)
{
    if(element.ValueStr() == "scene")
    {
        return (element.Parent() == element.GetDocument());
    }
    else if (element.ValueStr() == "materials")
    {
        return true;
    }
    else if (element.ValueStr() == "primitives")
    {
        return true;
    }
    else if (element.ValueStr() == "plane")
    {
        m_scene->insert_primitive(m_current);
        m_current = NULL;
        return true;
    }
	else if (element.ValueStr() == "box")
    {
        m_scene->insert_primitive(m_current);
        m_current = NULL;
        return true;
    }
    else if (element.ValueStr() == "sphere")
    {
        m_scene->insert_primitive(m_current);
        m_current = NULL;
        return true;
    }
    else
        return false;
}
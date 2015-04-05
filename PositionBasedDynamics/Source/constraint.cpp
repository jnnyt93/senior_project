// Cloth Simulation using Position Based Dynamics
// Copyright 2013 Xing Du

#include "constraint.h"
#include <cassert>

#ifndef EPSILON
#define EPSILON 0.00001f
#define M_PI 3.14159265358979323846  /* pi */
#endif

//----------Constraint Class----------//
Constraint::Constraint() : 
    m_vertices(NULL),
    m_stiffness(1.0f)
{
   ;
}

Constraint::Constraint(ParticleList *verts, float stiff) : 
    m_vertices(verts),
    m_stiffness(stiff)
{
    ;
}

Constraint::Constraint(const Constraint& other) : 
    m_vertices(other.m_vertices),
    m_stiffness(other.m_stiffness)
{
    ;
}

Constraint::~Constraint()
{
    m_vertices = NULL;
}

bool Constraint::project_constraint()
{
    return true;
}

//----------DensityConstraint Class----------//
DensityConstraint::DensityConstraint() : 
    Constraint()
{
    ;
}

DensityConstraint::DensityConstraint(ParticleList *verts, std::vector<std::vector<unsigned int>>* neighbors, std::vector<float> lambdas, unsigned int p0, float h) : 
    Constraint(verts, 1.0f),
	m_neighbors(neighbors),
	m_lambdas(lambdas),
	m_p0(p0),
	rest_density(1000.0f),
	h(h)
{
    ;
}

DensityConstraint::DensityConstraint(const DensityConstraint& other) : 
    Constraint(other)
{
    ;
}

DensityConstraint::~DensityConstraint()
{
    ;
}

glm::vec3 W_spiky(glm::vec3 r, float h) {
	if (r.x == 0 && r.y == 0 && r.z == 0) return glm::vec3(0,0,0);
	//if (r.x < 0) r.x = -r.x;
	//if (r.y < 0) r.y = -r.y;
	//if (r.z < 0) r.z = -r.z;
	float a = 45.0f / (M_PI * glm::pow(h, 6.f));
	float b = glm::pow(h - glm::length(r), 2.0f);
	glm::vec3 c = r / glm::length(r);
	return a * b * c;
}

bool DensityConstraint::project_constraint()
{// TODO: implement the project function for FixedPointConstraint.
    //return true if current position is OK. return false if the position is being projected

	m_vertices->lock_pos(m_p0);
	//return true;
	glm::vec3 dp0 = glm::vec3(0.0f);
	std::vector<unsigned int> neighbors_of_p0 = m_neighbors->at(m_p0);
	for (int j = 0; j < neighbors_of_p0.size(); j++) {
		unsigned int neighbor_index = neighbors_of_p0[j];
		glm::vec3 pi = m_vertices->pos(m_p0);
		glm::vec3 pj = m_vertices->pos(neighbor_index);
		glm::vec3 w = W_spiky(pi - pj, h);
		dp0 += (m_lambdas.at(m_p0) + m_lambdas.at(neighbor_index)) * w;
	}
	dp0 *= 1.0f / rest_density;
	m_vertices->predicted_pos(m_p0) += dp0 * m_stiffness;

	return false;
}

//----------CollisionConstraint Class----------//
CollisionConstraint::CollisionConstraint() : 
    Constraint()
{
    ;
}

CollisionConstraint::CollisionConstraint(ParticleList *verts, unsigned int p0, const glm::vec3& q, const glm::vec3& n) : 
    Constraint(verts, 1.0f),
    m_p0(p0),
    m_ref_point(q),
    m_normal(n)
{
    ;
}

CollisionConstraint::CollisionConstraint(const CollisionConstraint& other) : 
    Constraint(other),
    m_p0(other.m_p0),
    m_ref_point(other.m_ref_point),
    m_normal(other.m_normal)
{
    ;
}

CollisionConstraint::~CollisionConstraint()
{
    ;
}

bool CollisionConstraint::project_constraint()
{// TODO: implement the project function for CollisionConstraint.
    //return true if current position is OK. return false if the position is being projected.

    glm::vec3 p0 = m_vertices->predicted_pos(m_p0);
    float value = glm::dot(p0-m_ref_point, m_normal);
    if(value > 0.0f)
        return true;

    glm::vec3 dp0 = m_ref_point - p0;
    m_vertices->predicted_pos(m_p0) += dp0 * m_stiffness;

    return false;
}

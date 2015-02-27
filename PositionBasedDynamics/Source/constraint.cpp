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

DensityConstraint::DensityConstraint(ParticleList *verts, std::vector<std::vector<unsigned int>>* neighbors, std::vector<float> lambdas, unsigned int p0) : 
    Constraint(verts, 1.0f),
	m_neighbors(neighbors),
	m_lambdas(lambdas),
	m_p0(p0),
	rest_density(1000.0f),
	h(7.0f)
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

//----------FixedPointConstraint Class----------//
FixedPointConstraint::FixedPointConstraint() : 
    Constraint()
{
    ;
}

FixedPointConstraint::FixedPointConstraint(ParticleList *verts, unsigned int p0, const glm::vec3& fixedpoint) : 
    Constraint(verts, 1.0f),
    m_p0(p0),
    m_fixd_point(fixedpoint)
{
    ;
}

FixedPointConstraint::FixedPointConstraint(const FixedPointConstraint& other) : 
    Constraint(other),
    m_p0(other.m_p0),
    m_fixd_point(other.m_fixd_point)
{
    ;
}

FixedPointConstraint::~FixedPointConstraint()
{
    ;
}

bool FixedPointConstraint::project_constraint()
{// TODO: implement the project function for FixedPointConstraint.
    //return true if current position is OK. return false if the position is being projected.
    m_vertices->lock_pos(m_p0);
    float value = glm::length(m_vertices->pos(m_p0)-m_fixd_point);
    if(value < EPSILON)
        return true;

    glm::vec3 dp0 = m_fixd_point - m_vertices->predicted_pos(m_p0);
    m_vertices->predicted_pos(m_p0) += dp0 * m_stiffness;
    return false;
}

//----------StretchConstraint Class----------//
StretchConstraint::StretchConstraint() : 
    Constraint()
{
    ;
}

StretchConstraint::StretchConstraint(ParticleList *verts, float stiff, unsigned int p1, unsigned int p2, float length) : 
    Constraint(verts, stiff),
    m_p1(p1),
    m_p2(p2),
    m_rest_length(length)
{
    ;
}

StretchConstraint::StretchConstraint(const StretchConstraint& other) : 
    Constraint(other),
    m_p1(other.m_p1),
    m_p2(other.m_p2),
    m_rest_length(other.m_rest_length)
{
    ;
}

StretchConstraint::~StretchConstraint()
{
    ;
}

bool StretchConstraint::project_constraint()
{// TODO: implement the project function for StretchConstraint.
    //return true if current position is OK. return false if the position is being projected.
    glm::vec3 p1, p2;
    p1 = m_vertices->predicted_pos(m_p1);
    p2 = m_vertices->predicted_pos(m_p2);

    float length = glm::length(p1-p2);

    if(fabs(length - m_rest_length) < EPSILON)
        return true;

	float w1 = m_vertices->inv_mass(m_p1);
	float w2 = m_vertices->inv_mass(m_p2);

	float d = m_rest_length;
	float s = (length - d) / (w1 + w2);
	glm::vec3 mag = (p1-p2)/length;
    glm::vec3 dp1 = -1*w1 * s * mag;
	glm::vec3 dp2 = w2 * s * mag;

    m_vertices->predicted_pos(m_p1) += dp1 * m_stiffness;
    m_vertices->predicted_pos(m_p2) += dp2 * m_stiffness;

    return false;
}

//----------BendConstraint Class----------//
BendConstraint::BendConstraint() : 
    Constraint()
{
    ;
}

BendConstraint::BendConstraint(ParticleList *verts, float stiff, unsigned int p1, unsigned int p2, unsigned int p3, unsigned int p4, float phi) : 
    Constraint(verts, stiff),
    m_p1(p1), m_p2(p2), m_p3(p3), m_p4(p4),
    m_phi(phi)
{
    ;
}

BendConstraint::BendConstraint(const BendConstraint& other) : 
    Constraint(other),
    m_p1(other.m_p1), m_p2(other.m_p2), m_p3(other.m_p3), m_p4(other.m_p4),
    m_phi(other.m_phi)
{
    ;
}

BendConstraint::~BendConstraint()
{
    ;
}

bool BendConstraint::project_constraint()
{// TODO: implement the project function for BendConstraint.
    //return true if current position is OK. return false if the position is being projected
	glm::vec3 dp1, dp2, dp3, dp4, q1, q2, q3, q4;
    glm::vec3 p1 = m_vertices->predicted_pos(m_p1),
              p2 = m_vertices->predicted_pos(m_p2),
              p3 = m_vertices->predicted_pos(m_p3),
              p4 = m_vertices->predicted_pos(m_p4);
	glm::vec3 n1 = glm::cross(p2-p1, p3-p1);
	glm::vec3 n2 = glm::cross(p2-p1, p4-p1);
	float d = glm::dot(n1, n2);
	float w1 = m_vertices->inv_mass(m_p1);
	float w2 = m_vertices->inv_mass(m_p2);
	float w3 = m_vertices->inv_mass(m_p3);
	float w4 = m_vertices->inv_mass(m_p4);
	q3 = (glm::cross(p2, n2) + glm::cross(n1, p2)*d) / glm::length(glm::cross(p2, p3));
	q4 = (glm::cross(p2, n1) + glm::cross(n2, p2)*d) / glm::length(glm::cross(p2, p4));
	q2 = -1.f*(glm::cross(p3, n2) + glm::cross(n1, p3)*d) / glm::length(glm::cross(p2, p3));
	q2 = q2 - (glm::cross(p4, n1) + glm::cross(n2, p4)*d) / glm::length(glm::cross(p2, p4));
	q1 = -1.f * q2 - q3 - q4;
    
	float sum = w1*glm::length(q1*q1) +
		w2*glm::length(q1*q2) +
		w3*glm::length(q3*q3) +
		w4*glm::length(q4*q4);

	dp1 = -1*w1*glm::sqrt(1 - d*d) * (glm::acos(d) - m_phi) * q1 / sum;
	dp2 = -1*w2*glm::sqrt(1 - d*d) * (glm::acos(d) - m_phi) * q2 / sum;
	dp3 = -1*w3*glm::sqrt(1 - d*d) * (glm::acos(d) - m_phi) * q3 / sum;
	dp4 = -1*w4*glm::sqrt(1 - d*d) * (glm::acos(d) - m_phi) * q4 / sum;

    m_vertices->predicted_pos(m_p1) += dp1 * m_stiffness;
    m_vertices->predicted_pos(m_p2) += dp2 * m_stiffness;
    m_vertices->predicted_pos(m_p3) += dp3 * m_stiffness;
    m_vertices->predicted_pos(m_p4) += dp4 * m_stiffness;

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

//----------CollisionConstraint Class----------//
SelfCollisionConstraint::SelfCollisionConstraint() : 
    Constraint()
{
    ;
}

SelfCollisionConstraint::SelfCollisionConstraint(ParticleList *verts, unsigned int q, unsigned int p1, unsigned int p2, unsigned int p3, float h) :
    Constraint(verts, 1.0f),
    m_q(q), m_p1(p1), m_p2(p2), m_p3(p3),
    m_h(h)
{
    ;
}
SelfCollisionConstraint::SelfCollisionConstraint(const SelfCollisionConstraint& other) :
    Constraint(other),
    m_q(other.m_q), m_p1(other.m_p1), m_p2(other.m_p2), m_p3(other.m_p3),
    m_h(other.m_h)
{
    ;
}

SelfCollisionConstraint::~SelfCollisionConstraint()
{

}

bool SelfCollisionConstraint::project_constraint()
{
    glm::vec3 q, p1, p2, p3;
    q =  m_vertices->predicted_pos(m_q);
    p1 = m_vertices->predicted_pos(m_p1);
    p2 = m_vertices->predicted_pos(m_p2);
    p3 = m_vertices->predicted_pos(m_p3);

    q = q - p1;
    p2 = p2 - p1;
    p3 = p3 - p1;
    p1 = glm::vec3(0.0f);
    
    glm::vec3 normal(glm::cross(p2, p3));
    float c23 = glm::length(normal);
    normal = glm::normalize(normal);

    float value = glm::dot(q, normal) - m_h;
    if(value > 0.0f)
        return true;

    glm::vec3 dcq, dcp1, dcp2, dcp3;
    dcq = normal;
    dcp2 = (glm::cross(p3, q) + glm::cross(normal, p3) * glm::dot(normal, q)) / c23;
    dcp3 = -(glm::cross(p2, q) + glm::cross(normal, p2) * glm::dot(normal, q)) / c23;
    dcp1 = -dcq - dcp2 - dcp3;

    float wq, w1, w2, w3;
    wq = m_vertices->inv_mass(m_q);
    w1 = m_vertices->inv_mass(m_p1);
    w2 = m_vertices->inv_mass(m_p2);
    w3 = m_vertices->inv_mass(m_p3);

    float denominator = w1 * glm::dot(dcp1, dcp1) + w2 * glm::dot(dcp2, dcp2) + w3 * glm::dot(dcp3, dcp3) + wq * glm::dot(dcq, dcq);
    assert(denominator < EPSILON);

    glm::vec3 dq, dp1, dp2, dp3;
    float s = value / denominator;
    dq = -wq * s * dcq;
    dp1 = -w1 * s * dcp1;
    dp2 = -w2 * s * dcp2;
    dp3 = -w3 * s * dcp3;
    
    m_vertices->predicted_pos(m_q) += dq * m_stiffness;
    m_vertices->predicted_pos(m_p1) += dp1 * m_stiffness;
    m_vertices->predicted_pos(m_p2) += dp2 * m_stiffness;
    m_vertices->predicted_pos(m_p3) += dp3 * m_stiffness;
    return false;
}
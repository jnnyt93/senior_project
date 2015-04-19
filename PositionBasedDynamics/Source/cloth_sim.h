// Cloth Simulation using Position Based Dynamics
// Copyright 2013 Xing Du

#ifndef CLOTH_SIM_H_INCLUDED
#define CLOTH_SIM_H_INCLUDED
#define M_PI 3.14159265358979323846  /* pi */

#include "openGL_headers.h"
#include "math_headers.h"
#include "particlelist.h"
#include "constraint.h"
#include "scene.h"
#include <vector>

class ClothSim
{
public:
    ClothSim();
    ClothSim(unsigned int n);
    virtual ~ClothSim();


    void initialize(unsigned int dim_x, unsigned int dim_z, unsigned int dim_y, const glm::vec3& cloth_min, const glm::vec3& cloth_max);
	void uploadPoint();
    void update(const Scene* const scene, float dt);
    void draw(const VBO& vbos);
    void flip_draw_mode()
    {
        m_draw_wire = !m_draw_wire;
    }
	
protected:
    struct Edge
    {
        unsigned int m_v1, m_v2;
        unsigned int m_tri1, m_tri2;
    };
protected:
    unsigned int m_dimx, m_dimy, m_dimz;
    float m_thick;
    unsigned int m_solver_iterations;
    // vertices and estimated position.
    ParticleList m_vertices;
	// neightbors of each of the particles.
	std::vector<std::vector<unsigned int>> m_neighbors;
	std::vector<float> m_lambdas;
	std::vector<float> m_densities;
	std::vector<float> m_C;
	std::vector<float> m_gradC;
	std::vector<bool> contact_box;
    // internal and external constraints.
    std::vector<Constraint*> m_constraints_int;
    std::vector<CollisionConstraint> m_constraints_ext;
    std::vector<SelfCollisionConstraint> m_self_collision;
    //std::vector<Constraint*> m_constraints_ext;
    // for visualize the cloth.
    bool m_draw_wire;
	std::vector<glm::vec3> m_positions;
    std::vector<glm::vec3> m_normals;
    std::vector<glm::vec3> m_colors;
    std::vector<unsigned int> m_indices;
    // for generating constraints.
    std::vector<Edge> m_edge_list;
private:
    // generate edge list from the geometry representation.
    void generate_edge_list();
    // generate all the internal constraints based on the edge list. 
    void generate_internal_constraints();
    // update the normal per frame for visualization.
    void compute_normal();
    // apply external force to the system.
    void apply_external_force(const glm::vec3& force, float dt);
    // damp velocity for all vertices.
    void damp_velocity(float k_damp);
    // compute predicted position based on velocity.
    void compute_predicted_position(float dt);

	// PBF lines 8 to 19
    void project_constraints();
	// resolve all the constraints, both internal and external.
    void resolve_constriants();
    // update the position and velocity.
    void integration(float dt);
    void update_velocity(float dt);
    void clean_collision_constraints();

	void find_neighboring_particles();
	float sph_density_estimator(unsigned int pi);
	void compute_lambda();
	glm::vec3 W_spiky(glm::vec3 r, float h);
	glm::vec3 ClothSim::gradient_C(unsigned int i, unsigned int k);
	void sphere_init_visualization(glm::vec3 m_center);
	float m_radius;

	void resolve_box_collision(float dt);
	void compute_position();
	float h; //smoothing radius
	float rest_density;
	float epsilon;
	float friction;
	float restitution;

};

#endif
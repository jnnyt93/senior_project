// Cloth Simulation using Position Based Dynamics
// Copyright 2013 Xing Du

#include "cloth_sim.h"
#define FOR_EACH_PARTICLE \
	for(int i = 0; i < m_vertices.size(); i++)  \


ClothSim::ClothSim() : 
    m_dimx(0), m_dimy(0), m_dimz(0),
    m_thick(0.05f),
    m_solver_iterations(10),
    m_draw_wire(false)
{
    ;
}

ClothSim::ClothSim(unsigned int n) : 
    m_dimx(0), m_dimy(0), m_dimz(0),
    m_thick(0.05f),
    m_solver_iterations(n),
    m_draw_wire(false)
{
    ;
}

ClothSim::~ClothSim()
{
    m_normals.clear();
    m_colors.clear();

    unsigned int i, size;
    size = m_constraints_int.size();
    for(i = 0; i < size; ++i)
        delete m_constraints_int[i];
    m_constraints_int.clear();

    m_constraints_ext.clear();
    m_self_collision.clear();
}

void ClothSim::initialize(unsigned int dim_x, unsigned int dim_y, unsigned int dim_z, const glm::vec3& cloth_min, const glm::vec3& cloth_max)
{// initialize the cloth here. feel free to create your own initialization.
	h = 1.0f;
	rest_density = 1000.0f;
	epsilon = 100.0f;
	friction = 0.98;
	restitution = 1.5f;
	m_radius = 0.01f;
    m_dimx = dim_x;
	m_dimy = dim_y;
    m_dimz = dim_z;
    glm::vec3 delta;
    delta.x = (cloth_max.x - cloth_min.x) / (float)(m_dimx - 1);
    delta.y = (cloth_max.y - cloth_min.y) / (float)(m_dimy - 1);
    delta.z = (cloth_max.z - cloth_min.z) / (float)(m_dimz - 1);
    // if you want, you can substitute this part using a obj file loader.
	// We'll be dealing with the most simple case, so things are done manually here.
	m_vertices.resize(m_dimx * m_dimy * m_dimz);
	m_normals.resize(m_dimx * m_dimy * m_dimz);
	m_colors.resize(m_dimx * m_dimy * m_dimz);
	m_neighbors.resize(m_dimx * m_dimy * m_dimz);
	m_lambdas.resize(m_dimx * m_dimy * m_dimz);
	// Assign initial position, velocity and mass to all the vertices.
	unsigned int i, k, j, index;
	index = 0;
	float d = 0.2f;
	for(i = 0; i < m_dimx; ++i) {
		for (j = 0; j < dim_y; ++j) {
			for(k = 0; k < m_dimz; ++k) {
				if (j % 2 == 0) d = -d;
				m_vertices.pos(index) = glm::vec3(delta.x * i + cloth_min.x + d, delta.y * j + cloth_min.y + d, delta.z * k + cloth_min.z - d);
				std::cout << m_vertices.pos(index)[0] << ", " << m_vertices.pos(index)[1] << ", " << m_vertices.pos(index)[2] << std::endl;
				m_vertices.vel(index) = glm::vec3(0.0f);
				m_vertices.set_inv_mass(index, 1.0f);
				m_neighbors.at(index) = std::vector<unsigned int>();
				m_lambdas.at(index) = 0.0f;
				index++;
			}
		}
	}



}


void ClothSim::sphere_init_visualization(glm::vec3 m_center)
{

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

void ClothSim::update(const Scene* const scene, float dt)
{// update the system for a certain time step dt.
    glm::vec3 gravity(0.0f, -98.f, 0.0f);
    apply_external_force(gravity, dt);
    compute_predicted_position(dt);
	find_neighboring_particles();
    resolve_constriants();
    integration(dt);
    //update_velocity(dt);
    //clean_collision_constraints();
}

void ClothSim::draw(const VBO& vbos)
{// visualize the cloth on the screen.
	//clear color and depth buffer 
    
	glColor3f(1.0f,1.0f,1.0f); //blue color
	glPointSize(10.0f);//set point size to 10 pixels
	glBegin(GL_POINTS); //starts drawing of points
	for (int i = 0; i < m_vertices.size(); i++) {
		glColor3f(0.0f,0.0f,1.0f); //blue color
		glVertex3f(m_vertices.pos(i)[0],m_vertices.pos(i)[1],m_vertices.pos(i)[2]);//upper-right corner
	}
	glEnd();//end drawing of points

	//glPolygonMode(GL_FRONT_AND_BACK, (m_draw_wire ? GL_LINE : GL_FILL));

 //   unsigned int size = m_vertices.size();
 //   // position
 //   glBindBuffer(GL_ARRAY_BUFFER, vbos.m_vbo);
 //   glBufferData(GL_ARRAY_BUFFER, 3 * size * sizeof(float), &m_vertices.pos(0), GL_DYNAMIC_DRAW);
 //   // color
 //   glBindBuffer(GL_ARRAY_BUFFER, vbos.m_cbo);
 //   glBufferData(GL_ARRAY_BUFFER, 3 * size * sizeof(float), &m_colors[0], GL_STATIC_DRAW);
 //   // normal
 //   glBindBuffer(GL_ARRAY_BUFFER, vbos.m_nbo);
 //   glBufferData(GL_ARRAY_BUFFER, 3 * size * sizeof(float), &m_normals[0], GL_DYNAMIC_DRAW);


 //   glEnableVertexAttribArray(0);
 //   glEnableVertexAttribArray(1);
 //   glEnableVertexAttribArray(2);

 //   glBindBuffer(GL_ARRAY_BUFFER, vbos.m_vbo);
 //   glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);

 //   glBindBuffer(GL_ARRAY_BUFFER, vbos.m_cbo);
 //   glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, 0);

 //   glBindBuffer(GL_ARRAY_BUFFER, vbos.m_nbo);
 //   glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 0, 0);

 //   glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vbos.m_ibo);
 //   glDrawElements(GL_POINTS, 1, GL_UNSIGNED_INT, 0);

 //   glDisableVertexAttribArray(0);
 //   glDisableVertexAttribArray(1);
 //   glDisableVertexAttribArray(2);

 //   glBindBuffer(GL_ARRAY_BUFFER, 0);
 //   glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

}

/* poly6 kernal function */
float W(glm::vec3 r, float h) {
	float frac = 315.0f/ (64.0f * M_PI * glm::pow(h,9.0f));
	float r2 = glm::length(r) * glm::length(r);
	float temp = h*h - r2;
	float temp2 = glm::pow(temp, 3.0f);
	return frac*temp2;
}

glm::vec3 ClothSim::W_spiky(glm::vec3 r, float h) {
	if(glm::length(r)>h)
		return glm::vec3(0.00001,0.00001,0.00001);
	float a = 45.0f / (M_PI * glm::pow(h, 6.f));
	float b = glm::pow(h - glm::length(r), 2.0f);
	glm::vec3 c = r/(glm::length(r) + 0.001f);
	glm::vec3 ret = a * b * c;
	return ret;
}

float ClothSim::sph_density_estimator(unsigned int pi) {
	float density = 0.0f;
	std::vector<unsigned int> neighbors_of_pi = m_neighbors.at(pi);
	for (int j = 0; j < neighbors_of_pi.size(); j++) {
		unsigned int neighbor_index = neighbors_of_pi[j];
		float mass = 1.0f/m_vertices.inv_mass(pi);
		glm::vec3 p_i = m_vertices.pos(pi);
		glm::vec3 p_j = m_vertices.pos(neighbor_index);
		glm::vec3 r = p_i - p_j;
		density += mass * W(r, h);
	}
	return density;
}

glm::vec3 ClothSim::gradient_C(unsigned int i, unsigned int k) {
	glm::vec3 pi = m_vertices.pos(i);
	glm::vec3 pj;
	glm::vec3 gradientC = glm::vec3(0.0f);
	glm::vec3 sum = glm::vec3(0.0f);
	std::vector<unsigned int> neighbors = m_neighbors.at(i);
	for (int j = 0; j < neighbors.size(); j++) {
		unsigned int j_indx = neighbors.at(j);
		pj = m_vertices.pos(j_indx);
		if (k == i) {
			sum += W_spiky(pi-pj, h);
		} else if (k == j) {
			sum -= -1.0f*W_spiky(pi - pj, h);
		}
	}
	gradientC += sum/rest_density;
	return gradientC;
}

void ClothSim::calculate_lambda(unsigned int i) {
	float density = sph_density_estimator(i);
	float Ci = density / rest_density - 1.0f;
	float sumk = 0.0f;
	glm::vec3 grad_c = glm::vec3(0.0f);
	for (int k = 0; k < m_vertices.size(); k++) {
		grad_c = gradient_C(i, k);
		float l = 0;
		if (grad_c != glm::vec3(0,0,0))
			l = glm::length(grad_c);
		sumk += l*l;
	}
	if (sumk == 0.0f) sumk = 1.0f;
	
	m_lambdas.at(i) = Ci/(sumk + epsilon);
}

void ClothSim::generate_internal_constraints()
{// TODO: generate all internal constraints in this function.
	unsigned int i, k, index;

	/* Calculate lambda i (Line 10) */
	for (i = 0; i < m_vertices.size(); i++) {
		calculate_lambda(i);
	}

	for (i = 0; i < m_vertices.size(); i++) {
		float density_i = sph_density_estimator(i);
		Constraint* density_constraint = new DensityConstraint(&m_vertices, &m_neighbors, m_lambdas, i, h);
		m_constraints_int.push_back(density_constraint);
	}

}

void ClothSim::apply_external_force(const glm::vec3& force, float dt)
{// apply external force to cloth.
    unsigned int size = m_vertices.size();
    for(unsigned int i = 0; i < size; ++i)
    {
		m_vertices.vel(i) = m_vertices.vel(i) + dt*m_vertices.inv_mass(i)*force;
    }
}

void ClothSim::damp_velocity(float k_damp)
{
    // TODO: apply damping according to chapter 3.5 in original paper.
	unsigned int i, k, index;
	glm::vec3 sum_xm = glm::vec3(0.f);
	float sum_mass = 0;
	glm::vec3 sum_vm = glm::vec3(0.f);
	for(i = 0; i < m_dimx; ++i) {
		for(k = 0; k < m_dimz; ++k){
			index = m_dimz * i + k;
			sum_xm = sum_xm + m_vertices.pos(index)/m_vertices.inv_mass(index);
			sum_mass = sum_mass + 1.0f/m_vertices.inv_mass(index);
			sum_vm = sum_vm + m_vertices.vel(index)/m_vertices.inv_mass(index);
		}
	}

	glm::vec3 xcm = sum_xm / sum_mass; // (1)
	glm::vec3 vcm = sum_vm / sum_mass; // (2)
	glm::vec3 L = glm::vec3(0.f); // (3)
	glm::mat3 I = glm::mat3(0.f); // (4)
	for(i = 0; i < m_dimx; ++i) {
		for(k = 0; k < m_dimz; ++k){
			index = m_dimz * i + k;
			glm::vec3 ri = m_vertices.pos(index) - xcm;
			glm::vec3 mivi = m_vertices.vel(index)/m_vertices.inv_mass(index);
			L = L + glm::cross(ri, mivi);
			glm::vec3 v1 = glm::vec3(0.0f, ri[2], -ri[1]);
			glm::vec3 v2 = glm::vec3(-ri[2], 0.0f, ri[0]);
			glm::vec3 v3 = glm::vec3(ri[1], -ri[0], 0.0f);
			glm::mat3 r_tilda = glm::mat3(v1,v2,v3);
			I = I + r_tilda * glm::transpose(r_tilda) / m_vertices.inv_mass(index); 
		}
	}
	glm::vec3 w = glm::inverse(I) * L; // (5)
	for(i = 0; i < m_dimx; ++i) {
		for(k = 0; k < m_dimz; ++k){
			index = m_dimz * i + k;
			glm::vec3 ri = m_vertices.pos(index) - xcm;
			glm::vec3 vi = m_vertices.vel(index);
			glm::vec3 delta_vi = vcm + glm::cross(w, ri) - vi; // (7)
			m_vertices.vel(index) = vi + k_damp * delta_vi; // (8)
		}
	}
}

void ClothSim::compute_predicted_position(float dt)
{
    FOR_EACH_PARTICLE {
        m_vertices.predicted_pos(i) = m_vertices.pos(i) + dt*m_vertices.vel(i);
		resolve_box_collision(i, dt);
    }

	//for (int i = 0; i < m_vertices.size(); i++) {
	//	for (int j = 0; j < m_vertices.size(); j++) {
	//		resolve_particle_collision(i, j, dt);
	//	}
	//}
}

void ClothSim::find_neighboring_particles(){
	unsigned int i, j, size;
    size = m_vertices.size();
	glm::vec3 p1, p2;
	float radius = 0.5f;
    for(i = 0; i < size; ++i) {
        p1 = m_vertices.pos(i);
		std::vector<unsigned int> p1_neighbors = m_neighbors.at(i);
		p1_neighbors.clear();
		for (j = 0; j < size; ++j) {
			p2 = m_vertices.pos(j);
			if (glm::distance(p1, p2) <= radius && i != j) {
				p1_neighbors.push_back(j);
			}
		}
		m_neighbors.at(i) = p1_neighbors;
	}
}

bool ClothSim::detect_particle_collision(int i, int j, float dt) {
	if (i == j) return false;
	glm::vec3 pi = m_vertices.predicted_pos(i);
	glm::vec3 pj = m_vertices.predicted_pos(j);
	glm::vec3 norm, reflected_dir;
	glm::vec3 vel = m_vertices.vel(i);

	float dist_btwn_centers = glm::abs(glm::dot(pi, pj));
	float sum_radii = m_radius*2;

	if (dist_btwn_centers <= sum_radii) {
		return true;
	} else {
		return false;
	}
}

void ClothSim::resolve_particle_collision(int i, int j, float dt) {
	bool collision = detect_particle_collision(i, j, dt);
	glm::vec3 norm, v, vn, vt, newpos, newvel;
	glm::vec3 pi = m_vertices.pos(i);
	if (collision) {
		v = m_vertices.vel(i);
		norm = glm::normalize(m_vertices.pos(j) - m_vertices.pos(i));
		vn = glm::dot(v, norm) * norm;
		vt = v - vn;
		newvel = friction*vt + restitution*vn;
		newpos = pi + m_vertices.vel(i)*dt;

		m_vertices.vel(i) = newvel;
		m_vertices.predicted_pos(i) = newpos;
	} else {
		return;
	}

}

void ClothSim::resolve_box_collision(int i, float dt) {
	glm::vec3 pi = m_vertices.predicted_pos(i);
	glm::vec3 vel = m_vertices.vel(i);
	glm::vec3 dim = glm::vec3(5,5,5);
	glm::vec3 norm, reflected_dir;

	if (pi.x < -dim.x) {
		norm = glm::vec3(0,1,0);
		reflected_dir = glm::reflect(vel, norm);
		vel.x = restitution*reflected_dir.x;
	}
	else if (pi.x > dim.x) {
		norm = glm::vec3(0,-1,0);
		reflected_dir = glm::reflect(vel, norm);
		vel.x = restitution*reflected_dir.x;
	}
	else if (pi.y < 0) {
		norm = glm::vec3(0,1,0);
		reflected_dir = glm::reflect(vel, norm);
		vel.y = restitution*reflected_dir.y;
	}
	else if (pi.z < -dim.z) {
		norm = glm::vec3(0,0,1);
		reflected_dir = glm::reflect(vel, norm);
		vel.z = restitution*reflected_dir.z;
	}
	else if (pi.z > dim.z) {
		norm = glm::vec3(0,0,-1);
		reflected_dir = glm::reflect(vel, norm);
		vel.z = restitution*reflected_dir.z;
	}
	else return;

	m_vertices.vel(i) = vel;
	glm::vec3 new_pos = pi + vel*dt;
	m_vertices.predicted_pos(i) = pi + vel*dt;
}

void ClothSim::resolve_constriants()
{// resolve all the constraints, including both internal and external constraints.
    bool all_solved = true;
    int i, size;
    for(unsigned int n = 0; n < m_solver_iterations; ++n) {
        // solve all the internal constraints.
		generate_internal_constraints();
        size = m_constraints_int.size();
		for (i = 0; i  < size; i++) {
			all_solved &= m_constraints_int[i]->project_constraint();
		}
        if(all_solved)
            break;
    }
    m_vertices.unlock_pos_all();
}

void ClothSim::integration(float dt)
{// integration the position based on optimized prediction, and determine velocity based on position. (12-15)
    unsigned int size = m_vertices.size();
    float inv_dt = 1.0f / dt;
    FOR_EACH_PARTICLE {
        m_vertices.vel(i) = (m_vertices.predicted_pos(i)-m_vertices.pos(i))/dt;
		m_vertices.pos(i) = m_vertices.predicted_pos(i);
    }
}

void ClothSim::update_velocity(float dt)
{
	FOR_EACH_PARTICLE {
		m_vertices.vel(i) = (m_vertices.pos(i) - m_vertices.predicted_pos(i)) / dt;
	}
}

void ClothSim::clean_collision_constraints()
{
    m_constraints_ext.clear();
    m_self_collision.clear();
}
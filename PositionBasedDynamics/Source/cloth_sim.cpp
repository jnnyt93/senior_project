// Cloth Simulation using Position Based Dynamics
// Copyright 2013 Xing Du

#include "cloth_sim.h"
#define FOR_EACH_PARTICLE \
	for(unsigned int i = 0; i < m_vertices.size(); i++)  \

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
	h = 2.0f;
	rest_density = 1000.0f;
	epsilon = 0.01f;
	friction = 0.98;
	restitution = 1.0f;
	m_radius = 1.5f;
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
	m_densities.resize(m_dimx * m_dimy * m_dimz);
	m_C.resize(m_dimx * m_dimy * m_dimz);
	m_gradC.resize(m_dimx * m_dimy * m_dimz);
	m_deltaP.resize(m_dimx * m_dimy * m_dimz);
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
				m_densities.at(index) = 0.0f;
				m_C.at(index) = 0.0f;
				m_gradC.at(index) = 0.0f;
				m_deltaP.at(index) = glm::vec3(0.0f);
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
	find_neighboring_particles();

	for(unsigned int n = 0; n < m_solver_iterations; ++n) {
		compute_lambda();
		compute_deltaPi();
		resolve_box_collision(dt);
		update_predicted_position();
	}
	
    update_velocity(dt);
	update_position();
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
		glm::vec3 p_i = m_vertices.predicted_pos(pi);
		glm::vec3 p_j = m_vertices.predicted_pos(neighbor_index);
		glm::vec3 r = p_i - p_j;
		density += mass * W(r, h);
	}
	return density;
}

glm::vec3 ClothSim::gradient_C(unsigned int i, unsigned int k) {
	glm::vec3 pi = m_vertices.predicted_pos(i);
	glm::vec3 pj;
	glm::vec3 gradientC = glm::vec3(0.0f);
	glm::vec3 sum = glm::vec3(0.0f);
	float sum_grad = 0.0f;
	std::vector<unsigned int> neighbors = m_neighbors.at(i);
	for (int j = 0; j < neighbors.size(); j++) {
		unsigned int j_indx = neighbors.at(j);
		pj = m_vertices.predicted_pos(j_indx);
		if (k == i) {
			sum += W_spiky(pi-pj, h);
		} else if (k == j_indx) {
			sum -= W_spiky(pi-pj, h);
		}
	}
	gradientC += sum/rest_density;
	return gradientC;
}

void ClothSim::compute_deltaPi() {
	FOR_EACH_PARTICLE {
		glm::vec3 pj = m_vertices.predicted_pos(i);
		float li = m_lambdas[i];
		glm::vec3 dp = glm::vec3(0.0f);
		for (int j = 0; j < m_neighbors[i].size(); j++) {
			unsigned int neighbor_index = m_neighbors[i].at(j);
			glm::vec3 pi = m_vertices.pos(i);
			glm::vec3 pj = m_vertices.pos(neighbor_index);
			glm::vec3 w = W_spiky(pi - pj, h);
			dp += (m_lambdas.at(i) + m_lambdas.at(neighbor_index)) * w;
		}
		dp *= 1.0f / rest_density;
		m_deltaP[i] = dp;
	}
}

void ClothSim::update_predicted_position() {
	FOR_EACH_PARTICLE {
		m_vertices.predicted_pos(i) = m_vertices.predicted_pos(i) + m_deltaP[i];
	}
}

void ClothSim::update_position() {
	FOR_EACH_PARTICLE {
		m_vertices.pos(i) = m_vertices.predicted_pos(i); 
	}
}

void ClothSim::compute_lambda() {
	/* Calculate C_i */
	FOR_EACH_PARTICLE {
		float density = sph_density_estimator(i);
		m_C[i] = (density/rest_density) - 1.0f;
	}
	//printf("here\n");
	/* Calculate gradient of C_i */
	FOR_EACH_PARTICLE {
		float Ci = m_C[i];
		float sumk = 0.0f;
		glm::vec3 grad_c = glm::vec3(0.0f);
		for (int k = 0; k < m_vertices.size(); k++) {
			grad_c = gradient_C(i, k);
			float l = 0;
			if (grad_c != glm::vec3(0,0,0))
				l = glm::length(grad_c);
			sumk += l*l;
		}
		sumk += epsilon;
		m_lambdas.at(i) = -1.0f * Ci/sumk;
	}
}

void ClothSim::apply_external_force(const glm::vec3& force, float dt)
{
	FOR_EACH_PARTICLE {
		glm::vec3 a = m_vertices.inv_mass(i)*force;
		m_vertices.vel(i) = m_vertices.vel(i) + a*dt;
		m_vertices.predicted_pos(i) = m_vertices.pos(i) + dt*m_vertices.vel(i);
    }
}

void ClothSim::find_neighboring_particles(){
	unsigned int i, j, size;
    size = m_vertices.size();
	glm::vec3 p1, p2;
    for(i = 0; i < size; ++i) {
        p1 = m_vertices.pos(i);
		std::vector<unsigned int> p1_neighbors = m_neighbors.at(i);
		p1_neighbors.clear();
		for (j = 0; j < size; ++j) {
			p2 = m_vertices.pos(j);
			if (glm::distance(p1, p2) <= m_radius && i != j) {
				p1_neighbors.push_back(j);
			}
		}
		m_neighbors.at(i) = p1_neighbors;
	}
}

void ClothSim::resolve_box_collision(float dt) {
	float buffer = 0.5;
	FOR_EACH_PARTICLE {
		glm::vec3 pi = m_vertices.predicted_pos(i);
		glm::vec3 vel = m_vertices.vel(i);
		glm::vec3 dim = glm::vec3(2,2,6);
		glm::vec3 norm, reflected_dir;

		if (pi.x <= -dim.x + buffer) {
			norm = glm::vec3(0,1,0);
			reflected_dir = glm::reflect(vel, norm);
			vel.x = restitution*reflected_dir.x;
			m_vertices.predicted_pos(i).x = -dim.x + buffer;
		}
		else if (pi.x >= dim.x - buffer) {
			norm = glm::vec3(0,-1,0);
			reflected_dir = glm::reflect(vel, norm);
			vel.x = restitution*reflected_dir.x;
			m_vertices.predicted_pos(i).x = dim.x - buffer;
		}
		if (pi.y <= 0 + buffer) {
			norm = glm::vec3(0,1,0);
			reflected_dir = glm::reflect(vel, norm);
			vel.y = restitution*reflected_dir.y;
			m_vertices.predicted_pos(i).y = buffer;
		}
		if (pi.z <= -dim.z + buffer) {
			norm = glm::vec3(0,0,1);
			reflected_dir = glm::reflect(vel, norm);
			vel.z = restitution*reflected_dir.z;
			m_vertices.predicted_pos(i).z = -dim.z + buffer;
		}
		if (pi.z >= dim.z - buffer) {
			norm = glm::vec3(0,0,-1);
			reflected_dir = glm::reflect(vel, norm);
			vel.z = restitution*reflected_dir.z;
			m_vertices.predicted_pos(i).z = dim.z - buffer;
		}
		m_vertices.vel(i) = vel;
	}
}

void ClothSim::integration(float dt)
{// integration the position based on optimized prediction, and determine velocity based on position. (12-15)
    FOR_EACH_PARTICLE {
		m_vertices.vel(i) = (m_vertices.predicted_pos(i)-m_vertices.pos(i))/dt;
		m_vertices.pos(i) = m_vertices.predicted_pos(i);
    }
}

void ClothSim::update_velocity(float dt)
{
	FOR_EACH_PARTICLE {
		m_vertices.vel(i) = (m_vertices.predicted_pos(i)-m_vertices.pos(i)) / dt;
	}
}

void ClothSim::clean_collision_constraints()
{
    m_constraints_ext.clear();
    m_self_collision.clear();
}
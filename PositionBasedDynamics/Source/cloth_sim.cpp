// Cloth Simulation using Position Based Dynamics
// Copyright 2013 Xing Du

#include "cloth_sim.h"

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
    m_triangle_list.clear();

    m_edge_list.clear();

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
	for(i = 0; i < m_dimx; ++i) {
		for (j = 0; j < dim_y; ++j) {
			for(k = 0; k < m_dimz; ++k) {
				m_vertices.pos(index) = glm::vec3(delta.x * i + cloth_min.x, delta.y * j + cloth_min.y, delta.z * k + cloth_min.z);
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

void ClothSim::update(const Scene* const scene, float dt)
{// update the system for a certain time step dt.
    glm::vec3 gravity(0.0f, -9.8f, 0.0f);
    apply_external_force(gravity, dt);
    damp_velocity(0.01f);
    compute_predicted_position(dt);
	find_neighboring_particles();
	collision_detection(scene);
    resolve_constriants();
    integration(dt);
    update_velocity(0.98f, 0.4f);
    clean_collision_constraints();
    
}

void ClothSim::draw(const VBO& vbos)
{// visualize the cloth on the screen.
	//clear color and depth buffer 
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();//load identity matrix
    
	glColor3f(1.0f,1.0f,1.0f); //blue color
    glPointSize(10.0f);//set point size to 10 pixels
    glBegin(GL_POINTS); //starts drawing of points
	for (int i = 0; i < m_vertices.size(); i++) {
		glColor3f(1.0f,1.0f,1.0f); //blue color
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

void ClothSim::generate_edge_list()
{// generate all the edges from the vertices and triangle list.
    // courtesy of Eric Lengyel, "Building an Edge List for an Arbitrary Mesh". Terathon Software 3D Graphics Library.
    // http://www.terathon.com/code/edges.html
    unsigned int vert_num = m_vertices.size();
    unsigned int tri_num = m_triangle_list.size() / 3;

    unsigned int *first_edge = new unsigned int[vert_num + 3 * tri_num];
    unsigned int *next_edge = first_edge + vert_num;
    
    for(unsigned int i = 0; i < vert_num; ++i)
        first_edge[i] = 0xFFFFFFFF;
    // First pass over all triangles. Finds out all the edges satisfying the condition that
    // the first vertex index is less than the second vertex index when the direction from 
    // the first to the second represents a counterclockwise winding around the triangle to
    // which the edge belongs. For each edge found, the edge index is stored in a linked 
    // list of edges belonging to the lower-numbered vertex index i. This allows us to 
    // quickly find an edge in the second pass whose higher-numbered vertex is i.

    unsigned int edge_count = 0;
    const unsigned int* triangle = &m_triangle_list[0];
    unsigned int i1, i2;
    for(unsigned int t = 0; t < tri_num; ++t)
    {
        i1 = triangle[2];
        for(unsigned int n = 0; n < 3; ++n)
        {
            i2 = triangle[n];
            if(i1 < i2)
            {
                Edge new_edge;
                new_edge.m_v1 = i1;
                new_edge.m_v2 = i2;
                new_edge.m_tri1 = t;
                new_edge.m_tri2 = t;
                m_edge_list.push_back(new_edge);

                unsigned int edge_idx = first_edge[i1];
                if(edge_idx == 0xFFFFFFFF)
                {
                    first_edge[i1] = edge_count;
                }
                else
                {
                    while(true)
                    {
                        unsigned int idx = next_edge[edge_idx];
                        if(idx == 0xFFFFFFFF)
                        {
                            next_edge[edge_idx] = edge_count;
                            break;
                        }
                        edge_idx = idx;
                    }
                }

                next_edge[edge_count] = 0xFFFFFFFF;
                edge_count++;
            }
            i1 = i2;
        }
        triangle += 3;
    }

    // Second pass over all triangles. Finds out all the edges satisfying the condition that
    // the first vertex index is greater than the second vertex index when the direction from 
    // the first to the second represents a counterclockwise winding around the triangle to
    // which the edge belongs. For each of these edges, the same edge should have already been
    // found in the first pass for a different triangle. So we search the list of edges for the
    // higher-numbered index for the matching edge and fill in the second triangle index. The 
    // maximum number of the comparisons in this search for any vertex is the number of edges
    // having that vertex as an endpoint.
    triangle = &m_triangle_list[0];
    for(unsigned int t = 0; t < tri_num; ++t)
    {
        i1 = triangle[2];
        for(unsigned int n = 0; n < 3; ++n)
        {
            i2 = triangle[n];
            if(i1 > i2)
            {
                bool is_new_edge = true;
                for(unsigned int edge_idx = first_edge[i2]; edge_idx != 0xFFFFFFFF; edge_idx = next_edge[edge_idx])
                {
                    Edge *edge = &m_edge_list[edge_idx];
                    if((edge->m_v2 == i1) && (edge->m_tri1 == edge->m_tri2))
                    {
                        edge->m_tri2 = t;
                        is_new_edge = false;
                        break;
                    }
                }
                // for case where a edge belongs to only one triangle. i.e. mesh is not watertight.
                if(is_new_edge)
                {
                    Edge new_edge;
                    new_edge.m_v1 = i1;
                    new_edge.m_v2 = i2;
                    new_edge.m_tri1 = t;
                    new_edge.m_tri2 = t;
                    m_edge_list.push_back(new_edge);

                    unsigned int edge_idx = first_edge[i1];
                    if(edge_idx == 0xFFFFFFFF)
                    {
                        first_edge[i1] = edge_count;
                    }
                    else
                    {
                        while(true)
                        {
                            unsigned int idx = next_edge[edge_idx];
                            if(idx == 0xFFFFFFFF)
                            {
                                next_edge[edge_idx] = edge_count;
                                break;
                            }
                            edge_idx = idx;
                        }
                    }

                    next_edge[edge_count] = 0xFFFFFFFF;
                    edge_count++;
                }
            }
            i1 = i2;
        }
        triangle += 3;
    }

    delete[] first_edge;
    printf("Edge number: %u.\n", m_edge_list.size());
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
	if (r.x == 0 && r.y == 0 && r.z == 0) return glm::vec3(0,0,0);
	float a = 45.0f / (M_PI * glm::pow(h, 6.f));
	float b = glm::pow(h - glm::length(r), 2.0f);
	glm::vec3 c = glm::normalize(r);
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
	if (k == i) {
		for (int j = 0; j < m_neighbors.at(i).size(); j++) {
			sum += W_spiky(pi - pj, h);
		}
	} else {
		for (int j = 0; j < m_neighbors.at(i).size(); j++) {
			if (k == j) {
				pj = m_vertices.pos(j);
				sum = -1.0f*W_spiky(pi - pj, h);
				break;
			}
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
	float epsilon = 200;
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
		Constraint* density_constraint = new DensityConstraint(&m_vertices, &m_neighbors, m_lambdas, i);
		m_constraints_int.push_back(density_constraint);
	}

}

void ClothSim::compute_normal()
{// compute normal for all the vertex. only necessary for visualization.
    // reset all the normal.
    glm::vec3 zero(0.0f);
    for(std::vector<glm::vec3>::iterator n = m_normals.begin(); n != m_normals.end(); ++n) {
        *n = zero;
    }
    // calculate normal for each individual triangle
    unsigned int triangle_num = m_triangle_list.size() / 3;
    unsigned int id0, id1, id2;
    glm::vec3 p0, p1, p2;
    glm::vec3 normal;
    for(unsigned int i = 0; i < triangle_num; ++i)
    {
        id0 = m_triangle_list[3 * i];
        id1 = m_triangle_list[3 * i + 1];
        id2 = m_triangle_list[3 * i + 2];

        p0 = m_vertices.pos(id0);
        p1 = m_vertices.pos(id1);
        p2 = m_vertices.pos(id2);
        
        normal = glm::normalize(glm::cross(p1 - p0, p2 - p1));

        m_normals[id0] += normal;
        m_normals[id1] += normal;
        m_normals[id2] += normal;
    }
    // re-normalize all the normals.
    for(std::vector<glm::vec3>::iterator n = m_normals.begin(); n != m_normals.end(); ++n) {
        *n = glm::normalize(*n);
    }
}

void ClothSim::apply_external_force(const glm::vec3& force, float dt)
{// apply external force to cloth.
    unsigned int size = m_vertices.size();
    for(unsigned int i = 0; i < size; ++i)
    {// TODO: apply force to velocity of all vertices.
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
    unsigned int size = m_vertices.size();
    for(unsigned int i = 0; i < size; ++i) {
        m_vertices.predicted_pos(i) = m_vertices.pos(i) + dt*m_vertices.vel(i);
    }
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

void ClothSim::collision_detection(const Scene* const scene)
{// detect collision between cloth and the scene, and generate external constraints.
    unsigned int i, size;
    size = m_vertices.size();
    glm::vec3 x, p, q, n;
    for(i = 0; i < size; ++i)
    {
        x = m_vertices.pos(i);
        p = m_vertices.predicted_pos(i);
        if(scene->line_intersection(x, p, m_thick, q, n))
        {
            CollisionConstraint c(&m_vertices, i, q, n);
            m_constraints_ext.push_back(c);
        }
    }
}

void ClothSim::resolve_constriants()
{// resolve all the constraints, including both internal and external constraints.
    bool all_solved = true;
    bool reverse = false;
    int i, size;
    for(unsigned int n = 0; n < m_solver_iterations; ++n) {
        // solve all the internal constraints.
		generate_internal_constraints();

        size = m_constraints_int.size();
        for(i = reverse ? (size - 1) : 0; (i < size) && (i >= 0); reverse ? --i : ++i)
        {
            all_solved &= m_constraints_int[i]->project_constraint();
        }
        // solve all the external constraints.
        size = m_constraints_ext.size();
        for(i = reverse ? (size - 1) : 0; (i < size) && (i >= 0); reverse ? --i : ++i)
        {
            all_solved &= m_constraints_ext[i].project_constraint();
        }
        // solve all the self collisions.
        size = m_self_collision.size();
        for(i = reverse ? (size - 1) : 0; (i < size) && (i >= 0); reverse ? --i : ++i)
        {
            all_solved &= m_self_collision[i].project_constraint();
        }

        if(all_solved)
            break;
        reverse = !reverse;
    }
    m_vertices.unlock_pos_all();
}

void ClothSim::integration(float dt)
{// integration the position based on optimized prediction, and determine velocity based on position. (12-15)
    unsigned int size = m_vertices.size();
    float inv_dt = 1.0f / dt;
    for(unsigned int i = 0; i < size; ++i) {
        m_vertices.vel(i) = (m_vertices.predicted_pos(i)-m_vertices.pos(i))/dt;
		m_vertices.pos(i) = m_vertices.predicted_pos(i);
    }
}

void ClothSim::update_velocity(float friction, float restitution)
{// update velocity based on collision restitution or friction. (16)
    glm::vec3 normal, vn, vt;
    float norm_fraction;
    for(std::vector<CollisionConstraint>::iterator s = m_constraints_ext.begin(); s != m_constraints_ext.end(); ++s) {
		glm::vec3 v = m_vertices.vel(s->index());
		vn = glm::dot(v, s->normal()) * s->normal();
		vt = v - vn;
		m_vertices.vel(s->index()) = friction*vt + restitution*vn; 
    }
}

void ClothSim::clean_collision_constraints()
{
    m_constraints_ext.clear();
    m_self_collision.clear();
}
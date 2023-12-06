#include <igl/readOBJ.h>
#include "OPSim.h"
#include "Particle.h" // Oriented particle data structure
#include <unordered_set> // For adjacency list


bool OPSim::advance() {
    
    m_time += m_dt*num_steps;
    m_step++; 
    for(int i = 0; i < num_steps; i++){ 
        m_particleHash->create(m_particles);
        predict();
    }
    return false;
}

void OPSim::predict(){
    for (int i = 0; i < xp.rows(); i++) {
        m_particleColors[i] = Eigen::Vector3d(1, 0, 0);

        cubeCollision(i);
    }
    xp = xp + p_vel * m_dt;
    
    solve();   
}

bool OPSim::trianglesAreNeighbors(const Eigen::Vector3i& tri1, const Eigen::Vector3i& tri2) {
    int commonVertices = 0;

    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            if (tri1[i] == tri2[j]) {
                ++commonVertices;
                if (commonVertices == 2) {
                    return true; // Two common vertices found, triangles are neighbors
                }
            }
        }
    }

    return false; // Less than 2 common vertices, triangles are not neighbors
}

void OPSim::addEdge(int v1, int v2) {
    if (v1 >= 0 && v1 < m_particles.size() && v2 >= 0 && v2 < m_particles.size()) {
        // m_adjacencyList[v1].insert(v2); // Set edge from v1 to v2
        // m_adjacencyList[v2].insert(v1); // Set edge from v2 to v1

        adjacencyMatrix(v1, v2) = 1;
        adjacencyMatrix(v2, v1) = 1;
    }
}

int OPSim::getParticleIndexByPosition(const Eigen::Vector3d& position) {
    for (size_t i = 0; i < m_particles.size(); ++i) {
        if (m_particles[i].getPosition() == position) {
            return i;
        }
    }
    return -1;
}

void OPSim::solve(){
    for(int i = 0; i < xp.rows(); i++){
        xp.row(i) = groundConstraint(xp.row(i));
    }
    distanceConstraint(xp, m_renderE);
    
    for (int i = 0; i < m_particles.size(); i++) {
        m_particleHash->query(m_particles[i], 2 * particle_radius);
        std::vector<int> query = m_particleHash->queryIds;
        for (int j = 0; j < query.size(); j++) {
            collisionConstraint(i, query[j]);
        }
    }
    update();
}

void OPSim::update(){
    //p_vel = (xp-m_renderV) / m_dt;
    for(int i = 0; i < p_vel.rows(); i++){
        p_vel.row(i) += m_gravity * m_dt;
    }
    m_renderV = xp;
    //p_obj->setMesh(m_renderV, m_renderF); 
    for (int i = 0; i < m_particles.size(); i++) {
        m_particles[i].setPosition(m_renderV.row(i));
        m_particleSim[i] = m_renderV.row(i);
    }
}

void OPSim::cubeCollision(int pid) {
    // chech X
    if (xp.row(pid).x() < -1 || xp.row(pid).x() > 1) {
        p_vel.row(pid).x() = -p_vel.row(pid).x();
        xp.row(pid).x() = xp.row(pid).x();
    }
    // check Y
    if (xp.row(pid).y() < 0 || xp.row(pid).y() > 2) {
        p_vel.row(pid).y() = -p_vel.row(pid).y();
        xp.row(pid).y() = xp.row(pid).y();

    }
    // check Z
    if (xp.row(pid).z() < -1 || xp.row(pid).z() > 1) {
        p_vel.row(pid).z() = -p_vel.row(pid).z();
        xp.row(pid).z() = xp.row(pid).z();
    }
};

Eigen::Vector3d OPSim::groundConstraint(Eigen::Vector3d pa_pos){
        if(pa_pos.y() >= 0){
            return pa_pos;
        }
        float C = pa_pos.y();
        Eigen::Vector3d dC = Eigen::Vector3d(0, 1, 0);
        float lambda = -C/(dC.norm()*dC.norm() + alpha/pow(m_dt,2));
        pa_pos += lambda * dC;
        return pa_pos;
};

void OPSim::distanceConstraint(Eigen::MatrixXd& pa_pos, Eigen::MatrixXi edges){
    
    for(int i = 0; i < edges.rows(); i++){
        Eigen::Vector2i edge = edges.row(i);
        Eigen::Vector3d p1 = pa_pos.row(edge.x());
        Eigen::Vector3d p2 = pa_pos.row(edge.y());
        float dist= (p1-p2).norm();
        float C = -(edge_dist[i]-dist);
        Eigen::Vector3d dC1 = (p1 - p2)/dist;
        Eigen::Vector3d dC2 = (p2 - p1)/dist;
        float lambda = -C/(pow(dC1.norm(),2) + pow(dC2.norm(),2) + alpha/pow(m_dt,2));
        pa_pos.row(edge.x()) = p1 + lambda*dC1;
        pa_pos.row(edge.y()) = p2 + lambda*dC2;
    }
};

void OPSim::collisionConstraint(int pid1, int pid2){
    Eigen::Vector3d normal = xp.row(pid1) - xp.row(pid2);
    float dist = normal.norm();
    if (dist > 0 && dist < 2 * particle_radius) {
        float C = (dist - 2 * particle_radius) ;
        Eigen::Vector3d dC1 = normal / dist * C;
        Eigen::Vector3d dC2 = -normal / dist * C;
        //float lambda = -C / (pow(dC1.norm(), 2) + pow(dC2.norm(), 2) + alpha / pow(m_dt, 2));
        xp.row(pid1) +=  dC1;
        xp.row(pid2) +=  dC2;

        double v1 = p_vel.row(pid1).dot(normal);
        double v2 = p_vel.row(pid2).dot(normal);

        p_vel.row(pid1) += normal * (v2 - v1);
        p_vel.row(pid2) += normal * (v1 - v2);
        m_particleColors[pid1] = Eigen::Vector3d(1, 1, 0);
        m_particleColors[pid2] = Eigen::Vector3d(1, 1, 0);

    }
};


void OPSim::updateAdjacencyList(Eigen::MatrixXi m_renderF) {

    // ADJACENCY LIST FOR PARTICLES PLACED ON THE VERTICES OF THE MESH
    int numTriangles = m_renderF.rows();

    for (int i = 0; i < numTriangles; ++i) {
        Eigen::Vector3d v1 = m_renderV.row(m_renderF(i, 0));
        Eigen::Vector3d v2 = m_renderV.row(m_renderF(i, 1));
        Eigen::Vector3d v3 = m_renderV.row(m_renderF(i, 2));

        int i1 = getParticleIndexByPosition(v1);
        int i2 = getParticleIndexByPosition(v2);
        int i3 = getParticleIndexByPosition(v3);

        if(i1 != -1 && i2 != -1) addEdge(i1, i2);
        if(i2 != -1 && i3 != -1) addEdge(i2, i3);
        if(i3 != -1 && i1 != -1) addEdge(i3, i1);
    }


    // ADJACENCY LIST FOR PARTICLES PLACED ON THE CENTER OF THE MESH
    // int numTriangles = m_renderF.rows();

    // for (int i = 0; i < numTriangles; ++i) {
    //     for (int j = i + 1; j < numTriangles; ++j) {
    //         if (trianglesAreNeighbors(m_renderF.row(i), m_renderF.row(j))) {
    //             addEdge(i, j);
    //         }
    //     }
    // }

    // Now adjacency[i][j] is true if triangles i and j are neighbors
}

void OPSim::assignParticles() {
    // Get loaded vertex positions
    if (m_objects.size() == 0) {
        return;
    }
    p_obj->getMesh(m_renderV, m_renderF);

    // Clear previous particles and initialize new particles at each vertex
    m_particles.clear();
    p_pos = Eigen::MatrixXd(m_renderV.rows(), 3);

    // For placing the particles on the center of the mesh, change the loop to: m_renderF.rows()
    for (int i = 0; i < m_renderV.rows(); i++) {
        Particle particle;

        // PARTICLES PLACED ON THE VERTICES OF THE MESH
        Eigen::VectorXd rowVector = m_renderV.row(i);
        particle.setPosition(rowVector);

        // PARTICLES PLACED ON THE CENTER OF THE MESH
        // Eigen::Vector3d v1 = m_renderV.row(m_renderF(i, 0));
        // Eigen::Vector3d v2 = m_renderV.row(m_renderF(i, 1));
        // Eigen::Vector3d v3 = m_renderV.row(m_renderF(i, 2));
        // Eigen::Vector3d centroid = (v1 + v2 + v3) / 3.0f;
        // particle.setPosition(centroid);

        // Set orientation and velocity to zero in the beginning
        particle.setOrientation(Eigen::Quaterniond::Identity());
        particle.setVelocity(Eigen::Vector3d::Zero());

        m_particles.push_back(particle);

        // Add particle position and color to the vectors for rendering
        m_particleSim.push_back(particle.getPosition()); 
        m_particleColors.push_back(m_color);
        p_pos.row(i) = particle.getPosition();
    }

    // Update adjacency matrix
    adjacencyMatrix = Eigen::MatrixXi::Zero(m_particles.size(), m_particles.size());
    updateAdjacencyList(m_renderF);
}

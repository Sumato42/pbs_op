#include <igl/readOBJ.h>
#include "OPSim.h"
#include "Particle.h" // Oriented particle data structure
#include <unordered_set> // For adjacency list

std::vector<Particle> m_particles; // Container for oriented particles
// Define adjacency list as a vector of unordered sets
std::vector<std::unordered_set<int>> m_adjacencyList;

bool OPSim::advance() {
    
    m_time += m_dt;
    m_step++; 
    for(int i = 0; i < num_steps; i++){ 
        predict();
    }
    return false;
}

void OPSim::predict(){
    for(int i = 0; i < num_steps;i++){
        xp = xp + p_vel * m_dt;
        solve();   
    }    
}

bool trianglesAreNeighbors(const Eigen::Vector3i& tri1, const Eigen::Vector3i& tri2) {
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

void addEdge(int v1, int v2) {
    if (v1 >= 0 && v1 < m_particles.size() && v2 >= 0 && v2 < m_particles.size()) {
        m_adjacencyList[v1].insert(v2); // Set edge from v1 to v2
        m_adjacencyList[v2].insert(v1); // Set edge from v2 to v1
    }
}

void OPSim::solve(){
    for(int i = 0; i < xp.rows(); i++){
        xp.row(i) = groundConstraint(xp.row(i));
    }
    distanceConstraint(xp, m_renderE);
    update();
}

void OPSim::update(){
    p_vel = (xp-m_renderV) / m_dt;
    p_vel1 = (pp-p_obj->getPosition()) / m_dt + m_gravity * m_dt;
    for(int i = 0; i < p_vel.rows(); i++){
        p_vel.row(i) += m_gravity * m_dt;
    }
    m_renderV = xp;
    p_obj->setMesh(m_renderV, m_renderF);    
}

Eigen::Vector3d OPSim::groundConstraint(Eigen::Vector3d particle_pos){
        if(particle_pos.y() >= 0){
            return particle_pos;
        }
        float C = particle_pos.y();
        Eigen::Vector3d dC = Eigen::Vector3d(0, 1, 0);
        float lambda = -C/(dC.norm()*dC.norm() + alpha/pow(m_dt,2));
        particle_pos += lambda * dC;
        return particle_pos;
};

void OPSim::distanceConstraint(Eigen::MatrixXd& particle_pos, Eigen::MatrixXi edges){
    
    for(int i = 0; i < edges.rows(); i++){
        Eigen::Vector2i edge = edges.row(i);
        Eigen::Vector3d p1 = particle_pos.row(edge.x());
        Eigen::Vector3d p2 = particle_pos.row(edge.y());
        float dist= (p1-p2).norm();
        float C = -(edge_dist[i]-dist);
        Eigen::Vector3d dC1 = (p1 - p2)/dist;
        Eigen::Vector3d dC2 = (p2 - p1)/dist;
        float lambda = -C/(pow(dC1.norm(),2) + pow(dC2.norm(),2) + alpha/pow(m_dt,2));
        particle_pos.row(edge.x()) = p1 + lambda*dC1;
        particle_pos.row(edge.y()) = p2 + lambda*dC2;
    }
};

void OPSim::collisionConstraint(Eigen::MatrixXd& particle_pos){
    
};

void OPSim::updateAdjacencyList(Eigen::MatrixXi m_renderF) {

    // ADJACENCY LIST FOR PARTICLES PLACED ON THE VERTICES OF THE MESH
    int numTriangles = m_renderF.rows();

    for (int i = 0; i < numTriangles; ++i) {
        int v1 = m_renderF(i, 0);
        int v2 = m_renderF(i, 1);
        int v3 = m_renderF(i, 2);

        addEdge(v1, v2);
        addEdge(v2, v3);
        addEdge(v3, v1);
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
    p_obj->getMesh(m_renderV, m_renderF);

    //std::cout << m_renderF.cols() << std::endl;
    // std::cout << m_renderV.row(m_renderF(2, 0)) << std::endl;
    // std::cout << m_renderV.row(m_renderF(2, 1)) << std::endl;
    // std::cout << m_renderV.row(m_renderF(2, 2)) << std::endl;

    // Clear previous particles and initialize new particles at each vertex
    m_particles.clear();

    // For placing the particles on the center of the mesh, change the loop to: m_renderF.rows()
    for (int i = 0; i < m_renderV.rows(); i++) {
        Particle particle;

        // PARTICLES PLACED ON THE VERTICES OF THE MESH
        Eigen::VectorXd rowVector = m_renderV.row(i);
        particle.setPosition(rowVector.cast<float>()); 

        // PARTICLES PLACED ON THE CENTER OF THE MESH
        // Eigen::Vector3f v1 = m_renderV.row(m_renderF(i, 0)).cast<float>();
        // Eigen::Vector3f v2 = m_renderV.row(m_renderF(i, 1)).cast<float>();
        // Eigen::Vector3f v3 = m_renderV.row(m_renderF(i, 2)).cast<float>();
        // Eigen::Vector3f centroid = (v1 + v2 + v3) / 3.0f;
        // particle.setPosition(centroid);

        // Print particle position
        // std::cout << "particle position: " << particle.getPosition() << std::endl;

        // Set orientation and velocity to zero in the beginning
        particle.setOrientation(Eigen::Quaternionf::Identity());
        particle.setVelocity(Eigen::Vector3f::Zero());
        
        m_particles.push_back(particle);
    }

    // Print the size of particles
    std::cout << "particles size: " << m_particles.size() << std::endl;
}

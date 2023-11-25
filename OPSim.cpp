#include <igl/readOBJ.h>
#include "OPSim.h"
#include "Particle.h" // Oriented particle data structure
#include <unordered_set> // For adjacency list

std::vector<Particle> m_particles; // Container for oriented particles
// Define adjacency list as a vector of unordered sets
//std::vector<std::unordered_set<int>> m_adjacencyList;

// Define the adjacency matrix as a matrix of booleans
Eigen::MatrixXi adjacencyMatrix;

bool OPSim::advance() {
    
    m_time += m_dt;
    m_step++;  

    return false;
}

void OPSim::loadOBJ(){
    std::string path = "circular.obj";
    m_objects.clear();
    m_objects.push_back(RigidObject(path));
    p_obj = &m_objects.back();
    p_obj->setScale(0.1);
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
        // m_adjacencyList[v1].insert(v2); // Set edge from v1 to v2
        // m_adjacencyList[v2].insert(v1); // Set edge from v2 to v1

        adjacencyMatrix(v1, v2) = 1;
        adjacencyMatrix(v2, v1) = 1;
    }
}

int getParticleIndexByPosition(const Eigen::Vector3d& position) {
    for (size_t i = 0; i < m_particles.size(); ++i) {
        if (m_particles[i].getPosition() == position) {
            return i;
        }
    }
    return -1;
}


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
    p_obj->getMesh(m_renderV, m_renderF);

    // Clear previous particles and initialize new particles at each vertex
    m_particles.clear();

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
    }

    // Update adjacency matrix
    adjacencyMatrix = Eigen::MatrixXi::Zero(m_particles.size(), m_particles.size());
    updateAdjacencyList(m_renderF);
}

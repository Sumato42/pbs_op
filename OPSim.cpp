#include <igl/readOBJ.h>
#include "OPSim.h"
#include "Particle.h" // Oriented particle data structure

std::vector<Particle> m_particles; // Container for oriented particles

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

void OPSim::assignParticles() {
    // Get loaded vertex positions
    p_obj->getMesh(m_renderV, m_renderF);
    std::cout << m_renderF.rows() << std::endl;

    // Clear previous particles and initialize new particles at each vertex
    m_particles.clear();
    for (int i = 0; i < m_renderF.rows(); i++) {
        Particle particle;

        // Place particles on the vertices of the mesh
        // Eigen::VectorXd rowVector = m_renderV.row(i);
        // particle.setPosition(rowVector.cast<float>()); 

        // Place particles on the center of the mesh
        Eigen::Vector3f v1 = m_renderV.row(m_renderF(i, 0)).cast<float>();
        Eigen::Vector3f v2 = m_renderV.row(m_renderF(i, 1)).cast<float>();
        Eigen::Vector3f v3 = m_renderV.row(m_renderF(i, 2)).cast<float>();

        Eigen::Vector3f centroid = (v1 + v2 + v3) / 3.0f;
        particle.setPosition(centroid);

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

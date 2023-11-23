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
    std::cout << m_renderV.rows() << std::endl;

    // Clear previous particles and initialize new particles at each vertex
    m_particles.clear();
    for (int i = 0; i < m_renderV.rows(); i++) {
        Particle particle;

        Eigen::VectorXd rowVector = m_renderV.row(i);  // Extract the row vector from MatrixXd
        particle.setPosition(rowVector.cast<float>()); 

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

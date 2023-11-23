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
    std::string path = "circular.obj";

    // Read OBJ file to get vertex positions
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    igl::readOBJ(path, V, m_renderTC, m_renderN, F, m_renderFTC, m_renderFN);

    // Clear previous particles and initialize new particles at each vertex
    m_particles.clear();
    for (int i = 0; i < V.rows(); i++) {
        Particle particle;

        Eigen::VectorXd rowVector = V.row(i);  // Extract the row vector from MatrixXd
        particle.setPosition(rowVector.cast<float>()); 

        // Set orientation and velocity to zero for now
        particle.setOrientation(Eigen::Quaternionf::Identity());
        particle.setVelocity(Eigen::Vector3f::Zero());
        
        m_particles.push_back(particle);
    }
}

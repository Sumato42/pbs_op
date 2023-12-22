#ifndef PARTICLE_H
#define PARTICLE_H

#include <Eigen/Dense>

class Particle {
private:
    Eigen::Vector3d m_position; // Position of the particle
    Eigen::Vector3d m_velocity; // Velocity of the particle
    Eigen::Quaterniond m_orientation; // Orientation of the particle (stored as a quaternion)
    

public:
    Particle() : m_position(0, 0, 0), m_velocity(0, 0, 0), m_orientation(1, 0, 0, 0) {}

    // Setter functions for position, velocity, and orientation
    void setPosition(const Eigen::Vector3d& position) { m_position = position; }
    void setVelocity(const Eigen::Vector3d& velocity) { m_velocity = velocity; }
    void setOrientation(const Eigen::Quaterniond& orientation) { m_orientation = orientation.normalized(); }

    // Getter functions for position, velocity, and orientation
    Eigen::Vector3d getPosition() const { return m_position; }
    Eigen::Vector3d getVelocity() const { return m_velocity; }
    Eigen::Quaterniond getOrientation() const { return m_orientation; }

    // Other functions for particle behavior or manipulation...
};

#endif // PARTICLE_H

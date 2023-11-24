#include <igl/readOBJ.h>
#include "OPSim.h"

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

void OPSim::collisionConstraint(){
    
};


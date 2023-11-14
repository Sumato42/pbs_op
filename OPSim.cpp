#include <igl/readOBJ.h>
#include "OPSim.h"

bool OPSim::advance() {
    
    m_time += m_dt;
    m_step++;  

    return false;
}

void OPSim::loadOBJ(){
        std::string path = "Frank/Frank.obj";
        m_objects.clear();
        m_objects.push_back(RigidObject(path));
        p_obj = &m_objects.back();
        //igl::readOBJ(path, m_renderV, m_renderTC, m_renderN, m_renderF, m_renderFTC, m_renderFN);
}



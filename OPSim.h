#include <igl/readOBJ.h>
#include "Simulation.h"
#include <igl/edges.h>

using namespace std;

struct Spring {
   public:
    float stiffness;
    float length;
    float damping;
    Eigen::Vector3d start;
    Eigen::Vector3d end;
};

/*
 * Simulation of Oriented Particles.
 */
class OPSim : public Simulation {
   public:
    OPSim() : Simulation() {
        init();
    }

    virtual void init() override {
        std::string path = "cube.off";
        m_objects.clear();
        m_objects.push_back(RigidObject(path));
        p_obj = &m_objects.back();
        m_gravity = Eigen::Vector3d(0, -9.81, 0);
        m_dt = 0.02/num_steps;
    }

    virtual void resetMembers() override {
        p_obj->reset();
        p_obj->setPosition(Eigen::Vector3d(0, 5, 0));
        p_obj->getMesh(m_renderV, m_renderF);
        p_vel = Eigen::MatrixXd::Zero(m_renderV.rows(), m_renderV.cols());
        xp = m_renderV ;
        igl::edges(m_renderF, m_renderE);
        edge_dist = Eigen::VectorXd(m_renderE.rows());
        
        for(int i = 0; i < edge_dist.rows(); i++){
            Eigen::Vector2i edge = m_renderE.row(i);
            edge_dist[i]= (m_renderV.row(edge.x())-m_renderV.row(edge.y())).norm();
        }
        
        p_obj->setMesh(xp, m_renderF);
        assignParticles();
        // updateAdjacencyList();
    }

    virtual void updateRenderGeometry() override {
        p_obj->getMesh(m_renderV, m_renderF);
        p_obj->getColors(m_renderC);
    }

	virtual bool advance() override;

    virtual void predict();

    virtual void solve();

    virtual void update();

    virtual Eigen::Vector3d groundConstraint(Eigen::Vector3d particle_pos);

    virtual void distanceConstraint(Eigen::MatrixXd& particle_pos, Eigen::MatrixXi edges);

    virtual void collisionConstraint(Eigen::MatrixXd& particle_pos);

    
    virtual void renderRenderGeometry(
        igl::opengl::glfw::Viewer &viewer) override {
            
        viewer.data().set_mesh(m_renderV, m_renderF);
        viewer.data().set_colors(m_renderC);
    }

    virtual void assignParticles();

    virtual void updateAdjacencyList(Eigen::MatrixXi m_renderF);

#pragma region SettersAndGetters

    void setLogFrequency(int f) { m_log_frequency = f; }

#pragma endregion SettersAndGetters

   private:
    
    RigidObject *p_obj;
	//RigidObject *p_plane;

    
    Eigen::MatrixXd m_renderV;  // vertex positions for rendering
    Eigen::MatrixXi m_renderF;  // face indices for rendering
    Eigen::MatrixXd m_renderC;  // face colors for rendering

    Eigen::MatrixXd p_pos;
    Eigen::MatrixXd p_vel;

    Eigen::MatrixXd xp;
    Eigen::Vector3d pp;
    Eigen::Vector3d p_vel1;
    Eigen::Vector3d m_gravity;
    
    int num_steps = 1;
    float alpha = 0.0;
    float particle_radius = 1.0;
    Eigen::MatrixXi m_renderE;
    Eigen::VectorXd edge_dist;

    int m_log_frequency;  // how often should we log the COM in the GUI
    Eigen::RowVector3d m_color;
};
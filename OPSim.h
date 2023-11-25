#include <igl/readOBJ.h>
#include "Simulation.h"

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
        m_particleSim.clear();
        m_particleColors.clear();
        m_color = Eigen::RowVector3d(1.0, 0.0, 0.0);
    }

    virtual void init() override {
        std::string path = "cube.off";
        //m_objects.clear();
        //m_objects.push_back(RigidObject(path));
        //p_obj = &m_objects.back();
        loadOBJ();
        reset();
    }

    virtual void resetMembers() override {
        p_obj->reset();
        m_particleSim.clear();
        m_particleColors.clear();
        assignParticles();
    }

    virtual void updateRenderGeometry() override {
        p_obj->getMesh(m_renderV, m_renderF);
        p_obj->getColors(m_renderC);
    }

	virtual bool advance() override;

    virtual void loadOBJ();


    virtual void renderRenderGeometry(
        igl::opengl::glfw::Viewer &viewer) override {
        
        viewer.data().set_mesh(m_renderV, m_renderF);
        viewer.data().set_colors(m_renderC);

        // Render oriented particles
        for (size_t i = 0; i < m_particleSim.size(); i++) {
            viewer.data().add_points(m_particleSim[i].transpose(), m_particleColors[i]);
        }
           
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

    Eigen::MatrixXd m_renderTC; // Texture Coordinates
    Eigen::MatrixXd m_renderN;  // Normals
    Eigen::MatrixXi m_renderFTC;  // not needed
    Eigen::MatrixXi m_renderFN;  // not needed

    int m_log_frequency;  // how often should we log the COM in the GUI
    Eigen::RowVector3d m_color;

    vector<Eigen::Vector3d> m_particleSim;
    vector<Eigen::RowVector3d> m_particleColors;
};
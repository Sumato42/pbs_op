#include <igl/readOBJ.h>
#include "Simulation.h"
#include <igl/edges.h>
#include "Particle.h" // Oriented particle data structure
#include <unordered_set> // For adjacency list
#include "Hash.h"

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
    OPSim(igl::opengl::glfw::Viewer& viewer) : Simulation() {
        init(viewer);
        m_particleSim.clear();
        m_particleColors.clear();
        m_color = Eigen::RowVector3d(1.0, 0.0, 0.0);
    }

    virtual void init(igl::opengl::glfw::Viewer &viewer) {
        loadSceneMeshes();
        for(int i = 1; i < m_objects.size(); i++){
            viewer.append_mesh();
        }
        m_gravity = Eigen::Vector3d(0, -9.81, 0);
        m_dt = 0.1/num_steps;
    }

    virtual void loadSceneMeshes(){

        // COLLISION SCENE
        m_gravity = Eigen::Vector3d(0, 0, 0);

        m_particles.clear();
        m_particleSim.clear();
        m_particleColors.clear();
        int num_particles_per_dim = 8+2;
        double cube_size = 2;

        Eigen::Vector3d start = Eigen::Vector3d(-cube_size/2, 0, -cube_size/2);
        Eigen::Vector3d step_X = Eigen::Vector3d(cube_size, 0, 0) / num_particles_per_dim;
        Eigen::Vector3d step_Y = Eigen::Vector3d(0, cube_size, 0) / num_particles_per_dim;
        Eigen::Vector3d step_Z = Eigen::Vector3d(0, 0, cube_size) / num_particles_per_dim;
        Eigen::Vector3d position;
        for (int i = 1; i < num_particles_per_dim-1; i++) {
            for (int j = 1; j < num_particles_per_dim-1; j++) {
                for (int k = 1; k < num_particles_per_dim-1; k++) {
                    position = start + i * step_X + j * step_Y + k * step_Z;
                    Particle part;
                    part.setPosition(position);
                    part.setVelocity(Eigen::Vector3d::Random());
                    m_particles.push_back(part);
                    m_particleSim.push_back(position);
                    m_particleColors.push_back(Eigen::Vector3d(1, 0, 0));
                }
            }
        }
        
        /*

        // CUBE SCENE
        
        std::string path = "cube.off";
        m_objects.clear();
        //cube_0
        m_objects.push_back(RigidObject(path));
        m_objects[0].setPosition(Eigen::Vector3d(2, 2, 0));

        //cube_1
        m_objects.push_back(RigidObject(path));
        m_objects[1].setPosition(Eigen::Vector3d(0, 5, 0));

        //cube_2
        m_objects.push_back(RigidObject(path));
        m_objects[2].setPosition(Eigen::Vector3d(0, 1, -2));

        
        

        // used if we want only one huge single mesh istead of multiple ones
        combineMeshes();
        */
        
    }

    virtual void combineMeshes(){
        if (m_objects.size() == 0) {
            std::cout << m_objects.size() << std::endl;
            return;
        }
        Eigen::MatrixXd V1;
        Eigen::MatrixXi F1;

        int rows_combinedV = 0;
        int rows_combinedF = 0;
        int cols_V = 3;
        int cols_F = 3;

        for (size_t i = 0; i < m_objects.size(); ++i)
        {   
            m_objects[i].getMesh(V1, F1);
            rows_combinedV += V1.rows();
            rows_combinedF += F1.rows();
        }

        // Concatenate vertices
        Eigen::MatrixXd V_combined(rows_combinedV, cols_V);
        Eigen::MatrixXi F_combined(rows_combinedF, cols_F);
        int current_rowV = 0;
        int current_rowF = 0;
        for (size_t i = 0; i < m_objects.size(); ++i)
        {
            m_objects[i].getMesh(V1, F1);
            V_combined.block(current_rowV, 0, V1.rows(), cols_V) = V1;
            F_combined.block(current_rowF, 0, F1.rows(), cols_F) = F1.array() + current_rowV;
            current_rowV += V1.rows();
            current_rowF += F1.rows();
        }
        m_renderV = V_combined;
        m_renderF = F_combined;

        // clear m_objects such that we can load a placeholder RigidObject 
        // and later set the mesh of this object to the combined meshes
        m_objects.clear();
        m_objects.push_back(RigidObject("cube.off"));
        p_obj = &m_objects.back();
        p_obj->setMesh(m_renderV, m_renderF);
    }

    virtual void resetMembers() override {

        //Reseting the objects by loading them again
        loadSceneMeshes();

        /* Needed if we want to avoid to combine all meshes to a hube single one

        //reset cube_0
        m_objects[0].reset();
        m_objects[0].setPosition(Eigen::Vector3d(2, 2, 0));

        //reset cube_1
        p_obj->reset();   
        p_obj->setPosition(Eigen::Vector3d(0, 5, 0));
        p_obj->getMesh(m_renderV, m_renderF);

        */
        
        // assign particles after reseting the objects
        //m_particleSim.clear();
        //m_particleColors.clear();
        //assignParticles();
        
        // set the initial parameter values for PBD
        //xp = p_pos;
        //p_vel = Eigen::MatrixXd::Zero(xp.rows(), 3);
        std::cout <<"Num Particles: "<< m_particles.size() << endl;
        m_particleHash = new Hash(2 * particle_radius, m_particles.size());
        m_particleHash->create(m_particles);

        // Collision example
        //std::cout << m_particleSim.size() << std::endl;
        m_renderV = Eigen::MatrixXd(m_particles.size(), 3);
        p_vel = Eigen::MatrixXd(m_particles.size(), 3);
        for (int i = 0; i < m_particles.size(); i++) {
            m_renderV.row(i) = m_particles[i].getPosition();
            p_vel.row(i) = m_particles[i].getVelocity();
        }
         
        xp = m_renderV;
        
        if (m_renderF.rows() != 0) {
            igl::edges(m_renderF, m_renderE);
            edge_dist = Eigen::VectorXd(m_renderE.rows());

            for (int i = 0; i < edge_dist.rows(); i++) {
                Eigen::Vector2i edge = m_renderE.row(i);
                int x = edge.x();
                int y = edge.y();
                int rows = m_renderV.rows();
                edge_dist[i] = (m_renderV.row(edge.x()) - m_renderV.row(edge.y())).norm();
            }
            std::cout << "Faces: " << m_renderF.rows() << std::endl;
        }
        
                
        // updateAdjacencyList();
    }

    virtual void updateRenderGeometry() override {
        //p_obj->getMesh(m_renderV, m_renderF);
        //p_obj->getColors(m_renderC);
    }

	virtual bool advance() override;

    virtual void predict();

    virtual void solve();

    virtual void update();

    virtual void cubeCollision(int pid);


    virtual Eigen::Vector3d groundConstraint(Eigen::Vector3d pa_pos);

    virtual void distanceConstraint(Eigen::MatrixXd& pa_pos, Eigen::MatrixXi edges);

    virtual void collisionConstraint(int pid1, int pid2);

    
    virtual void renderRenderGeometry(
        igl::opengl::glfw::Viewer &viewer) override {
        // Used if scene contains meshes
        if (m_objects.size() != 0) {
            viewer.data().set_mesh(m_renderV, m_renderF);
            viewer.data().set_colors(Eigen::RowVector3d(0.0, 0.0, 1.0));
        }
        
        /* Code for using different meshes instead of a huge mesh
        int cube_0 = viewer.data_list[0].id;
        int cube_1 = viewer.data_list[1].id;

        //show different objects in the scene by changing the selected_data_index

        //cube_0
        Eigen::MatrixXd G;
        Eigen::MatrixXi H;
        m_objects[0].getMesh(G, H);
        viewer.selected_data_index = 0;
        viewer.data().set_mesh(G, H);
        viewer.data().set_colors(m_renderC);
        
        //cube_1
        viewer.selected_data_index = 1;
        viewer.data().set_mesh(m_renderV, m_renderF);
        viewer.data().set_colors(m_renderC);
        */        

        // Render oriented particles
        for (size_t i = 0; i < xp.rows(); i++) {
            viewer.data().add_points(xp.row(i), m_particleColors[i]);
        }
           
    }

    virtual void assignParticles();

    virtual void updateAdjacencyList(Eigen::MatrixXi m_renderF);

    virtual void addEdge(int v1, int v2);

    virtual bool trianglesAreNeighbors(const Eigen::Vector3i& tri1, const Eigen::Vector3i& tri2);

    virtual int getParticleIndexByPosition(const Eigen::Vector3d& position);

#pragma region SettersAndGetters

    void setLogFrequency(int f) { m_log_frequency = f; }

#pragma endregion SettersAndGetters

   private:
    
    RigidObject *p_obj;
	//RigidObject *p_plane;

    
    Eigen::MatrixXd m_renderV;  // vertex positions for rendering
    Eigen::MatrixXi m_renderF;  // face indices for rendering
    Eigen::MatrixXd m_renderC;  // face colors for rendering

    //Eigen::MatrixXd p_pos;
    Eigen::MatrixXd p_vel;

    Eigen::MatrixXd xp;
    Eigen::Vector3d pp;
    Eigen::Vector3d p_vel1;
    Eigen::Vector3d m_gravity;
    
    int num_steps = 1;
    float alpha = 0.0;
    float particle_radius = 0.05;
    Eigen::MatrixXi m_renderE;
    Eigen::VectorXd edge_dist;

    int m_log_frequency;  // how often should we log the COM in the GUI
    Eigen::RowVector3d m_color;

    vector<Eigen::Vector3d> m_particleSim;
    vector<Eigen::RowVector3d> m_particleColors;
    Hash* m_particleHash = nullptr;
    std::vector<Particle> m_particles; // Container for oriented particles
    // Define adjacency list as a vector of unordered sets
    //std::vector<std::unordered_set<int>> m_adjacencyList;

    // Define the adjacency matrix as a matrix of booleans
    Eigen::MatrixXi adjacencyMatrix;
};
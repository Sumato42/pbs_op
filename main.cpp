#include <igl/writeOFF.h>
#include <thread>
#include "Gui.h"
#include "Simulator.h"
#include "OPSim.h"

/*
 * GUI for the spring simulation. This time we need additional paramters,
 * e.g. which integrator to use for the simulation and the force applied to the
 * cube, and we also add some more visualizations (trajectories).
 */
class OPGui : public Gui {
   public:
    // global simulation parameters
    float example_param = 0;
    OPSim *p_opsim = NULL;
    OPGui() {
        /////// initialize the parameters for the simulation
        // this contains all parameter initialisation for objects in the scene
        // could be handled by a default initialization of the object
        

        /////// define a OPSim instance with reference

        // Example:
        // p_springSim = new SpringSim();
        // p_springSim->setSpring(m_spring);
        // setSimulation(p_springSim);
        p_opsim = new OPSim();
        setSimulation(p_opsim);

        /////// interactive part of the simulator
        // Example:
        //callback_clicked_vertex =
        //    [&](int clickedVertexIndex, int clickedObjectIndex,
        //        Eigen::Vector3d &pos, Eigen::Vector3d &dir) {
        //        RigidObject &o = p_springSim->getObjects()[clickedObjectIndex];
        //        pos = o.getVertexPosition(clickedVertexIndex);
        //        dir = o.getVelocity(pos);
        //    };
        
        ////// start the simulator
        start();
    }

    virtual void updateSimulationParameters() override {
        // change all parameters of the simulation to the values that are set in
        // the GUI

        // Example:
        // p_springSim->setTimestep(m_dt);
        
    }

    virtual void clearSimulation() override {
        // clear the simulation

        // Example:
        // p_springSim->clearTrajectories();
    }

    // Keyboard interaction for simulation control
    virtual bool childKeyCallback(igl::opengl::glfw::Viewer &viewer,
                                  unsigned int key, int modifiers) override {
        switch (key) {
            case 'E':
                return true;
            case 'e':
                return true;
        }
        return false;
    }

    virtual void drawSimulationParameterMenu() override {
        // Example of using Buttons with methods
        // if (ImGui::Button("Export Trajectories", ImVec2(-1, 0))) {
        //     exportTrajectories();
        // }
        ImGui::InputFloat("Example parameter", &example_param, 0, 0);
    }
};

int main(int argc, char *argv[]) {
    // create a new instance of the GUI for the simulation
    new OPGui();

    return 0;
}
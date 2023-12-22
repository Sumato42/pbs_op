
# Physically-Based Simulation Project: Adding Physics to Animated Characters with Oriented Particles
In this project, we implemented a physically based cloth simulation scene with consideration of the paper "Adding Physics to Animated Characters with Oriented Particles" by Müller et al.
Due to time constraints, we only managed to simulate the cloth using particles and Position Based Dynamics (PBD). We used Unity to improve the rendering pipeline of the scene and thus improve the performance of the simulation. All collision detection and simulation are handled in the PBS.cs file, which is assigned to a Unity GameObject so that the user can interact with the simulation during runtime.

# How to run our code:
1. Clone this repository or download the folder PBS_Unity.
2. In Unity, add the PBS_Unity project folder to your list of projects and open it. This might take a while to import all necessary dependencies.
3. In Unity, in the Project window, select "Assets/Scenes/SampleScene.unity" to open the simulation scene.
4. Run the simulation by pressing the play button at the top of the Unity window.

# How to interact with the simulation:
In Unity, in the hierarchy window, select the GameObject called "PBS". This should show the corresponding components in the Inspector window. Under PBS(Script), you should find a list of attributes. To simplify the interaction, press the lock symbol at the top right corner of the Inspector window to lock the interface.
The first two attributes are used for connecting the meshes of the scene to the simulation, where Object_Anim stores all animated GameObjects (running character) and Object_Sim all objects which should be simulated (Carpet_small).
The following attributes are used for parameter tuning:
- Gravitation: the gravitational force vector applied to all simulated objects in the scene.
- Num_substeps: the number of substeps used during each iteration to improve collision detection and simulation.
- M_dt: the timestep used for each substep (not for each iteration!)
- Particle_radius: radius for all particles
- *_Compliance: the compliance used for the corresponding constraint. If set to 0, the constraint is followed strictly. The larger the compliance is set, the less strict the constraint.

Lastly, there are the boolean parameters for showing the simulated particles and which constraints to use. Additionally, one can hold the animation of the character to see how the cloth will calm itself or how it will fall on top of the character.


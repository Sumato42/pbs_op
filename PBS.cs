using System.Collections;
using System.Collections.Generic;
using System.Linq;
using Unity.VisualScripting;
using UnityEngine;
using UnityEngine.Android;

public class Particle
{
    public Vector3 Position;
    public Vector3 OldPosition;
    public Vector3 Velocity;
    public Quaternion Orientation;
    public Quaternion Angular_velocity;
    public bool IsSimulationParticle = true;
    public float Radius;
    public GameObject Sphere;
    public Particle(Vector3 pos, float radius)
    {
        this.Position = pos;
        this.OldPosition = pos;
        this.Velocity = Vector3.zero;
        this.Orientation = Quaternion.identity;
        this.Angular_velocity = Quaternion.identity;
        this.Radius = radius;
        //this.Sphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        //this.Sphere.transform.position = pos;
        //this.Sphere.transform.localScale = Vector3.one * radius;
    }
}

public class Hash
{
    public double Spacing;
    public int TableSize;
    public int MaxNumObjects;
    public List<int> CellStart;
    public List<int> CellEntries;
    public int QuerySize;

    public List<int> QueryIds;

    public Hash(double spacing, int maxNumObjects)
    {
        this.Spacing = spacing;
        this.TableSize = 2 * maxNumObjects;
        this.CellStart = Enumerable.Repeat(0, this.TableSize + 1).ToList();
        this.CellEntries = Enumerable.Repeat(0, this.MaxNumObjects).ToList();

        //this.QuerySize = 0;
        this.MaxNumObjects = maxNumObjects;
    }

    public void create(List<Particle> particles)
    {
        int numObjects = Mathf.Min(particles.Count, this.CellEntries.Count);
        this.CellStart = Enumerable.Repeat(0, this.TableSize + 1).ToList();
        this.CellEntries = Enumerable.Repeat(0, this.MaxNumObjects).ToList();

        for (int i = 0; i < numObjects; i++)
        {
            int h = hashPos(particles[i].Position);
            this.CellStart[h]++;
        }

        int start = 0;
        for (int i = 0; i < this.TableSize; i++)
        {
            start += this.CellStart[i];
            this.CellStart[i] = start;
        }

        this.CellStart[this.TableSize] = start;

        for (int i = 0; i < numObjects; i++)
        {
            int h = hashPos(particles[i].Position);
            this.CellStart[h]--;
            this.CellEntries[this.CellStart[h]] = i;
        }
    }

    public void query(Particle particle, double maxDist)
    {
        Vector3 pos = particle.Position;
        int x0 = intCoord(pos.x - maxDist);
        int y0 = intCoord(pos.y - maxDist);
        int z0 = intCoord(pos.z - maxDist);

        int x1 = intCoord(pos.x + maxDist);
        int y1 = intCoord(pos.y + maxDist);
        int z1 = intCoord(pos.z + maxDist);

        //this.QuerySize = 0; 
        this.QueryIds = new List<int>();

        for (int xi = x0; xi <= x1; xi++)
        {
            for (int yi = y0; yi <= y1; yi++)
            {
                for (int zi = z0; zi <= z1; zi++)
                {
                    int h = hashCoords(xi, yi, zi);
                    int start = this.CellStart[h];
                    int end = this.CellStart[h + 1];
                    for (int i = start; i < end; i++)
                    {
                        this.QueryIds.Add(this.CellEntries[i]);
                        //QuerySize++;
                    }
                }
            }
        }

    }

    public int hashCoords(int xi, int yi, int zi)
    {
        int h = (xi * 92837111) ^ (yi * 689287499) ^ (zi * 283923481);
        return Mathf.Abs(h) % this.TableSize;
    }

    public int intCoord(double coord)
    {
        return (int)(Mathf.Floor((float)(coord / this.Spacing)));
    }

    public int hashPos(Vector3 pos)
    {
        return hashCoords(intCoord(pos.x), intCoord(pos.y), intCoord(pos.z));
    }
}

public class PBS : MonoBehaviour
{
    [SerializeField]
    GameObject[] Objects_Anim = null;
    [SerializeField]
    GameObject[] Objects_Sim = null;

    List<Particle> Particles_Anim = new();
    List<Particle> Particles_Sim = new();

    List<Animator> Animators = new();

    int[,] adjacencyMatrix;

    List<Mesh> Meshes_Anim = new();
    List<Mesh> Meshes_Sim = new();

    public Vector3 Gravitation = new Vector3(0, -9.81f, 0);
    public int Num_substep = 1;
    public float M_dt = 0.1f;
    public float Particle_radius = 0.05f;
    public float Particle_max_vel;
    public float Particle_max_travel_dist;

    public Hash ParticleHash;
    List<List<int>> AdjacencyList = new();
    List<List<float>> AdjacencyDistList = new();

    // Start is called before the first frame update
    void Start()
    {   
        Debug.Log("Start!");

        // setting the parameters for the simulation
        M_dt /= Num_substep;
        Particle_max_vel = Particle_radius / M_dt;
        Particle_max_travel_dist = Particle_max_vel * M_dt * Num_substep;
        Debug.Log("Max Velocity: " +  Particle_max_vel);
        Debug.Log("Max Travel Distance: " + Particle_max_travel_dist);
        
        if (Objects_Anim.Length != 0)
        {            
            // generate and store Particles
            foreach (GameObject m in Objects_Anim)
            {
                // Moved this block to assignParticles for getting access to the world coordinates of each point
                /*Transform beta_surface = m.transform.Find("Beta_Surface");
                SkinnedMeshRenderer m_render = beta_surface.GetComponent<SkinnedMeshRenderer>();
                Mesh beta_mesh = m_render.sharedMesh;
                Meshes_Anim.Add(beta_mesh);
                */
                Animators.Add(m.transform.GetComponent<Animator>());
                assingParticles(m, false);
            }
            Animators[0].enabled = false;
            Debug.Log("Animated Particles assigned!");
        }

        if (Objects_Sim.Length != 0)
        {
            foreach (GameObject m in Objects_Sim)
            {
                // Moved this block to assignParticles for getting access to the world coordinates of each point
                /* MeshFilter m_render = m.transform.GetComponent<MeshFilter>();
                Mesh beta_mesh = m_render.mesh;
                Meshes_Sim.Add(beta_mesh);
                */
                assingParticles(m, true);
            }
            Debug.Log("Simulated Particles assigned!");

            // Assign the adjacency matrix after particles are assigned
            adjacencyMatrix = new int[Particles_Sim.Count, Particles_Sim.Count];
            InitializeMatrixToZero(adjacencyMatrix);

            foreach (Mesh m in Meshes_Sim)
            {
                updateAdjacencyList(m);
            }

            for(int i = 0; i < Particles_Sim.Count; i++)
            {
                AdjacencyList.Add(new List<int>());
                AdjacencyDistList.Add(new List<float>());
                for(int j = 0; j < Particles_Sim.Count; j++)
                {
                    if (adjacencyMatrix[i,j] == 1)
                    {
                        AdjacencyList[i].Add(j);
                        AdjacencyDistList[i].Add((Particles_Sim[i].Position - Particles_Sim[j].Position).magnitude);
                    }
                }
            }

            Debug.Log("Simulated Adjacency list assigned!");
        }

        // create a hash
        ParticleHash = new Hash(Particle_radius, Particles_Sim.Count + Particles_Anim.Count);
        Debug.Log("Numbeer of particles: " + Particles_Sim.Count);

    }

    // Update is called once per frame
    void Update()
    {
        // Update animated particle positions
        AdvanceAnimation();
        foreach (GameObject m in Objects_Anim)
        {
            Transform beta_surface = m.transform.Find("Beta_Surface");
            SkinnedMeshRenderer m_render = beta_surface.GetComponent<SkinnedMeshRenderer>();
            Mesh beta_mesh = new Mesh();
            m_render.BakeMesh(beta_mesh);

            updateAnimParticles(m);
        }

        // Predict simulation particles position
        for (int i = 0; i < Num_substep; i++)
        {
            //////////////////////////////////////// prediction 
            for (int j = 0; j < Particles_Sim.Count; j++)
            {
                Particles_Sim[j].Velocity += Gravitation * M_dt;
                float vel = Particles_Sim[j].Velocity.magnitude;
                if(vel > Particle_max_vel)
                {
                    Particles_Sim[j].Velocity *= Particle_max_vel / vel;
                }
                Particles_Sim[j].Position = Particles_Sim[j].OldPosition + Particles_Sim[j].Velocity * M_dt;
            }   
            
            // update Hash
            ParticleHash.create(Enumerable.Concat(Particles_Sim, Particles_Anim).ToList());

            //////////////////////////////////////// solve constraints
            for (int j = 0; j < Particles_Sim.Count; j++)
            {
                groundConstraint(this.Particles_Sim[j]);
            }
            distanceConstraint();
            collisionConstraint();
            Particles_Sim[0].Position = Particles_Sim[0].OldPosition;
            Particles_Sim[10].Position = Particles_Sim[10].OldPosition;
            foreach (Particle p in Particles_Sim)
            {
                p.Velocity = (p.Position - p.OldPosition) / M_dt;
                p.OldPosition = p.Position;
            }
        }

        //////////////////////////////////////// update meshes
        foreach (GameObject m in Objects_Sim)
        {
            UpdateMeshFromParticles(m);
        }

    }

    void UpdateMeshFromParticles(GameObject m)
    {
        MeshFilter meshFilter = m.transform.GetComponent<MeshFilter>();
        if (meshFilter != null)
        {
            Mesh mesh = meshFilter.mesh;
            Vector3[] vertices = mesh.vertices;
            Vector3 object_pos;
            Quaternion object_rot;
            m.transform.GetPositionAndRotation(out object_pos, out object_rot);
            object_rot = Quaternion.Inverse(object_rot);
            // Update mesh vertices based on Particle positions
            for (int i = 0; i < Particles_Sim.Count; i++)
            {
                vertices[i] = Vector3.Scale(object_rot * (Particles_Sim[i].Position - m.transform.position), new Vector3(1 / m.transform.localScale.x, 1 / m.transform.localScale.y, 1 / m.transform.localScale.z));
                //Particles_Sim[i].Sphere.transform.position = Particles_Sim[i].Position;
            }

            // Assign updated vertices to the mesh
            mesh.vertices = vertices;
            mesh.RecalculateBounds();
            meshFilter.mesh = mesh;
            
        }
    }

    void InitializeMatrixToZero(int[,] matrix)
    {
        int rows = matrix.GetLength(0);
        int columns = matrix.GetLength(1);

        for (int i = 0; i < rows; i++)
        {
            for (int j = 0; j < columns; j++)
            {
                matrix[i, j] = 0;
            }
        }
    }

    void assingParticles(GameObject m, bool flag)
    {
        Mesh mesh = new Mesh();
        if(flag)
        {
            MeshFilter m_render = m.transform.GetComponent<MeshFilter>();
            mesh = m_render.mesh;
            Meshes_Sim.Add(mesh);
        }
        else
        {
            Transform beta_surface = m.transform.Find("Beta_Surface");
            SkinnedMeshRenderer m_render = beta_surface.GetComponent<SkinnedMeshRenderer>();
            m_render.BakeMesh(mesh);
            Meshes_Anim.Add(mesh);
        }
        
        Vector3[] vertices = mesh.vertices;
        Vector3 pos, object_pos;
        Quaternion object_rot;
        for (int i = 0; i < vertices.Length; i++)
        {
            
            m.transform.GetPositionAndRotation(out object_pos, out object_rot);
            pos = object_rot * Vector3.Scale(vertices[i], m.transform.localScale) + object_pos;
            Particle part = new Particle(pos, Particle_radius);

            if (flag)
            {
                Particles_Sim.Add(part);
                part.IsSimulationParticle = true;
            }
            else
            {
                Particles_Anim.Add(part);
                part.IsSimulationParticle = false;
            }

            // Moved to Particle such that the shperes positions can be updated according to the Particles
            /* GameObject sphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            sphere.transform.position = vertices[i];
            sphere.transform.localScale= Vector3.one*0.01f;
            */
        }
    }

    private void updateAnimParticles(GameObject m)
    {
        Transform beta_surface = m.transform.Find("Beta_Surface");
        SkinnedMeshRenderer m_render = beta_surface.GetComponent<SkinnedMeshRenderer>();
        Mesh beta_mesh = new Mesh();
        m_render.BakeMesh(beta_mesh);
        Vector3[] vertices = beta_mesh.vertices;
        Vector3 object_pos;
        Quaternion object_rot;
        for (int i = 0; i < vertices.Length; i++)
        {
            m.transform.GetPositionAndRotation(out object_pos, out object_rot);
            Particles_Anim[i].Position = object_rot * Vector3.Scale(vertices[i], m.transform.localScale) + object_pos; 
            //Particles_Anim[i].Sphere.transform.position = Particles_Anim[i].Position;
        }
    }

    void updateAdjacencyList(Mesh mesh)
    {
        int[] triangles = mesh.triangles;
        int numTriangles = triangles.Length / 3;

        for (int i = 0; i < numTriangles; ++i)
        {
            int index1 = triangles[i * 3];
            int index2 = triangles[i * 3 + 1];
            int index3 = triangles[i * 3 + 2];

            Particle v1 = Particles_Sim[index1];
            Particle v2 = Particles_Sim[index2];
            Particle v3 = Particles_Sim[index3];

            int i1 = Particles_Sim.IndexOf(v1);
            int i2 = Particles_Sim.IndexOf(v2);
            int i3 = Particles_Sim.IndexOf(v3);

            if (i1 != -1 && i2 != -1) addEdge(i1, i2);
            if (i2 != -1 && i3 != -1) addEdge(i2, i3);
            if (i3 != -1 && i1 != -1) addEdge(i3, i1);
        }
    }

    void addEdge(int v1, int v2)
    {
        if (v1 >= 0 && v1 < Particles_Sim.Count && v2 >= 0 && v2 < Particles_Sim.Count)
        {
            adjacencyMatrix[v1, v2] = 1;
            adjacencyMatrix[v2, v1] = 1;
        }
    }


    void groundConstraint(Particle p)
    {
        if (p.Position.y >= 0)
            return;
        p.Position.y = 0;
        return;
        /*float C = p.Position.y;
        Vector3 dC = new Vector3(0, 1, 0);
        float lambda = -C / dC.sqrMagnitude;
        p.Position += lambda * dC;
        */
    }

    void distanceConstraint()
    {
        // For each Particle, iterate over all neighbours and compair current distance to initial rest distance
        for(int i = 0; i < Particles_Sim.Count; i++)
        {
            for(int j = 0; j < AdjacencyList[i].Count; j++)
            {
                Vector3 pi = Particles_Sim[i].Position;
                Vector3 pj = Particles_Sim[AdjacencyList[i][j]].Position;
                float dist = (pi - pj).magnitude;
                float restDist = AdjacencyDistList[i][j];
                float C = dist - restDist;
                Vector3 dC1 = (pi - pj) /dist;
                Vector3 dC2 = (pj - pi) /dist;
                float lambda = -C/(dC1.sqrMagnitude + dC2.sqrMagnitude);
                Particles_Sim[i].Position += lambda * dC1;
                Particles_Sim[AdjacencyList[i][j]].Position += lambda * dC2;
            }
        }
    }

    void collisionConstraint()
    {
        // check for all particles for collisions with the help of the ParticleHash
        for (int i = 0; i < Particles_Sim.Count; i++)
        {
            ParticleHash.query(Particles_Sim[i], 2 * Particle_radius);

            foreach (int j in ParticleHash.QueryIds)
            {
                Vector3 pi = Particles_Sim[i].Position;
                Vector3 pj;
                if (j >= Particles_Sim.Count)
                {
                    int anim_j = j - Particles_Sim.Count;
                    pj = Particles_Anim[anim_j].Position;
                    Vector3 normal = pi - pj;
                    float dist = normal.magnitude;
                    //float mindist = Mathf.Min(dist, 2 * Particle_radius);
                    if (dist > 0 && dist < 2 * Particle_radius)
                    {
                        float C = 2 * Particle_radius - dist;
                        Vector3 dC1 = normal / dist * C;
                        Vector3 dC2 = -normal / dist * C;
                        Particles_Sim[i].Position += dC1/2;

                        Vector3.Cross(dC1, dC2);
                        float v1 = Vector3.Dot(Particles_Sim[i].Velocity, normal);

                        // should be zero since we assume no velocity
                        //float v2 = Vector3.Dot(Particles_Sim[j].Velocity, normal);
                        float v2 = 0.0f;
                        Particles_Sim[i].Velocity += normal * (v2 - v1);
                    }
                }
                else
                {
                    pj = Particles_Sim[j].Position;
                    Vector3 normal = pi - pj;
                    float dist = Mathf.Sqrt(normal.sqrMagnitude);
                    if (dist > 0 && dist < 2 * Particle_radius)
                    {
                        float C = 2 * Particle_radius - dist;
                        Vector3 dC1 = normal / dist * C;
                        Vector3 dC2 = -normal / dist * C;
                        Particles_Sim[i].Position += dC1/2;
                        Particles_Sim[j].Position += dC2/2;
                        // ToDo Cloth Friction
                        Vector3.Cross(dC1, dC2);
                        float v1 = Vector3.Dot(Particles_Sim[i].Velocity, normal);
                        float v2 = Vector3.Dot(Particles_Sim[j].Velocity, normal);

                        Particles_Sim[i].Velocity += normal * (v2 - v1);
                        Particles_Sim[j].Velocity += normal * (v1 - v2);
                    }
                }
            }
        }
    }

    public void AdvanceAnimation()
    {
        if(Animators.Count == 0) {
            return;
        }
        float currentNormalizedTime = GetCurrentNormalizedTime();
        float newNormalizedTime = currentNormalizedTime + M_dt * Num_substep;
        Animators[0].Play("Jog In Circle", -1, newNormalizedTime);
        Animators[0].Update(0); // Manually update the animator to apply the new time
    }

    // Get the current normalized time of the animation
    private float GetCurrentNormalizedTime()
    {
        AnimatorStateInfo stateInfo = Animators[0].GetCurrentAnimatorStateInfo(0);
        return stateInfo.normalizedTime;
    }
}
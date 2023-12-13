using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;


public class Particle
{
    public Vector3 Position;
    public Vector3 OldPosition;
    public Vector3 Velocity;
    public Quaternion Orientation;
    public Quaternion Angular_velocity;
    public bool IsSimulationParticle = true;
    public Particle(Vector3 pos)
    {
        this.Position = pos;
        this.OldPosition = pos;
        this.Velocity = Vector3.zero;
        this.Orientation = Quaternion.identity;
        this.Angular_velocity = Quaternion.identity;
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
    GameObject[] Objects_Sim = null;

    List<Particle> Particles_Anim = new();
    List<Particle> Particles_Sim= new();

    int[,] adjacencyMatrix;

    List<Mesh> Meshes_Anim = new();
    List<Mesh> Meshes_Sim= new();

    public Vector3 Gravitation = new Vector3(0, -9.81f, 0);
    public int Num_substep = 1;
    public float M_dt = 0.1f;
    public float Particle_radius = 0.05f;

    public Hash ParticleHash;

    // Start is called before the first frame update
    void Start()
    {
        Debug.Log("Start!");
        if (Objects_Anim.Length == 0)
        {
            return;
        }

        // setting the parameters for the simulation
        M_dt /= Num_substep;

        // generate and store Particles
        foreach (GameObject m in Objects_Anim)
        {
            Transform beta_surface = m.transform.Find("Beta_Surface");
            SkinnedMeshRenderer m_render = beta_surface.GetComponent<SkinnedMeshRenderer>();
            Mesh beta_mesh = m_render.sharedMesh;
            Meshes_Anim.Add(beta_mesh);

            assingParticles(beta_mesh, false);
        }
        Debug.Log("Animated Particles assigned!");

        if(Objects_Sim.Length != 0)
        {
            foreach (GameObject m in Objects_Sim)
            {
                Transform beta_surface = m.transform.Find("Beta_Surface");
                SkinnedMeshRenderer m_render = beta_surface.GetComponent<SkinnedMeshRenderer>();
                Mesh beta_mesh = m_render.sharedMesh;
                Meshes_Sim.Add(beta_mesh);

                assingParticles(beta_mesh, true);
            }
            Debug.Log("Simulated Particles assigned!");

            // Assign the adjacency matrix after particles are assigned
            adjacencyMatrix = new int[Particles_Sim.Count, Particles_Sim.Count];
            InitializeMatrixToZero(adjacencyMatrix);

            foreach (Mesh m in Meshes_Sim)
            {
                updateAdjacencyList(m);
            }

            Debug.Log("Simulated Adjacency list assigned!");
        }

        // create a hash
        ParticleHash = new Hash(Particle_radius, Particles_Sim.Count + Particles_Anim.Count);
        Debug.Log(Particles_Sim.Count);

    }

    // Update is called once per frame
    void Update()
    {
        // Update animated particle positions
        foreach (Mesh m in Meshes_Anim)
        {
            updateAnimParticles(m);
        }

        // Predict simulation particles position
        for (int i = 0; i < Num_substep; i++)
        {
            // update Hash
            ParticleHash.create(Enumerable.Concat(Particles_Sim, Particles_Anim).ToList());
            // prediction 
            for (int j = 0; j < Particles_Sim.Count; j++)
            {
                Particles_Sim[j].Position = Particles_Sim[j].OldPosition + Particles_Sim[j].Velocity * M_dt;
            }
            // solve constraints
            for (int j = 0; j < Particles_Sim.Count; j++)
            {
                groundConstraint(this.Particles_Sim[j].Position);
            }
            collisionConstraint();
            // update meshes 
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

    void assingParticles(Mesh mesh, bool flag)
    {
        Vector3[] vertices = mesh.vertices;

        for (int i = 0; i < vertices.Length; i++)
        {
            Particle part = new Particle(vertices[i]);
            if(flag)
            {
                Particles_Sim.Add(part);
                part.IsSimulationParticle = true;
            } else {
                Particles_Anim.Add(part);
                part.IsSimulationParticle = false;
            }
            
            /* GameObject sphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            sphere.transform.position = vertices[i];
            sphere.transform.localScale= Vector3.one*0.01f;
            */
        }
    }

    private void updateAnimParticles(Mesh mesh)
    {
        Vector3[] vertices = mesh.vertices;
        for (int i = 0; i < vertices.Length; i++)
        {
            Particles_Anim[i].Position = vertices[i];
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


    void groundConstraint(Vector3 pos)
    {
        if (pos.y >= 0)
            return;
        float C = pos.y;
        Vector3 dC = new Vector3(0, 1, 0);
        float lambda = -C / dC.sqrMagnitude;
        pos += lambda * dC;
    }

    void distanceConstraint()
    {
        // Need the Edges of the mesh and the corresponding distances
    }

    void collisionConstraint()
    {
        // check for all particles for collisions with the help of the ParticleHash
        bool collision_particle_belongs_to_anim = false;
        for (int i = 0; i < Particles_Sim.Count; i++)
        {
            ParticleHash.query(Particles_Sim[i], 2 * Particle_radius);

            foreach (int j in ParticleHash.QueryIds)
            {
                Vector3 pi = Particles_Sim[i].Position;
                Vectro3 pj;
                if (j >= Particles_Sim.Count)
                {
                    collision_particle_belongs_to_anim = true;
                    j -= Particles_Sim.Count;
                    Vector3 pj = Particles_Anim[j].Position;
                    Vector3 normal = pi - pj;
                    float dist = Mathf.Sqrt(normal.sqrMagnitude);
                    if (dist > 0 && dist < 2 * Particle_radius)
                    {
                        float C = dist - 2 * Particle_radius;
                        Vector3 dC1 = normal / dist * C;
                        Vector3 dC2 = -normal / dist * C;
                        Particles_Sim[i].Position += dC1;

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
                    Vector3 pj = Particles_Sim[j].Position;
                    Vector3 normal = pi - pj;
                    float dist = Mathf.Sqrt(normal.sqrMagnitude);
                    if (dist > 0 && dist < 2 * Particle_radius)
                    {
                        float C = dist - 2 * Particle_radius;
                        Vector3 dC1 = normal / dist * C;
                        Vector3 dC2 = -normal / dist * C;
                        Particles_Sim[i].Position += dC1;
                        Particles_Sim[j].Position += dC2;
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

}
using System.Collections;
using System.Collections.Generic;
using UnityEngine;


public class Particle
{
    public Vector3 Position;
    public Vector3 OldPosition;
    public Vector3 Velocity;
    public Quaternion Orientation;
    public Quaternion Angular_velocity;
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

    public Hash(double spacing, int maxNumObjects){
        this.Spacing = spacing;
        this.TableSize = 2 * maxNumObjects;
        this.CellStart = new List<int>(this.TableSize + 1);
        this.CellEntries = new List<int>(maxNumObjects);
        //this.QuerySize = 0;
        this.MaxNumObjects = maxNumObjects;
    }

    public void create(List<Particle> particles){
        int numObjects = Math.Min(particles.Length, this.CellEntries.Length);
        this.CellStart = new List<int>(this.TableSize + 1);
        this.CellEntries = new List<int>(this.MaxNumObjects);

        for (int i = 0; i < numObjects; i++){
            int h = hashPos(particles[i].Position);
            this.CellStart[h]++;
        }

        int start = 0;
        for(int i = 0; i < this.TableSize; i++){
            start += this.CellStart[i];
            this.CellStart[i] = start;
        }
        this.CellStart[this.TableSize] = start;

        for(int i = 0; i < numObjects; i++){
            int h = hashPos(particles[i].Position);
            this.CellStart[h]--;
            this.CellEntries[this.CellStart[h]] = i;
        }
    }

    public void query(Particle particle, double maxDist){
        Vectro3 pos = particle.Position;
        int x0 = intCoord(pos.x - maxDist);
        int y0 = intCoord(pos.y - maxDist);
        int z0 = intCoord(pos.z - maxDist);

        int x1 = intCoord(pos.x + maxDist);
        int y1 = intCoord(pos.y + maxDist);
        int z1 = intCoord(pos.z + maxDist);

        //this.QuerySize = 0; 
        this.QueryIds = new List<int>();
        for(int xi = x0; xi <= x1; xi++){
            for(int yi = y0; yi <= y1; y0++){
                for(int zi = z0; zi <= z1; zi++){
                    int h = hashCoords(xi, yi, zi);
                    int start = this.CellStart[h];
                    int end = this.CellStart[h + 1];

                    for(int i = start; i < end; i++){
                        this.QueryIds.Add(this.CellEntries[i]);
                        //QuerySize++;
                    }
                }
            }
        }
    }

    public int hashCoords(int xi, int yi, int zi){
        int h = (xi * 92837111) ^ (yi * 689287499) ^ (zi * 283923481);
        return Math.Abs(h) % this.TableSize;
    }

    public int intCoord(double coord){
        return int(Math.floor(coord / this.Spacing));
    }

    public int hashPos(const Vector3 pos){
        return hashCoords(intCoord(pos.x), intCoord(pos.y), intCoord(pos.z));
    }
}

public class PBS : MonoBehaviour
{
    [SerializeField]
    GameObject[] M_Objects = null;

    List<Particle> M_Particles = new();

    public Vector3 Gravitation = new Vector3(0, -9.81, 0);
    public int Num_substep = 1;
    public double M_dt = 0.1;
    public float Particle_radius = 0.05;

    public Hash ParticleHash;

    // Start is called before the first frame update
    void Start()
    {
        Debug.Log("Start!");
        if (M_Objects.Length == 0) {
            return;
        }

        // setting the parameters for the simulation
        M_dt /= Num_substep;
        
        // generate and store Particles
        foreach(GameObject m in M_Objects)
        {
            Transform beta_surface = m.transform.Find("Beta_Surface");
            SkinnedMeshRenderer m_render = beta_surface.GetComponent<SkinnedMeshRenderer>();
            Mesh beta_mesh = m_render.sharedMesh;
            
            assingParticles(beta_mesh);
        }
        Debug.Log("Particles assigned!");

        // create a hash
        ParticleHash = new Hash(Particle_radius, M_Particles.Length);

    }

    // Update is called once per frame
    void Update()
    {
        for (int i = 0; i < Num_substep; i++){
            // update Hash
            ParticleHash.create(M_Particles);
            // prediction 
            for(int i = 0; i < M_Particles.Length; i++){
                M_Particles[i].Position = M_Particles[i].OldPosition + M_Particles[i].Velocity * M_dt;
            }
            // solve constraints
            for(int i = 0; i < M_Particles.Length; i++){
                groundConstraint(M_Particles[i].Position);
            }
            collisionConstraint();
            // update meshes 
        }
        
    }

    void assingParticles(Mesh mesh)
    {
        Vector3[] vertices = mesh.vertices;
        
        for(int i = 0; i < vertices.Length; i++)
        {
            Particle part = new Particle(vertices[i]);
            M_Particles.Add(part);
            /* GameObject sphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            sphere.transform.position = vertices[i];
            sphere.transform.localScale= Vector3.one*0.01f;
            */
        }
    }

    void groundConstraint(Vector3& pos)
    {   
        if(pos.y >= 0)
            return;
        float C = pos.y();
        Vector3 dC = new Vector3(0, 1, 0);
        float lambda = -C/(dC.norm()*dC.norm())
        pos += lambda * dC;
    }

    void distanceConstraint()
    {
        // Need the Edges of the mesh and the corresponding distances
    }

    void collisionConstraint()
    {
        // check for all particles for collisions with the help of the ParticleHash
        for (int i = 0; i < M_Particles.Length; i++){
            ParticleHash.query(Particle[i].Position, 2 * Particle_radius);
            foreach(int j in ParticleHash.QueryIds){
                Vector3 pi = M_Particles[i].Position;
                Vector3 pj = M_Particles[j].Position;
                Vector3 normal = pi - pj;
                float dist = normal.norm();
                if(dist > 0 and 2 * Particle_radius){
                    float C = dist - 2 * Particle_radius;
                    Vector3 dC1 = normal / dist * C;
                    Vector3 dC2 = -normal / dist * C;
                    M_Particles[i].Position += dC1;
                    M_Particles[j].Position += dC2;

                    double v1 = M_Particles[i].Velocity.dot(normal);
                    double v2 = M_Particles[j].Velocity.dot(normal);

                    M_Particles[i].Velocity += normal * (v2 - v1);
                    M_Particles[j].Velocity += normal * (v1 - v2);
                }
            }
        }
    }

}

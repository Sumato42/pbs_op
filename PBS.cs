using System.Collections;
using System.Collections.Generic;
using UnityEngine;


public class Particle
{
    public Vector3 Position;
    public Vector3 Velocity;
    public Quaternion Orientation;
    public Quaternion Angular_velocity;
    public Particle(Vector3 pos)
    {
        this.Position = pos;
        this.Velocity = Vector3.zero;
        this.Orientation = Quaternion.identity;
        this.Angular_velocity = Quaternion.identity;
    }
}

public class PBS : MonoBehaviour
{
    [SerializeField]
    GameObject[] M_Objects = null;

    List<Particle> M_Particles = new();
    // Start is called before the first frame update
    void Start()
    {
        Debug.Log("Start!");
        if (M_Objects.Length == 0) {
            return;
        }

        foreach(GameObject m in M_Objects)
        {
            Transform beta_surface = m.transform.Find("Beta_Surface");
            SkinnedMeshRenderer m_render = beta_surface.GetComponent<SkinnedMeshRenderer>();
            Mesh beta_mesh = m_render.sharedMesh;
            
            assingParticles(beta_mesh);
        }
        Debug.Log("Particles assigned!");

        // generate and store Particles
        // create a hash
    }

    // Update is called once per frame
    void Update()
    {
        // update Hash
        // prediction 
        // solve constraints
        // update meshes
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
}

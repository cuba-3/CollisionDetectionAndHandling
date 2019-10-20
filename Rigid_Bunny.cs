using UnityEngine;
using System.Collections;

public class Rigid_Bunny : MonoBehaviour 
{
	public bool animated;
	public bool launched;
	public bool collided;

	Vector3[] vertices;
	Mesh mesh;

	public float dt = 0.02f;
	
	public Vector3 v;							// velocity
	public Vector3 w;							// angular velocity
	
	public float m;
	public float mass;							// mass
	public Matrix4x4 I_body;					// body inertia

	public float linear_damping;				// for damping
	public float angular_damping;
	public float restitution;					// for collision
						
	public Vector3 N_ground = new Vector3(0, 1, 0);
	public Vector3 N_wall = new Vector3(-1, 0, 0);

    public Vector3 pos;							// position
	public Quaternion q; 						// transform.rotation
	Vector3 pos_init;

	// Use this for initialization
	void Start () 
	{
		//Initialize coefficients
		linear_damping  = 0.999f;
		angular_damping = 0.98f;
		restitution 	= 0.5f;		//elastic collision
		m 				= 1;
		mass 			= 0;

		//w = new Vector3 (0, 0, 0);
		w = new Vector3(5, 0, 0);
		v = new Vector3(0, 0, 0);
		pos_init = transform.position;
		pos = transform.position;
		q = Quaternion.identity;
		
		launched = false;
		collided = false;
	
		mesh = GetComponent<MeshFilter>().mesh;
	    vertices = mesh.vertices;
		for (int i=0; i<vertices.Length; i++) 
		{
			mass += m;
			float diag=m*vertices[i].sqrMagnitude;
			I_body[0, 0]+=diag;
			I_body[1, 1]+=diag;
			I_body[2, 2]+=diag;
			I_body[0, 0]-=m*vertices[i][0]*vertices[i][0];
			I_body[0, 1]-=m*vertices[i][0]*vertices[i][1];
			I_body[0, 2]-=m*vertices[i][0]*vertices[i][2];
			I_body[1, 0]-=m*vertices[i][1]*vertices[i][0];
			I_body[1, 1]-=m*vertices[i][1]*vertices[i][1];
			I_body[1, 2]-=m*vertices[i][1]*vertices[i][2];
			I_body[2, 0]-=m*vertices[i][2]*vertices[i][0];
			I_body[2, 1]-=m*vertices[i][2]*vertices[i][1];
			I_body[2, 2]-=m*vertices[i][2]*vertices[i][2];
		}
		I_body [3, 3] = 1;
	}
	
	Matrix4x4 Get_Cross_Matrix(Vector3 a)
	{
		//Get the cross product matrix of vector a
		Matrix4x4 A = Matrix4x4.zero;
		A [0, 0] = 0; 
		A [0, 1] = -a[2]; 
		A [0, 2] = a[1]; 
		A [1, 0] = a[2]; 
		A [1, 1] = 0; 
		A [1, 2] = -a[0]; 
		A [2, 0] = -a[1]; 
		A [2, 1] = a[0]; 
		A [2, 2] = 0; 
		A [3, 3] = 1;
		return A;
	}


	// Update is called once per frame
	void Update () 
	{
		Quaternion p;
		
		// reset
		if(Input.GetKey(KeyCode.R)){
			launched = false;
			transform.position = pos_init;
			v = new Vector3(0, 0, 0);
			w = new Vector3(5, 0, 0);
		}

		//launch
		if(Input.GetKey(KeyCode.L)){
			v = new Vector3(4, 3, 0);
			launched = true;
		}

		if(launched){
			Vector3 forces = new Vector3(0, -9.8f, 0) * linear_damping;
			v.x = v.x + dt * forces.x;
			v.y = v.y + dt * forces.y;
			v.z = v.z + dt * forces.z;
  			w = w*angular_damping;
			CollisionHandler(N_ground);
			CollisionHandler(N_wall);
		}
		

		// Part III: Update position & orientation
		p.x = dt * w.x * 0.5f;
		p.y = dt * w.y * 0.5f;
		p.z = dt * w.z * 0.5f;
		p.w = 0;
		Quaternion pq;
		pq = p * q;
		q.x += pq.x;
		q.y += pq.y;
		q.z += pq.z;
		q.w += pq.w;

		// Part IV: Assign to the bunny object
		transform.position += dt * v;
		transform.rotation = q;

		pos = transform.position;
		q = transform.rotation;	
	}

	public void CollisionHandler(Vector3 N){
		Matrix4x4 R = Matrix4x4.Rotate(q);
		Vector3 ri_sum = new Vector3(0, 0, 0);
		Mesh mesh = GetComponent<MeshFilter>().mesh;
		int c = 0;
		Vector3 ri;
		Vector3 vi;

		Vector3[] vertices = GetComponent<MeshFilter>().mesh.vertices;
		for(int i = 0; i < vertices.Length; i++){
			Vector3 vertex = vertices[i];
			ri = R * vertex; // should be using multiply3x4?
			Vector3 xi = ri + pos;
			vi = v + (Vector3.Cross(w, ri));

			if(N == N_wall){
				if (Vector3.Dot(xi, N) + 2 < 0 && Vector3.Dot(vi, N) < 0){
					ri_sum = ri_sum + ri;
					c += 1;
				}
			} else if(N == N_ground){
				if (Vector3.Dot(xi, N) < 0 && Vector3.Dot(vi, N) < 0) {
					ri_sum = ri_sum + ri;
					c += 1;
				}
			}
		}

		if (c != 0) {
			ri = ri_sum / c;
			vi = v + (Vector3.Cross(w, ri));
			Matrix4x4 Ri = Get_Cross_Matrix(ri);
			Matrix4x4 I = R*I_body*R.transpose;
			
			//Find Matrix K
			Matrix4x4 mass_identity = Matrix4x4.zero;
            mass_identity[0, 0] = 1/mass;
            mass_identity[1, 1] = 1/mass;
            mass_identity[2, 2] = 1/mass;
            mass_identity[3, 3] = 1/mass;
            Matrix4x4 matrix2 = Ri*I.inverse*Ri;
            Matrix4x4 K = Matrix4x4.zero;
            K[0, 0] = mass_identity[0, 0] - matrix2[0, 0];
            K[0, 1] = mass_identity[0, 1] - matrix2[0, 1];
            K[0, 2] = mass_identity[0, 2] - matrix2[0, 2];
            K[0, 3] = mass_identity[0, 3] - matrix2[0, 3];
            K[1, 0] = mass_identity[1, 0] - matrix2[1, 0];
            K[1, 1] = mass_identity[1, 1] - matrix2[1, 1];
            K[1, 2] = mass_identity[1, 2] - matrix2[1, 2];
            K[1, 3] = mass_identity[1, 3] - matrix2[1, 3];
            K[2, 0] = mass_identity[2, 0] - matrix2[2, 0];
            K[2, 1] = mass_identity[2, 1] - matrix2[2, 1];
            K[2, 2] = mass_identity[2, 2] - matrix2[2, 2];
            K[2, 3] = mass_identity[2, 3] - matrix2[2, 3];
            K[3, 0] = mass_identity[3, 0] - matrix2[3, 0];
            K[3, 1] = mass_identity[3, 1] - matrix2[3, 1];
            K[3, 2] = mass_identity[3, 2] - matrix2[3, 2];
            K[3, 3] = mass_identity[3, 3] - matrix2[3, 3];

			float mu = 0.5f;
            if (Mathf.Abs(Vector3.Dot(vi, N)) < 0.2){
                mu = 0;
                v= new Vector3(0, 0, 0);
                w= new Vector3(0, 0, 0);
            }
            else{
				Vector3 j = K.inverse * (-vi - mu * Vector3.Dot(vi, N) * N);
				v = v + j/mass;
				w = w + (Vector3)(I.inverse * (Vector3.Cross(ri, j))); 
        	}
            
			// pos.x = pos.x + (0.02f * v.x);
            // pos.y = pos.y + (0.02f * v.y);
            // pos.z = pos.z + (0.02f * v.z);
            // transform.position = pos;
        }
       	collided = true;
	}
}
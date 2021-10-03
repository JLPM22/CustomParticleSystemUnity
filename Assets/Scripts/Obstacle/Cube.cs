using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace CustomParticleSystem
{
    public class Cube : Obstacle
    {
        private Vector3 N1 { get { return transform.up; } }
        private float D1 { get { return -Vector3.Dot(N1, transform.position + transform.up * transform.localScale.y * 0.5f); } }
        private Vector3 N2 { get { return -transform.up; } }
        private float D2 { get { return -Vector3.Dot(N2, transform.position - transform.up * transform.localScale.y * 0.5f); } }
        private Vector3 N3 { get { return transform.forward; } }
        private float D3 { get { return -Vector3.Dot(N3, transform.position + transform.forward * transform.localScale.z * 0.5f); } }
        private Vector3 N4 { get { return -transform.forward; } }
        private float D4 { get { return -Vector3.Dot(N4, transform.position - transform.forward * transform.localScale.z * 0.5f); } }
        private Vector3 N5 { get { return transform.right; } }
        private float D5 { get { return -Vector3.Dot(N5, transform.position + transform.right * transform.localScale.x * 0.5f); } }
        private Vector3 N6 { get { return -transform.right; } }
        private float D6 { get { return -Vector3.Dot(N6, transform.position - transform.right * transform.localScale.x * 0.5f); } }

        private Vector4[][] TrianglesLocalPos;

        private void Awake()
        {
            MeshFilter mf = GetComponent<MeshFilter>();
            if (mf != null)
            {
                int[] tris = mf.sharedMesh.triangles;
                if (tris.Length != 36)
                {
                    Debug.LogWarning("Cube mesh has " + tris.Length + " indices (tris=indices/3), expected 36 indices");
                    Destroy(this);
                    return;
                }
                Vector3[] vertices = mf.sharedMesh.vertices;
                TrianglesLocalPos = new Vector4[tris.Length / 3][];
                for (int i = 0; i < tris.Length; i += 3)
                {
                    int index = i / 3;
                    TrianglesLocalPos[index] = new Vector4[3];
                    Vector3 v1 = vertices[tris[i]];
                    TrianglesLocalPos[index][0] = new Vector4(v1.x, v1.y, v1.z, 1);
                    Vector3 v2 = vertices[tris[i + 1]];
                    TrianglesLocalPos[index][1] = new Vector4(v2.x, v2.y, v2.z, 1);
                    Vector3 v3 = vertices[tris[i + 2]];
                    TrianglesLocalPos[index][2] = new Vector4(v3.x, v3.y, v3.z, 1);
                }
            }
        }

        public override bool HasCollisionParticle(Particle p)
        {
            return Plane.IsNegativeSpacePlane(p, N1, D1) && Plane.IsNegativeSpacePlane(p, N2, D2) &&
                   Plane.IsNegativeSpacePlane(p, N3, D3) && Plane.IsNegativeSpacePlane(p, N4, D4) &&
                   Plane.IsNegativeSpacePlane(p, N5, D5) && Plane.IsNegativeSpacePlane(p, N6, D6);
        }

        public override void CorrectCollisionParticle(Particle p, float deltaTime)
        {
            bool found = false;
            for (int i = 0; i < TrianglesLocalPos.Length && !found; i++)
            {
                Vector4[] triangle = TrianglesLocalPos[i];
                Vector3 v1 = transform.localToWorldMatrix * triangle[0];
                Vector3 v2 = transform.localToWorldMatrix * triangle[1];
                Vector3 v3 = transform.localToWorldMatrix * triangle[2];

                Vector3 n = Vector3.Cross(v2 - v1, v3 - v1).normalized;
                float d = -Vector3.Dot(n, v1);

                if (Triangle.OnTriangleParticle(p, v1, v2, v3, n, d))
                {
                    Plane.CollisionPlaneParticle(p, n, d, Friction);
                    found = true;
                }
            }
        }
    }
}
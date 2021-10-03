using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace CustomParticleSystem
{
    public class Triangle : Obstacle
    {
        private Vector3 Normal
        {
            get { return transform.up; }
        }

        // ax + by + cz + D = 0
        private float D
        {
            get { return -Vector3.Dot(Normal, transform.position); }
        }

        private Vector4[] VerticesLocalPos;
        private Vector3 V1 { get { return transform.localToWorldMatrix * VerticesLocalPos[0]; } }
        private Vector3 V2 { get { return transform.localToWorldMatrix * VerticesLocalPos[1]; } }
        private Vector3 V3 { get { return transform.localToWorldMatrix * VerticesLocalPos[2]; } }

        private void Awake()
        {
            MeshFilter mf = GetComponent<MeshFilter>();
            if (mf != null)
            {
                Vector3[] vertices = mf.sharedMesh.vertices;
                if (vertices.Length != 3)
                {
                    Debug.LogWarning("Triangle mesh must have exactly 3 vertices.");
                    Destroy(this);
                    return;
                }
                VerticesLocalPos = new Vector4[vertices.Length];
                for (int i = 0; i < vertices.Length; i++) VerticesLocalPos[i] = new Vector4(vertices[i].x, vertices[i].y, vertices[i].z, 1);
            }
        }

        public override bool HasCollisionParticle(Particle p)
        {
            float sign = Vector3.Dot(p.Position, Normal) + D;
            sign *= Vector3.Dot(p.PreviousPosition, Normal) + D;
            if (sign <= 0)
            {
                Vector3 line = p.Position - p.PreviousPosition;
                float t = (-D - Vector3.Dot(Normal, p.PreviousPosition)) / Vector3.Dot(Normal, line);
                Vector3 intersectionPoint = p.PreviousPosition + line * t;
                const float epsilon = 0.0001f;
                return Mathf.Abs(Area(intersectionPoint, V2, V3) + Area(V1, intersectionPoint, V3) + Area(V1, V2, intersectionPoint) - Area(V1, V2, V3)) < epsilon;
            }
            return false;
        }

        public override void CorrectCollisionParticle(Particle p, float deltaTime)
        {
            Plane.CollisionPlaneParticle(p, Normal, D, Friction);
        }

        private float Area(Vector3 vi, Vector3 vj, Vector3 vk)
        {
            return 0.5f * Vector3.Cross(vj - vi, vk - vi).magnitude;
        }
    }
}
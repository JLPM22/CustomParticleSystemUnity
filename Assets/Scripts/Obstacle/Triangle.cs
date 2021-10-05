using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace CustomParticleSystem
{
    public class Triangle : Obstacle
    {
        private Vector3 Normal;
        private float D; // ax + by + cz + D = 0

        private Vector4[] VerticesLocalPos;
        private Vector3 V1, V2, V3;

        private void Update()
        {
            Normal = transform.up;
            D = -Vector3.Dot(Normal, transform.position);
            V1 = transform.localToWorldMatrix * VerticesLocalPos[0];
            V2 = transform.localToWorldMatrix * VerticesLocalPos[1];
            V3 = transform.localToWorldMatrix * VerticesLocalPos[2];
        }

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
            if (Plane.IsCrossingPlane(p, Normal, D))
            {
                return OnTriangleParticle(p, V1, V2, V3, Normal, D);
            }
            return false;
        }

        public override void CorrectCollisionParticle(Particle p, float deltaTime)
        {
            Plane.CollisionPlaneParticle(p, Normal, D, Friction, deltaTime);
        }

        private static float Area(Vector3 vi, Vector3 vj, Vector3 vk)
        {
            return 0.5f * Vector3.Cross(vj - vi, vk - vi).magnitude;
        }

        public static bool OnTriangleParticle(Particle p, Vector3 v1, Vector3 v2, Vector3 v3, Vector3 N, float d)
        {
            Vector3 previousPosition = p.GetBoundary(p.PreviousPosition, -N);
            Vector3 line = p.GetBoundary(p.Position, -N) - previousPosition;
            float t = (-d - Vector3.Dot(N, previousPosition)) / Vector3.Dot(N, line);
            if (t < 0 || t > 1) return false;
            Vector3 intersectionPoint = previousPosition + line * t;
            const float epsilon = 0.0001f;
            return Mathf.Abs(Area(intersectionPoint, v2, v3) + Area(v1, intersectionPoint, v3) + Area(v1, v2, intersectionPoint) - Area(v1, v2, v3)) < epsilon;
        }
    }
}
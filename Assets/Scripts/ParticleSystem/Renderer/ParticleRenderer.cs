using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace CustomParticleSystem
{
    public abstract class ParticleRenderer
    {
        protected Mesh Mesh;
        protected Material Material;

        public ParticleRenderer(Mesh mesh, Material material)
        {
            Mesh = mesh;
            Material = material;
        }

        public abstract void UpdateInstances();
        public abstract void Render(bool shadows);
        public abstract void SolveMovement(Solver solver, float deltaTime, float kVerlet);
        public abstract void SolveCollisions(Obstacle[] obstacles, float deltaTime);
        public abstract void SpawnParticle(ParticleSpawner spawner);
        public abstract void SetMaximumParticles(int maxParticles);
        public abstract void SetProperties(float mass, float radius, float bouncing);
        public abstract void Release();

        protected struct InstanceData
        {
            public Matrix4x4 TRSMatrix;
            public float Lifetime;

            public InstanceData(Matrix4x4 tRSMatrix, float lifetime)
            {
                TRSMatrix = tRSMatrix;
                Lifetime = lifetime;
            }

            public InstanceData(Vector3 position, Quaternion rotation, Vector3 scale, float lifetime)
            {
                TRSMatrix = Matrix4x4.TRS(position, rotation, scale);
                Lifetime = lifetime;
            }

            public static int Size()
            {
                return sizeof(float) * (4 * 4 + 1);
            }
        }
    }
}
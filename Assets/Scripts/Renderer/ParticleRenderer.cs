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
        public abstract void Render();
        public abstract void SolveMovement(Solver solver, float deltaTime, float kVerlet);
        public abstract void SolveCollisions(Obstacle[] obstacles, float deltaTime);
        public abstract void SpawnParticle(ParticleSpawner spawner);
        public abstract void SetMaximumParticles(int maxParticles);
        public abstract void SetRadius(float radius);
        public abstract void Release();

        protected struct InstanceData
        {
            public Matrix4x4 TRSMatrix;

            public InstanceData(Matrix4x4 tRSMatrix)
            {
                TRSMatrix = tRSMatrix;
            }

            public InstanceData(Vector3 position, Quaternion rotation, Vector3 scale)
            {
                TRSMatrix = Matrix4x4.TRS(position, rotation, scale);
            }

            public static int Size()
            {
                return sizeof(float) * 4 * 4;
            }
        }
    }
}
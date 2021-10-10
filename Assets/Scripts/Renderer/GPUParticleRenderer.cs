using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace CustomParticleSystem
{
    public class GPUParticleRenderer : ParticleRenderer
    {
        public GPUParticleRenderer(Mesh mesh, Material material) : base(mesh, material) { }

        public override void Render()
        {
            throw new System.NotImplementedException();
        }

        public override void SetMaximumParticles(int maxParticles)
        {
            throw new System.NotImplementedException();
        }

        public override void SetRadius(float radius)
        {
            throw new System.NotImplementedException();
        }

        public override void SolveCollisions(Obstacle[] obstacles, float deltaTime)
        {
            throw new System.NotImplementedException();
        }

        public override void SolveMovement(Solver solver, float deltaTime, float kVerlet)
        {
            throw new System.NotImplementedException();
        }

        public override void SpawnParticle(ParticleSpawner spawner)
        {
            throw new System.NotImplementedException();
        }

        public override void UpdateInstances()
        {
            throw new System.NotImplementedException();
        }

        public override void Release()
        {
            throw new System.NotImplementedException();
        }
    }
}
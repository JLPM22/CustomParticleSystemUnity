using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using Shape = CustomParticleSystem.ParticleSpawner.Shape;

namespace CustomParticleSystem
{
    public class CPUParticleRenderer : ParticleRenderer
    {
        private struct InstanceData
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

        private List<Particle> Particles = new List<Particle>();
        private Bounds Bounds = new Bounds(Vector3.zero, 10000 * Vector3.one);
        private ComputeBuffer InstancesBuffer;
        private ComputeBuffer ArgsBuffer;
        private InstanceData[] InstancesData;
        private int MaximumParticles;
        private Vector3 Scale = Vector3.one;

        public CPUParticleRenderer(Mesh mesh, Material material) : base(mesh, material) { }

        public override void SetMaximumParticles(int maxParticles)
        {
            MaximumParticles = maxParticles;
            InstancesData = new InstanceData[MaximumParticles];
            InitBuffers();
            UpdateParticlesList();
        }

        public override void SetRadius(float radius)
        {
            Scale = new Vector3(radius * 2, radius * 2, radius * 2);
        }

        private void InitBuffers()
        {
            ReleaseBuffers();
            uint[] args = new uint[5] { 0, 0, 0, 0, 0 };
            args[0] = (uint)Mesh.GetIndexCount(0);
            args[1] = (uint)MaximumParticles;
            args[2] = (uint)Mesh.GetIndexStart(0);
            args[3] = (uint)Mesh.GetBaseVertex(0);
            ArgsBuffer = new ComputeBuffer(1, args.Length * sizeof(uint), ComputeBufferType.IndirectArguments);
            ArgsBuffer.SetData(args);

            InstancesBuffer = new ComputeBuffer(MaximumParticles, InstanceData.Size());
            InstancesBuffer.SetData(new InstanceData[MaximumParticles]);
            Material.SetBuffer("_PerInstanceData", InstancesBuffer);
        }

        private void UpdateParticlesList()
        {
            if (Particles.Count > MaximumParticles)
            {
                Particles.RemoveRange(MaximumParticles, Particles.Count - MaximumParticles);
            }
            else
            {
                while (Particles.Count < MaximumParticles)
                {
                    Particles.Add(new Particle());
                }
            }
        }

        public override void UpdateInstances()
        {
            for (int i = 0; i < Mathf.Min(Particles.Count, MaximumParticles); i++)
            {
                if (Particles[i].LifeTime > 0)
                {
                    InstancesData[i] = new InstanceData(Particles[i].Position, Quaternion.identity, Scale);
                }
                else
                {
                    InstancesData[i] = new InstanceData(Vector3.zero, Quaternion.identity, Vector3.zero);
                }
            }
            InstancesBuffer.SetData(InstancesData);
        }

        public override void Render()
        {
            Graphics.DrawMeshInstancedIndirect(Mesh, 0, Material, Bounds, ArgsBuffer);
        }

        public override void SolveMovement(Solver solver, float deltaTime, float kVerlet)
        {
            switch (solver)
            {
                case Solver.EulerOrig:
                    for (int i = 0; i < Particles.Count; i++)
                    {
                        Particles[i].UpdateEulerOrig(deltaTime);
                    }
                    break;
                case Solver.EulerSemi:
                    for (int i = 0; i < Particles.Count; i++)
                    {
                        Particles[i].UpdateEulerSemi(deltaTime);
                    }
                    break;
                case Solver.Verlet:
                    for (int i = 0; i < Particles.Count; i++)
                    {
                        Particles[i].UpdateVerlet(deltaTime, kVerlet);
                    }
                    break;
            }
            LastIndexSearchAvailable = 0;
        }

        public override void SolveCollisions(Obstacle[] obstacles, float deltaTime)
        {
            // Check Collisions
            for (int i = 0; i < Particles.Count; i++)
            {
                Particle p = Particles[i];
                if (p.LifeTime > 0.0f)
                {
                    bool found = false;
                    for (int j = 0; j < obstacles.Length && !found; j++)
                    {
                        Obstacle o = obstacles[j];
                        if (o.HasCollisionParticle(p))
                        {
                            o.CorrectCollisionParticle(p, deltaTime);
                            found = true;
                        }
                    }
                }
            }
        }

        private int LastIndexSearchAvailable = 0;
        private int FindFirstAvailableParticle()
        {
            for (int i = LastIndexSearchAvailable; i < Particles.Count; i++)
            {
                LastIndexSearchAvailable = i + 1;
                if (Particles[i].LifeTime <= 0.0f) return i;
            }
            return -1;
        }

        public override void SpawnParticle(ParticleSpawner spawner)
        {
            int index = FindFirstAvailableParticle();
            if (index != -1)
            {
                // Emission Shape
                Vector3 spawnPos = new Vector3();
                Vector3 velocitySpawn = new Vector3();
                switch (spawner.EmissionShape)
                {
                    case Shape.Point:
                        spawnPos = spawner.transform.position;
                        break;
                    case Shape.Explosion:
                        float alpha = (360.0f * (Random.value - 0.5f)) * Mathf.Deg2Rad;
                        float beta = (180.0f * (Random.value - 0.5f)) * Mathf.Deg2Rad;
                        spawnPos = new Vector3(Mathf.Cos(alpha) * Mathf.Cos(beta), Mathf.Sin(beta), Mathf.Sin(alpha) * Mathf.Cos(beta));
                        velocitySpawn = spawner.EmissionExplosionSpeed * spawnPos;
                        spawnPos = spawnPos * 0.01f + spawner.transform.position;
                        break;
                    case Shape.Fountain:
                        spawnPos = spawner.transform.position;
                        velocitySpawn = new Vector3(Random.value - 0.5f, spawner.EmissionFountainSpeed, Random.value - 0.5f);
                        break;
                    case Shape.Waterfall:
                        spawnPos = spawner.transform.position + Vector3.up * spawner.EmissionWaterfallHeight;
                        velocitySpawn = new Vector3(Random.value - 0.5f, -spawner.EmissionWaterfallSpeed, Random.value - 0.5f);
                        break;
                    case Shape.SemiSphere:
                        alpha = (360.0f * (Random.value - 0.5f)) * Mathf.Deg2Rad;
                        beta = (90.0f * Random.value) * Mathf.Deg2Rad;
                        spawnPos = new Vector3(Mathf.Cos(alpha) * Mathf.Cos(beta), Mathf.Sin(beta), Mathf.Sin(alpha) * Mathf.Cos(beta));
                        velocitySpawn = spawner.EmissionSemiSphereSpeed * spawnPos;
                        spawnPos = spawnPos * spawner.EmissionSemiSphereRadius + spawner.transform.position;
                        break;
                }
                // Init Particle
                Particles[index].Init(spawnPos, velocitySpawn, spawner.ParticleMass * new Vector3(0.0f, -spawner.Gravity, 0.0f), spawner.ParticleLifeTime, spawner.SimulationTimestep);
            }
        }

        private void ReleaseBuffers()
        {
            if (InstancesBuffer != null)
            {
                InstancesBuffer.Release();
                InstancesBuffer = null;
            }
            if (ArgsBuffer != null)
            {
                ArgsBuffer.Release();
                ArgsBuffer = null;
            }
        }

        public override void Release()
        {
            ReleaseBuffers();
        }
    }
}
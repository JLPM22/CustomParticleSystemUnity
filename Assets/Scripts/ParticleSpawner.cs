using System;
using System.Collections;
using System.Collections.Generic;
using System.Diagnostics;
using UnityEngine;

using Debug = UnityEngine.Debug;
using Random = UnityEngine.Random;

namespace CustomParticleSystem
{
    public class ParticleSpawner : MonoBehaviour
    {
        [Tooltip("Simulation timestep (independent from the rendering timestep)")]
        public float SimulationTimestep = 1.0f / 60.0f;
        public Solver ParticleSolver;
        [Range(0.95f, 1.0f)] public float KVerlet = 1.0f;
        public float Gravity = 9.81f;
        [Range(0.0f, 1.0f)] public float ParticleBouncing = 0.2f;
        public float ParticleMass = 1.0f;
        public float ParticleRadius = 0.05f;
        public float ParticleLifeTime = 5.0f;
        public Mesh ParticleMesh;
        public Material ParticleMaterial;
        public int MaximumNumberParticles;
        public float EmissionRate = 1.0f;
        public Method ExecutionMethod = Method.CPU_Single_Thread;
        public Shape EmissionShape = Shape.Point;
        public float EmissionExplosionSpeed = 10.0f;
        public float EmissionFountainSpeed = 10.0f;
        public float EmissionWaterfallHeight = 10.0f;
        public float EmissionWaterfallSpeed = 10.0f;
        public float EmissionSemiSphereRadius = 0.5f;
        public float EmissionSemiSphereSpeed = 10.0f;

        private List<Particle> Particles = new List<Particle>();
        private Obstacle[] Obstacles;

        private float AccumulatedDeltaTime = 0.0f;
        private int LastMaximumNumberParticles;
        private float AccumulatedTimeEmission = 0.0f;

        private CPUParticleRenderer CPURenderer;

        private IEnumerator Start()
        {
            // Init variables
            LastMaximumNumberParticles = MaximumNumberParticles;
            UpdateBuffers();
            // Wait one frame for the Destroy() calls to be done (objects bad inited)
            enabled = false;
            yield return null;
            enabled = true;
            // Find obstacles
            Obstacles = FindObjectsOfType<Obstacle>();
            Array.Sort(Obstacles, (a, b) => a.GetPriority() > b.GetPriority() ? -1 : 1);
            // Renderer
            CPURenderer = new CPUParticleRenderer(ParticleMesh, ParticleMaterial);
            CPURenderer.SetMaximumParticles(MaximumNumberParticles);
            CPURenderer.SetRadius(ParticleRadius);
            // Application framerate
            Application.targetFrameRate = -1;
        }

        private void Update()
        {
            // Check Maximum Number of Particles
            if (MaximumNumberParticles != LastMaximumNumberParticles)
            {
                CPURenderer.SetMaximumParticles(MaximumNumberParticles);
                UpdateBuffers();
                LastMaximumNumberParticles = MaximumNumberParticles;
            }

            // Emit particles
            LastIndexSearchAvailable = 0;
            AccumulatedTimeEmission += Time.deltaTime;
            float timeEmission = 1.0f / EmissionRate;
            while (AccumulatedTimeEmission >= timeEmission)
            {
                SpawnParticle(SimulationTimestep);
                AccumulatedTimeEmission -= timeEmission;
            }

            // Update particles
            float deltaTime = Time.deltaTime + AccumulatedDeltaTime;
            while (deltaTime >= SimulationTimestep)
            {
                float deltaTimeStep = SimulationTimestep;
                deltaTime -= deltaTimeStep;

                switch (ExecutionMethod)
                {
                    case Method.CPU_Single_Thread:
                        SolveCPUSingleThread(deltaTimeStep);
                        break;
                    case Method.CPU_Multi_Thread:
                        SolveCPUMultithread(deltaTimeStep);
                        break;
                    case Method.GPU:
                        SolveGPU(deltaTimeStep);
                        break;
                }
            }

            // Render
            CPURenderer.UpdateInstances(Particles);
            CPURenderer.Render();

            // Update Variables
            AccumulatedDeltaTime = deltaTime;
            Particle.Mass = ParticleMass;
            Particle.Radius = ParticleRadius;
            Particle.Bouncing = ParticleBouncing;
        }

        private void SolveCPUSingleThread(float deltaTime)
        {
            // Update particles
            switch (ParticleSolver)
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
                        Particles[i].UpdateVerlet(deltaTime, KVerlet);
                    }
                    break;
            }

            // Check Collisions
            for (int i = 0; i < Particles.Count; i++)
            {
                Particle p = Particles[i];
                if (p.LifeTime > 0.0f)
                {
                    bool found = false;
                    for (int j = 0; j < Obstacles.Length && !found; j++)
                    {
                        Obstacle o = Obstacles[j];
                        if (o.HasCollisionParticle(p))
                        {
                            o.CorrectCollisionParticle(p, deltaTime);
                            found = true;
                        }
                    }
                }
            }
        }

        private bool Test(Particle p)
        {
            float d = -Vector3.Dot(Vector3.up, Vector3.zero);
            float sign = Vector3.Dot(p.Position - Vector3.up * ParticleRadius, Vector3.up) + d;
            sign *= Vector3.Dot(p.PreviousPosition - Vector3.up * ParticleRadius, Vector3.up) + d;
            return sign <= 0;
        }

        private void SolveCPUMultithread(float deltaTime)
        {

        }

        private void SolveGPU(float deltaTime)
        {
            throw new NotImplementedException();
        }

        private void SpawnParticle(float deltaTime)
        {
            int index = FindFirstAvailableParticle();
            if (index != -1)
            {
                // Emission Shape
                Vector3 spawnPos = new Vector3();
                Vector3 velocitySpawn = new Vector3();
                switch (EmissionShape)
                {
                    case Shape.Point:
                        spawnPos = transform.position;
                        break;
                    case Shape.Explosion:
                        float alpha = (360.0f * (Random.value - 0.5f)) * Mathf.Deg2Rad;
                        float beta = (180.0f * (Random.value - 0.5f)) * Mathf.Deg2Rad;
                        spawnPos = new Vector3(Mathf.Cos(alpha) * Mathf.Cos(beta), Mathf.Sin(beta), Mathf.Sin(alpha) * Mathf.Cos(beta));
                        velocitySpawn = EmissionExplosionSpeed * spawnPos;
                        spawnPos = spawnPos * 0.01f + transform.position;
                        break;
                    case Shape.Fountain:
                        spawnPos = transform.position;
                        velocitySpawn = new Vector3(Random.value - 0.5f, EmissionFountainSpeed, Random.value - 0.5f);
                        break;
                    case Shape.Waterfall:
                        spawnPos = transform.position + Vector3.up * EmissionWaterfallHeight;
                        velocitySpawn = new Vector3(Random.value - 0.5f, -EmissionWaterfallSpeed, Random.value - 0.5f);
                        break;
                    case Shape.SemiSphere:
                        alpha = (360.0f * (Random.value - 0.5f)) * Mathf.Deg2Rad;
                        beta = (90.0f * Random.value) * Mathf.Deg2Rad;
                        spawnPos = new Vector3(Mathf.Cos(alpha) * Mathf.Cos(beta), Mathf.Sin(beta), Mathf.Sin(alpha) * Mathf.Cos(beta));
                        velocitySpawn = EmissionSemiSphereSpeed * spawnPos;
                        spawnPos = spawnPos * EmissionSemiSphereRadius + transform.position;
                        break;
                }
                // Init Particle
                Particles[index].Init(spawnPos, velocitySpawn, ParticleMass * new Vector3(0.0f, -Gravity, 0.0f), ParticleLifeTime, deltaTime);
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

        private void UpdateBuffers()
        {
            if (Particles.Count > MaximumNumberParticles)
            {
                Particles.RemoveRange(MaximumNumberParticles, Particles.Count - MaximumNumberParticles);
            }
            else
            {
                while (Particles.Count < MaximumNumberParticles)
                {
                    Particles.Add(new Particle());
                }
            }
        }

        private void OnDestroy()
        {
            if (CPURenderer != null) CPURenderer.Release();
        }

        public enum Method
        {
            CPU_Single_Thread,
            CPU_Multi_Thread,
            GPU
        }

        public enum Shape
        {
            Point,
            Fountain,
            Waterfall,
            SemiSphere,
            Explosion
        }
    }
}
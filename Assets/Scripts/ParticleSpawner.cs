using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

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

        private List<Particle> Particles = new List<Particle>();
        private Obstacle[] Obstacles;

        private float AccumulatedDeltaTime = 0.0f;
        private int LastMaximumNumberParticles;
        private float TimeToNextEmission = 0.0f;

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
            TimeToNextEmission -= Time.deltaTime;
            if (TimeToNextEmission <= 0.0f)
            {
                SpawnParticle(transform.position, Vector3.zero, SimulationTimestep);
                TimeToNextEmission = 1.0f / EmissionRate;
            }

            // Update particles
            float deltaTime = Time.deltaTime + AccumulatedDeltaTime;
            while (deltaTime >= SimulationTimestep)
            {
                float deltaTimeStep = SimulationTimestep;
                deltaTime -= deltaTimeStep;

                // Update particles
                switch (ParticleSolver)
                {
                    case Solver.EulerOrig:
                        for (int i = 0; i < Particles.Count; i++)
                        {
                            Particles[i].UpdateEulerOrig(deltaTimeStep);
                        }
                        break;
                    case Solver.EulerSemi:
                        for (int i = 0; i < Particles.Count; i++)
                        {
                            Particles[i].UpdateEulerSemi(deltaTimeStep);
                        }
                        break;
                    case Solver.Verlet:
                        for (int i = 0; i < Particles.Count; i++)
                        {
                            Particles[i].UpdateVerlet(deltaTimeStep, KVerlet);
                        }
                        break;
                }
                // Check Collisions
                for (int i = 0; i < Particles.Count; i++)
                {
                    bool found = false;
                    for (int j = 0; j < Obstacles.Length && !found; j++)
                    {
                        if (Obstacles[j].HasCollisionParticle(Particles[i]))
                        {
                            Obstacles[j].CorrectCollisionParticle(Particles[i], deltaTimeStep);
                            found = true;
                        }
                    }
                }
            }

            // Render
            CPURenderer.UpdateInstances(Particles);
            CPURenderer.Render();

            // Update Variables
            AccumulatedDeltaTime = deltaTime;
        }

        private void SpawnParticle(Vector3 pos, Vector3 velocity, float deltaTime)
        {
            int index = FindFirstAvailableParticle();
            if (index != -1)
            {
                Particles[index].Init(pos, velocity, ParticleMass * new Vector3(0.0f, -Gravity, 0.0f), ParticleMass, ParticleBouncing, ParticleRadius, ParticleLifeTime, deltaTime);
            }
        }

        private int FindFirstAvailableParticle()
        {
            for (int i = 0; i < Particles.Count; i++)
            {
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
    }
}
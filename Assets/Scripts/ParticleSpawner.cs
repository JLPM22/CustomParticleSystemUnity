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

        private List<Particle> Particles = new List<Particle>();
        private List<GameObject> ParticlesVisual = new List<GameObject>(); // TEMPORAL (waiting for GPU Instancing)
        private Obstacle[] Obstacles;

        private float AccumulatedDeltaTime = 0.0f;

        private IEnumerator Start()
        {
            yield return null; // Wait one frame for the Destroy() calls to be done (objects bad inited)
            // Find obstacles
            Obstacles = FindObjectsOfType<Obstacle>();
            Array.Sort(Obstacles, (a, b) => a.GetPriority() > b.GetPriority() ? -1 : 1);
            // Spawn first particle
            SpawnParticle(transform.position, Vector3.zero, SimulationTimestep);
            // Application framerate
            Application.targetFrameRate = -1;
        }

        private void Update()
        {
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
                            ParticlesVisual[i].transform.position = Particles[i].Position;
                        }
                        break;
                    case Solver.EulerSemi:
                        for (int i = 0; i < Particles.Count; i++)
                        {
                            Particles[i].UpdateEulerSemi(deltaTimeStep);
                            ParticlesVisual[i].transform.position = Particles[i].Position;
                        }
                        break;
                    case Solver.Verlet:
                        for (int i = 0; i < Particles.Count; i++)
                        {
                            Particles[i].UpdateVerlet(deltaTimeStep, KVerlet);
                            ParticlesVisual[i].transform.position = Particles[i].Position;
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

            AccumulatedDeltaTime = deltaTime;
        }

        private void SpawnParticle(Vector3 pos, Vector3 velocity, float deltaTime)
        {
            Particle p = new Particle();
            p.Init(pos, velocity, ParticleMass * new Vector3(0.0f, -Gravity, 0.0f), ParticleMass, ParticleBouncing, ParticleRadius, deltaTime);
            Particles.Add(p);

            GameObject go = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            go.transform.position = pos;
            go.transform.localScale = new Vector3(ParticleRadius * 2, ParticleRadius * 2, ParticleRadius * 2);
            ParticlesVisual.Add(go);
        }
    }
}
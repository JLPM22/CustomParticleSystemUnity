using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace CustomParticleSystem
{
    public class CustomParticleSystem : MonoBehaviour
    {
        [Tooltip("Simulation timestep (independent from the rendering timestep)")] public float SimulationTimestep = 1.0f / 60.0f;
        public Solver ParticleSolver;
        public float Gravity = 9.81f;
        [Range(0.0f, 1.0f)] public float ParticleBouncing = 0.2f;
        public float ParticleMass = 1.0f;

        private List<Particle> Particles = new List<Particle>();
        private List<GameObject> ParticlesVisual = new List<GameObject>(); // TEMPORAL (waiting for GPU Instancing)
        private Obstacle[] Obstacles;

        private void Awake()
        {
            Obstacles = FindObjectsOfType<Obstacle>();
        }

        private void Start()
        {
            SpawnParticle(transform.position, Vector3.zero);
        }

        private void Update()
        {
            float deltaTime = Time.deltaTime;

            while (deltaTime > 0)
            {
                float deltaTimeStep = Mathf.Min(deltaTime, SimulationTimestep);
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
                            Particles[i].UpdateVerlet(deltaTimeStep);
                            ParticlesVisual[i].transform.position = Particles[i].Position;
                        }
                        break;
                }
                // Check Collisions
                for (int i = 0; i < Particles.Count; i++)
                {
                    for (int j = 0; j < Obstacles.Length; j++)
                    {
                        if (Obstacles[j].HasCollisionParticle(Particles[i]))
                        {
                            Obstacles[j].CorrectCollisionParticle(Particles[i]);
                        }
                    }
                }
            }
        }

        private void SpawnParticle(Vector3 pos, Vector3 velocity)
        {
            Particle p = new Particle();
            p.Init(pos, velocity, new Vector3(0.0f, -Gravity, 0.0f), ParticleMass, ParticleBouncing);
            Particles.Add(p);

            GameObject go = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            go.transform.position = pos;
            go.transform.localScale = new Vector3(0.1f, 0.1f, 0.1f);
            ParticlesVisual.Add(go);
        }
    }
}
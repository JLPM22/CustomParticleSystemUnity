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

        private Obstacle[] Obstacles;

        private float AccumulatedDeltaTime = 0.0f;
        private int LastMaximumNumberParticles;
        private float AccumulatedTimeEmission = 0.0f;

        private ParticleRenderer ParticleRenderer;

        private IEnumerator Start()
        {
            // Init variables
            LastMaximumNumberParticles = MaximumNumberParticles;
            // Wait one frame for the Destroy() calls to be done (objects bad inited)
            enabled = false;
            yield return null;
            enabled = true;
            // Find obstacles
            Obstacles = FindObjectsOfType<Obstacle>();
            Array.Sort(Obstacles, (a, b) => a.GetPriority() > b.GetPriority() ? -1 : 1);
            // Renderer
            switch (ExecutionMethod)
            {
                case Method.CPU_Single_Thread:
                    ParticleRenderer = new CPUParticleRenderer(ParticleMesh, ParticleMaterial);
                    break;
                case Method.CPU_Multi_Thread:
                    throw new NotImplementedException();
                    break;
                case Method.GPU:
                    ParticleRenderer = new GPUParticleRenderer(ParticleMesh, ParticleMaterial);
                    break;
            }
            ParticleRenderer.SetMaximumParticles(MaximumNumberParticles);
            ParticleRenderer.SetRadius(ParticleRadius);
            // Application framerate
            Application.targetFrameRate = -1;
        }

        private void Update()
        {
            // Check Maximum Number of Particles
            if (MaximumNumberParticles != LastMaximumNumberParticles)
            {
                ParticleRenderer.SetMaximumParticles(MaximumNumberParticles);
                LastMaximumNumberParticles = MaximumNumberParticles;
            }

            // Emit particles
            AccumulatedTimeEmission += Time.deltaTime;
            float timeEmission = 1.0f / EmissionRate;
            while (AccumulatedTimeEmission >= timeEmission)
            {
                ParticleRenderer.SpawnParticle(this);
                AccumulatedTimeEmission -= timeEmission;
            }

            // Update particles
            float deltaTime = Time.deltaTime + AccumulatedDeltaTime;
            while (deltaTime >= SimulationTimestep)
            {
                float deltaTimeStep = SimulationTimestep;
                deltaTime -= deltaTimeStep;

                ParticleRenderer.SolveMovement(ParticleSolver, deltaTimeStep, KVerlet);
                ParticleRenderer.SolveCollisions(Obstacles, deltaTimeStep);
            }

            // Render
            ParticleRenderer.UpdateInstances();
            ParticleRenderer.Render();

            // Update Variables
            AccumulatedDeltaTime = deltaTime;
            Particle.Mass = ParticleMass;
            Particle.Radius = ParticleRadius;
            Particle.Bouncing = ParticleBouncing;
        }

        private void OnDestroy()
        {
            if (ParticleRenderer != null) ParticleRenderer.Release();
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
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using CustomParticleSystem;
using System;

namespace CustomSpringSystem
{
    public class SpringSpawner : MonoBehaviour
    {
        [Tooltip("Simulation timestep (independent from the rendering timestep)")]
        public float SimulationTimestep = 1.0f / 60.0f;
        public bool EnableInput;
        public Solver SpringSolver = Solver.Verlet;
        [Range(0.95f, 1.0f)] public float KVerlet = 0.99f;
        public RenderType SpringRenderType = RenderType.Particles;
        public int NumberParticles;
        public float DistanceBetweenParticles;
        public Vector3 InitParticleSpawnDir;
        public bool[] FixedParticles;
        public float Gravity = 9.81f;
        public float Damping = 1.00f;
        public float Elasticity = 1.0f;
        [Range(0.0f, 1.0f)] public float ParticleBouncing = 0.2f;
        public float ParticleMass = 1.0f;
        public float ParticleRadius = 0.05f;
        public float HairRadius = 0.05f;
        public Mesh ParticleMesh;
        public Mesh HairMesh;
        public Material ParticleMaterial;
        public Material HairMaterial;
        public bool Shadows;

        private Obstacle[] Obstacles;
        private Wind Wind;
        private float AccumulatedDeltaTime = 0.0f;
        private int LastNumberParticles;
        private RenderType LastSpringRenderType;
        private BurstSpringRenderer SpringRenderer;
        private bool IsMovingParticle;
        private int MovingParticleIndex;
        private UnityEngine.Plane MovingPlane;
        private Camera MainCamera;

        private void Awake()
        {
            MainCamera = Camera.main;
        }

        private IEnumerator Start()
        {
            enabled = false;
            yield return null;
            // Init variables
            LastNumberParticles = NumberParticles;
            LastSpringRenderType = SpringRenderType;
            // Wait one frame for the Destroy() calls to be done (objects bad inited)
            yield return null;
            enabled = true;
            // Find obstacles
            Obstacles = FindObjectsOfType<Obstacle>();
            Array.Sort(Obstacles, (a, b) => a.GetPriority() > b.GetPriority() ? -1 : 1);
            // Find wind
            Wind = FindObjectOfType<Wind>();
            // Renderer
            SpringRenderer = new BurstSpringRenderer(this);
            UpdateMaximumParticles();
            // Application framerate
            Application.targetFrameRate = -1;
            QualitySettings.vSyncCount = 0;
        }

        private void Update()
        {
            // Check Maximum Number of Particles
            if (NumberParticles != LastNumberParticles)
            {
                UpdateMaximumParticles();
                LastNumberParticles = NumberParticles;
            }

            // Update particles
            float deltaTime = Time.deltaTime + AccumulatedDeltaTime;
            while (deltaTime >= SimulationTimestep)
            {
                float deltaTimeStep = SimulationTimestep;
                deltaTime -= deltaTimeStep;

                SpringRenderer.SolveForces(deltaTimeStep, Wind == null ? Vector3.zero : Wind.WindForce);
                SpringRenderer.SolveMovement(SpringSolver, deltaTimeStep, KVerlet);
                SpringRenderer.SolveCollisions(Obstacles, deltaTimeStep);
            }

            // Input
            if (EnableInput)
            {
                if (Input.GetKeyDown(KeyCode.F))
                {
                    FixParticle();
                }
                if (Input.GetMouseButtonDown(0))
                {
                    StartMovingParticle();
                }
                else if (Input.GetMouseButtonUp(0))
                {
                    IsMovingParticle = false;
                }
                if (IsMovingParticle)
                {
                    MoveParticle();
                }
            }

            // Render
            SpringRenderer.UpdateInstances(SpringRenderType);
            SpringRenderer.Render(Shadows, SpringRenderType);

            // Update Variables
            AccumulatedDeltaTime = deltaTime;
        }

        private void StartMovingParticle()
        {
            Ray line = MainCamera.ScreenPointToRay(Input.mousePosition);
            for (int i = 0; i < NumberParticles; i++)
            {
                if (TestSphereLineIntersection(i, line.origin, line.direction))
                {
                    MovingParticleIndex = i;
                    IsMovingParticle = true;
                    MovingPlane = new UnityEngine.Plane(-line.direction, SpringRenderer.GetParticlePosition(MovingParticleIndex));
                    break;
                }
            }
        }

        private void MoveParticle()
        {
            Ray line = MainCamera.ScreenPointToRay(Input.mousePosition);
            MovingPlane.Raycast(line, out float distance);
            SpringRenderer.SetParticlePosition(MovingParticleIndex, line.origin + line.direction * distance);
        }

        private void FixParticle()
        {
            Ray line = MainCamera.ScreenPointToRay(Input.mousePosition);
            for (int i = 0; i < NumberParticles; i++)
            {
                if (TestSphereLineIntersection(i, line.origin, line.direction))
                {
                    FixedParticles[i] = !FixedParticles[i];
                }
            }
        }

        private bool TestSphereLineIntersection(int indexParticle, Vector3 lineStart, Vector3 lineDir)
        {
            float radius = DistanceBetweenParticles * 0.5f;
            Vector3 particlePos = SpringRenderer.GetParticlePosition(indexParticle);
            float a = Vector3.Dot(lineDir, (lineStart - particlePos));
            float l = Vector3.Magnitude(lineStart - particlePos);
            float d = a * a - (l * l - radius * radius);
            return d >= 0.0f;
        }

        public Vector3 GetParticlePosition(int index)
        {
            if (SpringRenderer != null)
                return SpringRenderer.GetParticlePosition(index);
            return new Vector3(100000.0f, 100000.0f, 100000.0f);
        }

        public Vector3 GetParticlePreviousPosition(int index)
        {
            if (SpringRenderer != null)
                return SpringRenderer.GetParticlePreviousPostion(index);
            return new Vector3(100000.0f, 100000.0f, 100000.0f);
        }

        public void SetParticlePosition(int index, Vector3 position)
        {
            if (SpringRenderer != null)
                SpringRenderer.SetParticlePosition(index, position);
        }

        public void SetParticlePreviousPosition(int index, Vector3 position)
        {
            if (SpringRenderer != null)
                SpringRenderer.SetParticlePreviousPosition(index, position);
        }

        public void RecomputeElasticityAndDamping()
        {
            Elasticity = (ParticleMass / (SimulationTimestep * SimulationTimestep)) * (1.0f / (1.0f + 2.0f));
            Damping = (ParticleMass / SimulationTimestep) * (1.0f / (1.0f + 2.0f));
        }

        private void UpdateMaximumParticles()
        {
            SpringRenderer.SetMaximumParticles(NumberParticles, InitParticleSpawnDir);
        }

        private void OnDestroy()
        {
            if (SpringRenderer != null) SpringRenderer.Release();
        }

#if UNITY_EDITOR
        private void OnDrawGizmosSelected()
        {
            if (!Application.isPlaying)
            {
                Vector3 dirNormalized = InitParticleSpawnDir.normalized;
                for (int i = 0; i < NumberParticles; i++)
                {
                    Gizmos.color = FixedParticles[i] ? Color.red : Color.blue;
                    Vector3 pos = transform.position + dirNormalized * DistanceBetweenParticles * i;
                    Gizmos.DrawWireSphere(pos, ParticleRadius);
                }
            }
        }
#endif

        public enum Solver
        {
            EulerOrig,
            EulerSemi,
            Verlet
        }

        [System.Flags]
        public enum RenderType
        {
            Particles = 1 << 0,
            Hair = 1 << 1
        }
    }
}
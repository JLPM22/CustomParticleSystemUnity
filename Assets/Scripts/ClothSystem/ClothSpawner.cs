using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using CustomParticleSystem;
using System;

namespace CustomClothSystem
{
    public class ClothSpawner : MonoBehaviour
    {
        [Tooltip("Simulation timestep (independent from the rendering timestep)")]
        public float SimulationTimestep = 1.0f / 60.0f;
        public bool EnableInput;
        public Solver ClothSolver = Solver.Verlet;
        [Range(0.95f, 1.0f)] public float KVerlet = 0.99f;
        public RenderType ClothRenderType = RenderType.Particles;
        public Vector2Int NumberParticles;
        public Vector2 DistanceBetweenParticles;
        public Vector3 InitParticleSpawnDirX;
        public Vector3 InitParticleSpawnDirY;
        public bool[] FixedParticles;
        public float Gravity = 9.81f;
        public float Damping = 1.00f;
        public float Elasticity = 1.0f;
        [Range(0.0f, 1.0f)] public float ParticleBouncing = 0.2f;
        public float ParticleMass = 1.0f;
        public float ParticleRadius = 0.05f;
        public Mesh ParticleMesh;
        public Material ParticleMaterial;
        public Material ClothMaterial;
        public bool Shadows;
        public bool Stretch = true, Shear = true, Bend = true;

        private Obstacle[] Obstacles;
        private Wind Wind;
        private float AccumulatedDeltaTime = 0.0f;
        private Vector2Int LastNumberParticles;
        private RenderType LastClothRenderType;
        private BurstClothRenderer ClothRenderer;
        private bool IsMovingParticle;
        private bool IsMovingRow;
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
            LastClothRenderType = ClothRenderType;
            // Wait one frame for the Destroy() calls to be done (objects bad inited)
            yield return null;
            enabled = true;
            // Find obstacles
            Obstacles = FindObjectsOfType<Obstacle>();
            Array.Sort(Obstacles, (a, b) => a.GetPriority() > b.GetPriority() ? -1 : 1);
            // Find wind
            Wind = FindObjectOfType<Wind>();
            // Renderer
            ClothRenderer = new BurstClothRenderer(this);
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

                ClothRenderer.SolveForces(deltaTimeStep, Wind == null ? Vector3.zero : Wind.WindForce);
                ClothRenderer.SolveMovement(ClothSolver, deltaTimeStep, KVerlet);
                ClothRenderer.SolveCollisions(Obstacles, deltaTimeStep);
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
            ClothRenderer.UpdateInstances(ClothRenderType);
            ClothRenderer.Render(Shadows, ClothRenderType);

            // Update Variables
            AccumulatedDeltaTime = deltaTime;
        }

        private void StartMovingParticle()
        {
            Ray line = MainCamera.ScreenPointToRay(Input.mousePosition);
            for (int i = 0; i < (NumberParticles.x * NumberParticles.y); i++)
            {
                if (TestSphereLineIntersection(i, line.origin, line.direction))
                {
                    MovingParticleIndex = i;
                    IsMovingParticle = true;
                    IsMovingRow = Input.GetKey(KeyCode.R);
                    MovingPlane = new UnityEngine.Plane(-line.direction, ClothRenderer.GetParticlePosition(MovingParticleIndex));
                    break;
                }
            }
        }

        private void MoveParticle()
        {
            Ray line = MainCamera.ScreenPointToRay(Input.mousePosition);
            MovingPlane.Raycast(line, out float distance);
            Vector3 position = line.origin + line.direction * distance;
            Vector3 offset = position - ClothRenderer.GetParticlePosition(MovingParticleIndex);
            ClothRenderer.SetParticlePosition(MovingParticleIndex, position);
            if (IsMovingRow)
            {
                int row = MovingParticleIndex / NumberParticles.x;
                int column = MovingParticleIndex % NumberParticles.x;
                for (int i = 0; i < NumberParticles.x; i++)
                {
                    if (i == column) continue;
                    int index = row * NumberParticles.x + i;
                    ClothRenderer.SetParticlePosition(index, ClothRenderer.GetParticlePosition(index) + offset);
                }
            }
        }

        private void FixParticle()
        {
            Ray line = MainCamera.ScreenPointToRay(Input.mousePosition);
            for (int i = 0; i < (NumberParticles.x * NumberParticles.y); i++)
            {
                if (TestSphereLineIntersection(i, line.origin, line.direction))
                {
                    FixedParticles[i] = !FixedParticles[i];
                }
            }
        }

        private bool TestSphereLineIntersection(int indexParticle, Vector3 lineStart, Vector3 lineDir)
        {
            float radius = Mathf.Min(DistanceBetweenParticles.x * 0.5f, DistanceBetweenParticles.y * 0.5f);
            Vector3 particlePos = ClothRenderer.GetParticlePosition(indexParticle);
            float a = Vector3.Dot(lineDir, (lineStart - particlePos));
            float l = Vector3.Magnitude(lineStart - particlePos);
            float d = a * a - (l * l - radius * radius);
            return d >= 0.0f;
        }

        public Vector3 GetParticlePosition(int index)
        {
            if (ClothRenderer != null)
                return ClothRenderer.GetParticlePosition(index);
            return new Vector3(100000.0f, 100000.0f, 100000.0f);
        }

        public Vector3 GetParticlePreviousPosition(int index)
        {
            if (ClothRenderer != null)
                return ClothRenderer.GetParticlePreviousPostion(index);
            return new Vector3(100000.0f, 100000.0f, 100000.0f);
        }

        public void SetParticlePosition(int index, Vector3 position)
        {
            if (ClothRenderer != null)
                ClothRenderer.SetParticlePosition(index, position);
        }

        public void SetParticlePreviousPosition(int index, Vector3 position)
        {
            if (ClothRenderer != null)
                ClothRenderer.SetParticlePreviousPosition(index, position);
        }

        public void RecomputeElasticityAndDamping()
        {
            float nSpringsConnected = (Stretch ? 4.0f : 0.0f) + (Shear ? 4.0f : 0.0f) + (Bend ? 4.0f : 0.0f);
            Elasticity = (ParticleMass / (SimulationTimestep * SimulationTimestep)) * (1.0f / (1.0f + nSpringsConnected));
            Damping = (ParticleMass / SimulationTimestep) * (1.0f / (1.0f + nSpringsConnected));
        }

        private void UpdateMaximumParticles()
        {
            ClothRenderer.SetMaximumParticles(NumberParticles.x * NumberParticles.y, InitParticleSpawnDirX, InitParticleSpawnDirY);
        }

        private void OnDestroy()
        {
            if (ClothRenderer != null) ClothRenderer.Release();
        }

#if UNITY_EDITOR
        private void OnDrawGizmosSelected()
        {
            if (!Application.isPlaying)
            {
                Vector3 dirXNormalized = InitParticleSpawnDirX.normalized;
                Vector3 dirYNormalized = InitParticleSpawnDirY.normalized;
                for (int y = 0; y < NumberParticles.y; y++)
                {
                    for (int x = 0; x < NumberParticles.x; x++)
                    {
                        Vector3 pos = transform.position + (dirXNormalized * x * DistanceBetweenParticles.x) + (dirYNormalized * y * DistanceBetweenParticles.y);
                        Gizmos.color = FixedParticles[y * NumberParticles.x + x] ? Color.red : Color.blue;
                        Gizmos.DrawWireSphere(pos, ParticleRadius);
                    }
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
            Cloth = 1 << 1
        }
    }
}
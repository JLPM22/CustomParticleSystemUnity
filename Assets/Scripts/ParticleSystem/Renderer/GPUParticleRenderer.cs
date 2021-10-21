using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using Shape = CustomParticleSystem.ParticleSpawner.Shape;

namespace CustomParticleSystem
{
    public class GPUParticleRenderer : ParticleRenderer
    {
        private ComputeShader ComputeShader;
        private int SolveMovementKernel, SolveCollisionsPlanesKernel, SolveCollisionsSpheresKernel, SolveCollisionsTrianglesKernel, UpdateInstancesKernel;
        private int ParticlesBufferKey, InstancesBufferKey, CollisionsBufferKey;
        private int PlanesBufferKey, SpheresBufferKey, TrianglesBufferKey;
        private int MassKey, RadiusKey, BouncingKey, KVerletKey, DeltaTimeKey;
        private int NumberPlanesKey, NumberSpheresKey, NumberTrianglesKey;
        private Bounds Bounds = new Bounds(Vector3.zero, 10000 * Vector3.one);
        private ComputeBuffer ParticlesBuffer;
        private ComputeBuffer InstancesBuffer;
        private ComputeBuffer CollisionsBuffer;
        private ComputeBuffer ArgsBuffer;
        private ComputeBuffer PlanesBuffer;
        private ComputeBuffer SpheresBuffer;
        private ComputeBuffer TrianglesBuffer;

        private float[] Lifetimes;
        private int MaximumParticles;

        public GPUParticleRenderer(Mesh mesh, Material material) : base(mesh, material)
        {
            ComputeShader = Resources.Load<ComputeShader>("GPUParticleRendererCS");
            SolveMovementKernel = ComputeShader.FindKernel("SolveMovement");
            SolveCollisionsPlanesKernel = ComputeShader.FindKernel("SolveCollisionsPlanes");
            SolveCollisionsSpheresKernel = ComputeShader.FindKernel("SolveCollisionsSpheres");
            SolveCollisionsTrianglesKernel = ComputeShader.FindKernel("SolveCollisionsTriangles");
            UpdateInstancesKernel = ComputeShader.FindKernel("UpdateInstances");
            ParticlesBufferKey = Shader.PropertyToID("Particles");
            InstancesBufferKey = Shader.PropertyToID("Instances");
            CollisionsBufferKey = Shader.PropertyToID("Collisions");
            PlanesBufferKey = Shader.PropertyToID("Planes");
            SpheresBufferKey = Shader.PropertyToID("Spheres");
            TrianglesBufferKey = Shader.PropertyToID("Triangles");
            MassKey = Shader.PropertyToID("Mass");
            RadiusKey = Shader.PropertyToID("Radius");
            BouncingKey = Shader.PropertyToID("Bouncing");
            KVerletKey = Shader.PropertyToID("KVerlet");
            DeltaTimeKey = Shader.PropertyToID("DeltaTime");
            NumberPlanesKey = Shader.PropertyToID("NumberPlanes");
            NumberSpheresKey = Shader.PropertyToID("NumberSpheres");
            NumberTrianglesKey = Shader.PropertyToID("NumberTriangles");
        }

        public override void Render(bool shadows)
        {
            Graphics.DrawMeshInstancedIndirect(mesh: Mesh,
                                               submeshIndex: 0,
                                               material: Material,
                                               bounds: Bounds,
                                               bufferWithArgs: ArgsBuffer,
                                               castShadows: shadows ? UnityEngine.Rendering.ShadowCastingMode.On : UnityEngine.Rendering.ShadowCastingMode.Off,
                                               receiveShadows: shadows);
        }

        public override void SetMaximumParticles(int maxParticles)
        {
            maxParticles = ((maxParticles + 63) / 64) * 64; // make the number multiple of 64
            MaximumParticles = maxParticles;
            Lifetimes = new float[MaximumParticles];
            InitBuffers();
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

            ParticlesBuffer = new ComputeBuffer(MaximumParticles, ParticleGPU.Size());
            ParticlesBuffer.SetData(new ParticleGPU[MaximumParticles]);

            InstancesBuffer = new ComputeBuffer(MaximumParticles, InstanceData.Size());
            InstancesBuffer.SetData(new InstanceData[MaximumParticles]);
            Material.SetBuffer("_PerInstanceData", InstancesBuffer);

            CollisionsBuffer = new ComputeBuffer(MaximumParticles, sizeof(int));
        }

        public override void SetProperties(float mass, float radius, float bouncing)
        {
            ComputeShader.SetFloat(MassKey, mass);
            ComputeShader.SetFloat(RadiusKey, radius);
            ComputeShader.SetFloat(BouncingKey, bouncing);
        }

        public override void SolveMovement(Solver solver, float deltaTime, float kVerlet)
        {
            ComputeShader.SetFloat(KVerletKey, kVerlet);
            ComputeShader.SetFloat(DeltaTimeKey, deltaTime);

            ComputeShader.SetBuffer(SolveMovementKernel, ParticlesBufferKey, ParticlesBuffer);
            ComputeShader.Dispatch(SolveMovementKernel, MaximumParticles / 64, 1, 1);
            // Update CPU-Side Lifetime
            for (int i = 0; i < MaximumParticles; i++) Lifetimes[i] -= deltaTime;

            LastIndexSearchAvailable = 0;
        }

        public override void SolveCollisions(Obstacle[] obstacles, float deltaTime)
        {
            UpdateObstaclesBuffer(obstacles);
            // Plane
            if (PlanesGPU.Length > 0)
            {
                ComputeShader.SetBuffer(SolveCollisionsPlanesKernel, ParticlesBufferKey, ParticlesBuffer);
                ComputeShader.SetBuffer(SolveCollisionsPlanesKernel, CollisionsBufferKey, CollisionsBuffer);
                ComputeShader.SetBuffer(SolveCollisionsPlanesKernel, PlanesBufferKey, PlanesBuffer);
                ComputeShader.Dispatch(SolveCollisionsPlanesKernel, MaximumParticles / 64, 1, 1);
            }
            // Sphere
            if (SpheresGPU.Length > 0)
            {
                ComputeShader.SetBuffer(SolveCollisionsSpheresKernel, ParticlesBufferKey, ParticlesBuffer);
                ComputeShader.SetBuffer(SolveCollisionsSpheresKernel, CollisionsBufferKey, CollisionsBuffer);
                ComputeShader.SetBuffer(SolveCollisionsSpheresKernel, SpheresBufferKey, SpheresBuffer);
                ComputeShader.Dispatch(SolveCollisionsSpheresKernel, MaximumParticles / 64, 1, 1);
            }
            // Triangle
            if (TrianglesGPU.Length > 0)
            {
                ComputeShader.SetBuffer(SolveCollisionsTrianglesKernel, ParticlesBufferKey, ParticlesBuffer);
                ComputeShader.SetBuffer(SolveCollisionsTrianglesKernel, CollisionsBufferKey, CollisionsBuffer);
                ComputeShader.SetBuffer(SolveCollisionsTrianglesKernel, TrianglesBufferKey, TrianglesBuffer);
                ComputeShader.Dispatch(SolveCollisionsTrianglesKernel, MaximumParticles / 64, 1, 1);
            }
        }

        private ParticleGPU[] ParticleToGPU = new ParticleGPU[1];
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
                Lifetimes[index] = spawner.ParticleLifeTime;
                ParticleGPU particle = new ParticleGPU(spawnPos, velocitySpawn, spawner.ParticleMass * new Vector3(0.0f, -spawner.Gravity, 0.0f), spawner.ParticleLifeTime, spawner.SimulationTimestep);
                ParticleToGPU[0] = particle;
                ParticlesBuffer.SetData(ParticleToGPU, 0, index, 1);
            }
        }

        private int LastIndexSearchAvailable = 0;
        private int FindFirstAvailableParticle()
        {
            for (int i = LastIndexSearchAvailable; i < Lifetimes.Length; i++)
            {
                LastIndexSearchAvailable = i + 1;
                if (Lifetimes[i] <= 0.0f) return i;
            }
            return -1;
        }

        public override void UpdateInstances()
        {
            ComputeShader.SetBuffer(UpdateInstancesKernel, ParticlesBufferKey, ParticlesBuffer);
            ComputeShader.SetBuffer(UpdateInstancesKernel, InstancesBufferKey, InstancesBuffer);
            ComputeShader.SetBuffer(UpdateInstancesKernel, CollisionsBufferKey, CollisionsBuffer);
            ComputeShader.Dispatch(UpdateInstancesKernel, MaximumParticles / 64, 1, 1);
        }

        private PlaneGPU[] PlanesGPU;
        private SphereGPU[] SpheresGPU;
        private TriangleGPU[] TrianglesGPU;
        private void UpdateObstaclesBuffer(Obstacle[] obstacles)
        {
            if (PlanesBuffer == null)
            {
                int planeCount = 0;
                int sphereCount = 0;
                int triangleCount = 0;
                foreach (Obstacle o in obstacles)
                {
                    if (o is Plane) planeCount += 1;
                    else if (o is Sphere) sphereCount += 1;
                    else if (o is Triangle) triangleCount += 1;
                    else if (o is Cube) o.GetComponent<MeshRenderer>().enabled = false;
                }
                // Planes
                if (planeCount > 0)
                {
                    PlanesBuffer = new ComputeBuffer(planeCount, PlaneGPU.Size());
                    PlanesGPU = new PlaneGPU[planeCount];
                    ComputeShader.SetInt(NumberPlanesKey, planeCount);
                }
                else PlanesGPU = new PlaneGPU[0];
                // Spheres
                if (sphereCount > 0)
                {
                    SpheresBuffer = new ComputeBuffer(sphereCount, SphereGPU.Size());
                    SpheresGPU = new SphereGPU[sphereCount];
                    ComputeShader.SetInt(NumberSpheresKey, sphereCount);
                }
                else SpheresGPU = new SphereGPU[0];
                // Triangles
                if (triangleCount > 0)
                {
                    TrianglesBuffer = new ComputeBuffer(triangleCount, TriangleGPU.Size());
                    TrianglesGPU = new TriangleGPU[triangleCount];
                    ComputeShader.SetInt(NumberTrianglesKey, triangleCount);
                }
                else TrianglesGPU = new TriangleGPU[0];
            }

            int planeIndex = 0;
            int sphereIndex = 0;
            int triangleIndex = 0;
            foreach (Obstacle o in obstacles)
            {
                if (o is Plane)
                {
                    Plane p = o as Plane;
                    PlanesGPU[planeIndex] = new PlaneGPU(p.Normal, p.D, p.Friction);
                    planeIndex += 1;
                }
                else if (o is Sphere)
                {
                    Sphere s = o as Sphere;
                    SpheresGPU[sphereIndex] = new SphereGPU(s.Center, s.Radius, s.Friction);
                    sphereIndex += 1;
                }
                else if (o is Triangle)
                {
                    Triangle t = o as Triangle;
                    TrianglesGPU[triangleIndex] = new TriangleGPU(t.Normal, t.D, t.Friction, t.V1, t.V2, t.V3);
                    triangleIndex += 1;
                }
            }
            if (PlanesGPU.Length > 0) PlanesBuffer.SetData(PlanesGPU);
            if (SpheresGPU.Length > 0) SpheresBuffer.SetData(SpheresGPU);
            if (TrianglesGPU.Length > 0) TrianglesBuffer.SetData(TrianglesGPU);
        }

        public override void Release()
        {
            ReleaseBuffers();
        }

        private void ReleaseBuffers()
        {
            if (ParticlesBuffer != null)
            {
                ParticlesBuffer.Release();
                ParticlesBuffer = null;
            }
            if (InstancesBuffer != null)
            {
                InstancesBuffer.Release();
                InstancesBuffer = null;
            }
            if (CollisionsBuffer != null)
            {
                CollisionsBuffer.Release();
                CollisionsBuffer = null;
            }
            if (ArgsBuffer != null)
            {
                ArgsBuffer.Release();
                ArgsBuffer = null;
            }
            if (PlanesBuffer != null)
            {
                PlanesBuffer.Release();
                PlanesBuffer = null;
            }
            if (SpheresBuffer != null)
            {
                SpheresBuffer.Release();
                SpheresBuffer = null;
            }
            if (TrianglesBuffer != null)
            {
                TrianglesBuffer.Release();
                TrianglesBuffer = null;
            }
        }

        private struct ParticleGPU
        {
            public Vector3 Position;
            public Vector3 Velocity;
            public Vector3 PreviousPosition;
            public Vector3 PreviousVelocity;
            public Vector3 Force;
            public float Lifetime;

            public ParticleGPU(Vector3 position, Vector3 velocity, Vector3 force, float lifeTime, float deltaTime)
            {
                Position = position;
                PreviousPosition = Position - velocity * deltaTime;
                Velocity = velocity;
                PreviousVelocity = velocity;
                Force = force;
                Lifetime = lifeTime;
            }

            public static int Size()
            {
                return sizeof(float) * (3 * 5 + 1);
            }
        }

        private struct PlaneGPU
        {
            public Vector3 Normal;
            public float D;
            public float Friction;

            public PlaneGPU(Vector3 normal, float d, float friction)
            {
                Normal = normal;
                D = d;
                Friction = friction;
            }

            public static int Size()
            {
                return sizeof(float) * (3 + 1 + 1);
            }
        }

        private struct SphereGPU
        {
            public Vector3 Center;
            public float Radius;
            public float Friction;

            public SphereGPU(Vector3 center, float radius, float friction)
            {
                Center = center;
                Radius = radius;
                Friction = friction;
            }

            public static int Size()
            {
                return sizeof(float) * (3 + 1 + 1);
            }
        }

        private struct TriangleGPU
        {
            public Vector3 Normal;
            public float D;
            public float Friction;
            public Vector3 V1, V2, V3;

            public TriangleGPU(Vector3 normal, float d, float friction, Vector3 v1, Vector3 v2, Vector3 v3)
            {
                Normal = normal;
                D = d;
                Friction = friction;
                V1 = v1;
                V2 = v2;
                V3 = v3;
            }

            public static int Size()
            {
                return sizeof(float) * (3 + 1 + 1 + 3 * 3);
            }
        }
    }
}
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using CustomParticleSystem;

using static Unity.Mathematics.math;
using Plane = CustomParticleSystem.Plane;
using Solver = CustomClothSystem.ClothSpawner.Solver;
using RenderType = CustomClothSystem.ClothSpawner.RenderType;

namespace CustomClothSystem
{
    using System.Runtime.CompilerServices;
    using Unity.Mathematics;

    public class BurstClothRenderer
    {
        private ClothSpawner Spawner;

        public BurstClothRenderer(ClothSpawner spawner)
        {
            Spawner = spawner;
        }

        private NativeArray<BurstParticle> Particles;
        private NativeArray<BurstParticle> ParticlesRead;
        private NativeArray<BurstPlane> Planes;
        private NativeArray<BurstSphere> Spheres;
        private NativeArray<BurstTriangle> Triangles;

        private Bounds Bounds = new Bounds(Vector3.zero, 10000 * Vector3.one);
        private ComputeBuffer InstancesParticlesBuffer;
        private ComputeBuffer ArgsParticlesBuffer;
        private InstanceData[] InstancesParticlesData;
        private int NumberParticles;
        private bool ObstaclesInit;

        public void SetMaximumParticles(int nParticles, Vector3 spawnDirX, Vector3 spawnDirY)
        {
            NumberParticles = nParticles;
            if (Particles.IsCreated) Particles.Dispose();
            if (ParticlesRead.IsCreated) ParticlesRead.Dispose();
            Particles = new NativeArray<BurstParticle>(NumberParticles, Allocator.Persistent);
            ParticlesRead = new NativeArray<BurstParticle>(NumberParticles, Allocator.Persistent);
            InitBuffers();
            SpawnParticles(spawnDirX.normalized, spawnDirY.normalized);
        }

        public Vector3 GetParticlePosition(int index)
        {
            return Particles[index].Position;
        }

        public Vector3 GetParticlePreviousPostion(int index)
        {
            return Particles[index].PreviousPosition;
        }

        public void SetParticlePosition(int index, Vector3 position)
        {
            BurstParticle particle = Particles[index];
            BurstParticle particleRead = ParticlesRead[index];
            particle.Position = position;
            particleRead.Position = position;
            Particles[index] = particle;
            ParticlesRead[index] = particleRead;
        }

        public void SetParticlePreviousPosition(int index, Vector3 position)
        {
            BurstParticle particle = Particles[index];
            BurstParticle particleRead = ParticlesRead[index];
            particle.PreviousPosition = position;
            particleRead.PreviousPosition = position;
            Particles[index] = particle;
            ParticlesRead[index] = particleRead;
        }

        private void InitBuffers()
        {
            ReleaseBuffers();

            InstancesParticlesData = new InstanceData[NumberParticles];

            uint[] args = new uint[5] { 0, 0, 0, 0, 0 };
            args[0] = (uint)Spawner.ParticleMesh.GetIndexCount(0);
            args[1] = (uint)NumberParticles;
            args[2] = (uint)Spawner.ParticleMesh.GetIndexStart(0);
            args[3] = (uint)Spawner.ParticleMesh.GetBaseVertex(0);
            ArgsParticlesBuffer = new ComputeBuffer(1, args.Length * sizeof(uint), ComputeBufferType.IndirectArguments);
            ArgsParticlesBuffer.SetData(args);

            InstancesParticlesBuffer = new ComputeBuffer(NumberParticles, InstanceData.Size());
            InstancesParticlesBuffer.SetData(new InstanceData[NumberParticles]);
            Spawner.ParticleMaterial.SetBuffer("_PerInstanceData", InstancesParticlesBuffer);
        }

        public void UpdateInstances(RenderType renderType)
        {
            if (renderType.HasFlag(RenderType.Particles))
            {
                float radius = Spawner.ParticleRadius;
                for (int i = 0; i < NumberParticles; i++)
                {
                    InstancesParticlesData[i] = new InstanceData(Particles[i].Position,
                                                        Quaternion.identity,
                                                        new Vector3(radius * 2.0f, radius * 2.0f, radius * 2.0f),
                                                        Spawner.FixedParticles[i]);
                }
                InstancesParticlesBuffer.SetData(InstancesParticlesData);
            }
            if (renderType.HasFlag(RenderType.Cloth))
            {

            }
        }

        public void Render(bool shadows, RenderType renderType)
        {
            if (renderType.HasFlag(RenderType.Particles))
            {
                Graphics.DrawMeshInstancedIndirect(mesh: Spawner.ParticleMesh,
                                                   submeshIndex: 0,
                                                   material: Spawner.ParticleMaterial,
                                                   bounds: Bounds,
                                                   bufferWithArgs: ArgsParticlesBuffer,
                                                   castShadows: shadows ? UnityEngine.Rendering.ShadowCastingMode.On : UnityEngine.Rendering.ShadowCastingMode.Off,
                                                   receiveShadows: shadows);
            }
            if (renderType.HasFlag(RenderType.Cloth))
            {

            }
        }

        public void SolveForces(float deltaTime, Vector3 windForce)
        {
            NativeArray<BurstParticle> particles = Particles;
            Particles = ParticlesRead;
            ParticlesRead = particles;

            var job = new ComputeForces
            {
                ParticlesRead = ParticlesRead,
                Particles = Particles,
                Damping = Spawner.Damping,
                Elasticity = Spawner.Elasticity,
                Gravity = Spawner.Gravity,
                SpringLength = Spawner.DistanceBetweenParticles,
                NumberParticles = Spawner.NumberParticles,
                Mass = Spawner.ParticleMass,
                Wind = windForce,
                Stretch = Spawner.Stretch,
                Shear = Spawner.Shear,
                Bend = Spawner.Bend
            };

            JobHandle handle = job.Schedule(Particles.Length, 64);
            handle.Complete(); // ensure job has finished

            // Fixed Particles
            for (int i = 0; i < Particles.Length; ++i)
            {
                if (Spawner.FixedParticles[i])
                {
                    BurstParticle p = Particles[i];
                    p.Force = float3.zero;
                    p.Velocity = float3.zero;
                    p.PreviousPosition = p.Position;
                    p.PreviousVelocity = float3.zero;
                    Particles[i] = p;
                }
            }
        }

        public void SolveMovement(Solver solver, float deltaTime, float kVerlet)
        {
            if (solver == Solver.Verlet)
            {
                var job = new VerletSolver
                {
                    Particles = Particles,
                    KVerlet = kVerlet,
                    DeltaTime = deltaTime,
                    Mass = Spawner.ParticleMass
                };
                JobHandle handle = job.Schedule(Particles.Length, 64);
                handle.Complete(); // ensure job has finished
            }
            else if (solver == Solver.EulerOrig)
            {
                var job = new EulerSolver
                {
                    Particles = Particles,
                    DeltaTime = deltaTime,
                    Mass = Spawner.ParticleMass
                };
                JobHandle handle = job.Schedule(Particles.Length, 64);
                handle.Complete(); // ensure job has finished
            }
            else if (solver == Solver.EulerSemi)
            {
                var job = new EulerSemiSolver
                {
                    Particles = Particles,
                    DeltaTime = deltaTime,
                    Mass = Spawner.ParticleMass
                };
                JobHandle handle = job.Schedule(Particles.Length, 64);
                handle.Complete(); // ensure job has finished
            }
        }

        public void SolveCollisions(Obstacle[] obstacles, float deltaTime)
        {
            UpdateObstaclesArray(obstacles);

            var job = new SolveCollisionsBurst
            {
                Particles = Particles,
                Planes = Planes,
                Spheres = Spheres,
                Triangles = Triangles,
                DeltaTime = deltaTime,
                ParticleBouncing = Spawner.ParticleBouncing,
                ParticleRadius = Spawner.ParticleRadius
            };

            JobHandle handle = job.Schedule(Particles.Length, 64);
            handle.Complete(); // ensure job has finished
        }

        private void UpdateObstaclesArray(Obstacle[] obstacles)
        {
            if (!ObstaclesInit)
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
                Planes = new NativeArray<BurstPlane>(planeCount, Allocator.Persistent);
                Spheres = new NativeArray<BurstSphere>(sphereCount, Allocator.Persistent);
                Triangles = new NativeArray<BurstTriangle>(triangleCount, Allocator.Persistent);
                ObstaclesInit = true;
            }
            int planeIndex = 0;
            int sphereIndex = 0;
            int triangleIndex = 0;
            foreach (Obstacle o in obstacles)
            {
                if (o is Plane)
                {
                    Plane p = o as Plane;
                    Planes[planeIndex] = new BurstPlane(p.Normal, p.D, p.Friction);
                    planeIndex += 1;
                }
                else if (o is Sphere)
                {
                    Sphere s = o as Sphere;
                    Spheres[sphereIndex] = new BurstSphere(s.Center, s.Radius, s.Friction);
                    sphereIndex += 1;
                }
                else if (o is Triangle)
                {
                    Triangle t = o as Triangle;
                    Triangles[triangleIndex] = new BurstTriangle(t.Normal, t.D, t.V1, t.V2, t.V3, t.Friction);
                    triangleIndex += 1;
                }
            }
        }

        private void SpawnParticles(Vector3 spawnDirX, Vector3 spawnDirY)
        {
            for (int i = 0; i < NumberParticles; i++)
            {
                int x = i % Spawner.NumberParticles.x;
                int y = i / Spawner.NumberParticles.x;
                Vector3 pos = Spawner.transform.position + (spawnDirX * x * Spawner.DistanceBetweenParticles.x) + (spawnDirY * y * Spawner.DistanceBetweenParticles.y);
                // Init Particle
                Particles[i] = new BurstParticle(pos, Vector3.zero, Spawner.SimulationTimestep);
            }
        }

        private void ReleaseBuffers()
        {
            if (InstancesParticlesBuffer != null)
            {
                InstancesParticlesBuffer.Release();
                InstancesParticlesBuffer = null;
            }
            if (ArgsParticlesBuffer != null)
            {
                ArgsParticlesBuffer.Release();
                ArgsParticlesBuffer = null;
            }
        }

        public void Release()
        {
            ReleaseBuffers();
            if (Particles.IsCreated) Particles.Dispose();
            if (ParticlesRead.IsCreated) ParticlesRead.Dispose();
            if (Planes.IsCreated) Planes.Dispose();
            if (Spheres.IsCreated) Spheres.Dispose();
            if (Triangles.IsCreated) Triangles.Dispose();
        }

        private struct BurstParticle
        {
            public float3 Position;
            public float3 Velocity;
            public float3 PreviousPosition;
            public float3 PreviousVelocity;
            public float3 Force;

            public BurstParticle(float3 position, float3 velocity, float deltaTime)
            {
                Position = position;
                PreviousPosition = Position - velocity * deltaTime;
                Velocity = velocity;
                PreviousVelocity = velocity;
                Force = float3.zero;
            }
        }

        private struct BurstPlane
        {
            public float3 N;
            public float D;
            public float Friction;

            public BurstPlane(float3 n, float d, float friction)
            {
                N = n;
                D = d;
                Friction = friction;
            }
        }

        private struct BurstSphere
        {
            public float3 Center;
            public float Radius;
            public float Friction;

            public BurstSphere(float3 center, float radius, float friction)
            {
                Center = center;
                Radius = radius;
                Friction = friction;
            }
        }

        private struct BurstTriangle
        {
            public float3 Normal;
            public float D;
            public float3 V1, V2, V3;
            public float Friction;

            public BurstTriangle(float3 normal, float d, float3 v1, float3 v2, float3 v3, float friction)
            {
                Normal = normal;
                D = d;
                V1 = v1;
                V2 = v2;
                V3 = v3;
                Friction = friction;
            }
        }

        [BurstCompile]
        private struct ComputeForces : IJobParallelFor
        {
            private static readonly int[] StretchY = { -1, 1, 0, 0 };
            private static readonly int[] StretchX = { 0, 0, -1, 1 };
            private static readonly int[] ShearY = { -1, -1, 1, 1 };
            private static readonly int[] ShearX = { -1, 1, -1, 1 };
            private static readonly int[] BendY = { -2, 2, 0, 0 };
            private static readonly int[] BendX = { 0, 0, -2, 2 };

            [ReadOnly] public NativeArray<BurstParticle> ParticlesRead;
            public NativeArray<BurstParticle> Particles;
            public Vector2 SpringLength;
            public Vector2Int NumberParticles;
            public float Gravity;
            public float Elasticity;
            public float Damping;
            public float Mass;
            public float3 Wind;
            public bool Stretch;
            public bool Shear;
            public bool Bend;

            public void Execute(int index)
            {
                BurstParticle p = ParticlesRead[index];
                int x = index % NumberParticles.x;
                int y = index / NumberParticles.x;
                float3 gravityForce = new float3(0, -Gravity, 0) * Mass;
                // Stretch
                float3 stretch = new float3(0, 0, 0);
                if (Stretch)
                {
                    for (int i = 0; i < StretchX.Length; i++)
                    {
                        int iy = StretchY[i];
                        int ix = StretchX[i];
                        int x1 = x + ix;
                        int y1 = y + iy;
                        if (x1 >= 0 && x1 < NumberParticles.x && y1 >= 0 && y1 < NumberParticles.y)
                        {
                            BurstParticle p1 = ParticlesRead[x1 + y1 * NumberParticles.x];
                            float springLength = SpringLength.x * abs(ix) + SpringLength.y * abs(iy);
                            if (ix > 0 || iy > 0)
                            {
                                float3 p1p = p1.Position - p.Position;
                                float lengthp1p = length(p1p);
                                float3 p1pUnit = p1p / lengthp1p;
                                stretch += (Elasticity * (lengthp1p - springLength) + Damping * dot(p1.Velocity - p.Velocity, p1pUnit)) * p1pUnit;
                            }
                            else
                            {
                                float3 pp1 = p.Position - p1.Position;
                                float lengthpp1 = length(pp1);
                                float3 pp1Unit = pp1 / lengthpp1;
                                stretch += -((Elasticity * (lengthpp1 - springLength) + Damping * dot(p.Velocity - p1.Velocity, pp1Unit)) * pp1Unit);
                            }
                        }
                    }
                }
                // Shear
                float3 shear = new float3(0, 0, 0);
                if (Shear)
                {
                    float springLength = sqrt(SpringLength.x * SpringLength.x + SpringLength.y * SpringLength.y);
                    for (int i = 0; i < ShearX.Length; i++)
                    {
                        int iy = ShearY[i];
                        int ix = ShearX[i];
                        int x1 = x + ix;
                        int y1 = y + iy;
                        if (x1 >= 0 && x1 < NumberParticles.x && y1 >= 0 && y1 < NumberParticles.y)
                        {
                            BurstParticle p1 = ParticlesRead[x1 + y1 * NumberParticles.x];
                            if (iy > 0)
                            {
                                float3 p1p = p1.Position - p.Position;
                                float lengthp1p = length(p1p);
                                float3 p1pUnit = p1p / lengthp1p;
                                shear += (Elasticity * (lengthp1p - springLength) + Damping * dot(p1.Velocity - p.Velocity, p1pUnit)) * p1pUnit;
                            }
                            else
                            {
                                float3 pp1 = p.Position - p1.Position;
                                float lengthpp1 = length(pp1);
                                float3 pp1Unit = pp1 / lengthpp1;
                                shear += -((Elasticity * (lengthpp1 - springLength) + Damping * dot(p.Velocity - p1.Velocity, pp1Unit)) * pp1Unit);
                            }
                        }
                    }
                }
                // Bend
                float3 bend = new float3(0, 0, 0);
                if (Bend)
                {
                    for (int i = 0; i < BendX.Length; i++)
                    {
                        int iy = BendX[i];
                        int ix = BendY[i];
                        int x1 = x + ix;
                        int y1 = y + iy;
                        if (x1 >= 1 && x1 < NumberParticles.x - 1 && y1 >= 1 && y1 < NumberParticles.y - 1)
                        {
                            BurstParticle p1 = ParticlesRead[x1 + y1 * NumberParticles.x];
                            float springLength = SpringLength.x * abs(ix) + SpringLength.y * abs(iy);
                            if (ix > 0 || iy > 0)
                            {
                                float3 p1p = p1.Position - p.Position;
                                float lengthp1p = length(p1p);
                                float3 p1pUnit = p1p / lengthp1p;
                                bend += (Elasticity * (lengthp1p - springLength) + Damping * dot(p1.Velocity - p.Velocity, p1pUnit)) * p1pUnit;
                            }
                            else
                            {
                                float3 pp1 = p.Position - p1.Position;
                                float lengthpp1 = length(pp1);
                                float3 pp1Unit = pp1 / lengthpp1;
                                bend += -((Elasticity * (lengthpp1 - springLength) + Damping * dot(p.Velocity - p1.Velocity, pp1Unit)) * pp1Unit);
                            }
                        }
                    }
                }
                p.Force = gravityForce + stretch + shear + bend + Wind;
                Particles[index] = p;
            }
        }

        [BurstCompile]
        private struct EulerSolver : IJobParallelFor
        {
            public NativeArray<BurstParticle> Particles;
            public float DeltaTime;
            public float Mass;

            public void Execute(int index)
            {
                BurstParticle p = Particles[index];
                p.PreviousPosition = p.Position;
                p.PreviousVelocity = p.Velocity;
                p.Position += p.Velocity * DeltaTime;
                p.Velocity += (p.Force / Mass) * DeltaTime;
                Particles[index] = p;
            }
        }

        [BurstCompile]
        private struct EulerSemiSolver : IJobParallelFor
        {
            public NativeArray<BurstParticle> Particles;
            public float DeltaTime;
            public float Mass;

            public void Execute(int index)
            {
                BurstParticle p = Particles[index];
                p.PreviousPosition = p.Position;
                p.PreviousVelocity = p.Velocity;
                p.Velocity += (p.Force / Mass) * DeltaTime;
                p.Position += p.Velocity * DeltaTime;
                Particles[index] = p;
            }
        }

        [BurstCompile]
        private struct VerletSolver : IJobParallelFor
        {
            public NativeArray<BurstParticle> Particles;
            public float KVerlet;
            public float DeltaTime;
            public float Mass;

            public void Execute(int index)
            {
                BurstParticle particle = Particles[index];
                float3 previousPreviousPosition = particle.PreviousPosition;
                particle.PreviousPosition = particle.Position;
                particle.PreviousVelocity = particle.Velocity;
                particle.Position = particle.PreviousPosition + KVerlet * (particle.PreviousPosition - previousPreviousPosition) + ((DeltaTime * DeltaTime) * particle.Force) / Mass;
                particle.Velocity = (particle.Position - particle.PreviousPosition) / DeltaTime;
                Particles[index] = particle;
            }
        }

        [BurstCompile]
        private struct SolveCollisionsBurst : IJobParallelFor
        {
            public NativeArray<BurstParticle> Particles;
            [ReadOnly] public NativeArray<BurstPlane> Planes;
            [ReadOnly] public NativeArray<BurstSphere> Spheres;
            [ReadOnly] public NativeArray<BurstTriangle> Triangles;
            public float DeltaTime;
            public float ParticleRadius;
            public float ParticleBouncing;

            public void Execute(int index)
            {
                BurstParticle p = Particles[index];
                bool collision = false;
                // Planes
                for (int j = 0; j < Planes.Length; j++)
                {
                    BurstPlane plane = Planes[j];
                    if (IsCrossingPlane(p, plane.N, plane.D))
                    {
                        p = CollisionPlaneParticle(p, plane.N, plane.D, plane.Friction);
                        collision = true;
                    }
                }
                // Spheres
                for (int j = 0; j < Spheres.Length; ++j)
                {
                    BurstSphere sphere = Spheres[j];
                    float3 dir = (p.Position + normalize(sphere.Center - p.Position) * ParticleRadius) - sphere.Center;
                    if (dot(dir, dir) < sphere.Radius * sphere.Radius)
                    {
                        p = CollisionSphereParticle(p, sphere, index == 17);
                        collision = true;
                    }
                }
                // Triangles
                for (int j = 0; j < Triangles.Length; ++j)
                {
                    BurstTriangle triangle = Triangles[j];
                    if (IsCrossingPlane(p, triangle.Normal, triangle.D) &&
                        OnTriangleParticle(p, triangle.V1, triangle.V2, triangle.V3, triangle.Normal, triangle.D))
                    {
                        p = CollisionPlaneParticle(p, triangle.Normal, triangle.D, triangle.Friction);
                        collision = true;
                    }
                }
                if (collision) p.PreviousPosition = p.Position - p.Velocity * DeltaTime;
                Particles[index] = p;
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            private bool IsCrossingPlane(BurstParticle p, float3 N, float d)
            {
                float sign = dot(p.Position - N * ParticleRadius, N) + d;
                sign *= dot(p.PreviousPosition + N * ParticleRadius, N) + d;
                return sign <= 0;
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            private BurstParticle CollisionPlaneParticle(BurstParticle p, float3 N, float d, float friction)
            {
                float dotVN = dot(p.Velocity, N);

                // Friction
                float3 velocityNormal = dotVN * N;
                float3 velocityTangent = p.Velocity - velocityNormal;

                // Elastic collision
                p.Velocity -= (1 + ParticleBouncing) * velocityNormal;

                // Apply friction
                p.Velocity -= friction * velocityTangent;

                // Update position
                p.Position -= (1 + ParticleBouncing) * (dot(p.Position - N * ParticleRadius, N) + d) * N;

                return p;
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            private BurstParticle CollisionSphereParticle(BurstParticle p, BurstSphere sphere, bool debug)
            {
                float3 previousPosition = p.PreviousPosition + normalize(sphere.Center - p.PreviousPosition) * ParticleRadius;
                // Segment-Sphere intersection
                float alpha = dot(p.PreviousVelocity, p.PreviousVelocity);
                float beta = dot(2 * p.PreviousVelocity, (previousPosition - sphere.Center));
                float gamma = dot(sphere.Center, sphere.Center) + dot(previousPosition, previousPosition) - dot(sphere.Center, 2 * previousPosition) - sphere.Radius * sphere.Radius;
                // Solve Second order equation
                float num = beta * beta - 4 * alpha * gamma;
                if (num < 0)
                {
                    // No intersection
                    return p;
                }
                float sqrtVar = sqrt(num);
                float s1 = Mathf.Abs((-beta + sqrtVar) / (2 * alpha));
                float s2 = Mathf.Abs((-beta - sqrtVar) / (2 * alpha));
                float s = s1 < s2 ? s1 : s2;
                // Intersection point with the sphere
                float3 P = previousPosition + s * p.PreviousVelocity;
                // Define tangent plane on P
                float3 N = normalize(P - sphere.Center);
                // Apply collision plane-particle
                return CollisionPlaneParticle(p, N, -dot(N, P), sphere.Friction);
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            private bool OnTriangleParticle(BurstParticle p, float3 v1, float3 v2, float3 v3, float3 N, float d)
            {
                float3 previousPosition = p.PreviousPosition - N * ParticleRadius;
                float3 line = (p.Position - N * ParticleRadius) - previousPosition;
                float t = (-d - dot(N, previousPosition)) / dot(N, line);
                if (t < 0 || t > 1) return false;
                float3 intersectionPoint = previousPosition + line * t;
                const float epsilon = 0.0001f;
                return Mathf.Abs(Area(intersectionPoint, v2, v3) + Area(v1, intersectionPoint, v3) + Area(v1, v2, intersectionPoint) - Area(v1, v2, v3)) < epsilon;
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            private float Area(Vector3 vi, Vector3 vj, Vector3 vk)
            {
                return 0.5f * length(cross(vj - vi, vk - vi));
            }
        }

        protected struct InstanceData
        {
            public Matrix4x4 TRSMatrix;
            public float Lifetime;

            public InstanceData(Matrix4x4 tRSMatrix, bool fix)
            {
                TRSMatrix = tRSMatrix;
                Lifetime = fix ? 1.0f : 0.0f;
            }

            public InstanceData(Vector3 position, Quaternion rotation, Vector3 scale, bool fix)
            {
                TRSMatrix = Matrix4x4.TRS(position, rotation, scale);
                Lifetime = fix ? 1.0f : 0.0f;
            }

            public static int Size()
            {
                return sizeof(float) * (4 * 4 + 1);
            }
        }

        protected struct InstanceHairData
        {
            public Matrix4x4 TRSMatrix;
            public float3x3 NormalMatrix;
            public float Lifetime;

            public InstanceHairData(Matrix4x4 tRSMatrix, bool fix)
            {
                TRSMatrix = tRSMatrix;
                NormalMatrix = (float3x3)transpose(inverse((float4x4)tRSMatrix));
                Lifetime = fix ? 1.0f : 0.0f;
            }

            public InstanceHairData(Vector3 position, Quaternion rotation, Vector3 scale, bool fix)
            {
                TRSMatrix = Matrix4x4.TRS(position, rotation, scale);
                NormalMatrix = (float3x3)transpose(inverse((float4x4)TRSMatrix));
                Lifetime = fix ? 1.0f : 0.0f;
            }

            public static int Size()
            {
                return sizeof(float) * (4 * 4 + 3 * 3 + 1);
            }
        }
    }
}
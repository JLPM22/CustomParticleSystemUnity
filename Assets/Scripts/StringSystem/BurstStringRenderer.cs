using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using CustomParticleSystem;

using static Unity.Mathematics.math;
using Plane = CustomParticleSystem.Plane;
using Solver = CustomStringSystem.StringSpawner.Solver;
using RenderType = CustomStringSystem.StringSpawner.RenderType;

namespace CustomStringSystem
{
    using System.Runtime.CompilerServices;
    using Unity.Mathematics;

    public class BurstStringRenderer
    {
        private StringSpawner Spawner;

        public BurstStringRenderer(StringSpawner spawner)
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
        private ComputeBuffer InstancesHairBuffer;
        private ComputeBuffer ArgsParticlesBuffer;
        private ComputeBuffer ArgsHairBuffer;
        private InstanceData[] InstancesParticlesData;
        private InstanceHairData[] InstancesHairData;
        private int NumberParticles;
        private bool ObstaclesInit;

        public void SetMaximumParticles(int nParticles, Vector3 spawnDir)
        {
            NumberParticles = nParticles;
            if (Particles.IsCreated) Particles.Dispose();
            if (ParticlesRead.IsCreated) ParticlesRead.Dispose();
            Particles = new NativeArray<BurstParticle>(NumberParticles, Allocator.Persistent);
            ParticlesRead = new NativeArray<BurstParticle>(NumberParticles, Allocator.Persistent);
            InitBuffers();
            SpawnParticles(spawnDir.normalized);
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
            InstancesHairData = new InstanceHairData[NumberParticles - 1];

            uint[] args = new uint[5] { 0, 0, 0, 0, 0 };
            args[0] = (uint)Spawner.ParticleMesh.GetIndexCount(0);
            args[1] = (uint)NumberParticles;
            args[2] = (uint)Spawner.ParticleMesh.GetIndexStart(0);
            args[3] = (uint)Spawner.ParticleMesh.GetBaseVertex(0);
            ArgsParticlesBuffer = new ComputeBuffer(1, args.Length * sizeof(uint), ComputeBufferType.IndirectArguments);
            ArgsParticlesBuffer.SetData(args);

            args[0] = (uint)Spawner.HairMesh.GetIndexCount(0);
            args[1] = (uint)(NumberParticles - 1);
            args[2] = (uint)Spawner.HairMesh.GetIndexStart(0);
            args[3] = (uint)Spawner.HairMesh.GetBaseVertex(0);
            ArgsHairBuffer = new ComputeBuffer(1, args.Length * sizeof(uint), ComputeBufferType.IndirectArguments);
            ArgsHairBuffer.SetData(args);

            InstancesParticlesBuffer = new ComputeBuffer(NumberParticles, InstanceData.Size());
            InstancesParticlesBuffer.SetData(new InstanceData[NumberParticles]);
            Spawner.ParticleMaterial.SetBuffer("_PerInstanceData", InstancesParticlesBuffer);

            InstancesHairBuffer = new ComputeBuffer(NumberParticles - 1, InstanceHairData.Size());
            InstancesHairBuffer.SetData(new InstanceHairData[NumberParticles - 1]);
            Spawner.HairMaterial.SetBuffer("_PerInstanceHairData", InstancesHairBuffer);
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
            if (renderType.HasFlag(RenderType.Hair))
            {
                float radius = Spawner.HairRadius;
                for (int i = 0; i < NumberParticles - 1; i++)
                {
                    Vector3 p1 = Particles[i].Position;
                    Vector3 p2 = Particles[i + 1].Position;
                    Vector3 norm = normalize(p2 - p1);
                    Vector3 axisRot = Vector3.Cross(Vector3.up, norm);
                    Quaternion rot = Quaternion.AngleAxis(acos(Vector3.Dot(norm, Vector3.up)) * Mathf.Rad2Deg, axisRot);
                    InstancesHairData[i] = new InstanceHairData((p1 + p2) / 2.0f,
                                                            rot,
                                                            new Vector3(radius * 2.0f, length(p1 - p2) * 0.5f, radius * 2.0f),
                                                            Spawner.FixedParticles[i]);
                }
                InstancesHairBuffer.SetData(InstancesHairData);
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
            if (renderType.HasFlag(RenderType.Hair))
            {
                Graphics.DrawMeshInstancedIndirect(mesh: Spawner.HairMesh,
                                                   submeshIndex: 0,
                                                   material: Spawner.HairMaterial,
                                                   bounds: Bounds,
                                                   bufferWithArgs: ArgsHairBuffer,
                                                   castShadows: shadows ? UnityEngine.Rendering.ShadowCastingMode.On : UnityEngine.Rendering.ShadowCastingMode.Off,
                                                   receiveShadows: shadows);
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
                StringLength = Spawner.DistanceBetweenParticles,
                Mass = Spawner.ParticleMass,
                Wind = windForce
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

        private void SpawnParticles(Vector3 spawnDir)
        {

            for (int i = 0; i < NumberParticles; i++)
            {
                Vector3 pos = Spawner.transform.position + spawnDir * Spawner.DistanceBetweenParticles * i;
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
            if (InstancesHairBuffer != null)
            {
                InstancesHairBuffer.Release();
                InstancesHairBuffer = null;
            }
            if (ArgsParticlesBuffer != null)
            {
                ArgsParticlesBuffer.Release();
                ArgsParticlesBuffer = null;
            }
            if (ArgsHairBuffer != null)
            {
                ArgsHairBuffer.Release();
                ArgsHairBuffer = null;
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
            [ReadOnly] public NativeArray<BurstParticle> ParticlesRead;
            public NativeArray<BurstParticle> Particles;
            public float StringLength;
            public float Gravity;
            public float Elasticity;
            public float Damping;
            public float Mass;
            public float3 Wind;

            public void Execute(int index)
            {
                BurstParticle p0 = ParticlesRead[(index - 1) < 0 ? 0 : (index - 1)];
                BurstParticle p1 = ParticlesRead[index];
                BurstParticle p2 = ParticlesRead[(index + 1) % ParticlesRead.Length];
                float3 gravityForce = new float3(0, -Gravity, 0) * Mass;
                float3 force1 = new float3(0, 0, 0);
                if (index != ParticlesRead.Length - 1)
                {
                    float3 p2p1 = p2.Position - p1.Position;
                    float lengthp2p1 = length(p2p1);
                    float3 p2p1unit = p2p1 / lengthp2p1;
                    force1 = (Elasticity * (lengthp2p1 - StringLength) + Damping * dot((p2.Velocity - p1.Velocity), p2p1unit)) * p2p1unit;
                }
                float3 force2 = new float3(0, 0, 0);
                if (index != 0)
                {
                    float3 p1p0 = p1.Position - p0.Position;
                    float lengthp1p0 = length(p1p0);
                    float3 p1p0unit = p1p0 / lengthp1p0;
                    force2 = -((Elasticity * (lengthp1p0 - StringLength) + Damping * dot((p1.Velocity - p0.Velocity), p1p0unit)) * p1p0unit);
                }
                p1.Force = gravityForce + force1 + force2 + Wind;
                Particles[index] = p1;
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

        // [BurstCompile]
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
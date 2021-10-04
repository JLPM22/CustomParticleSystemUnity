using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace CustomParticleSystem
{
    public class CPUParticleRenderer
    {
        private struct InstanceData
        {
            public Matrix4x4 TRSMatrix;

            public InstanceData(Matrix4x4 tRSMatrix)
            {
                TRSMatrix = tRSMatrix;
            }

            public InstanceData(Vector3 position, Quaternion rotation, Vector3 scale)
            {
                TRSMatrix = Matrix4x4.TRS(position, rotation, scale);
            }

            public static int Size()
            {
                return sizeof(float) * 4 * 4;
            }
        }

        private Bounds Bounds = new Bounds(Vector3.zero, 10000 * Vector3.one);
        private ComputeBuffer InstancesBuffer;
        private ComputeBuffer ArgsBuffer;
        private InstanceData[] InstancesData;
        private int MaximumParticles;
        private Mesh Mesh;
        private Material Material;
        private Vector3 Scale = Vector3.one;

        public CPUParticleRenderer(Mesh mesh, Material material)
        {
            Mesh = mesh;
            Material = material;
        }

        public void SetMaximumParticles(int maxParticles)
        {
            MaximumParticles = maxParticles;
            InstancesData = new InstanceData[MaximumParticles];
            InitBuffers();
        }

        public void SetRadius(float radius)
        {
            Scale = new Vector3(radius * 2, radius * 2, radius * 2);
        }

        public void UpdateInstances(List<Particle> particles)
        {
            for (int i = 0; i < Mathf.Min(particles.Count, MaximumParticles); i++)
            {
                if (particles[i].LifeTime > 0)
                {
                    InstancesData[i] = new InstanceData(particles[i].Position, Quaternion.identity, Scale);
                }
                else
                {
                    InstancesData[i] = new InstanceData(Vector3.zero, Quaternion.identity, Vector3.zero);
                }
            }
            InstancesBuffer.SetData(InstancesData);
        }

        public void Render()
        {
            Graphics.DrawMeshInstancedIndirect(Mesh, 0, Material, Bounds, ArgsBuffer);
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

            InstancesBuffer = new ComputeBuffer(MaximumParticles, InstanceData.Size());
            InstancesBuffer.SetData(new InstanceData[MaximumParticles]);
            Material.SetBuffer("_PerInstanceData", InstancesBuffer);
        }

        private void ReleaseBuffers()
        {
            if (InstancesBuffer != null)
            {
                InstancesBuffer.Release();
                InstancesBuffer = null;
            }
            if (ArgsBuffer != null)
            {
                ArgsBuffer.Release();
                ArgsBuffer = null;
            }
        }

        public void Release()
        {
            ReleaseBuffers();
        }
    }
}
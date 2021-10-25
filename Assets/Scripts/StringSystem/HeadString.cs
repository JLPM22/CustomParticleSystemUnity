using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace CustomStringSystem
{
    public class HeadString : MonoBehaviour
    {
        public StringSpawner StringSpawnerPrefab;
        public int NumberHair;
        public int ParticlesPerHair;
        public float DistanceBetweenParticles;
        public float HairRadius;
        public float ParticleRadius;
        public float ParticleMass;
        public StringSpawner.RenderType RenderType;
        public Material ParticleMaterial;
        public Material HairMaterial;

        private List<StringSpawner> StringSpawners = new List<StringSpawner>();
        private List<Vector3> Directions = new List<Vector3>();

        private void Start()
        {
            float radius = (transform.localScale.x / 2) + ParticleRadius * 1.2f;

            for (int i = 0; i < NumberHair; i++)
            {
                Vector3 dir = Random.onUnitSphere;
                while (!(Vector3.Dot(dir, Vector3.up) > 0.0f && Vector3.Dot(dir, Vector3.forward) < 0.0f)) dir = Random.onUnitSphere;
                Vector3 pos = transform.position + dir * radius;
                StringSpawner stringSpawner = Instantiate<StringSpawner>(StringSpawnerPrefab, pos, Quaternion.identity, transform);
                stringSpawner.StringRenderType = RenderType;
                stringSpawner.NumberParticles = ParticlesPerHair;
                stringSpawner.FixedParticles = new bool[ParticlesPerHair];
                stringSpawner.FixedParticles[0] = true;
                stringSpawner.DistanceBetweenParticles = DistanceBetweenParticles;
                stringSpawner.InitParticleSpawnDir = -transform.forward;
                stringSpawner.ParticleRadius = ParticleRadius;
                stringSpawner.HairRadius = HairRadius;
                stringSpawner.KVerlet = 0.97f;
                stringSpawner.StringSolver = StringSpawner.Solver.Verlet;
                stringSpawner.ParticleBouncing = 0.5f;
                stringSpawner.ParticleMass = ParticleMass;
                stringSpawner.EnableInput = false;
                stringSpawner.ParticleMaterial = new Material(ParticleMaterial);
                stringSpawner.HairMaterial = new Material(HairMaterial);
                stringSpawner.SimulationTimestep = 0.005f;
                stringSpawner.Shadows = false;
                stringSpawner.RecomputeElasticityAndDamping();
                StringSpawners.Add(stringSpawner);
                Directions.Add(dir);
            }
        }

        private void Update()
        {
            float radius = (transform.localScale.x / 2) + ParticleRadius * 1.2f;

            for (int i = 0; i < StringSpawners.Count; i++)
            {
                StringSpawner stringSpawner = StringSpawners[i];
                stringSpawner.SetParticlePosition(0, transform.position + Directions[i] * radius);
                stringSpawner.SetParticlePreviousPosition(0, transform.position + Directions[i] * radius);
            }
        }

        private void OnDestroy()
        {
            foreach (StringSpawner stringSpawner in StringSpawners)
            {
                Destroy(stringSpawner.ParticleMaterial);
                Destroy(stringSpawner.HairMaterial);
            }
        }
    }
}
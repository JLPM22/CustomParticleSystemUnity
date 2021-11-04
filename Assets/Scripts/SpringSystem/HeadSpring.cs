using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace CustomSpringSystem
{
    public class HeadSpring : MonoBehaviour
    {
        public SpringSpawner SpringSpawnerPrefab;
        public int NumberHair;
        public int ParticlesPerHair;
        public float DistanceBetweenParticles;
        public float HairRadius;
        public float ParticleRadius;
        public float ParticleMass;
        public SpringSpawner.RenderType RenderType;
        public Material ParticleMaterial;
        public Material HairMaterial;

        private List<SpringSpawner> StringSpawners = new List<SpringSpawner>();
        private List<Vector3> Directions = new List<Vector3>();

        private void Start()
        {
            float radius = (transform.localScale.x / 2) + ParticleRadius * 1.2f;

            for (int i = 0; i < NumberHair; i++)
            {
                Vector3 dir = Random.onUnitSphere;
                while (!(Vector3.Dot(dir, Vector3.up) > 0.0f && Vector3.Dot(dir, Vector3.forward) < 0.5f)) dir = Random.onUnitSphere;
                Vector3 pos = transform.position + dir * radius;
                SpringSpawner springSpawner = Instantiate<SpringSpawner>(SpringSpawnerPrefab, pos, Quaternion.identity, transform);
                springSpawner.SpringRenderType = RenderType;
                springSpawner.NumberParticles = ParticlesPerHair;
                springSpawner.FixedParticles = new bool[ParticlesPerHair];
                springSpawner.FixedParticles[0] = true;
                springSpawner.DistanceBetweenParticles = DistanceBetweenParticles;
                springSpawner.InitParticleSpawnDir = -transform.forward;
                springSpawner.ParticleRadius = ParticleRadius;
                springSpawner.HairRadius = HairRadius;
                springSpawner.KVerlet = 0.97f;
                springSpawner.SpringSolver = SpringSpawner.Solver.Verlet;
                springSpawner.ParticleBouncing = 0.5f;
                springSpawner.ParticleMass = ParticleMass;
                springSpawner.EnableInput = false;
                springSpawner.ParticleMaterial = new Material(ParticleMaterial);
                springSpawner.HairMaterial = new Material(HairMaterial);
                springSpawner.SimulationTimestep = 0.005f;
                springSpawner.Shadows = true;
                springSpawner.RecomputeElasticityAndDamping();
                StringSpawners.Add(springSpawner);
                Directions.Add(dir);
            }
        }

        private void Update()
        {
            float radius = (transform.localScale.x / 2) + ParticleRadius * 1.2f;

            for (int i = 0; i < StringSpawners.Count; i++)
            {
                SpringSpawner springSpawner = StringSpawners[i];
                springSpawner.SetParticlePosition(0, transform.position + Directions[i] * radius);
                springSpawner.SetParticlePreviousPosition(0, transform.position + Directions[i] * radius);
            }
        }

        private void OnDestroy()
        {
            foreach (SpringSpawner springSpawner in StringSpawners)
            {
                Destroy(springSpawner.ParticleMaterial);
                Destroy(springSpawner.HairMaterial);
            }
        }
    }
}
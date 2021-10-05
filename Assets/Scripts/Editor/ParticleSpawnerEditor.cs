using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;
using CustomParticleSystem;

using ParticleSpawner = CustomParticleSystem.ParticleSpawner;
using Solver = CustomParticleSystem.Solver;
using UnityEditor.SceneManagement;

[CustomEditor(typeof(ParticleSpawner))]
public class ParticleSpawnerEditor : Editor
{
    public override void OnInspectorGUI()
    {
        const int space = 5;

        ParticleSpawner ps = (ParticleSpawner)target;

        EditorGUILayout.BeginHorizontal();
        ps.SimulationTimestep = EditorGUILayout.FloatField("Simulation Timestep", ps.SimulationTimestep);
        if (ps.SimulationTimestep <= 0.0f) ps.SimulationTimestep = 0.001f;
        EditorGUILayout.LabelField("FPS: " + (1 / ps.SimulationTimestep).ToString("F0"));
        EditorGUILayout.EndHorizontal();
        ps.ExecutionMethod = (ParticleSpawner.Method)EditorGUILayout.EnumPopup("Execution Method", ps.ExecutionMethod);
        EditorGUILayout.Space(space);

        ps.EmissionShape = (ParticleSpawner.Shape)EditorGUILayout.EnumPopup("Emission Shape", ps.EmissionShape);
        if (ps.EmissionShape == ParticleSpawner.Shape.Explosion)
        {
            ps.EmissionExplosionSpeed = EditorGUILayout.FloatField("Explosion Speed", ps.EmissionExplosionSpeed);
        }
        else if (ps.EmissionShape == ParticleSpawner.Shape.Fountain)
        {
            ps.EmissionFountainSpeed = EditorGUILayout.FloatField("Fountain Speed", ps.EmissionFountainSpeed);
        }
        else if (ps.EmissionShape == ParticleSpawner.Shape.Waterfall)
        {
            ps.EmissionWaterfallHeight = EditorGUILayout.FloatField("Waterfall Height", ps.EmissionWaterfallHeight);
            ps.EmissionWaterfallSpeed = EditorGUILayout.FloatField("Waterfall Speed", ps.EmissionWaterfallSpeed);
        }
        else if (ps.EmissionShape == ParticleSpawner.Shape.SemiSphere)
        {
            ps.EmissionSemiSphereRadius = EditorGUILayout.FloatField("Semi-Sphere Radius", ps.EmissionSemiSphereRadius);
            if (ps.EmissionSemiSphereRadius <= 0.0f) ps.EmissionSemiSphereRadius = 0.01f;
            ps.EmissionSemiSphereSpeed = EditorGUILayout.FloatField("SemiSphere Speed", ps.EmissionSemiSphereSpeed);
        }
        EditorGUILayout.Space(space);

        ps.ParticleSolver = (Solver)EditorGUILayout.EnumPopup("Particle Solver", ps.ParticleSolver);
        if (ps.ParticleSolver == Solver.Verlet)
        {
            ps.KVerlet = EditorGUILayout.Slider("K Verlet", ps.KVerlet, 0.95f, 1.0f);
        }
        EditorGUILayout.Space(space);

        ps.EmissionRate = EditorGUILayout.FloatField("Emission Rate", ps.EmissionRate);
        if (ps.EmissionRate <= 0) ps.EmissionRate = 0.1f;
        ps.Gravity = EditorGUILayout.FloatField("Gravity", ps.Gravity);
        ps.MaximumNumberParticles = EditorGUILayout.IntField("Max Particles", ps.MaximumNumberParticles);
        if (ps.MaximumNumberParticles < 1) ps.MaximumNumberParticles = 1;
        EditorGUILayout.Space(space);

        ps.ParticleLifeTime = EditorGUILayout.FloatField("Particle Life Time", ps.ParticleLifeTime);
        if (ps.ParticleLifeTime <= 0) ps.ParticleLifeTime = 0.1f;
        ps.ParticleBouncing = EditorGUILayout.Slider("Particle Bouncing", ps.ParticleBouncing, 0.0f, 1.0f);
        ps.ParticleMass = EditorGUILayout.FloatField("Particle Mass", ps.ParticleMass);
        ps.ParticleRadius = EditorGUILayout.FloatField("Particle Radius", ps.ParticleRadius);
        ps.ParticleMesh = (Mesh)EditorGUILayout.ObjectField("Particle Mesh", ps.ParticleMesh, typeof(Mesh), false);
        ps.ParticleMaterial = (Material)EditorGUILayout.ObjectField("Particle Material", ps.ParticleMaterial, typeof(Material), false);
        if (ps.ParticleRadius < 0.001f) ps.ParticleRadius = 0.001f;

        // Save changes
        if (GUI.changed && !Application.isPlaying)
        {
            EditorUtility.SetDirty(ps);
            EditorSceneManager.MarkSceneDirty(ps.gameObject.scene);
        }
    }
}
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;
using UnityEditor.SceneManagement;
using CustomClothSystem;

using Solver = CustomClothSystem.ClothSpawner.Solver;

[CustomEditor(typeof(ClothSpawner))]
public class ClothSpawnerEditor : Editor
{
    private bool ShowFixedParticles;

    public override void OnInspectorGUI()
    {
        const int space = 5;

        ClothSpawner ss = (ClothSpawner)target;

        EditorGUILayout.BeginHorizontal();
        ss.SimulationTimestep = EditorGUILayout.FloatField("Simulation Timestep", ss.SimulationTimestep);
        if (ss.SimulationTimestep <= 0.0f) ss.SimulationTimestep = 0.001f;
        EditorGUILayout.LabelField("FPS: " + (1 / ss.SimulationTimestep).ToString("F0"));
        EditorGUILayout.EndHorizontal();
        ss.EnableInput = EditorGUILayout.Toggle("Enable Input", ss.EnableInput);
        ss.Shadows = EditorGUILayout.Toggle("Shadows", ss.Shadows);
        EditorGUILayout.Space(space);

        ss.ClothSolver = (Solver)EditorGUILayout.EnumPopup("Particle Solver", ss.ClothSolver);
        if (ss.ClothSolver == Solver.Verlet)
        {
            ss.KVerlet = EditorGUILayout.Slider("K Verlet", ss.KVerlet, 0.95f, 1.0f);
        }
        EditorGUILayout.Space(space);

        // String Render Type is a Mask
        ss.ClothRenderType = (ClothSpawner.RenderType)EditorGUILayout.EnumFlagsField("String Render Type", ss.ClothRenderType);
        EditorGUILayout.Space(space);

        ss.NumberParticles = EditorGUILayout.Vector2IntField("Number Particles", ss.NumberParticles);
        if (ss.NumberParticles.x < 1) ss.NumberParticles = new Vector2Int(1, ss.NumberParticles.y);
        if (ss.NumberParticles.y < 1) ss.NumberParticles = new Vector2Int(ss.NumberParticles.x, 1);
        ss.DistanceBetweenParticles = EditorGUILayout.Vector2Field("Distance Between Particles", ss.DistanceBetweenParticles);
        if (ss.DistanceBetweenParticles.x <= 0.0f) ss.DistanceBetweenParticles = new Vector2(0.001f, ss.DistanceBetweenParticles.y);
        if (ss.DistanceBetweenParticles.y <= 0.0f) ss.DistanceBetweenParticles = new Vector2(ss.DistanceBetweenParticles.x, 0.001f);
        ss.InitParticleSpawnDirX = EditorGUILayout.Vector3Field("Initial Particle Spawn Direction X", ss.InitParticleSpawnDirX);
        ss.InitParticleSpawnDirY = EditorGUILayout.Vector3Field("Initial Particle Spawn Direction Y", ss.InitParticleSpawnDirY);
        if (ss.InitParticleSpawnDirX == Vector3.zero) ss.InitParticleSpawnDirX = Vector3.right;
        if (ss.InitParticleSpawnDirY == Vector3.zero) ss.InitParticleSpawnDirY = Vector3.down;
        if (ss.FixedParticles == null || (ss.FixedParticles.Length != (ss.NumberParticles.x * ss.NumberParticles.y)))
        {
            ss.FixedParticles = new bool[ss.NumberParticles.x * ss.NumberParticles.y];
        }
        ShowFixedParticles = EditorGUILayout.BeginFoldoutHeaderGroup(ShowFixedParticles, "Fixed Particles");
        if (ShowFixedParticles)
        {
            for (int i = 0; i < ss.NumberParticles.y; ++i)
            {
                bool row = true;
                for (int x = 0; x < ss.NumberParticles.x; ++x)
                {
                    row = row && ss.FixedParticles[x + i * ss.NumberParticles.x];
                }
                row = EditorGUILayout.Toggle("Row " + i.ToString(), row);
                for (int x = 0; x < ss.NumberParticles.x; ++x)
                {
                    ss.FixedParticles[x + i * ss.NumberParticles.x] = row;
                }
            }
        }
        EditorGUILayout.EndFoldoutHeaderGroup();
        EditorGUILayout.Space(space);

        ss.Elasticity = EditorGUILayout.FloatField("Elasticity", ss.Elasticity);
        if (ss.Elasticity < 0.0f) ss.Elasticity = 0.0f;
        ss.Damping = EditorGUILayout.FloatField("Damping", ss.Damping);
        if (ss.Damping < 0.0f) ss.Damping = 0.0f;
        ss.Gravity = EditorGUILayout.FloatField("Gravity", ss.Gravity);
        EditorGUILayout.Space(space);

        ss.Stretch = EditorGUILayout.Toggle("Stretch", ss.Stretch);
        ss.Shear = EditorGUILayout.Toggle("Shear", ss.Shear);
        ss.Bend = EditorGUILayout.Toggle("Bend", ss.Bend);
        EditorGUILayout.Space(space);

        ss.ParticleBouncing = EditorGUILayout.Slider("Particle Bouncing", ss.ParticleBouncing, 0.0f, 1.0f);
        ss.ParticleMass = EditorGUILayout.FloatField("Particle Mass", ss.ParticleMass);
        EditorGUILayout.Space(space);

        ss.ParticleRadius = EditorGUILayout.FloatField("Particle Radius", ss.ParticleRadius);
        if (ss.ParticleRadius < 0.001f) ss.ParticleRadius = 0.001f;
        ss.ParticleMesh = (Mesh)EditorGUILayout.ObjectField("Particle Mesh", ss.ParticleMesh, typeof(Mesh), false);
        ss.ParticleMaterial = (Material)EditorGUILayout.ObjectField("Particle Material", ss.ParticleMaterial, typeof(Material), false);
        EditorGUILayout.Space(space);

        ss.ClothMaterial = (Material)EditorGUILayout.ObjectField("Cloth Material", ss.ClothMaterial, typeof(Material), false);
        EditorGUILayout.Space(space);

        if (GUILayout.Button("Automatic Elasticity and Dumping"))
        {
            ss.RecomputeElasticityAndDamping();
        }

        // Save changes
        if (GUI.changed && !Application.isPlaying)
        {
            EditorUtility.SetDirty(ss);
            EditorSceneManager.MarkSceneDirty(ss.gameObject.scene);
        }
    }
}

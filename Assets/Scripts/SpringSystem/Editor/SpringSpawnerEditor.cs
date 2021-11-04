using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;
using UnityEditor.SceneManagement;
using CustomSpringSystem;

using Solver = CustomSpringSystem.SpringSpawner.Solver;

[CustomEditor(typeof(SpringSpawner))]
public class SpringSpawnerEditor : Editor
{
    private bool ShowFixedParticles;

    public override void OnInspectorGUI()
    {
        const int space = 5;

        SpringSpawner ss = (SpringSpawner)target;

        EditorGUILayout.BeginHorizontal();
        ss.SimulationTimestep = EditorGUILayout.FloatField("Simulation Timestep", ss.SimulationTimestep);
        if (ss.SimulationTimestep <= 0.0f) ss.SimulationTimestep = 0.001f;
        EditorGUILayout.LabelField("FPS: " + (1 / ss.SimulationTimestep).ToString("F0"));
        EditorGUILayout.EndHorizontal();
        ss.EnableInput = EditorGUILayout.Toggle("Enable Input", ss.EnableInput);
        ss.Shadows = EditorGUILayout.Toggle("Shadows", ss.Shadows);
        EditorGUILayout.Space(space);

        ss.SpringSolver = (Solver)EditorGUILayout.EnumPopup("Particle Solver", ss.SpringSolver);
        if (ss.SpringSolver == Solver.Verlet)
        {
            ss.KVerlet = EditorGUILayout.Slider("K Verlet", ss.KVerlet, 0.95f, 1.0f);
        }
        EditorGUILayout.Space(space);

        // String Render Type is a Mask
        ss.SpringRenderType = (SpringSpawner.RenderType)EditorGUILayout.EnumFlagsField("String Render Type", ss.SpringRenderType);
        EditorGUILayout.Space(space);

        ss.NumberParticles = EditorGUILayout.IntField("Number Particles", ss.NumberParticles);
        if (ss.NumberParticles < 1) ss.NumberParticles = 1;
        ss.DistanceBetweenParticles = EditorGUILayout.FloatField("Distance Between Particles", ss.DistanceBetweenParticles);
        if (ss.DistanceBetweenParticles <= 0.0f) ss.DistanceBetweenParticles = 0.001f;
        ss.InitParticleSpawnDir = EditorGUILayout.Vector3Field("Initial Particle Spawn Direction", ss.InitParticleSpawnDir);
        if (ss.InitParticleSpawnDir == Vector3.zero) ss.InitParticleSpawnDir = Vector3.down;
        if (ss.FixedParticles == null || (ss.FixedParticles.Length != ss.NumberParticles)) ss.FixedParticles = new bool[ss.NumberParticles];
        ShowFixedParticles = EditorGUILayout.BeginFoldoutHeaderGroup(ShowFixedParticles, "Fixed Particles");
        if (ShowFixedParticles)
        {
            for (int i = 0; i < ss.FixedParticles.Length + 4; i += 4)
            {
                EditorGUILayout.BeginHorizontal();
                for (int j = i; j < i + 4 && j < ss.FixedParticles.Length; j++)
                {
                    EditorGUILayout.LabelField(j.ToString(), GUILayout.Width(20));
                    ss.FixedParticles[j] = EditorGUILayout.Toggle(ss.FixedParticles[j], GUILayout.Width(20));
                }
                EditorGUILayout.EndHorizontal();
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

        ss.ParticleBouncing = EditorGUILayout.Slider("Particle Bouncing", ss.ParticleBouncing, 0.0f, 1.0f);
        ss.ParticleMass = EditorGUILayout.FloatField("Particle Mass", ss.ParticleMass);
        EditorGUILayout.Space(space);

        ss.ParticleRadius = EditorGUILayout.FloatField("Particle Radius", ss.ParticleRadius);
        if (ss.ParticleRadius < 0.001f) ss.ParticleRadius = 0.001f;
        ss.ParticleMesh = (Mesh)EditorGUILayout.ObjectField("Particle Mesh", ss.ParticleMesh, typeof(Mesh), false);
        ss.ParticleMaterial = (Material)EditorGUILayout.ObjectField("Particle Material", ss.ParticleMaterial, typeof(Material), false);
        EditorGUILayout.Space(space);

        ss.HairRadius = EditorGUILayout.FloatField("Hair Radius", ss.HairRadius);
        if (ss.HairRadius < 0.001f) ss.HairRadius = 0.001f;
        ss.HairMesh = (Mesh)EditorGUILayout.ObjectField("Hair Mesh", ss.HairMesh, typeof(Mesh), false);
        ss.HairMaterial = (Material)EditorGUILayout.ObjectField("Hair Material", ss.HairMaterial, typeof(Material), false);
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

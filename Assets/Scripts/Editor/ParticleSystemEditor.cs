using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;
using CustomParticleSystem;

using ParticleSystem = CustomParticleSystem.ParticleSystem;
using Solver = CustomParticleSystem.Solver;
using UnityEditor.SceneManagement;

[CustomEditor(typeof(ParticleSystem))]
public class ParticleSystemEditor : Editor
{
    public override void OnInspectorGUI()
    {
        const int space = 5;

        ParticleSystem ps = (ParticleSystem)target;

        EditorGUILayout.BeginHorizontal();
        ps.SimulationTimestep = EditorGUILayout.FloatField("Simulation Timestep", ps.SimulationTimestep);
        EditorGUILayout.LabelField("FPS: " + (1 / ps.SimulationTimestep).ToString("F0"));
        EditorGUILayout.EndHorizontal();
        EditorGUILayout.Space(space);

        ps.ParticleSolver = (Solver)EditorGUILayout.EnumPopup("Particle Solver", ps.ParticleSolver);
        if (ps.ParticleSolver == Solver.Verlet)
        {
            ps.KVerlet = EditorGUILayout.Slider("K Verlet", ps.KVerlet, 0.95f, 1.0f);
        }
        EditorGUILayout.Space(space);

        ps.Gravity = EditorGUILayout.FloatField("Gravity", ps.Gravity);
        EditorGUILayout.Space(space);

        ps.ParticleBouncing = EditorGUILayout.Slider("Particle Bouncing", ps.ParticleBouncing, 0.0f, 1.0f);
        ps.ParticleMass = EditorGUILayout.FloatField("Particle Mass", ps.ParticleMass);

        // Save changes
        if (GUI.changed)
        {
            EditorUtility.SetDirty(ps);
            EditorSceneManager.MarkSceneDirty(ps.gameObject.scene);
        }
    }
}
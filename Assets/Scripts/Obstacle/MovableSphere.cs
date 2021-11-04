using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using CustomSpringSystem;

public class MovableSphere : MonoBehaviour
{
    public float Speed = 10.0f;
    private bool IsMoving = false;
    private UnityEngine.Plane MovingPlane;
    private Camera MainCamera;
    private Vector3 TargetPosition;

    private SpringSpawner[] StringSpawners;

    private void Awake()
    {
        MainCamera = Camera.main;
        TargetPosition = transform.position;
    }

    private IEnumerator Start()
    {
        enabled = false;
        for (int i = 0; i < 5; i++) yield return null;
        enabled = true;
        StringSpawners = FindObjectsOfType<SpringSpawner>();
    }

    private void LateUpdate()
    {
        if (Input.GetMouseButtonDown(0))
        {
            StartMoving();
        }
        else if (Input.GetMouseButtonUp(0))
        {
            IsMoving = false;
        }
        if (IsMoving)
        {
            Move();
        }

        Vector3 prev = transform.position;
        transform.position = Vector3.Lerp(transform.position, TargetPosition, Time.deltaTime * Speed);

        float radius = transform.localScale.x / 2.0f;
        Vector3 offset = (transform.position - prev) * 2.0f;
        foreach (SpringSpawner spawner in StringSpawners)
        {
            for (int i = 0; i < spawner.NumberParticles; i++)
            {
                Vector3 particlePos = spawner.GetParticlePosition(i);
                if (Vector3.Dot(particlePos - transform.position, particlePos - transform.position) < (radius + spawner.ParticleRadius * 1.25f) * (radius + spawner.ParticleRadius * 1.25f))
                {
                    spawner.SetParticlePosition(i, particlePos + offset);
                    spawner.SetParticlePreviousPosition(i, spawner.GetParticlePreviousPosition(i) + offset);
                }
            }
        }
    }

    private void Move()
    {
        Ray line = MainCamera.ScreenPointToRay(Input.mousePosition);
        MovingPlane.Raycast(line, out float distance);
        TargetPosition = line.origin + line.direction * distance;
    }

    private void StartMoving()
    {
        Ray line = MainCamera.ScreenPointToRay(Input.mousePosition);
        IsMoving = TestSphereLineIntersection(line.origin, line.direction);
        MovingPlane = new Plane(-line.direction, transform.position);
    }

    private bool TestSphereLineIntersection(Vector3 lineStart, Vector3 lineDir)
    {
        float radius = transform.localScale.x * 0.5f;
        Vector3 pos = transform.position;
        float a = Vector3.Dot(lineDir, (lineStart - pos));
        float l = Vector3.Magnitude(lineStart - pos);
        float d = a * a - (l * l - radius * radius);
        return d >= 0.0f;
    }
}

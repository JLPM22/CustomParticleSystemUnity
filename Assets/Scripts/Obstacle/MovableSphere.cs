using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using CustomSpringSystem;
using CustomClothSystem;

public class MovableSphere : MonoBehaviour
{
    public float Speed = 10.0f;
    private bool IsMoving = false;
    private Vector3 TargetPosition;

    private CameraController MainCamera;
    private SpringSpawner[] StringSpawners;
    private ClothSpawner[] ClothSpawners;

    private void Awake()
    {
        TargetPosition = transform.position;
        MainCamera = Camera.main.GetComponent<CameraController>();
    }

    private IEnumerator Start()
    {
        enabled = false;
        for (int i = 0; i < 5; i++) yield return null;
        enabled = true;
        StringSpawners = FindObjectsOfType<SpringSpawner>();
        ClothSpawners = FindObjectsOfType<ClothSpawner>();
    }

    private void LateUpdate()
    {
        if (!IsMoving && Input.GetMouseButtonDown(0))
        {
            Ray ray = MainCamera.GetComponent<Camera>().ScreenPointToRay(Input.mousePosition);
            if (TestSphereLineIntersection(ray.origin, ray.direction.normalized))
            {
                IsMoving = true;
                MainCamera.SetMovement(false);
            }
        }
        else if (IsMoving && Input.GetMouseButtonDown(0))
        {
            Ray ray = MainCamera.GetComponent<Camera>().ScreenPointToRay(Input.mousePosition);
            if (TestSphereLineIntersection(ray.origin, ray.direction.normalized))
            {
                IsMoving = false;
                MainCamera.SetMovement(true);
            }
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
        foreach (ClothSpawner spawner in ClothSpawners)
        {
            for (int y = 0; y < spawner.NumberParticles.y; y++)
            {
                for (int x = 0; x < spawner.NumberParticles.x; x++)
                {
                    int index = x + y * spawner.NumberParticles.x;
                    Vector3 particlePos = spawner.GetParticlePosition(index);
                    if (Vector3.Dot(particlePos - transform.position, particlePos - transform.position) < (radius + spawner.ParticleRadius * 1.25f) * (radius + spawner.ParticleRadius * 1.25f))
                    {
                        spawner.SetParticlePosition(index, particlePos + offset);
                        spawner.SetParticlePreviousPosition(index, spawner.GetParticlePreviousPosition(index) + offset);
                    }
                }
            }
        }
    }

    private void Move()
    {
        float horizontal = Input.GetAxis("Horizontal");
        float vertical = Input.GetAxis("Vertical");
        float up = Input.GetKey(KeyCode.E) ? 1.0f : (Input.GetKey(KeyCode.Q) ? -1.0f : 0.0f);
        Vector3 movement = new Vector3(horizontal, up, vertical);
        TargetPosition += movement * Time.deltaTime * Speed * 0.1f;
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

using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Wind : MonoBehaviour
{
    public Vector3 MaxWindForce;
    public float OscilationSpeed = 1.0f;
    public Vector3 WindForce { get; private set; }

    private void Update()
    {
        WindForce = Mathf.Abs(Mathf.Sin(Time.time * OscilationSpeed)) * MaxWindForce;
    }
}

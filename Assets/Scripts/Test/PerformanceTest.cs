using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Diagnostics;

using Debug = UnityEngine.Debug;

public class PerformanceTest : MonoBehaviour
{
    public int TestCount = 10000;

    private void Start()
    {
        DoTest();
    }

    private void DoTest()
    {
        Vector3[] a = new Vector3[TestCount];
        Vector3[] b = new Vector3[TestCount];
        float[] c = new float[TestCount];

        // Init vectors
        for (int i = 0; i < TestCount; i++)
        {
            a[i] = new Vector3(Random.Range(-100, 100), Random.Range(-100, 100), Random.Range(-100, 100));
            b[i] = new Vector3(Random.Range(-100, 100), Random.Range(-100, 100), Random.Range(-100, 100));
        }

        Stopwatch stopwatch = new Stopwatch();
        stopwatch.Start();

        for (int i = 0; i < TestCount; i++)
        {
            c[i] = Vector3.Dot(a[i], b[i]);
        }

        stopwatch.Stop();
        Debug.Log("Time: " + stopwatch.ElapsedMilliseconds);
    }

}

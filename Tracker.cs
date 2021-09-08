using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Tracker : MonoBehaviour
{
    static int timeStep;
    // Start is called before the first frame update
    void Start()
    {
        timeStep = 0;
    }

    // Update is called once per frame
    void Update()
    {
        timeStep++;
    }
}

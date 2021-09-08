using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PSOswarm : MonoBehaviour
{
    static System.Random ran = null;
    public GameObject pedestrian;
    public GameObject target;
    GameObject _target;

    float ObjectiveFunction(Vector3 x)
    {
        return Vector3.Distance(x, _target.transform.position);
    }

    public int numberParticles = 1000;
    public int numberIterations = 1000;
    int iteration;
    float minX;
    float maxX;
    GameObject[] swarm;
    Vector3 bestGlobalPosition;
    float bestGlobalFitness;
    float minV;
    float maxV;

    public int v_factor = 2;

    public double w = 0.729; // inertia weight
    public double c1 = 1.49445; // cognitive weight
    public double c2 = 1.49445; // social weight
    public double r1, r2; // randomizations

    float time;

    Vector3 goal;

    bool loop = true;

    float randomInRange(float _min, float _max)
    {
        return (float)((_max - _min) * ran.NextDouble() + _min);
    }

    // Start is called before the first frame update
    void Start()
    {
        time = 0.0f;
        Debug.Log("\nBegin PSO demo\n");
        ran = new System.Random(0);

        iteration = 0;
        minX = -100.0f;
        maxX = 100.0f;

        goal = new Vector3(randomInRange(minX, maxX), 0, randomInRange(minX, maxX));
        _target = Instantiate(target, goal, target.transform.rotation);

        swarm = new GameObject[numberParticles];
        bestGlobalFitness = float.MaxValue;

        minV = minX / v_factor;
        maxV = maxX / v_factor;

        // Initialize all Particle objects

        for (int i = 0; i < swarm.Length; ++i)
        {
            float x = randomInRange(minX, maxX);
            float y = randomInRange(minX, maxX);
            Vector3 pos = new Vector3(x, 1, y);

            double fitness = ObjectiveFunction(pos);
            //double[] randomVelocity = new double[Dim];

            float lo = (float)(-1.0 * Math.Abs(maxX - minX));
            float hi = Math.Abs(maxX - minX);
            float vx = randomInRange(lo, hi);
            float vz = randomInRange(lo, hi);

            //Random position
            GameObject NGObject = Instantiate(pedestrian, pos, pedestrian.transform.rotation);
            //Random velocity
            NGObject.GetComponent<Rigidbody>().velocity = new Vector3(vx, 0, vz);
            NGObject.GetComponent<PSOagent>().fitness = ObjectiveFunction(NGObject.transform.position);
            swarm[i] = NGObject;
            var comp = swarm[i].GetComponent<PSOagent>().fitness;
            
            if (comp < bestGlobalFitness)
            {
                bestGlobalFitness = comp;
                bestGlobalPosition = swarm[i].transform.position;
            }
        }
    } // End Start()

    // bool reset = false;
    // Update is called once per frame
    void FixedUpdate()
    {
        if (iteration < numberIterations)
        {
            update_swarm();
            iteration++;
        }
        else
        {
            Debug.Log("Moving target to new location");
            goal = new Vector3(randomInRange(minX, maxX), 0, randomInRange(minX, maxX));
            GameObject.Destroy(_target);
            _target = Instantiate(target, goal, target.transform.rotation);
            bestGlobalFitness = float.MaxValue;
            for (int i = 0; i < swarm.Length; ++i)
            {
                //if(reset)
                    //swarm[i].transform.position = new Vector3(randomInRange(minX, maxX), 1, randomInRange(minX, maxX));

                swarm[i].GetComponent<PSOagent>().fitness = ObjectiveFunction(swarm[i].transform.position);
                swarm[i].GetComponent<PSOagent>().bestFitness = bestGlobalFitness;

                var comp = swarm[i].GetComponent<PSOagent>().fitness;
                if (comp < bestGlobalFitness)
                {
                    bestGlobalFitness = comp;
                    bestGlobalPosition = swarm[i].transform.position;
                }
            }
            iteration = 0;
        }
    }

    void update_swarm()
    {
        for (int i = 0; i < swarm.Length; i++)
        {
            Rigidbody currPR = swarm[i].GetComponent<Rigidbody>();

            r1 = ran.NextDouble();
            r2 = ran.NextDouble();

            // calculate velocities
            double _vx = (w * currPR.velocity.x) +
                (c1 * r1 * (swarm[i].GetComponent<PSOagent>().bestPosition.x - swarm[i].transform.position.x)) +
                (c2 * r2 * (bestGlobalPosition.x - swarm[i].transform.position.x));

            r1 = ran.NextDouble();
            r2 = ran.NextDouble();

            double _vz = (w * currPR.velocity.z) +
                (c1 * r1 * (swarm[i].GetComponent<PSOagent>().bestPosition.z - swarm[i].transform.position.z)) +
                (c2 * r2 * (bestGlobalPosition.z - swarm[i].transform.position.z));

            // clamp min velocities
            if (_vx < minV)
                _vx = minV;
            else if (_vx > maxV)
                _vx = maxV;

            // clamp max velocities
            if (_vz < minV)
                _vz = minV;
            else if (_vz > maxV)
                _vz = maxV;

            // set new velocity
            Vector3 newV = new Vector3((float)_vx, 0, (float)_vz);
            swarm[i].GetComponent<Rigidbody>().velocity = newV;

            //update particle position
            swarm[i].transform.position = swarm[i].transform.position + Time.deltaTime * newV;

            // clamp x and z min positions
            if (swarm[i].transform.position.x < minX)
                swarm[i].transform.position = new Vector3(minX, 0, swarm[i].transform.position.z);
            if (swarm[i].transform.position.z < minX)
                swarm[i].transform.position = new Vector3(swarm[i].transform.position.x, 0, minX);
            // clamp x and z max positions
            if (swarm[i].transform.position.x > maxX)
                swarm[i].transform.position = new Vector3(maxX, 0, swarm[i].transform.position.z);
            if (swarm[i].transform.position.z > maxX)
                swarm[i].transform.position = new Vector3(swarm[i].transform.position.x, 0, maxX);
            
            // calculate fitness to goal
            float newFitness = (float)ObjectiveFunction(swarm[i].transform.position);
            swarm[i].GetComponent<PSOagent>().fitness = newFitness;
            // update local fitness
            if (newFitness < swarm[i].GetComponent<PSOagent>().bestFitness)
            {
                swarm[i].GetComponent<PSOagent>().bestPosition = swarm[i].transform.position;
                swarm[i].GetComponent<PSOagent>().bestFitness = newFitness;
            }
            // update global fitness
            if (newFitness < bestGlobalFitness)
            {
                bestGlobalPosition = swarm[i].transform.position;
                bestGlobalFitness = newFitness;
            }
        }
    }
}

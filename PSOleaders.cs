using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PSOleaders : MonoBehaviour
{
    static System.Random ran = null;
    public GameObject pedestrian;
    public GameObject leader;
    public GameObject target;
    GameObject _target;

    public int num_goals = 1;
    public int num_leaders = 1;
    GameObject[] goals;
    GameObject[] leaders;
    Vector3[] vswarm;
    Vector3[] vleader;
    Color[] colors;

    public int numberParticles = 1000;
    public int numberIterations = 1000;
    int iteration;
    float minX;
    float maxX;
    GameObject[] swarm;
    List<float> bestGFit;
    List<Vector3> bestGPos;
    float bestGlobalFitness;
    Vector3 bestGlobalPosition; 
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

    // pedestrians use leaders as the goal
    float LeaderFunction(Vector3 x, Vector3 l)
    {
        return Vector3.Distance(x, l);
    }

    // leaders use targets as goal
    float ObjectiveFunction(Vector3 x)
    {
        float min_dist = float.MaxValue;
        GameObject g = null;
        foreach (GameObject _g in goals)
        {
            float d = Vector3.Distance(x, _g.transform.position);
            if (d < min_dist)
            {
                min_dist = d;
                g = _g;
            }
        }
        return min_dist;
    }

     
    void makeColors()
    {
        if (colors == null)
        {
            colors = new Color[num_goals];
            for (int i = 0; i < num_goals; i++)
            {
                colors[i] = UnityEngine.Random.ColorHSV(0f, 1f, 1f, 1f, 0.5f, 1f);
            }
        }
        for (int i = 0; i < num_goals; i++)
        {
            goals[i].GetComponent<Renderer>().material.color = colors[i];
        }
    }

    void makeGoals()
    {
        goals = new GameObject[num_goals];
        for (int i = 0; i < num_goals; i++)
        {
            float x = randomInRange(minX, maxX);
            float z = randomInRange(minX, maxX);
            goal = new Vector3(x, 0, z);
            goals[i] = Instantiate(target, goal, target.transform.rotation);
        }
        makeColors();
    }

    void destroyGoals()
    {
        foreach (GameObject g in goals)
        {
            GameObject.Destroy(g);
        }
    }

    void makeLeaders()
    {
        leaders = new GameObject[num_leaders];
        bestGFit = new List<float>();
        bestGPos = new List<Vector3>();
        for (int i = 0; i < num_leaders; i++)
        {
            float x = randomInRange(minX, maxX);
            float z = randomInRange(minX, maxX);
            goal = new Vector3(x, 0, z);
            leaders[i] = Instantiate(leader, goal, target.transform.rotation);
            float lo = (float)(-1.0 * Math.Abs(maxX - minX));
            float hi = Math.Abs(maxX - minX);
            float vx = randomInRange(lo, hi);
            float vz = randomInRange(lo, hi);
            leaders[i].GetComponent<Rigidbody>().velocity = new Vector3(vx, 0, vz);
            float newFitness = ObjectiveFunction(leaders[i].transform.position);
            if (newFitness < bestGlobalFitness)
            {
                bestGlobalPosition = leaders[i].transform.position;
                bestGlobalFitness = newFitness;
            }
            // Best fitness and positions for each leader,
            // these are directly affected by particle rather than the leaders
            bestGFit.Add(float.MaxValue);
            bestGPos.Add(new Vector3());
        }
    }

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

        //goal = new Vector3(randomInRange(minX, maxX), 0, randomInRange(minX, maxX));
        //_target = Instantiate(target, goal, target.transform.rotation);
        bestGlobalFitness = float.MaxValue;
        makeGoals();
        makeLeaders();

        swarm = new GameObject[numberParticles];

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
            float newFitness = ObjectiveFunction(NGObject.transform.position);
            //Random velocity
            NGObject.GetComponent<Rigidbody>().velocity = new Vector3(vx, 0, vz);

            NGObject.GetComponent<PSOagent>().fitness = newFitness;
            swarm[i] = NGObject;

            //updateSwarmFitness(pos, newFitness);
        }
        vswarm = new Vector3[swarm.Length];
        vleader = new Vector3[leaders.Length];
    } // End Start()

    //bool reset = false;
    // Update is called once per frame

    private void Update()
    {
        if (iteration < numberIterations)
        {
            update_leaders();
            update_swarm();
            iteration++;
        }
        else
        {
            Debug.Log("Moving target to new location");
            /*goal = new Vector3(randomInRange(minX, maxX), 0, randomInRange(minX, maxX));
            GameObject.Destroy(_target);
            _target = Instantiate(target, goal, target.transform.rotation);*/
            bestGlobalFitness = float.MaxValue;
            destroyGoals();
            makeGoals();
            for (int i = 0; i < num_leaders; i++)
            {
                float leaderFit = ObjectiveFunction(leaders[i].transform.position);
                leaders[i].GetComponent<PSOagent>().fitness = leaderFit;
                leaders[i].GetComponent<PSOagent>().bestFitness = float.MaxValue;
                if (leaderFit < leaders[i].GetComponent<PSOagent>().bestFitness)
                {
                    leaders[i].GetComponent<PSOagent>().bestPosition = leaders[i].transform.position;
                    leaders[i].GetComponent<PSOagent>().bestFitness = leaderFit;
                }
                if (leaderFit < bestGlobalFitness)
                {
                    bestGlobalFitness = leaderFit;
                    bestGlobalPosition = leaders[i].transform.position;
                }
            }
            resetSwarmFitness();
            for (int i = 0; i < swarm.Length; i++)
            {
                //if (reset)
                //swarm[i].transform.position = new Vector3(randomInRange(minX, maxX), 1, randomInRange(minX, maxX));
                float newFitness = getSwarmBestFitness(swarm[i].transform.position);
                swarm[i].GetComponent<PSOagent>().fitness = newFitness;
                swarm[i].GetComponent<PSOagent>().bestFitness = float.MaxValue;

                if (newFitness < swarm[i].GetComponent<PSOagent>().bestFitness)
                {
                    swarm[i].GetComponent<PSOagent>().bestPosition = swarm[i].transform.position;
                    swarm[i].GetComponent<PSOagent>().bestFitness = newFitness;
                }
                //updateSwarmFitness(swarm[i].transform.position, newFitness);
            }
            iteration = 0;
        }
    }

    void FixedUpdate()
    {
        for (int i = 0; i < swarm.Length; i++)
        {
            swarm[i].transform.position = swarm[i].transform.position + Time.deltaTime * vswarm[i];
        }
        for (int i = 0; i < num_leaders; i++)
        {
            leaders[i].transform.position = leaders[i].transform.position + Time.deltaTime * vleader[i];
        }
    }

    void update_leaders()
    {
        for (int i = 0; i < leaders.Length; i++)
        {
            Rigidbody currPR = leaders[i].GetComponent<Rigidbody>();

            r1 = ran.NextDouble();
            r2 = ran.NextDouble();

            // calculate velocities
            double _vx = (w * currPR.velocity.x) +
                (c1 * r1 * (leaders[i].GetComponent<PSOagent>().bestPosition.x - leaders[i].transform.position.x)) +
                (c2 * r2 * (bestGlobalPosition.x - leaders[i].transform.position.x));

            r1 = ran.NextDouble();
            r2 = ran.NextDouble();

            double _vz = (w * currPR.velocity.z) +
                (c1 * r1 * (leaders[i].GetComponent<PSOagent>().bestPosition.z - leaders[i].transform.position.z)) +
                (c2 * r2 * (bestGlobalPosition.z - leaders[i].transform.position.z));

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

            //set new velocity
            Vector3 newV = new Vector3((float)_vx, 0, (float)_vz);
            leaders[i].GetComponent<Rigidbody>().velocity = newV;
            vleader[i] = newV;

            // update particle position -> removing as Unity should be able to handle the physics
            // leaders[i].transform.position = leaders[i].transform.position + Time.deltaTime * newV;
            
            // clamp x and z min positions
            if (leaders[i].transform.position.x < minX)
                leaders[i].transform.position = new Vector3(minX, 0, leaders[i].transform.position.z);
            if (leaders[i].transform.position.z < minX)
                leaders[i].transform.position = new Vector3(leaders[i].transform.position.x, 0, minX);
            // clamp x and z max positions
            if (leaders[i].transform.position.x > maxX)
                leaders[i].transform.position = new Vector3(maxX, 0, leaders[i].transform.position.z);
            if (leaders[i].transform.position.z > maxX)
                leaders[i].transform.position = new Vector3(leaders[i].transform.position.x, 0, maxX);

            // calculate best fitness between goals, picks the minimal value between all goals
            float newFitness = (float)ObjectiveFunction(leaders[i].transform.position);
            leaders[i].GetComponent<PSOagent>().fitness = newFitness;
            // update local fitness
            if (newFitness < leaders[i].GetComponent<PSOagent>().bestFitness)
            {
                leaders[i].GetComponent<PSOagent>().bestPosition = leaders[i].transform.position;
                leaders[i].GetComponent<PSOagent>().bestFitness = newFitness;
            }
            if (newFitness < bestGlobalFitness)
            {
                bestGlobalPosition = leaders[i].transform.position;
                bestGlobalFitness = newFitness;
            }
            //update leader info for non-leaders
            bestGPos[i] = leaders[i].transform.position;
            bestGFit[i] = newFitness;
        }
    }

    void update_swarm()
    {
        for (int i = 0; i < swarm.Length; ++i)
        {
            Rigidbody currPR = swarm[i].GetComponent<Rigidbody>();

            //determine leader particle to use
            Vector3 lead = getSwarmBestPosition(swarm[i].transform.position);

            r1 = ran.NextDouble();
            r2 = ran.NextDouble();

            double _vx = (w * currPR.velocity.x) +
                (c1 * r1 * (swarm[i].GetComponent<PSOagent>().bestPosition.x - swarm[i].transform.position.x)) +
                (c2 * r2 * (lead.x - swarm[i].transform.position.x));

            r1 = ran.NextDouble();
            r2 = ran.NextDouble();

            double _vz = (w * currPR.velocity.z) +
                (c1 * r1 * (swarm[i].GetComponent<PSOagent>().bestPosition.z - swarm[i].transform.position.z)) +
                (c2 * r2 * (lead.z - swarm[i].transform.position.z));

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

            Vector3 newV = new Vector3((float)_vx, 0, (float)_vz);
            swarm[i].GetComponent<Rigidbody>().velocity = newV;
            vswarm[i] = newV;

            // update particle position -> removing as Unity should be able to handle the physics
            // swarm[i].transform.position = swarm[i].transform.position + Time.deltaTime * newV;

            // clamp x and z min positions
            swarm[i].transform.position = swarm[i].transform.position + Time.deltaTime * newV;
            if (swarm[i].transform.position.x < minX)
                swarm[i].transform.position = new Vector3(minX, 0, swarm[i].transform.position.z);
            if (swarm[i].transform.position.z < minX)
                swarm[i].transform.position = new Vector3(swarm[i].transform.position.x, 0, minX);
            // clamp x and z max positions
            if (swarm[i].transform.position.x > maxX)
                swarm[i].transform.position = new Vector3(maxX, 0, swarm[i].transform.position.z);
            if (swarm[i].transform.position.z > maxX)
                swarm[i].transform.position = new Vector3(swarm[i].transform.position.x, 0, maxX);

            float newFitness = getSwarmBestFitness(swarm[i].transform.position);
            swarm[i].GetComponent<PSOagent>().fitness = newFitness;
            // local parameter update
            if (newFitness < swarm[i].GetComponent<PSOagent>().bestFitness)
            {
                swarm[i].GetComponent<PSOagent>().bestPosition = swarm[i].transform.position;
                swarm[i].GetComponent<PSOagent>().bestFitness = newFitness;
            }
            // global update
            //updateSwarmFitness(swarm[i].transform.position, newFitness);
        }
    }

    void updateSwarmFitness(Vector3 v, float newFitness)
    {
        for(int i = 0; i < num_leaders; i++)
        {
            if(newFitness < bestGFit[i])
            {
                bestGFit[i] = newFitness;
                bestGPos[i] = v;
            }
        }
    }

    Vector3 getSwarmBestPosition(Vector3 v)
    {
        Vector3 pos = new Vector3();
        float best = float.MaxValue;
        for (int i = 0; i < num_leaders; i++)
        {
            // find the best position based on minimized fitness
            // relative to the particle's current position
            float temp = LeaderFunction(v, leaders[i].transform.position);
            if (temp < best)
            {
                best = temp;
                pos = leaders[i].transform.position;
            }
        }
        return pos;
    }

    float getSwarmBestFitness(Vector3 v)
    {
        float best = float.MaxValue;
        for (int i = 0; i < num_leaders; i++)
        {
            // find smallest value amongst leaders
            float temp = LeaderFunction(v, bestGPos[i]);
            if (temp < best)
            {
                best = temp;
            }
        }
        return best;
    }

    void resetSwarmFitness()
    {
        for(int i = 0; i < num_leaders; i++)
        {
            bestGFit[i] = leaders[i].GetComponent<PSOagent>().fitness;
            bestGPos[i] = leaders[i].transform.position;
        }
    }

    /*float LeaderFunction(Vector3 x)
    {
        float min_dist = float.MaxValue;
        GameObject g = null;
        foreach (GameObject _l in leaders)
        {
            float d = Vector3.Distance(x, _l.transform.position);
            if (d < min_dist)
            {
                min_dist = d;
                g = _l;
            }
        }
        //RaycastHit hit;
        Vector3 v = g.transform.position - x;
        *//*if ( Physics.Raycast(x, v, out hit, 50.0f))
        {
            Debug.DrawRay(x, v * hit.distance, Color.red);
        }*//*
        return min_dist;
    }*/
}

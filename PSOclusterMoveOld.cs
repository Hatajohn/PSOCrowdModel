using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PSOclusterMoveOld : MonoBehaviour
{
    static int _id = 1;
    int id;
    //UnityEngine.Random ran = null;
    public GameObject handler;
    public GameObject creator;

    public int num_leaders;
    public GameObject[] goals;
    public GameObject[] leaders;
    public GameObject[] swarm;
    Vector3[] vswarm;
    Vector3[] vleader;

    int numberIterations;
    public int iteration;

    public int reiteration = 0;
    float minX;
    float maxX;
    public float bestGlobalFitness;
    Vector3 bestGlobalPosition;
    float minV;
    float maxV;

    List<List<Vector3>> leaderPos;
    List<List<Vector3>> swarmPos;

    // PSO variables
    public int v_factor = 3;

    public double w = 0.729; // inertia weight
    public double c1 = 1.49445; // cognitive weight
    public double c2 = 1.49445; // social weight
    public double r1, r2; // randomizations
    public float mWeight = 0.7f;

    bool ready = false;
    bool done = false;

    float step = 0.005f;

    // Start is called before the first frame update
    void Start()
    {
        //ran = new UnityEngine.Random();
        id = _id;
        _id++;
        Debug.Log("\nCluster manager " + id + " operational\n");

        creator.GetComponent<PSOcluster>().CheckIn(id);

        bestGlobalFitness = float.MaxValue;

        leaderPos = new List<List<Vector3>>();
        swarmPos = new List<List<Vector3>>();

        Debug.LogWarning("Manager " + id + " is waiting for delay to finish");
        StartCoroutine(wait(5f));
    } // End Start()

    IEnumerator wait(float t)
    {
        yield return new WaitForSeconds(t);
    }

    IEnumerator sim()
    {

        //StartCoroutine(update_swarm());

        //Debug.Log(id + " : " + iteration + " / " + numberIterations);
        if (iteration < numberIterations)
        {
            update_leaders();
            update_swarm();

            updateLeaderV();
            updateSwarmV();

            //Debug.Log(id + " STEP: " + iteration);
            Physics.Simulate(step);

            waitForUpdate(step);
            setPositions();
            iteration++;
        }
        else
        {
            //Debug.Log("Desired iterations reached, reset requested");
            //handler.GetComponent<GoalHandler>().CheckIn(gameObject);
            Debug.Log("Desired iterations reached, manager " + id + " stopping");
            for (int i = 0; i < num_leaders; i++)
            {
                /*if (leaderPos.Count < num_leaders)
                    leaderPos.Add(new List<Vector3>());*/
                leaders[i].GetComponent<PSOagent>().hardStop();
                //leaderPos.Add(leaders[i].GetComponent<PSOagent>().getPositions());

            }
            for (int i = 0; i < swarm.Length; i++)
            {
                /*if (swarmPos.Count < swarm.Length)
                    swarmPos.Add(new List<Vector3>());*/
                swarm[i].GetComponent<PSOagent>().hardStop();
                //swarmPos.Add(swarm[i].GetComponent<PSOagent>().getPositions());

            }
            //bestGlobalFitness = float.MaxValue;
            ready = false;
            done = true;
            Debug.Log(leaderPos.Count + " leader positions");
            Debug.Log(leaderPos[0].Count + " positions");
        }
        yield return null;
    }

    void setPositions()
    {
        for (int i = 0; i < num_leaders; i++)
        {
            leaders[i].GetComponent<PSOagent>().setPosition(leaders[i].transform.position);
        }
        for (int i = 0; i < swarm.Length; i++)
        {
            swarm[i].GetComponent<PSOagent>().setPosition(swarm[i].transform.position);
        }
    }

    IEnumerator getPositions()
    {
        yield return wait(step);
        for (int i = 0; i < num_leaders; i++)
        {
            leaders[i].transform.position = leaderPos[i][reiteration];
        }
        for (int i = 0; i < swarm.Length; i++)
        {
            swarm[i].transform.position = swarmPos[i][reiteration];
        }
        if (reiteration < numberIterations - 1)
            reiteration++;
        else
            done = false;
        yield return null;
    }

    private void Update()
    {
        if (ready)
        {
            StartCoroutine(sim());
        }
        /*else if(done)
        {
            if(reiteration < numberIterations)
            {
                StartCoroutine(getPositions());
            }
        }*/
    }

    /*private void Update()
    {
        if (ready)
        {
            //StartCoroutine(update_swarm());

            //Debug.Log(id + " : " + iteration + " / " + numberIterations);
            if (iteration < numberIterations)
            {
                //StartCoroutine(update_leaders());
                //StartCoroutine(update_swarm());
                //StartCoroutine(update_velocities());
            }
            else
            {
                if (runitonce)
                {
                    //Debug.Log("Desired iterations reached, reset requested");
                    //handler.GetComponent<GoalHandler>().CheckIn(gameObject);
                    Debug.Log("Desired iterations reached, manager " + id + " stopping");
                    for (int i = 0; i < num_leaders; i++)
                    {
                        leaders[i].GetComponent<PSOagent>().hardStop();
                        //leaders[i].GetComponent<Rigidbody>().velocity = new Vector3();
                        //leaders[i].GetComponent<PSOagent>().momentum = new Vector3();
                        ///leaders[i].GetComponent<PSOagent>().bestFitness = float.MaxValue;
                    }
                    for (int i = 0; i < swarm.Length; i++)
                    {
                        swarm[i].GetComponent<PSOagent>().hardStop();
                        //swarm[i].GetComponent<Rigidbody>().velocity = new Vector3();
                        //swarm[i].GetComponent<PSOagent>().momentum = new Vector3();
                        //swarm[i].GetComponent<PSOagent>().bestFitness = float.MaxValue;
                    }
                    //bestGlobalFitness = float.MaxValue;
                    runitonce = false;
                    ready = false;
                }
            }
        }
    }*/


    /*void FixedUpdate()
    {
        if (ready)
        {
            //Debug.Log(id + " : " + iteration + " / " + numberIterations);
            if (iteration < numberIterations)
            {
                StartCoroutine(update_velocities());
            }
            else
            {
                if (runitonce)
                {
                    //Debug.Log("Desired iterations reached, reset requested");
                    //handler.GetComponent<GoalHandler>().CheckIn(gameObject);
                    Debug.Log("Desired iterations reached, manager " + id + " stopping");
                    for (int i = 0; i < num_leaders; i++)
                    {
                        leaders[i].GetComponent<PSOagent>().hardStop();
                        //leaders[i].GetComponent<Rigidbody>().velocity = new Vector3();
                        //leaders[i].GetComponent<PSOagent>().momentum = new Vector3();
                        ///leaders[i].GetComponent<PSOagent>().bestFitness = float.MaxValue;
                    }
                    for (int i = 0; i < swarm.Length; i++)
                    {
                        swarm[i].GetComponent<PSOagent>().hardStop();
                        //swarm[i].GetComponent<Rigidbody>().velocity = new Vector3();
                        //swarm[i].GetComponent<PSOagent>().momentum = new Vector3();
                        //swarm[i].GetComponent<PSOagent>().bestFitness = float.MaxValue;
                    }
                    //bestGlobalFitness = float.MaxValue;
                    runitonce = false;
                    ready = false;
                }
            }
        }
    }*/

    void waitForUpdate(float _t)
    {
        //update_velocities();

        for (int i = 0; i < num_leaders; i++)
        {
            leaders[i].GetComponent<Rigidbody>().velocity = new Vector3();
            //leaders[i].GetComponent<Rigidbody>().AddForce(vleader[i], ForceMode.VelocityChange);
        }
        for (int i = 0; i < swarm.Length; i++)
        {
            swarm[i].GetComponent<Rigidbody>().velocity = new Vector3();
            //swarm[i].GetComponent<Rigidbody>().AddForce(vswarm[i], ForceMode.VelocityChange);
        }
        //Debug.LogWarning("One iteration complete: " + iteration);
    }
    void updateLeaderV()
    {
        for (int i = 0; i < num_leaders; i++)
        {
            //leaders[i].GetComponent<Rigidbody>().velocity = new Vector3();
            leaders[i].GetComponent<Rigidbody>().AddForce(vleader[i], ForceMode.VelocityChange);
        }
    }

    void updateSwarmV()
    {
        for (int i = 0; i < swarm.Length; i++)
        {
            //swarm[i].GetComponent<Rigidbody>().velocity = new Vector3();
            swarm[i].GetComponent<Rigidbody>().AddForce(vswarm[i], ForceMode.VelocityChange);
        }
    }

    public void setup(GameObject[] _leaders, GameObject[] _swarm, int iter, float min, float max)
    {
        Debug.LogWarning("Setup called for manager #" + id + "; " + iter + ", " + min + ", " + max);
        numberIterations = iter;
        swarm = _swarm;
        leaders = _leaders;
        num_leaders = _leaders.Length;

        vswarm = new Vector3[_swarm.Length];
        vleader = new Vector3[_leaders.Length];

        goals = GameObject.FindGameObjectsWithTag("Exit");
        handler = GameObject.FindGameObjectWithTag("Handler");
        //Debug.Log("Number of exits: " + goals.Length);

        // Initialize parameters
        iteration = 0;
        minX = min;
        maxX = max;
        minV = minX / v_factor;
        maxV = maxX / v_factor;
    }

    public void readyCheck()
    {
        ready = true;
        //sim();
    }

    public void unready()
    {
        ready = false;
    }

    // leaders use targets as goal
    float ObjectiveFunction(Vector3 x)
    {
        float min_dist = float.MaxValue;
        GameObject g;
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

    Vector3 makeRandomVector()
    {
        float lo = (float)(-1.0 * Math.Abs(maxX - minX));
        float hi = Math.Abs(maxX - minX);
        float vx = UnityEngine.Random.Range(lo, hi);
        float vz = UnityEngine.Random.Range(lo, hi);

        return new Vector3(vx, 0, vz);
    }

    void resetLeaders()
    {
        for (int i = 0; i < num_leaders; i++)
        {
            leaders[i].GetComponent<Rigidbody>().velocity = new Vector3();
            leaders[i].GetComponent<Rigidbody>().AddForce(makeRandomVector(), ForceMode.VelocityChange);

            float leaderFit = ObjectiveFunction(leaders[i].transform.position);
            leaders[i].GetComponent<PSOagent>().fitness = leaderFit;
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
    }

    void resetSwarm()
    {
        for (int i = 0; i < swarm.Length; i++)
        {
            swarm[i].GetComponent<Rigidbody>().velocity = new Vector3();
            swarm[i].GetComponent<Rigidbody>().AddForce(makeRandomVector(), ForceMode.VelocityChange);

            float newFitness = getSwarmFitness(swarm[i].transform.position);
            swarm[i].GetComponent<PSOagent>().fitness = newFitness;

            if (newFitness < swarm[i].GetComponent<PSOagent>().bestFitness)
            {
                swarm[i].GetComponent<PSOagent>().bestPosition = swarm[i].transform.position;
                swarm[i].GetComponent<PSOagent>().bestFitness = newFitness;
            }
        }
    }

    public void resetEnv()
    {
        Debug.Log("Reset invoked!");

        Debug.Log("Target has been moved!");
        bestGlobalFitness = float.MaxValue;

        resetLeaders();
        resetSwarm();

        iteration = 0;
    }

    // agent dir, leader forward V
    bool angleCheck(Vector3 a, Vector3 b)
    {
        return Math.Atan2(b.z - a.z, b.x - a.x) < 90;
    }

    int num_updates = 0;
    void update_leaders()
    {
        for (int i = 0; i < leaders.Length; i++)
        {
            Rigidbody currPR = leaders[i].GetComponent<Rigidbody>();
            Vector3 m = leaders[i].GetComponent<PSOagent>().momentum;
            float mass = currPR.mass;
            Vector3 avg = leaders[i].GetComponent<PSOagent>().mExpAvg;

            m.x = Time.deltaTime * (mWeight * (avg.x) + (1 - mWeight) * currPR.velocity.x);
            m.z = Time.deltaTime * (mWeight * (avg.z) + (1 - mWeight) * currPR.velocity.z);
            leaders[i].GetComponent<PSOagent>().momentum = m;

            r1 = UnityEngine.Random.value;
            r2 = UnityEngine.Random.value;

            // calculate velocities
            double _vx = m.x + (w * currPR.velocity.x) +
                (c1 * r1 * (leaders[i].GetComponent<PSOagent>().bestPosition.x - leaders[i].transform.position.x)) +
                (c2 * r2 * (bestGlobalPosition.x - leaders[i].transform.position.x));

            r1 = UnityEngine.Random.value;
            r2 = UnityEngine.Random.value;

            double _vz = m.z + (w * currPR.velocity.z) +
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
            vleader[i] = new Vector3((float)_vx, 0, (float)_vz);

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
        }
        getLeaderAvgPosition();
    }

    // objective function for swarm
    float getSwarmFitness(Vector3 position)
    {
        float min_dist = float.MaxValue;
        GameObject g = null;
        foreach (GameObject l in leaders)
        {
            float d = Vector3.Distance(l.transform.position, position);
            if (d < min_dist)
            {
                min_dist = d;
                g = l;
            }
        }
        return min_dist;
    }

    void update_swarm()
    {
        for (int i = 0; i < swarm.Length; ++i)
        {
            Rigidbody currPR = swarm[i].GetComponent<Rigidbody>();
            Vector3 m = swarm[i].GetComponent<PSOagent>().momentum;
            float mass = currPR.mass;
            Vector3 avg = swarm[i].GetComponent<PSOagent>().mExpAvg;

            m.x = Time.deltaTime * (mWeight * (avg.x) + (1 - mWeight) * currPR.velocity.x);
            m.z = Time.deltaTime * (mWeight * (avg.z) + (1 - mWeight) * currPR.velocity.z);
            // store previous momentum to create the weighted average
            swarm[i].GetComponent<PSOagent>().momentum = m;
            //determine leader particle to use -> all leaders must add some velocity component when relevant as seen below
            //Vector3 lead = getSwarmAvgPosition(swarm[i].transform.position);

            r1 = UnityEngine.Random.value;
            r2 = UnityEngine.Random.value;

            Vector3 cleader = closestLeader(swarm[i].transform.position);

            double _vx = m.x + (w * currPR.velocity.x) +
                (c1 * r1 * (swarm[i].GetComponent<PSOagent>().bestPosition.x - swarm[i].transform.position.x)) +
                (c2 * r2 * (cleader.x - swarm[i].transform.position.x));

            r1 = UnityEngine.Random.value;
            r2 = UnityEngine.Random.value;

            double _vz = m.z + (w * currPR.velocity.z) +
                (c1 * r1 * (swarm[i].GetComponent<PSOagent>().bestPosition.z - swarm[i].transform.position.z)) +
                (c2 * r2 * (cleader.z - swarm[i].transform.position.z));

            // Move aside- adds velocity to move agents away from the direction a leader is moving
            /*foreach (GameObject l in leaders)
            {
                // find vector from leader to 
                Vector3 v = new Vector3();
                float _d = float.MaxValue;
                foreach (GameObject g in goals)
                {
                    float dist = Vector3.Distance(g.transform.position, l.transform.position);
                    if (dist < _d)
                    {
                        _d = dist;
                        v = g.transform.position - l.transform.position;
                    }
                }
                if (angleCheck(swarm[i].transform.position - l.transform.position, v))
                {
                    Vector3 side = Vector3.Cross(v, l.transform.up);
                    _vx += side.x / (Vector3.Distance(l.transform.position, swarm[i].transform.position) * num_leaders);
                    _vz += side.z / (Vector3.Distance(l.transform.position, swarm[i].transform.position) * num_leaders);
                }
            }*/

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

            vswarm[i] = new Vector3((float)_vx, 0, (float)_vz);

            // update particle position -> removing as Unity should be able to handle the physics
            // swarm[i].transform.position = swarm[i].transform.position + Time.deltaTime * newV;

            // clamp x and z min positions -> walls also prevent this
            if (swarm[i].transform.position.x < minX)
                swarm[i].transform.position = new Vector3(minX, 0, swarm[i].transform.position.z);
            if (swarm[i].transform.position.z < minX)
                swarm[i].transform.position = new Vector3(swarm[i].transform.position.x, 0, minX);
            // clamp x and z max positions
            if (swarm[i].transform.position.x > maxX)
                swarm[i].transform.position = new Vector3(maxX, 0, swarm[i].transform.position.z);
            if (swarm[i].transform.position.z > maxX)
                swarm[i].transform.position = new Vector3(swarm[i].transform.position.x, 0, maxX);

            float newFitness = getSwarmFitness(swarm[i].transform.position);
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

    Vector3 avgLeader;
    void getLeaderAvgPosition()
    {
        Vector3 pos = new Vector3();
        for (int i = 0; i < num_leaders; i++)
        {
            pos += leaders[i].transform.position;
        }
        pos /= num_leaders;
        avgLeader = pos;
    }

    Vector3 closestLeader(Vector3 position)
    {
        float min_dist = float.MaxValue;
        GameObject g = null;
        foreach (GameObject l in leaders)
        {
            float d = Vector3.Distance(l.transform.position, position);
            if (d < min_dist)
            {
                min_dist = d;
                g = l;
            }
        }
        return g.transform.position;
    }
}

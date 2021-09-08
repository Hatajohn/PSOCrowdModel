using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;

public class PSOcluster_move : MonoBehaviour
{
    static int _id = 1;
    int id;
    //UnityEngine.Random ran = null;
    public GameObject handler;
    public GameObject creator;

    public GameObject[] goals;
    public List<GameObject> leaders;
    public List<GameObject> swarm;
    bool[] leaderPath;
    bool[] swarmPath;
    Vector3[] vswarm;
    Vector3[] vleader;

    List<GameObject> leaderRemove;
    List<GameObject> swarmRemove;

    public Vector3 avgClusterPosition;

    int numberIterations;
    public int iteration;

    public int reiteration = 0;

    //Size of area
    public float minX;
    public float maxX;

    public float bestGlobalFitness;

    public float avgSwarmFitness;
    public float avgLeaderFitness;

    float pAvgSwarmFitness;
    float pAvgLeaderFitness;

    Vector3 bestGlobalPosition;
    float minV;
    float maxV;

    List<List<Vector3>> leaderPos;
    List<List<Vector3>> swarmPos;

    // PSO variables
    public int v_factor = 3;

    public double w = 0.729; // inertia weight
    // 1.49445 - previous
    public double c1 = .7; // cognitive weight - local
    public double c2 = 1.49445; // social weight - global
    public double r1, r2; // randomizations
    public float mWeight = 0.7f;

    bool ready = false;
    bool done = false;

    bool leaderControl = true;
    bool swarmControl = true;
    bool sent = false;

    float step = 0.1f;

    // Start is called before the first frame update
    void Start()
    {
        //ran = new UnityEngine.Random();
        id = _id;
        _id++;
        Debug.Log("\nCluster manager " + id + " operational\n");
        //Debug.LogError("RECORDER: " + recorder != null);

        creator.GetComponent<PSOcluster>().CheckIn(id);

        bestGlobalFitness = float.MaxValue;

        leaderPos = new List<List<Vector3>>();
        swarmPos = new List<List<Vector3>>();

       // Debug.LogWarning("Manager " + id + " is waiting for delay to finish");
        StartCoroutine(wait(5f));
    } // End Start()

    IEnumerator wait(float t)
    {
        yield return new WaitForSeconds(t);
    }

    Vector3 ClosestGoal(Vector3 x)
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
        return g.transform.position;
    }

    Vector3 ClosestFollow(Vector3 x)
    {
        float goal_dist = float.MaxValue;
        float lead_dist = float.MaxValue;
        GameObject g1 = null;
        GameObject g2 = null;
        // follow closest leader
        foreach(GameObject _g in leaders)
        {
            float d = Vector3.Distance(x, _g.transform.position);
            if (d < lead_dist)
            {
                lead_dist = d;
                g1 = _g;
            }
        }
        foreach (GameObject _g in goals)
        {
            float d = Vector3.Distance(x, _g.transform.position);
            if (d < goal_dist)
            {
                goal_dist = d;
                g2 = _g;
            }
        }
        RaycastHit[] hits;
        hits = Physics.RaycastAll(x, g2.transform.position - x, Vector3.Distance(g2.transform.position, x));
        //Debug.DrawRay(x, g2.transform.position - x);
        int exit = 0;
        int wall = 0;
        for(int i = 0; i < hits.Length; i++)
        {
            if (hits[i].collider.gameObject.tag == "Wall")
                wall = i;
            if (hits[i].collider.gameObject.tag == "Exit")
                exit = i;
        }
        Vector3 p = prevPosition(g1);
        Vector3 _p = new Vector3(0, 0, 0);
        if(g1 == null && g2 == null)
            Debug.LogError("Cannot find something to follow");
        if ((lead_dist < goal_dist && !(wall < exit)) && g1 != null && p != _p)
            return prevPosition(g1);
        return g2.transform.position;
    }

    Vector3 prevPosition(GameObject g)
    {
        if (g != null)
        {
            Vector3 pos = g.transform.position;
            Vector3 v = g.GetComponent<Rigidbody>().velocity.normalized;
            return pos - v;
        }
        return new Vector3();
    }

    void adjustPos(GameObject g)
    {
        if (g.transform.position.y < 1 || g.transform.position.y > 1) {
            Vector3 fix = g.transform.position;
            g.transform.position = new Vector3(fix.x, 1, fix.z); 
        }
    }

    float baseSpeed = 100f;
    void pathingLeader(GameObject g, int num)
    {
        adjustPos(g);
        if (!leaderPath[num])
        {
            leaderPath[num] = true;
            NavMeshPath path = new NavMeshPath();
            Vector3[] corners;
            //float rad = 1.5f;
            //float dist;
            Vector3 goal = ClosestGoal(g.transform.position);
            NavMesh.CalculatePath(g.transform.position, goal, NavMesh.AllAreas, path); // Calculate path for agent using A*
            corners = path.corners;
            int pathLength = corners.Length;
            if (pathLength > 0)
            {
                for (int i = 0; i < corners.Length - 1; i++)
                {
                    Debug.DrawLine(corners[i], corners[i + 1], g.GetComponent<Renderer>().material.color);
                }
                float _dist = Vector3.Distance(g.transform.position, goal);
                if (_dist > 3.0f)
                {
                    //Debug.Log("Heading to goal still");
                    //dist = Vector3.Distance(corners[1], g.transform.position);
                    Vector3 dir = (new Vector3(corners[1].x, 1, corners[1].z) - g.transform.position).normalized;
                    vleader[num] = dir * g.GetComponent<PSOagent>().speed;
                    g.GetComponent<PSOagent>().currV = dir * g.GetComponent<PSOagent>().speed;
                    /*if (dist < 3 * rad)
                    {
                        //Debug.Log("Pushing pash");

                        vleader[num] = g.GetComponent<PSOagent>().prev * g.GetComponent<PSOagent>().speed;
                    }
                    else
                    {
                        Vector3 dir = (corners[1] - g.transform.position).normalized;
                        vleader[num] = dir * g.GetComponent<PSOagent>().speed;
                        g.GetComponent<PSOagent>().prev = dir;
                    }*/
                }
                else
                {
                    leaderRemove.Add(g);
                }
            }
            leaderPath[num] = false;
        }
        //yield return null;
    }

    void pathingSwarm(GameObject g, int num)
    {
        try {
            adjustPos(g);
            if (!swarmPath[num])
            {
                swarmPath[num] = true;
                NavMeshPath path = new NavMeshPath();
                Vector3[] corners;
                //float rad = 1.5f;
                //float dist;
                Vector3 goal = ClosestGoal(g.transform.position);
                //Vector3 target = ClosestFollow(g.transform.position);

                //Debug.Log("Calculating path for agent " + num);
                NavMesh.CalculatePath(g.transform.position, goal, NavMesh.AllAreas, path); // Calculate path for agent using A*
                //g.GetComponent<PSOagent>().setPath(path);
                corners = path.corners;
                int pathLength = corners.Length;
                if (pathLength > 0)
                {
                    //Debug.Log("Drawing path for agent " + num);
                    for (int i = 0; i < corners.Length - 1; i++)
                    {
                        Debug.DrawLine(corners[i], corners[i + 1], g.GetComponent<Renderer>().material.color);
                    }
                    float _dist = Vector3.Distance(g.transform.position, goal);
                    if (_dist > 3.0f)
                    {
                        Vector3 dir = (new Vector3(corners[1].x, 1, corners[1].z) - g.transform.position).normalized;
                        vswarm[num] = dir * g.GetComponent<PSOagent>().speed;
                        g.GetComponent<PSOagent>().currV = dir * g.GetComponent<PSOagent>().speed;
                        //Debug.Log("Heading to goal still");
                        /*dist = Vector3.Distance(corners[1], g.transform.position);
                        if (dist < 3 * rad)
                        {
                            //Debug.Log("Pushing pash");

                            vswarm[num] = g.GetComponent<PSOagent>().prev * g.GetComponent<PSOagent>().speed;
                        }
                        else
                        {
                            Vector3 dir = (corners[1] - g.transform.position).normalized;
                            vswarm[num] = dir * g.GetComponent<PSOagent>().speed;
                            g.GetComponent<PSOagent>().prev = dir;
                        }*/
                    }
                    else
                    {
                        Debug.Log("Goal reached");
                        swarmRemove.Add(g);
                    }
                    //Debug.Log("Agent " + num + " pathed");
                }
                else
                {
                    Debug.Log("PathLength is 0");
                }
                swarmPath[num] = false;
            }
        }catch
        {
            Debug.LogWarning("Skipping due to missing component");
        }
        //yield return null;
    }

    void updateFitness()
    {
        float sum = 0;
        foreach (GameObject g in swarm)
            sum += g.GetComponent<PSOagent>().fitness;
        avgSwarmFitness = sum / swarm.Count;
        sum = 0;
        foreach (GameObject g in leaders)
            sum += g.GetComponent<PSOagent>().fitness;
        avgLeaderFitness = sum / leaders.Count;
    }

    bool starMode = false;
    float delta_time = 0.0f;
    IEnumerator sim()
    {

        //StartCoroutine(update_swarm());

        //Debug.Log(id + " : " + iteration + " / " + numberIterations);
        //                              && bestItsGonnaGet()
        bestItsGonnaGet();
        if (itsOver)
        {
            update_leaders();
            update_swarm();

            updateFitness();

            updateLeaderV();
            updateSwarmV();

            //Debug.Log(id + " STEP: " + iteration);
            Physics.Simulate(step);

            waitForUpdate();
            setPositions();
            iteration++;
        }
        else
        {
            if (!sent)
            {
                //Debug.LogWarning("Passing control to A*");
                starMode = true;
                sent = true;
                step = 0.1f;
            }
            leaderRemove = new List<GameObject>();
            swarmRemove = new List<GameObject>();
            for (int i = 0; i < leaders.Count; i++)
            {
                //StartCoroutine(pathingLeader(leaders[i], i));
                pathingLeader(leaders[i], i);
                //Debug.Log("Leader " + i + " pathed");
            }
            for (int i = 0; i < swarm.Count; i++)
            {
                //StartCoroutine(pathingSwarm(swarm[i], i));
                pathingSwarm(swarm[i], i);
            }
            updateLeaderV();
            updateSwarmV();

            Physics.Simulate(step);
            waitForUpdate();

            updateAvgPosition();
            //Debug.DrawRay(avgClusterPosition, Vector3.up * 50, Color.red, 1.0f);
            removal();
            /*if (howClose > .8f && !asked)
            {
                creator.GetComponent<PSOcluster>().CheckReset(id);
                Debug.Log(id + " is asking for a reset");
                asked = true;
            }*/
        }
        
        yield return null;
    }
    bool asked = false;
    public bool itsOver = true;
    bool bestItsGonnaGet()
    {
        if (itsOver)
        {
            if (iteration < 1000)
            {
                itsOver = true;
            } // If the current fitness is not significantly different from the last recorded
            else if (Math.Abs(pAvgLeaderFitness - avgLeaderFitness) > 0.001 || iteration > 3000 /*&& Math.Abs(pAvgSwarmFitness - avgSwarmFitness) < 0.0001*/)
            {
                itsOver = false;
            }
            else
                itsOver = true;
        }
        return itsOver;
    }

    void setPositions()
    {
        for (int i = 0; i < leaders.Count; i++)
        {
            leaders[i].GetComponent<PSOagent>().setPosition(leaders[i].transform.position);
        }
        for (int i = 0; i < swarm.Count; i++)
        {
            swarm[i].GetComponent<PSOagent>().setPosition(swarm[i].transform.position);
        }
    }

    IEnumerator getPositions()
    {
        yield return wait(step);
        for (int i = 0; i < leaders.Count; i++)
        {
            leaders[i].transform.position = leaderPos[i][reiteration];
        }
        for (int i = 0; i < swarm.Count; i++)
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
        if (delta_time >= 100 * step)
        {
            pAvgLeaderFitness = avgLeaderFitness;
            pAvgSwarmFitness = avgSwarmFitness;
        }
        delta_time += Time.deltaTime;
        /*else if(done)
        {
            if(reiteration < numberIterations)
            {
                StartCoroutine(getPositions());
            }
        }*/
    }

    void waitForUpdate()
    {
        //update_velocities();

        for (int i = 0; i < leaders.Count; i++)
        {
            leaders[i].GetComponent<Rigidbody>().velocity = new Vector3();
            //recorder.GetComponent<PSORecorder>().storeData(leaders[i].GetComponent<PSOagent>().agentId, Time.time, leaders[i].transform.position);
        }
        for (int i = 0; i < swarm.Count; i++)
        {
            swarm[i].GetComponent<Rigidbody>().velocity = new Vector3();
            //recorder.GetComponent<PSORecorder>().storeData(swarm[i].GetComponent<PSOagent>().agentId, Time.time, swarm[i].transform.position);
        }
        //Debug.LogWarning("One iteration complete: " + iteration);
    }
    void updateLeaderV()
    {
        for (int i = 0; i < leaders.Count; i++)
        {
            //leaders[i].GetComponent<Rigidbody>().velocity = new Vector3();
            leaders[i].GetComponent<Rigidbody>().AddForce(vleader[i], ForceMode.VelocityChange);

        }
    }

    void updateSwarmV()
    {
        for (int i = 0; i < swarm.Count; i++)
        {
            //swarm[i].GetComponent<Rigidbody>().velocity = new Vector3();
            swarm[i].GetComponent<Rigidbody>().AddForce(vswarm[i], ForceMode.VelocityChange);
        }
    }

    public void setup(List<GameObject> _leaders, List<GameObject> _swarm, int iter, float min, float max)
    {
        //Debug.LogWarning("Setup called for manager #" + id + "; " + iter + ", " + min + ", " + max);
        numberIterations = iter;
        swarm = _swarm;
        leaders = _leaders;

        vswarm = new Vector3[_swarm.Count];
        vleader = new Vector3[_leaders.Count];

        leaderPath = new bool[_leaders.Count];
        swarmPath = new bool[_swarm.Count];
        for (int i = 0; i < leaderPath.Length; i++)
            leaderPath[i] = false;
        for (int i = 0; i < swarmPath.Length; i++)
            swarmPath[i] = false;

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
    /*float ObjectiveFunction(Vector3 x)
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
        return Vector3.Distance(x, _g.transform.position);
    }*/

      float LeaderDistObjective(Vector3 x)
    {
        float min_dist = float.MaxValue;
        GameObject g;
        foreach (GameObject _g in leaders)
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

    float LeaderAverageObjective(Vector3 x)
    {
        getLeaderAvgPosition();
        return Vector3.Distance(getLeaderAvgPosition(), x);
        /*float min_dist = float.MaxValue;
        GameObject g;
        foreach (GameObject _g in leaders)
        {
            return Vector3.Distance(avgLeader, _g.transform.position);
            if (d < min_dist)
            {
                min_dist = d;
                g = _g;
            }
        }
        return min_dist;
        */
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
        for (int i = 0; i < leaders.Count; i++)
        {
            //leaders[i].GetComponent<Rigidbody>().velocity = new Vector3();
            leaders[i].GetComponent<Rigidbody>().AddForce(makeRandomVector(), ForceMode.VelocityChange);

            float leaderFit = LeaderAverageObjective(leaders[i].transform.position);
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
        for (int i = 0; i < swarm.Count; i++)
        {
            //swarm[i].GetComponent<Rigidbody>().velocity = new Vector3();
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
        for (int i = 0; i < leaders.Count; i++)
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
            //double _vx = m.x + (w * currPR.velocity.x) +
            //    (c1 * r1 * (leaders[i].GetComponent<PSOagent>().bestPosition.x - leaders[i].transform.position.x)) +
            //    (c2 * r2 * (bestGlobalPosition.x - leaders[i].transform.position.x));
            double _vx = (w * currPR.velocity.x) +
                (c1 * r1 * (leaders[i].GetComponent<PSOagent>().bestPosition.x - leaders[i].transform.position.x)) +
                (c2 * r2 * (bestGlobalPosition.x - leaders[i].transform.position.x));

            r1 = UnityEngine.Random.value;
            r2 = UnityEngine.Random.value;

            //double _vz = m.z + (w * currPR.velocity.z) +
            //   (c1 * r1 * (leaders[i].GetComponent<PSOagent>().bestPosition.z - leaders[i].transform.position.z)) +
            //    (c2 * r2 * (bestGlobalPosition.z - leaders[i].transform.position.z));
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
            //vleader[i] = new Vector3((float)_vx, 0, (float)_vz);
            vleader[i] = (new Vector3((float)_vx, 0, (float)_vz)).normalized * leaders[i].GetComponent<PSOagent>().speed;

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
            //float newFitness = (float)ObjectiveFunction(leaders[i].transform.position);

            // New fitness is meant to group the leaders
            float newFitness = (float)LeaderAverageObjective(leaders[i].transform.position);

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
        for (int i = 0; i < swarm.Count; ++i)
        {
            Rigidbody currPR = swarm[i].GetComponent<Rigidbody>();
            Vector3 m = swarm[i].GetComponent<PSOagent>().momentum;
            float mass = currPR.mass;
            Vector3 avg = swarm[i].GetComponent<PSOagent>().mExpAvg;

            //m.x = Time.deltaTime * (mWeight * (avg.x) + (1 - mWeight) * currPR.velocity.x);
            //m.z = Time.deltaTime * (mWeight * (avg.z) + (1 - mWeight) * currPR.velocity.z);

            // store previous momentum to create the weighted average
            swarm[i].GetComponent<PSOagent>().momentum = m;
            //determine leader particle to use -> all leaders must add some velocity component when relevant as seen below
            //Vector3 lead = getSwarmAvgPosition(swarm[i].transform.position);

            r1 = UnityEngine.Random.value;
            r2 = UnityEngine.Random.value;

            Vector3 cleader = closestLeader(swarm[i].transform.position);

            //double _vx = m.x + (w * currPR.velocity.x) +
            //    (c1 * r1 * (swarm[i].GetComponent<PSOagent>().bestPosition.x - swarm[i].transform.position.x)) +
            //    (c2 * r2 * (cleader.x - swarm[i].transform.position.x));

            double _vx = (w * currPR.velocity.x) +
                (c1 * r1 * (swarm[i].GetComponent<PSOagent>().bestPosition.x - swarm[i].transform.position.x)) +
                (c2 * r2 * (cleader.x - swarm[i].transform.position.x));

            r1 = UnityEngine.Random.value;
            r2 = UnityEngine.Random.value;

            //double _vz = m.z + (w * currPR.velocity.z) +
            //    (c1 * r1 * (swarm[i].GetComponent<PSOagent>().bestPosition.z - swarm[i].transform.position.z)) +
            //    (c2 * r2 * (cleader.z - swarm[i].transform.position.z));

            double _vz = (w * currPR.velocity.z) +
                (c1 * r1 * (swarm[i].GetComponent<PSOagent>().bestPosition.z - swarm[i].transform.position.z)) +
                (c2 * r2 * (cleader.z - swarm[i].transform.position.z));

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

            //vswarm[i] = new Vector3((float)_vx, 0, (float)_vz);
            vswarm[i] = (new Vector3((float)_vx, 0, (float)_vz)).normalized * swarm[i].GetComponent<PSOagent>().speed;

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
    
    Vector3 getLeaderAvgPosition()
    {
        Vector3 pos = new Vector3();
        for (int i = 0; i < leaders.Count; i++)
        {
            pos += leaders[i].transform.position;
        }
        pos /= leaders.Count;
            return pos;
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

    void enableFullSendLeaders()
    {
        foreach(GameObject l in leaders)
        {
            l.GetComponent<PSOagent>().sendIt();
        }
        leaderControl = false;
    }

    void enableFullSendSwarm()
    {
        foreach (GameObject s in swarm)
        {
            s.GetComponent<PSOagent>().sendIt();
        }
        swarmControl = false;
    }

    public float howClose = 0.0f;
    void updateAvgPosition()
    {
        howClose = 0.0f;
        foreach (GameObject g in leaders)
        {
            if (Vector3.Distance(ClosestGoal(g.transform.position), g.transform.position) < 10.0f)
                howClose += 1;
        }
        foreach (GameObject g in swarm)
        {
            if (Vector3.Distance(ClosestGoal(g.transform.position), g.transform.position) < 10.0f)
                howClose += 1;
        }
        howClose /= (leaders.Count + swarm.Count);
    }

    void removal()
    {
        foreach(GameObject i in leaderRemove)
        {
            try
            {
                leaders.Remove(i);
                i.GetComponent<CapsuleCollider>().isTrigger = true;
                i.GetComponent<Rigidbody>().velocity = new Vector3();
                i.GetComponent<PSOagent>().evacuated = true;
            }
            catch
            {
                Debug.Log("Skipping leader " + i);
            }
            //GameObject.Destroy(temp);
        }
        foreach (GameObject i in swarmRemove)
        {
            try
            {
                swarm.Remove(i);
                i.GetComponent<CapsuleCollider>().isTrigger = true;
                i.GetComponent<Rigidbody>().velocity = new Vector3();
                i.GetComponent<PSOagent>().evacuated = true;
            }
            catch
            {
                Debug.Log("Skipping follower " + i);
            }
            //GameObject.Destroy(temp);
        }
    }
}

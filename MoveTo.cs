// MoveTo.cs
using UnityEngine;
using UnityEngine.AI;
using System;
using System.Collections.Generic;
using System.Collections;
using Random = UnityEngine.Random;
using System.Linq;

public class MoveTo : MonoBehaviour
{
    float evac = 120.0f;
    NavMeshAgent agent;
    public GameObject[] goals;
    GameObject[] _goals;
    public Transform goal;
    public Transform dest = null;
    GameObject[] gameObjects;
    public bool panic = true;
    public bool debug = false;
    public GameObject knownExfil = null;
    public Vector3 dir;
    float c_time = 0.0f;
    float r_time = 0.0f;
    float e_time = 0.0f;
    float d_time = 0.0f;
    public bool sent = false;
    int hitLimit = 3;
    public int limit = 0;
    Vector3 lastPos;
    public string dp_status = "";
    float maxView = 50.0f;
    float fov = 60.0f;

    ArrayList positions;

    void Start()
    {
        agent = GetComponent<NavMeshAgent>();
        //Debug.Log("Old forward = " + transform.forward);
        float d1 = Random.value;
        float d2 = Random.value;
        //Debug.Log(d1 + ", " + d2);
        /*if(Random.value < 0.5f)
            d1 = -d1;
        if (Random.value < 0.5f)
            d2 = -d2;
        //Debug.Log(d1 + ", " + d2);
        transform.forward = new Vector3(d1, 0, d2);*/
        //Debug.Log("New forward = " + transform.forward);
        _goals = GameObject.FindGameObjectsWithTag("Goal");

        if (!panic) goals = GameObject.FindGameObjectsWithTag("Goal");
        if (goals.Length > 0)
        {
            Debug.Log("Exit found in start");
            var random = new System.Random();
            knownExfil = goals[random.Next(goals.Length)].GetComponent<ExitToGoal>().getExit();
            goal = knownExfil.transform;
            Debug.Log(knownExfil.name);
            dest = goal;
        }

        gameObjects = GameObject.FindGameObjectsWithTag("pedestrian");
        positions = new ArrayList();

        //Agents of lower priority are ignored, values themselves are arbitrary
        if (dp_status.Equals("NDP"))
        {
            agent.avoidancePriority = 99;
        }
        else
        {
            agent.avoidancePriority = 0;
        }

        //Initial position info
        positions.Add(dp_status);
        ArrayList temp = new ArrayList();
        temp.Add(transform.position.x);
        temp.Add(transform.position.z);
        temp.Add(agent.velocity);
        temp.Add(c_time);
        positions.Add(temp);
        r_time = 0.0f;
        lastPos = transform.position;
    }

    // Update is called once per frame
    void Update()
    {
        //Save agent position/velocity to arraylist, track all timers
        c_time += Time.deltaTime;
        r_time += Time.deltaTime;
        e_time += Time.deltaTime;
        d_time += Time.deltaTime;
        evac -= Time.deltaTime;
        if (r_time > 0.1f)
        {
            ArrayList temp = new ArrayList();
            temp.Add(transform.position.x);
            temp.Add(transform.position.z);
            temp.Add(agent.velocity);
            temp.Add(c_time);
            positions.Add(temp);
            r_time = 0.0f;
        }

        dir = this.GetComponent<NavMeshAgent>().desiredVelocity;
        //gameObjects = GameObject.FindGameObjectsWithTag("pedestrian");
        Debug.DrawRay(this.transform.position, this.transform.forward, Color.green, 0.01f);
        //elapsed += Time.deltaTime;
        //if (elapsed > 1.0f)
        //{

        //Pedestrian is in a state of panic and will attempt to reach the nearest exit on sight

        //exfil will be the GameObject of an exit
        //lead is another agent the pedestrian will attempt to follow
        // Initial path calculation when entering panic
        // Pedestrians will not attempt to avoid others
        //Agent will attempt to run to the first exit seen
        if (knownExfil == null)
        {
            // The exit has not been found
            GameObject exfil = findExit();
            if (exfil != null)
            {
                //Exit was found, save to pedestrian memory
                dest = exfil.transform;
                goal = dest;
                knownExfil = exfil;
            }
            else
            {
                //Exit was not found, find someone to follow
                GameObject lead = findClosestDir();
                if (lead != null)
                {
                    dest = lead.transform;
                }
                else
                    dest = runTo();
                // if the pedestrian has been moving
               /* bool quick = Vector3.Distance(lastPos, transform.position) > (GetComponent<NavMeshAgent>().speed * d_time / 6.0f);
                d_time = 0.0f;
                if (lead != null && quick)
                {
                    dest = lead.transform;
                }
                else if(e_time > 0.5f)
                {
                    dest = runTo();
                    e_time = 0.0f;
                }*/
            }
        }
        else
        {
            dest = knownExfil.transform;
            Debug.DrawRay(transform.position, dest.transform.position - transform.position, Color.black, 0.05f);
        }
        if ((knownExfil != null && Vector3.Distance(transform.position, knownExfil.transform.position) < 3.0f) || evac <= 0.0f)
        {
            if (!sent)
            {
                positions.Insert(1, "Radius: " + gameObject.GetComponent<CapsuleCollider>().radius);
                positions.Insert(2, "Max speed: " + gameObject.GetComponent<NavMeshAgent>().speed);
                positions.Insert(3, "Acceleration: " + gameObject.GetComponent<NavMeshAgent>().acceleration);
                positions.Insert(4, "Mass: " + gameObject.GetComponent<Rigidbody>().mass);
                positions.Insert(5, "FoV: " + fov);
                positions.Insert(6, "View distance: " + maxView);

                // The pedestrian has entered the radius of a goal, send pedestrian info to the recorder then destroy
                GameObject rec = GameObject.FindGameObjectWithTag("Recorder");
                rec.GetComponent<AgentRecorder>().sendData(positions);
                sent = true;
            }else
                Destroy(gameObject);
        }
        lastPos = transform.position;
    }

    public void setMaxView(float theta, float dist)
    {
        maxView = dist;
        fov = theta;
    }
    /*GameObject findClosest()
    {
        //Attemps to follow an pedestrian within view that has a similar forward vector to itself

        float _diff = 9999.0f;
        float diff = 0.0f;
        GameObject _obj = null;
        foreach (GameObject obj in gameObjects)
        {
            if (obj != null && obj.transform != this.transform)
            {
                Vector3 myDir = this.transform.forward;
                Vector3 otherDir = obj.transform.forward;

                Vector3 targetDir = obj.transform.position - this.transform.position;
                targetDir.y = 0.0f;
                Vector3 origin = this.transform.position;
                origin.y = 1.0f;
                diff = Vector3.Angle(myDir, otherDir);
                if (diff < _diff && diff < 60.0f)
                {
                    RaycastHit hit;
                    Physics.Raycast(transform.position, targetDir, out hit, 50);
                    if(debug)
                        Debug.DrawRay(origin, targetDir * hit.distance, Color.blue, 0.5f);
                    //If this pedestrian can "see" the other pedestrian
                    if (hit.transform == obj.transform)
                    {
                        _diff = diff;
                        _obj = obj;
                    }
                }
            }
        }
        return _obj;
    }*/
    
    //Uses closest object logic since a pedestrian would be interested in pursuing the nearest exit visible to them
    GameObject findExit()
    {
        if(debug)
            Debug.Log("Looking for exit");
        float _dist = 9999;
        float dist = 0.0f;
        GameObject _obj = null;
        GameObject[] goals = GameObject.FindGameObjectsWithTag("Goal");
        //Debug.Log(gameObjects.Length);
        foreach (GameObject obj in goals)
        {
            //For each pedestrian in the scene calculate the angle between from the foward vector and the direction of the other pedestrian
            Vector3 targetDir = obj.transform.position - this.transform.position;
            targetDir.y = 0.0f;
            Vector3 origin = this.transform.position;
            origin.y = 1.0f;
            if (debug)
            {
                Debug.DrawRay(origin, targetDir, Color.red, 0.01f);
            }
            float angle = Vector3.Angle(targetDir, this.transform.forward);
            // All GameObject in the FoV (120 degrees)
            if (angle < fov)
            {
                dist = Vector3.Distance(this.transform.position, obj.transform.position);
                if (dist < _dist)
                {
                    int layer_mask = LayerMask.GetMask("RayCheck");
                    RaycastHit[] hits = Physics.RaycastAll(transform.position, targetDir, maxView, layer_mask).OrderBy(h => h.distance).ToArray();

                    if (debug)
                    {
                        Debug.DrawLine(transform.position, targetDir * maxView, Color.cyan);
                        Debug.Log("Raycast exfil: " + hits.Length);
                        int index = 0;
                        foreach (RaycastHit h in hits)
                        {
                            Debug.LogError(index + " | " + h.transform.tag);
                        }
                    }

                    limit = hits.Length; 
                    foreach (RaycastHit h in hits) {
                        if (h.transform.tag.Equals("Wall"))
                        {
                            return null;
                        } else if(h.transform.tag.Equals("pedestrian"))
                        {
                            _obj = null;
                            return null;
                        }
                        else if (h.transform.tag.Equals("Goal"))
                        {
                            //Debug.Log(h.transform.gameObject.GetComponent<ExitToGoal>().getExit().name);
                            return h.transform.gameObject.GetComponent<ExitToGoal>().getExit();
                        }
                    }
                }
            }
        }
        if (debug && _obj != null)
        {
            Vector3 exitDir = _obj.transform.position - this.transform.position;
            Debug.DrawRay(this.transform.position, exitDir, Color.black, 0.01f);
        }
        return _obj;
    }

    Vector3 getWeightedDir(GameObject[] agents)
    {
        Vector3 v = new Vector3(0,0,0);
        float f = 0.0f;
        foreach(GameObject a in agents)
        {
            if (this != a)
            {
                v += a.transform.forward / Vector3.Distance(transform.position, a.transform.position);
                f += 1/Vector3.Distance(transform.position, a.transform.position);
            }

        }
        v /= f;
        v.Normalize();
        return v;
    }

    GameObject findClosestDir()
    {
        GameObject[] agents = GameObject.FindGameObjectsWithTag("pedestrian");
        Vector3 avg = getWeightedDir(agents);
        Vector3 myDir = this.transform.forward;

        float _dist = 9999.0f;
        float dist;

        float diff;
        float _diff = 360.0f;
        GameObject _obj = null;

        foreach (GameObject obj in agents)
        {
            if (obj != null && obj.transform != this.transform)
            {
                Vector3 otherDir = obj.transform.position - transform.position;
                float inView = Vector3.Angle(myDir, otherDir);
                /*if (inView < (fov+30.0f) || Vector3.Distance(obj.transform.position, transform.position) < transform.localScale.x * 3)
                {
                    dist = Vector3.Distance(obj.transform.position, transform.position);
                    diff = Vector3.Angle(avg, otherDir);
                    if (dist < _dist && diff < _diff)
                    {
                        _dist = dist;
                        _diff = diff;
                        _obj = obj;
                    }
                }*/
                if (inView <= fov)
                {
                    dist = Vector3.Distance(obj.transform.position, transform.position);
                    diff = Vector3.Angle(avg, otherDir);
                    if (dist < _dist && diff < _diff)
                    {
                        _dist = dist;
                        _diff = diff;
                        _obj = obj;
                    }
                }
            }
        }
        return _obj;
    }

    Transform runTo()
    {
        //Vector3 pos = new Vector3(0.0f, 0.0f, 0.0f);
        float d1 = Random.value;
        float d2 = Random.value;
        //Debug.Log(d1 + ", " + d2);
        //Debug.Log(d1 + ", " + d2);
        if (Random.value < 0.5f)
            d1 = -d1;
        if (Random.value < 0.5f)
            d2 = -d2;
        //Debug.Log(d1 + ", " + d2);
        RaycastHit[] hits = Physics.RaycastAll(transform.position, new Vector3(d1, 0, d2), 50.0f);
        if(debug)
            Debug.DrawLine(transform.position, transform.forward * maxView, Color.green);
        try
        {
            Transform t = hits[0].collider.transform;
            return t;
        }catch(Exception oops)
        {
            return null;
        }
    }
}
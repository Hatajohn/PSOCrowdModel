using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;

public class PSOagent : MonoBehaviour
{
    public int agentId;

    public Vector3 currV;

    public float fitness = float.MaxValue;
    public float prevFitness;
    public float bestFitness = float.MaxValue;
    public Vector3 bestPosition = new Vector3(0, 0, 0);
    public Vector3 momentum = new Vector3(0.0f, 0.0f, 0.0f);
    public float mass = 70.0f;
    public Vector3 mExpAvg = new Vector3(0.0f, 0.0f, 0.0f);
    public Vector3 leaderPos = new Vector3(0.0f, 0.0f, 0.0f);
    public float leaderFitness = float.MaxValue;
    public float minV, maxV;

    Vector3 lastPos;

    public float minSpeed = 4.0f;
    public float maxSpeed = 7.0f;
    public float speed;

    public bool evacuated = false;

    public bool isLeader = false;

    List<Vector3> positions;

    bool soloWalk = true;

    public void setPosition(Vector3 v)
    {
        if (positions == null)
            positions = new List<Vector3>();
        positions.Add(v);

        return;
    }

    public List<Vector3> getPositions()
    {
        return positions;
    }
   
    public void hardStop()
    {
        Rigidbody me = GetComponent<Rigidbody>();
        me.velocity = new Vector3(0, 0, 0);
        momentum = new Vector3(0, 0, 0);
        me.constraints = RigidbodyConstraints.FreezeAll;
    }

    float checkTime = 0.0f;
    /*private void FixedUpdate()
    {
        if (!evacuated)
        {
            if (Vector3.Distance(gameObject.transform.position, lastPos) < 0.01f && checkTime > 2.0f)
            {
                fixit(agentId);
                checkTime = 0.0f;
            }
            checkTime += Time.deltaTime;
            lastPos = gameObject.transform.position;
        }
    }*/

    /*void fixit(int id)
    {
        if (agentId != id)
            return;
        Debug.LogWarning("Fixing broken agent " + agentId);
        gameObject.transform.position = gameObject.transform.position + (new Vector3(0, 1, 0) - gameObject.transform.position).normalized * 0.01f;
    }*/

    int up = 0;
    NavMeshPath path;
    GameObject[] goals;
    //LineRenderer lineRenderer;
    Vector3[] corners;
    Rigidbody me;
    float rad;

    public void setPath(NavMeshPath p)
    {
        path = p;
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

    private void Start()
    {
        goals = GameObject.FindGameObjectsWithTag("Exit");
        path = new NavMeshPath();
        me = GetComponent<Rigidbody>();
        rad = GetComponent<CapsuleCollider>().radius;
        lastPos = gameObject.transform.position;
        currV = new Vector3();
    }

    public void sendIt()
    {
        fullSend = true;
    }

    public int pathLength = 0;
    public float dist;
    public Vector3 prev;
    bool fullSend = false;
    /*private void Update()
    {
        StartCoroutine(pathing());   
    }*/

    /*IEnumerator pathing()
    {
        if (fullSend)
        {
            Vector3 goal = ClosestGoal(transform.position);
            NavMesh.CalculatePath(transform.position, goal, NavMesh.AllAreas, path); // Calculate path for agent using A*
            corners = path.corners;
            pathLength = corners.Length;
            if (pathLength > 0)
            {
                for (int i = 0; i < corners.Length - 1; i++)
                {
                    Debug.DrawLine(corners[i], corners[i + 1], Color.red);
                }

                if (Vector3.Distance(transform.position, goal) > rad)
                {
                    //Debug.Log("Heading to goal still");
                    dist = Vector3.Distance(corners[1], transform.position);
                    if (dist < 3 * rad)
                    {
                        //Debug.Log("Pushing pash");
                        me.velocity = new Vector3();
                        me.AddForce(prev * 5, ForceMode.VelocityChange);
                    }
                    else
                    {
                        Vector3 dir = (corners[1] - transform.position).normalized;
                        prev = dir;
                        me.velocity = new Vector3();
                        me.AddForce(dir * 5, ForceMode.VelocityChange);
                    }
                }
                else
                {
                    //Debug.LogWarning("Goal reached");
                }
            }
            else
            {
                //Debug.LogWarning("Path has 0 corners");
            }
        }
        yield return null;
    }*/
}

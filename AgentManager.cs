using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;
using System.IO;

public class AgentManager : MonoBehaviour
{
    GameObject[] agents;
    NavMeshPath[] paths;
    public int numAgents = 0;
    public int numReady = 0;
    
    // Start is called before the first frame update
    void Start()
    {
        agents = GameObject.FindGameObjectsWithTag("pedestrian");
        paths = new NavMeshPath[agents.Length];
        GameObject.FindGameObjectWithTag("Recorder").GetComponent<AgentRecorder>().setAgents(agents.Length);
        calcPaths();
        int num = 0;
        bool pending = false;
        while (!pending)
        {
            foreach (GameObject g in agents)
            {
                if (g.GetComponent<NavMeshAgent>().pathPending)
                {
                    pending = false;
                    break;
                }
                else
                {
                    num++;
                    numReady = num;
                    pending = true;
                }
            }
        }
        //Debug.Log("Paths calculated");
        setPaths();
        //Debug.Log("Initial paths set");
    }

    // Update is called once per frame
    void Update()
    {
        // Check if paths are pending
        bool pending = false;
        foreach(GameObject g in agents)
        {
            if (g != null && g.scene.IsValid() && g.GetComponent<NavMeshAgent>().pathPending)
            {
                pending = true;
                break;
            }
        }
        if (!pending)
        {
            //Once all paths are ready, set them and begin new path recalculation
            setPaths();
            calcPaths();
        }
    }
    private void setPaths()
    {

        for(int i = 0; i < agents.Length; i++)
        {
            //Debug.Log("Agent " + agents[i]);
            //Debug.Log("Path " + paths[i] == null);
            if (agents[i] != null && agents[i].scene.IsValid() && paths[i] != null)
                if (agents[i].GetComponent<NavMeshAgent>().enabled == true)
                {
                    try
                    {
                        agents[i].GetComponent<NavMeshAgent>().SetPath(paths[i]);
                    }
                    catch (System.Exception e)
                    {
                        //Debug.Log("Agent path issue caught: " + e);
                    }
                }
        }
    }

    private void calcPaths()
    {
        int count = 0;
        for(int i = 0; i < agents.Length; i++)
        {
            if (agents[i]!=null && agents[i].scene.IsValid())
            {
                Transform d = agents[i].GetComponent<MoveTo>().dest;
                if (d != null)
                {
                    NavMeshPath path = new NavMeshPath();
                    NavMesh.CalculatePath(agents[i].transform.position, d.position, NavMesh.AllAreas, path);
                    paths[i] = path;
                    for (int p = 0; p < path.corners.Length - 1; p++)
                        Debug.DrawLine(path.corners[p], path.corners[p + 1], Color.red);
                }
                count++;
            }
        }
        numAgents = count;
    }
}

using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SpawnAgents : MonoBehaviour
{

    public int numAgents = 100;
    public float dpRatio = 0.1f;
    public float x = 2.8f;
    public float x_offset = 0.0f;
    public float z = 5.0f;
    public float z_offset = 6.0f;

    public GameObject agent;
    public GameObject d_agent;
    ArrayList aInfo;
    // Start is called before the first frame update
    void Start()
    {
        aInfo = new ArrayList();
        float x1 = x*5;
        float z1 = z*5;
        int dp_agents = (int)(dpRatio * numAgents);
        int ndp_agents = numAgents - dp_agents;

        //GameObject[] walls = GameObject.FindGameObjectsWithTag("Wall");

        for(int i = 0; i < dp_agents; i++)
        {
            float d1 = Random.value;
            float d2 = Random.value;
            //Debug.Log(d1 + ", " + d2);
            if (Random.value < 0.5f)
                d1 = -d1;
            if (Random.value < 0.5f)
                d2 = -d2;
            //Debug.Log(d1 + ", " + d2);
            transform.forward = new Vector3(d1, 0, d2);
            Vector3 pos = new Vector3(Random.Range(-x1 + x_offset, x1 + x_offset), 1.0f, Random.Range(-z1 + z_offset, z1 + z_offset));
            bool intersect = true;
            while (intersect)
            {
                bool _open = true;
                foreach(GameObject g in aInfo)
                {
                    if (Vector3.Distance(pos, g.transform.position) < 1.0f)
                    {
                        _open = false;
                        break;
                    }
                }
                /*foreach (GameObject g in walls)
                {
                    float x_max_pos = g.transform.position.x + 5 * g.transform.localScale.x;
                    float z_max_pos = g.transform.position.z + 5 * g.transform.localScale.z;
                    float x_min_pos = g.transform.position.x - 5 * g.transform.localScale.x;
                    float z_min_pos = g.transform.position.z - 5 * g.transform.localScale.z;
                    if (pos.x >= x_min_pos && pos.x <= x_max_pos && pos.z >= z_min_pos && pos.z <= z_max_pos)
                    {
                        _open = false;
                        break;
                    }
                }*/
                if (_open)
                    intersect = false;
                else
                    pos = new Vector3(Random.Range(-x1 + x_offset, x1 + x_offset), 1.0f, Random.Range(-z1 + z_offset, z1 + z_offset));
            }
            GameObject tempGObject = Instantiate(d_agent, pos, d_agent.transform.rotation);
            
            tempGObject.transform.forward = new Vector3(d1, 0, d2);
            aInfo.Add(tempGObject);
        }
        for(int j = 0; j < ndp_agents; j++)
        {
            float d1 = Random.value;
            float d2 = Random.value;
            //Debug.Log(d1 + ", " + d2);
            if (Random.value < 0.5f)
                d1 = -d1;
            if (Random.value < 0.5f)
                d2 = -d2;
            //Debug.Log(d1 + ", " + d2);
            transform.forward = new Vector3(d1, 0, d2);
            Vector3 pos = new Vector3(Random.Range(-x1 + x_offset, x1 + x_offset), 1.0f, Random.Range(-z1 + z_offset, z1 + z_offset));
            bool intersect = true;
            while (intersect)
            {
                bool _open = true;
                foreach (GameObject g in aInfo)
                {
                    if (Vector3.Distance(pos, g.transform.position) < 1.0f)
                    {
                        _open = false;
                        break;
                    }
                }
                /*foreach (GameObject g in walls)
                {
                    float x_max_pos = g.transform.position.x + 5 * g.transform.localScale.x;
                    float z_max_pos = g.transform.position.z + 5 * g.transform.localScale.z;
                    float x_min_pos = g.transform.position.x - 5 * g.transform.localScale.x;
                    float z_min_pos = g.transform.position.z - 5 * g.transform.localScale.z;
                    if (pos.x >= x_min_pos && pos.x <= x_max_pos && pos.z >= z_min_pos && pos.z <= z_max_pos)
                    {
                        _open = false;
                        break;
                    }
                }*/
                if (_open)
                    intersect = false;
                else
                    pos = new Vector3(Random.Range(-x1 + x_offset, x1 + x_offset), 1.0f, Random.Range(-z1 + z_offset, z1 + z_offset));
            }
            GameObject tempGObject = Instantiate(agent, pos, agent.transform.rotation);
            tempGObject.transform.forward = new Vector3(d1, 0, d2);
            aInfo.Add(tempGObject);
        }
    }

    // Update is called once per frame
    void Update()
    {
        
    }
}

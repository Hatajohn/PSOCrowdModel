using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class GoalHandler : MonoBehaviour
{
    static System.Random ran = null;
    public GameObject target;
    public float minX = -100;
    public float maxX = 100;
    GameObject[] goals;
    GameObject[] managers;
    Color[] colors;
    public bool manualMode = false;
    public int howMany = 1;
    public int numManagers;
    List<GameObject> checker;
    int check;
    Vector3 pos;
    // Start is called before the first frame update
    void Start()
    {
        ran = new System.Random((int)UnityEngine.Random.Range(1, 10000000));
        check = 0;
        managers = GameObject.FindGameObjectsWithTag("Manager");
        numManagers = managers.Length;
        checker = new List<GameObject>();
        if (!manualMode)
            makeGoals();
        else
        {
            Debug.LogWarning("Manual goal mode");
            goals = GameObject.FindGameObjectsWithTag("Exit");
            Debug.LogWarning(goals.Length);
        }
    }

    // Update is called once per frame
    void FixedUpdate()
    {
        if(numManagers != 0 && check == numManagers)
        {
            Debug.Log("All managers have requested a reset, confirmed");
            destroyGoals();
            makeGoals();
            checker = new List<GameObject>();
            check = 0;

            foreach (GameObject m in managers)
            {
                try
                {
                    m.GetComponent<PSO_MakingWay>().goals = goals;
                    m.GetComponent<PSO_MakingWay>().resetEnv();
                }
                catch
                {
                    Debug.LogError("Fuck, goal handler broke");
                }
            }
        }
    }


    void makeColors()
    {
        if (colors == null)
        {
            colors = new Color[howMany];
            for (int i = 0; i < howMany; i++)
            {
                colors[i] = UnityEngine.Random.ColorHSV(0f, 1f, 1f, 1f, 0.5f, 1f);
            }
        }
        for (int i = 0; i < howMany; i++)
        {
            goals[i].GetComponent<Renderer>().material.color = colors[i];
        }
    }

    void makeGoals()
    {
        goals = new GameObject[howMany];
        for (int i = 0; i < howMany; i++)
        {
            float x = Random.Range(minX, maxX);
            float z = Random.Range(minX, maxX);
            pos = new Vector3(x, 0, z);
            goals[i] = Instantiate(target, pos, target.transform.rotation);
        }
        makeColors();
    }

    void destroyGoals()
    {
        GameObject[] d_goals = goals;
        foreach (GameObject g in d_goals)
        {
            GameObject.Destroy(g);
        }
    }

    public void CheckIn(GameObject g)
    {
        Debug.Log("A manager has requested a reset");
        if (!checker.Contains(g))
        {
            checker.Add(g);
            check++;
            Debug.Log(check + "/" + numManagers);
            if (numManagers != 0 && check == numManagers)
            {
                //Debug.LogWarning("Resetting");
                checker = new List<GameObject>();
                check = 0;
                goal_reset();
            }
        }
    }

    bool no_spam = true;
    void goal_reset()
    {
        if (no_spam)
        {
            no_spam = false;
            Debug.Log("All managers have requested a reset, confirmed");
            try
            {
                foreach (GameObject manager in managers)
                    manager.GetComponent<PSOcluster_move>().unready();
            }
            catch
            {
                Debug.LogWarning("Managers in the instance do not have PSOcluster_move components");
            }
            destroyGoals();
            makeGoals();

            foreach (GameObject m in managers)
            {
                try
                {
                    m.GetComponent<PSOcluster_move>().goals = goals;
                    m.GetComponent<PSOcluster_move>().resetEnv();
                }
                catch
                {
                    Debug.LogWarning("Managers in the instance do not have PSOcluster_move components");
                }
                try
                {
                    m.GetComponent<PSO_MakingWay>().goals = goals;
                    m.GetComponent<PSO_MakingWay>().resetEnv();
                }
                catch
                {
                    Debug.LogWarning("Managers in the instance do not have PSO_MakingWay components");
                }
            }
            try {
            foreach (GameObject m in managers)
                m.GetComponent<PSOcluster_move>().readyCheck();
            }
            catch
            {
                Debug.LogWarning("Managers in the instance do not have PSOcluster_move components");
            }
        }
        no_spam = true;
    }

    public void manual_setup(int n)
    {
        Debug.Log("Number of managers manually set up to be " + n);
        numManagers = n;
    }
}

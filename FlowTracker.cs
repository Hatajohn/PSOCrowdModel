using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO;

public class FlowTracker : MonoBehaviour
{
    public float time;
    public int iter = 0;
    public int id = 0;
    int numAgents = 0;
    public float step = 1f;
    float d_time = 0.0f;
    public List<Vector2> data;
    public bool done = false;
    
    // Start is called before the first frame update
    void Start()
    {
        data = new List<Vector2>();
        time = Time.timeSinceLevelLoad;
        iter = PlayerPrefs.GetInt("iter");
    }

    private void OnTriggerEnter(Collider other)
    {
        //Debug.LogWarning("ENTRY: " + Time.time);
        numAgents += 1;
        addData();
    }

    private void OnTriggerExit(Collider other)
    {
        //Debug.LogWarning("EXIT: " + Time.time);
        numAgents = Mathf.Max(numAgents - 1, 0);
        addData();
    }

    void addData()
    {
        if (d_time >= step)
        {
            //Debug.LogWarning("Time: " + Time.time);
            data.Add(new Vector2(Time.timeSinceLevelLoad, numAgents));
            d_time = 0.0f;
        }
    }

    float mins = 0;
    private void FixedUpdate()
    {
        time += Time.deltaTime;
        d_time += Time.deltaTime;
        //addData();
        if (Time.timeSinceLevelLoad > 3600 && nospam) {
            Debug.LogWarning("Saving");
            saveFile();
            done = true;
        }
        else if (mins > 10)
        {
            Debug.LogWarning("Saving");
            saveFile();
            mins = 0;
        }
        mins += Time.deltaTime;
    }

    bool nospam = true;
    void saveFile()
    {
        if (nospam)
        {
            nospam = false;
            Debug.LogWarning("Flowtracker " + id + " saving");
            StreamWriter writer = new StreamWriter("Assets/Resources/Data/" + iter + "/FlowTracker" + "_" + id + ".txt", true);
            foreach (Vector2 v in data)
            {
                Debug.LogWarning("Flowtracker: " + v.x + " " + v.y);
                writer.WriteLine(v.x + " " + v.y);
            }
            data = new List<Vector2>();
            writer.Close();
        }
        else
        {
            using (StreamWriter sw = File.AppendText("Assets/Resources/Data/" + iter + "/FlowTracker" + "_" + id + ".txt"))
            {
                foreach (Vector2 v in data)
                {
                    Debug.LogWarning("Flowtracker: " + v.x + " " + v.y);
                    sw.WriteLine(v.x + " " + v.y);
                }
            }
            data = new List<Vector2>();
        }
    }
}

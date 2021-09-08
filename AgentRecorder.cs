using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;
using System.IO;
using System.Threading.Tasks;
using UnityEngine.SceneManagement;
using UnityEngine.UI;

public class AgentRecorder : MonoBehaviour
{
    int agents;
    string path;
    StreamWriter writer;
    ArrayList info;
    int arrL = 0;

    public bool reload = true;
    //string _scene = "TestingScene";
    public int iter = 10;
    public int time_scale = 4;
    int c_iter = 0;
    bool writing;
    //bool closed;

    // Start is called before the first frame update
    void Start()
    {
        Time.timeScale = time_scale;
        try
        {
            c_iter = PlayerPrefs.GetInt("c_iter");
        }
        catch (System.Exception kek)
        {
            Debug.Log("No kek yet");
        }
        path = "Assets/Resources/log_" + c_iter + ".txt";
        writer = new StreamWriter(path, true);
        info = new ArrayList();
        writing = false;
        //closed = false;
    }

    // Update is called once per frame
    void Update()
    {
        if (agents == arrL)
        {
            //Debug.LogError("Agents escaped!");
            if (!writing)
            {
                writing = true;
                try
                {
                    int a_num = 1;
                    string s = "";
                    // Each agent position Arraylist recorded
                    foreach (ArrayList t in info)
                    {
                        writer.WriteLine("Agent " + a_num);
                        writer.WriteLine("Agent status: " + t[0]);
                        writer.WriteLine(t[1]);
                        writer.WriteLine(t[2]);
                        writer.WriteLine(t[3]);
                        writer.WriteLine(t[4]);
                        writer.WriteLine(t[5]);
                        writer.WriteLine(t[6]);
                        /*positions.Insert(1, "Radius: " + gameObject.GetComponent<CapsuleCollider>().radius);
                        positions.Insert(2, "Max speed: " + gameObject.GetComponent<NavMeshAgent>().speed);
                        positions.Insert(3, "Acceleration: " + gameObject.GetComponent<NavMeshAgent>().acceleration);
                        positions.Insert(4, "Mass: " + gameObject.GetComponent<Rigidbody>().mass);*/
                        // Each position/velocity log in agent arraylist
                        //try
                        //{
                        for (int i = 7; i < t.Count; i++)
                            {
                                ArrayList y = (ArrayList)t[i];
                                //x position
                                float x = (float)y[0];
                                //z position
                                float z = (float)y[1];
                                //velocity
                                Vector3 xyz = (Vector3)y[2];
                                //Y values of agent velocity never change, remove them
                                Vector2 xz = new Vector2(xyz.x, xyz.z);
                                //time
                                float time = (float)y[3];
                                s = time + "," + x + "," + z + "," + xz.ToString();
                                writer.WriteLine(s);
                            }
                        //}
                        //catch (System.Exception e)
                        //{
                            //Debug.LogError(e);
                        //}
                        a_num += 1;
                    }
                    writer.Close();
                    Debug.LogError("Writer finished");
                    if (c_iter < iter)
                    {
                        //Debug.LogError("IM RELOADING, COVER ME!");
                        c_iter += 1;
                        PlayerPrefs.SetInt("c_iter", c_iter);
                        SceneManager.LoadScene("TestingScene");
                    }
                    else
                    {
                        PlayerPrefs.SetInt("c_iter", 0);
                        Application.Quit();
                    }
                    
                }
                catch (System.Exception e)
                {
                    Debug.Log("Issue when closing writer: " + e);
                }
            }
        }
    }

    public void sendData(ArrayList arr)
    {
        arrL += 1;
        info.Add(arr);
        //Debug.LogError(arrL + " / " + agents);
        return;
    }

    public void setAgents(int a)
    {
        agents = a;
    }
}

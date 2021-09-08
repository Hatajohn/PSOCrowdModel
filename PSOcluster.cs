using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using MathNet.Numerics.Statistics;
using System.IO;
using UnityEngine.SceneManagement;

public class PSOcluster : MonoBehaviour
{
    double cThreshold = 0.0001;
    int iterations;
    public int pso_iterations = 1000;
    UnityEngine.Random ran = null;
    public bool showPoints = true;
    public GameObject tempAgent;
    public GameObject manager;
    public float ratio = 0.10f;
    Vector3[] agents;
    GameObject[] g_agents;
    Vector3[] g_goals;
    List<double> distances;
    List<double> kde_uniform;

    public int clusterAmount = 3;

    public Dictionary<int, List<GameObject>> clusters;

    Color[] ccluster;

    public float bd = 5.0f;

    double[] centroids;
    double[] old_centroids;

    public GameObject point1;
    public GameObject point2;
    GameObject[] p1;
    GameObject[] p2;

    Color[] csp;

    public GameObject[] goals;

    public int numAgents = 300;
    int[] cluster_assignments;
    public float minMass = 60.0f;
    public float maxMass = 80.0f;
    public float avgRadius = .161f;
    public float offsetRadius = .0318f;
    float minX;
    float maxX;

    bool sent = false;
    GameObject[] managers;
    List<int> checker;
    List<int> resetlist;
    int check = 0;

    public int CurrentIteration = 0;
    public int IterationLimit = 10;

    GameObject[] recorders;

    bool appendTo = false;

    // Start is called before the first frame update
    void Start()
    {
        Physics.autoSimulation = false;
        try
        {
            CurrentIteration = PlayerPrefs.GetInt("iter");
            if (!Directory.Exists("Assets/Resources/Data/" + CurrentIteration))
            {
                string path = "Assets/Resources/Data/" + CurrentIteration;
                DirectoryInfo di = Directory.CreateDirectory(path);
                Console.WriteLine("The directory was created successfully at {0}.", Directory.GetCreationTime(path));
            }
        }
        catch
        {
            Debug.LogWarning("Failed to get iteration");
        }
        // Varibles:
        //  Number of agents
        //  Number of clusters
        //  Number of exits
        //  Wheelchair user ratio

        // Identify X positions in the scene (X1 agents X2 exits)
        // Slap down the exits -> get shortest distance among all exits for each agent
        // Use Kernel Density Estimation to assign agents to one of N clusters
        // Bandwidth will have to be tested for
        // double x = KernelDensity.EstimateGaussian(distance, bandwidth, all distances in List);

        // Within each cluster ratio wheelchair and walking agents
        //  Each will be an array of GameObjects
        //  Instantiate PSOmanager and assign both agent groups to GameObject arrays of new object
        // Move each cluster using leader-follower PSO

        GameObject h = GameObject.FindGameObjectWithTag("Handler");
        recorders = GameObject.FindGameObjectsWithTag("Recorder");

        evacDataL = new List<Vector2>();
        evacDataF = new List<Vector2>();
        evacDataA = new List<Vector2>();

        minX = h.GetComponent<GoalHandler>().minX;
        maxX = h.GetComponent<GoalHandler>().maxX;

        ran = new UnityEngine.Random();
        Debug.Log("\nBegin PSO Clustering\n");

        goals = GameObject.FindGameObjectsWithTag("Exit");
        //Debug.Log("Number of exits: " + goals.Length);
        csp = new Color[] { Color.red, Color.blue, Color.green, Color.cyan, Color.gray, Color.magenta, Color.yellow };
        /*foreach(GameObject g in goals)
        {
            Debug.Log(g.transform.position);
        }*/

        ccluster = new Color[clusterAmount];
        for(int i = 0; i < clusterAmount; i++)
        {
            ccluster[i] = csp[i % csp.Length];
        }

        makeAgents();
        kde();
        kmeans();

        if (showPoints)
        {
            plot_distances();
            plot_kde();
        }

        checker = new List<int>();

        make_managers();

    } // End Start()

    void kmeans()
    {
        centroids = initialize_centroids();
        cluster_assignments = cluster_assignment();
        mapClusters();
        iterations = 0;

        DebugDistances();
        updateKmeans();
    }

    void updateKmeans()
    {
        bool b;
        old_centroids = new double[centroids.Length];
        do
        {
            // Store old centroid values
            for (int i = 0; i < centroids.Length; i++)
                old_centroids[i] = centroids[i];
            // Move centroids to their cluster average
            update_centroids();
            // Save the new assignments if they changed
            cluster_assignments = cluster_assignment();
            // Recolor agents
            mapClusters();
            iterations++;
            // Check if centroids moved
            b = doItAgain();
            DebugDistances();
        } while (b);
        Debug.Log("It took only " + iterations + " updates");
    }

    bool doItAgain()
    {
        for(int i = 0; i < centroids.Length; i++)
        {
            // if any of the differences are larger than the threshold (a centroid was moved) then the kmeans centroids will need to be updated once again
            if (Mathf.Abs((float)(centroids[i] - old_centroids[i])) > cThreshold)
                return true;
        }
        return false;
    }

    // Randomly initizalize n centroids
    double[] initialize_centroids()
    {
        double[] output = new double[clusterAmount];
        List<double> kde_copy = new List<double>(kde_uniform);
        for(int i = 0; i < clusterAmount; i++)
        {
            int index = (int)UnityEngine.Random.Range(0, kde_copy.Count);
            double v = kde_copy[index];
            // Remove the selected centroid
            kde_copy.RemoveAt(index);
            output[i] = v;
        }
        return output;
    }

    // Assign values from kde to clusters based on minimum difference
    int[] cluster_assignment()
    {
        int[] assignments = new int[kde_uniform.Count];
        for (int j = 0; j < kde_uniform.Count; j++)
        {
            int closest = 0;
            double mdist = double.MaxValue;
            for(int i = 0; i < centroids.Length; i++)
            {
                // Using difference between values, assign each distance to a cluster 
                float cdist = Mathf.Abs((float)(centroids[i] - kde_uniform[j]));
                if(cdist < mdist)
                {
                    mdist = cdist;
                    // cluster denoted by int (ie 1-3 etc)
                    closest = i;
                }
            }
            assignments[j] = closest;
        }
        return assignments;
    }

    void mapClusters()
    {
        //int[] m = cluster_assignments;
        // cluster x <- GameObjects, ie cluster 1 is the list of GameObjects that were assigned to that cluster
        clusters = new Dictionary<int, List<GameObject>>();
        for (int i = 0; i < cluster_assignments.Length; i++)
        {
            try
            {
                g_agents[i].GetComponent<Renderer>().material.color = ccluster[cluster_assignments[i]];
                if (!clusters.ContainsKey(cluster_assignments[i]))
                    clusters[cluster_assignments[i]] = new List<GameObject>();
                clusters[cluster_assignments[i]].Add(g_agents[i]);
            }
            catch { 
                Debug.LogWarning("outside the bounds of the cluster most likely");
            }
        }
    }

    void update_centroids()
    {
        List<List<double>> a = new List<List<double>>();
        foreach (double d in centroids)
            a.Add(new List<double>());
        for (int i = 0; i < cluster_assignments.Length; i++)
        {
            a[cluster_assignments[i]].Add(kde_uniform[i]);
        }
        // Assignment index corresponds with centroid index
        for (int i = 0; i < a.Count; i++)
        {
            double sum = 0;
            foreach (double d in a[i])
                sum += d;
            centroids[i] = sum / a[i].Count;
        }
    }

    // leaders use targets as goal
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

    Vector3 makeRandomVector()
    {
        float vx = UnityEngine.Random.Range(minX, maxX);
        float vz = UnityEngine.Random.Range(minX, maxX);

        return new Vector3(vx, 0, vz);
    }

    void makeAgents()
    {
        try
        {
            // Initialize all Particle objects
            agents = new Vector3[numAgents];
            distances = new List<double>();
            g_agents = new GameObject[numAgents];
            g_goals = new Vector3[numAgents];

            for (int i = 0; i < agents.Length; i++)
            {
                agents[i] = makeRandomVector();
                g_goals[i] = ClosestGoal(agents[i]);
                distances.Add(Vector3.Distance(g_goals[i], agents[i]));

                g_agents[i] = Instantiate(tempAgent, new Vector3(agents[i].x, 2, agents[i].z), tempAgent.transform.rotation) as GameObject;
                g_agents[i].GetComponent<PSOagent>().agentId = i;
            }
        }
        catch
        {
            Debug.Log("Make agents failed");
        }
    }

    void kde()
    {
        try
        {
            kde_uniform = new List<double>();
            for (int i = 0; i < numAgents; i++)
            {
                kde_uniform.Add(KernelDensity.EstimateUniform(distances[i], bd, distances));
            }
        }
        catch
        {
            Debug.Log("KDE failed");
        }
    }

    void plot_distances()
    {
        try
        {
            p1 = new GameObject[numAgents];
            for (int i = 0; i < numAgents; i++)
            {
                float xPos = i * (maxX - minX) / numAgents;
                p1[i] = Instantiate(point1, new Vector3(xPos + minX, (float)distances[i], 110), point1.transform.rotation);
            }
        }
        catch
        {
            Debug.Log("Distance plotting failed");
        }
    }

    void plot_kde()
    {
        try
        {
            p2 = new GameObject[numAgents];
            for (int i = 0; i < numAgents; i++)
            {
                float xPos = i * (maxX - minX) / numAgents;
                p2[i] = Instantiate(point2, new Vector3(xPos + minX, 10 * numAgents * (float)kde_uniform[i], 101), point2.transform.rotation);
            }
        }
        catch
        {
            Debug.Log("KDE plotting failed");
        }
    }

    void DebugDistances()
    {
        for(int i = 0; i < numAgents; i++)
            Debug.DrawLine(agents[i], g_goals[i], g_agents[i].GetComponent<Renderer>().material.color, 0.01f);
    }

    void clearAgents()
    {
        foreach (GameObject g in g_agents)
            Destroy(g);
    }
    void clearDistance()
    {
        foreach (GameObject p in p1)
            Destroy(p);
    }

    void clearKDE()
    {
        foreach (GameObject p in p2)
            Destroy(p);
    }

    void FixClusters()
    {
        //Debug.LogError("Fix");
        for (int i = 0; i < clusters.Count; i++)
        {
            if (!clusters.ContainsKey(i) || clusters[i].Count == 0)
            {
                clusters.Remove(i);
            }
        }
    }

    void make_managers()
    {
        FixClusters();
        //Debug.LogWarning("Making managers");
        managers = new GameObject[clusters.Count];
        // Each cluster requires a manager to move the agents using PSO etc
        for (int i = 0; i < clusters.Count; i++)
        {
            // Create a copy of the manager GameObject
            managers[i] = Instantiate(manager, manager.transform.position, manager.transform.rotation);
            managers[i].GetComponent<PSOcluster_move>().creator = gameObject;
        }

        GameObject.FindGameObjectWithTag("Handler").GetComponent<GoalHandler>().manual_setup(clusters.Count);
    }

    public void CheckIn(int id)
    {
        if (!checker.Contains(id))
        {
            Debug.Log("Manager " + id + " has requested data");
            checker.Add(id);
            check++;
            Debug.Log(check + "/" + managers.Length);
            if (check == managers.Length)
            {
                prepare_managers();
                start_managers();
            }
        }
    }
    // Setp up the PSO managers for each cluster so they can begin moving
    Dictionary<int, List<GameObject>> clustersCopy;
    void prepare_managers()
    {
        clustersCopy = new Dictionary<int, List<GameObject>>();
        for (int i = 0; i < managers.Length; i++)
        {
            int c_size = 0;
            try
            {
                // Save the size of the cluster to reduce the number of times the reference is called
                c_size = clusters[i].Count;
                clustersCopy.Add(i, new List<GameObject>());
            }
            catch
            {
                //Debug.LogError("Cluster failure at " + i + ", " + clusters.ContainsKey(i) + " : " + clusters.Count);
                
                SceneManager.LoadScene("Experimental");
                return;
            }
            int expected = Mathf.FloorToInt(c_size * ratio);

            // Number of leaders and followers based on intended ratio
            List<GameObject> leaders = new List<GameObject>();
            List<GameObject> followers = new List<GameObject>();

            //shuffle the list
            /*for (int x = 0; x < c_size; x++)
            {
                GameObject t = clusters[i][x];
                int randomIndex = UnityEngine.Random.Range(x, c_size);
                clusters[i][x] = clusters[i][randomIndex];
                clusters[i][randomIndex] = t;
            }*/

            // Order the lists
            List<GameObject> clusterCopy = new List<GameObject>();
            while (clusters[i].Count > 0)
            {
                float d = float.MaxValue;
                GameObject t = null;
                int r = 0;
                for(int x = 0; x < clusters[i].Count; x++)
                {
                    float _d = Vector3.Distance(ClosestGoal(clusters[i][x].transform.position), clusters[i][x].transform.position);
                    if(_d < d)
                    {
                        t = clusters[i][x];
                        d = _d;
                        r = x;
                    }
                }
                clusterCopy.Add(t);
                clusters[i].RemoveAt(r);
                clustersCopy[i].Add(t);
            }

            int fIndex = 0;
            for (int j = 0; j < c_size; j++)
            {
                if(j < expected)
                {
                    leaders.Add(clusterCopy[j]);
                    leaders[j].GetComponent<PSOagent>().isLeader = true;
                    leaders[j].GetComponent<Outline>().OutlineColor = Color.yellow;

                    //Mass
                    float mass = UnityEngine.Random.Range(minMass + 10, maxMass + 10);
                    leaders[j].GetComponent<Rigidbody>().mass = mass;
                    //leaders[j].GetComponent<PSOagent>().speed = UnityEngine.Random.Range(leaders[j].GetComponent<PSOagent>().minSpeed, leaders[j].GetComponent<PSOagent>().maxSpeed);
                    leaders[j].GetComponent<PSOagent>().speed = UnityEngine.Random.Range(1.48f, 2.23f);
                    //recorder.GetComponent<PSORecorder>().storeData(leaders[j].GetComponent<PSOagent>().agentId, -5, new Vector3(mass, 0, 0));
                }
                else
                {
                    followers.Add(clusterCopy[j]);
                    followers[fIndex].GetComponent<Outline>().OutlineColor = Color.black;

                    //Mass
                    float mass = UnityEngine.Random.Range(minMass, maxMass);
                    //Debug.Log((followers[j] != null) + " " + (followers[j].GetComponent<Rigidbody>() != null));
                    followers[fIndex].GetComponent<Rigidbody>().mass = mass;
                    //followers[fIndex].GetComponent<PSOagent>().speed = UnityEngine.Random.Range(followers[fIndex].GetComponent<PSOagent>().minSpeed, followers[fIndex].GetComponent<PSOagent>().maxSpeed);
                    followers[fIndex].GetComponent<PSOagent>().speed = UnityEngine.Random.Range(1.3f, 1.7f);
                    fIndex++;
                }
            }

                Debug.Log("Leaders: " + leaders.Count + "; Follwers: " + followers.Count);
            // Send the arrays to the manager
            managers[i].GetComponent<PSOcluster_move>().setup(leaders, followers, pso_iterations, minX, maxX);
        }
    }

    void start_managers()
    {
        resetlist = new List<int>();
        //Debug.LogWarning("Starting managers");
        foreach (GameObject g in managers)
            g.GetComponent<PSOcluster_move>().readyCheck();
    }

    bool once2 = true;
    void resetSim()
    {
        if (once2 && CurrentIteration < IterationLimit)
        {
            once2 = false;
            CurrentIteration += 1;
            PlayerPrefs.SetInt("iter", CurrentIteration);
            SceneManager.LoadScene("Experimental");
            //GameObject recorder = GameObject.FindGameObjectWithTag("Recorder");
            //recorder.GetComponent<PSORecorder>().saveFile("PSOevac"+CurrentIteration+".txt");
        }
        else if(once2)
        {
            once2 = false;
            PlayerPrefs.SetInt("iter", 0);
            Debug.Log("Complete!");
        }
    }

    bool filesSaved()
    {
        foreach(GameObject r in recorders)
        {
            if (!r.GetComponent<FlowTracker>().done)
                return false;
        }
        return true;
    }

    // Change reset simulation to when time is up
    int resetCheck = 0;
    /*public void CheckReset(int id)
    {
        if (!resetlist.Contains(id))
        {
            Debug.Log("Manager " + id + " has requested sim reset");
            resetlist.Add(id);
            resetCheck++;
            Debug.Log(resetCheck + "/" + managers.Length);
            if (resetCheck == managers.Length - 1 || filesSaved())
            {
                resetSim();
            }
        }
    }*/

    bool once = true;
    float d_time = 0.0f;
    float step = 0.1f;
    float mins = 0;
    bool resetReady = false;
    private void FixedUpdate()
    {
        if (resetReady)
        {
            //parametersToFile();
            resetSim();
            return;
        }
        d_time += Time.deltaTime;
        if (d_time >= step)
        {
            recordData();
            d_time = 0.0f;
        }
        if(Time.timeSinceLevelLoad % 10 > mins)
        {
            writeData();
        }
        mins = Time.timeSinceLevelLoad % 60;
        if (evac == numAgents || Time.timeSinceLevelLoad > 600)
        {
            //Debug.LogError("WOAH, IT'S OVER");
            if (!nospam)
            {
                nospam = true;
                writeData();
                
                resetReady = true;
            }
        }
    }

    int escaped()
    {
        return evac;
    }


    // Number of agents evacuated over the course of the simulation
    // Agent speed distribution
    // Agent mass distribution
    List<Vector2> evacDataL;
    List<Vector2> evacDataF;
    List<Vector2> evacDataA;
    public int evac = 0;
    void recordData()
    {
        try
        {
            float t = Time.timeSinceLevelLoad;
            int leaderE = 0;
            int followE = 0;
            int allE = 0;
            //Debug.LogError(clustersCopy.Count + " number of objects");
            
            for (int i = 0; i < clustersCopy.Count; i++)
            {
                //Debug.LogWarning(clustersCopy[i].Count + " number of objects");
                foreach (GameObject g in clustersCopy[i])
                {
                    if (g.GetComponent<PSOagent>().isLeader && g.GetComponent<PSOagent>().evacuated)
                    {
                        leaderE += 1;
                        allE += 1;
                    }
                    if (!g.GetComponent<PSOagent>().isLeader && g.GetComponent<PSOagent>().evacuated)
                    {
                        followE += 1;
                        allE += 1;
                    }
                }
            }
            evacDataL.Add(new Vector2(t, leaderE));
            evacDataF.Add(new Vector2(t, followE));
            evacDataA.Add(new Vector2(t, allE));
            evac = (int)evacDataA[evacDataA.Count - 1].y;
        }
        catch
        {
            //Debug.LogWarning("LOL LMAO");
        }
    }

    bool nospam = false;
    bool wDone1 = false;
    public void writeData()
    {
        //Debug.LogError("Writing evac");
        if(!Directory.Exists("Assets/Resources/Data/" + PlayerPrefs.GetInt("iter"))){
            Directory.CreateDirectory("Assets/Resources/Data/" + PlayerPrefs.GetInt("iter"));
        }
        if (!appendTo)
        {
            appendTo = true;
            StreamWriter writer = new StreamWriter("Assets/Resources/Data/" + PlayerPrefs.GetInt("iter") + "/LeaderEvac.txt", false);
            writer.WriteLine("Time NumberOfLeadersEvacuated");
            foreach (Vector2 v in evacDataL)
            {
                writer.WriteLine(v.x + " " + v.y);
            }
            writer.Close();

            writer = new StreamWriter("Assets/Resources/Data/" + PlayerPrefs.GetInt("iter") + "/FollowerEvac.txt", false);
            writer.WriteLine("Time NumberOfFollowersEvacuated");
            foreach (Vector2 v in evacDataF)
            {
                writer.WriteLine(v.x + " " + v.y);
            }
            writer.Close();

            writer = new StreamWriter("Assets/Resources/Data/" + PlayerPrefs.GetInt("iter") + "/AllEvac.txt", false);
            writer.WriteLine("Time NumberOfAgentsEvacuated");
            foreach (Vector2 v in evacDataA)
            {
                writer.WriteLine(v.x + " " + v.y);
            }
            writer.Close();

            evacDataL = new List<Vector2>();
            evacDataF = new List<Vector2>();
            evacDataA = new List<Vector2>();

            //wDone1 = true;
        }
        else
        {
            using (StreamWriter sw = File.AppendText("Assets/Resources/Data/" + PlayerPrefs.GetInt("iter") + "/LeaderEvac.txt"))
            {
                //writer.WriteLine("Time NumberOfLeadersEvacuated");
                foreach (Vector2 v in evacDataL)
                {
                    sw.WriteLine(v.x + " " + v.y);
                }
            }
            using (StreamWriter sw = File.AppendText("Assets/Resources/Data/" + PlayerPrefs.GetInt("iter") + "/FollowerEvac.txt"))
            {
                //writer.WriteLine("Time NumberOfFollowersEvacuated");
                foreach (Vector2 v in evacDataF)
                {
                    sw.WriteLine(v.x + " " + v.y);
                }
            }
            using (StreamWriter sw = File.AppendText("Assets/Resources/Data/" + PlayerPrefs.GetInt("iter") + "/AllEvac.txt"))
            {
                //writer.WriteLine("Time NumberOfAgentsEvacuated");
                foreach (Vector2 v in evacDataA)
                {
                    sw.WriteLine(v.x + " " + v.y);
                }
            }
            evacDataL = new List<Vector2>();
            evacDataF = new List<Vector2>();
            evacDataA = new List<Vector2>();
            //wDone1 = true;
        }
        //Debug.LogError("evac written");
    }

    bool wDone2 = false;
    void parametersToFile()
    {
        //Debug.LogError("Writing parameters");
        int index = 0;
        StreamWriter writer = new StreamWriter("Assets/Resources/Data/" + PlayerPrefs.GetInt("iter") + "/AgentParameters.txt", true);
        writer.WriteLine("ID MASS SPEED isLEADER");
        for (int i = 0; i < clustersCopy.Count; i++)
        {
            Debug.Log("Writing for cluster " + i);
            foreach (GameObject g in clustersCopy[i])
            {
                writer.WriteLine(index + " " + g.GetComponent<Rigidbody>().mass + " " + g.GetComponent<PSOagent>().speed + " " + g.GetComponent<PSOagent>().isLeader);
                index += 1;
            }
        }
        writer.Close();
        wDone2 = true;
        //Debug.LogError("Parameters written");
    }
}

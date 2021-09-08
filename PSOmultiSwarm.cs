using System;
using System.Linq;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PSOmultiSwarm : MonoBehaviour
{
    static System.Random ran = null;
    public GameObject pedestrian;
    //prefab
    public GameObject target;
    //reference to instantiated prefab
    GameObject _target;

    public int num_goals = 1;
    public bool moveTarget = true;
    public float targetX = 0.0f;
    public float targetZ = 0.0f;
    public bool hard_clusters = false;
    public int num_clusters = 3;

    // Objective function returns the smallest distance from a particle position to a goal
    float ObjectiveFunction(Vector3 x)
    {
        float min_dist = float.MaxValue;
        GameObject g = null;
        foreach(GameObject _g in goals)
        {
            float d = Vector3.Distance(x, _g.transform.position);
            if (d < min_dist)
            {
                min_dist = d;
                g = _g;
            }
        }
        //RaycastHit hit;
        Vector3 v = g.transform.position - x;
        /*if ( Physics.Raycast(x, v, out hit, 50.0f))
        {
            Debug.DrawRay(x, v * hit.distance, Color.red);
        }*/
        return min_dist;
    }

    public int numberParticles = 1000;
    public int numberIterations = 1000;
    int iteration;
    public float minX = -50.0f;
    public float maxX = 50.0f;
    GameObject[] particle;
    List<List<GameObject>> swarms;
    float f_threshold = 2.0f;
    GameObject[] goals;
    Color[] colors;
    Vector3 bestGlobalPosition;
    float bestGlobalFitness;
    List<float> bestGfit;
    List<Vector3> bestGPos;

    public int v_factor = 2;

    public double w = 0.729; // inertia weight
    // Learning factors
    public double c1 = 1.49445; // cognitive weight
    public double c2 = 1.49445; // social weight
    public double r1, r2; // randomizations

    float time;

    Vector3 goal;

    bool loop = true;

    float randomInRange(float _min, float _max)
    {
        return (float)((_max - _min) * ran.NextDouble() + _min);
    }

    void makeColors()
    {
        if (colors == null)
        {
            colors = new Color[num_goals];
            for (int i = 0; i < num_goals; i++)
            {
                colors[i] = UnityEngine.Random.ColorHSV(0f, 1f, 1f, 1f, 0.5f, 1f);
            }
        }
        for (int i = 0; i < num_goals; i++)
        {
            goals[i].GetComponent<Renderer>().material.color = colors[i];
        }
    }

    Color[] makeColors2(int n)
    {
        Color[] output;
        Debug.Log(n + " colors");

        output = new Color[n];
        for (int z = 0; z < n; z++)
        {
            output[z] = UnityEngine.Random.ColorHSV(0f, 1f, 1f, 1f, 0.5f, 1f);
        }

        return output;
    }


    void makeGoals()
    {
        goals = new GameObject[num_goals];
        for (int i = 0; i < num_goals; i++)
        {
            float x = randomInRange(minX, maxX);
            float z = randomInRange(minX, maxX);
            goal = new Vector3(x, 0, z);
            _target = Instantiate(target, goal, target.transform.rotation);
            goals[i] = _target;
        }
        makeColors();
    }

    void destroyGoals()
    {
        foreach(GameObject g in goals)
        {
            GameObject.Destroy(g);
        }
    }

    // Start is called before the first frame update
    void Start()
    {
        time = 0.0f;
        Debug.Log("\nBegin PSO demo\n");
        ran = new System.Random(0);

        iteration = 0;
 
        makeGoals();

        particle = new GameObject[numberParticles];
        bestGlobalFitness = float.MaxValue;

        float maxV = maxX / v_factor; ;// randomInRange(1, maxX);
        float minV = minX / v_factor; ;// -maxV;

        // Initialize all Particle objects

        for (int i = 0; i < particle.Length; ++i)
        {
            float _x = randomInRange(minX, maxX);
            float _z = randomInRange(minX, maxX);
            Vector3 pos = new Vector3(_x, 1, _z);

            double fitness = ObjectiveFunction(pos);

            float lo = (float)(-1.0 * Math.Abs(maxV - minV));
            float hi = Math.Abs(maxV - minV);
            float vx = randomInRange(lo, hi);
            float vz = randomInRange(lo, hi);

            //Random position
            GameObject NGObject = Instantiate(pedestrian, pos, pedestrian.transform.rotation);
            //Random velocity
            NGObject.GetComponent<Rigidbody>().velocity = new Vector3(vx, 0, vz);
            NGObject.GetComponent<PSOagent>().fitness = ObjectiveFunction(NGObject.transform.position);
            particle[i] = NGObject;
            var comp = particle[i].GetComponent<PSOagent>().fitness;
            
            particle[i].GetComponent<PSOagent>().minV = minV;
            particle[i].GetComponent<PSOagent>().maxV = maxV;

            
            /*if (comp < bestGlobalFitness)
            {
                bestGlobalFitness = comp;
                bestGlobalPosition = particle[i].transform.position;
                //particle[i].GetComponent<PSOagent>().bestPosition = particle[i].transform.position;
            }*/
        }
        assignParticles();
    } // End Start()

    //bool reset = false;

    void Update()
    {
        if (iteration < numberIterations)
        {
            //assignParticles();
            update_swarms();
            iteration++;
        }
        else if(moveTarget)
        {
            Debug.Log("Moving target(s) to new location");
            destroyGoals();
            makeGoals();
            //bestGlobalFitness = float.MaxValue;
            //
            for (int i = 0; i < particle.Length; ++i)
            {
                particle[i].GetComponent<PSOagent>().fitness = ObjectiveFunction(particle[i].transform.position);
                particle[i].GetComponent<PSOagent>().bestFitness = float.MaxValue;
                particle[i].GetComponent<PSOagent>().bestPosition = particle[i].transform.position;
                /*
                var comp = particle[i].GetComponent<PSOagent>().fitness;
                if (comp < bestGlobalFitness)
                {
                    bestGlobalFitness = comp;
                    particle[i].GetComponent<PSOagent>().bestFitness = comp;
                    bestGlobalPosition = particle[i].transform.position;
                }
                */
            }
            //assignParticles();
            iteration = 0;
        }

    }

    void assignParticles()
    {
        int num_swarms = num_clusters;
        //float min = float.MaxValue;
        //float max = float.MinValue;
        Debug.Log("Expected bins: " + num_swarms);
        swarms = new List<List<GameObject>>();
        int size = particle.Length / num_swarms;
        List<GameObject> _L = particle.OfType<GameObject>().ToList();
        Color[] recolor = makeColors2(num_clusters);
        for (int i = 0; i < num_swarms; i+=size)
        {
            List<GameObject> microSwarm = _L.GetRange(i, Math.Min(size, _L.Count - i));
            Debug.Log(i);
            foreach(GameObject g in microSwarm)
                g.GetComponent<Renderer>().material.color = recolor[i];
            swarms.Add(microSwarm);
            Debug.Log("Swarm " + i + "; " + microSwarm.Count);
        }
        
        if (swarms.Count > 0)
        {
            // Calculate best fitness and position for each swarm
            bestGfit = new List<float>();
            bestGPos = new List<Vector3>();
            foreach (List<GameObject> l in swarms)
            {
                float bestLocal = float.MaxValue;
                Vector3 bestPos = new Vector3();
                float fit;
                foreach (GameObject g in l)
                {
                    fit = g.GetComponent<PSOagent>().fitness;
                    if (fit < bestLocal)
                    {
                        bestLocal = fit;
                        bestPos = g.transform.position;
                    }
                }
                bestGfit.Add(bestLocal);
                bestGPos.Add(bestPos);
            }
            Debug.Log(bestGfit.Count + " : " + bestGPos.Count);
        }
    }

    /*void assignParticles()
    {
        int num_swarms = 1;
        if (hard_clusters)
            num_swarms = num_clusters;
        else
            num_swarms = (int)Math.Ceiling(Math.Sqrt(numberParticles));
        float min = float.MaxValue;
        float max = float.MinValue;
        Debug.Log("Expected bins: " + num_swarms);
        swarms = new List<List<GameObject>>();
        for (int i = 0; i < num_swarms; i++)
        {
            swarms.Add(new List<GameObject>());
        }
        foreach(GameObject p in particle)
        {
            float newFitness = p.GetComponent<PSOagent>().fitness;
            if (newFitness < min)
                min = newFitness;
            if (newFitness > max)
                max = newFitness;
        }
        float range = max - min;
        float width = range / num_swarms;
        Debug.Log("Range: " + range + " Width: " + width);
        foreach (GameObject p in particle)
        {
            float v = p.GetComponent<PSOagent>().fitness;
            int i = 0;
            bool placed = false;
            while(max - width * (i+1) > min && !placed)
            {
                if (v >= max - width * (i + 1))
                {
                    swarms[i].Add(p);
                    placed = true;
                }
                i++;
            }
        }
        List<List<GameObject>> r = new List<List<GameObject>>();
        // remove empty bins
        foreach(List<GameObject> s in swarms)
        {
            if(s.Count > 0)
            {
                r.Add(s);
            }
        }
        swarms = r;
        if (swarms.Count > 0)
        {
            bestGfit = new List<float>();
            bestGPos = new List<Vector3>();
            foreach (List<GameObject> l in swarms)
            {
                float bestLocal = float.MaxValue;
                Vector3 bestPos = new Vector3();
                float fit;
                foreach (GameObject g in l)
                {
                    fit = g.GetComponent<PSOagent>().fitness;
                    if (fit < bestLocal)
                    {
                        bestLocal = fit;
                        bestPos = g.transform.position;
                    }
                }
                bestGfit.Add(bestLocal);
                bestGPos.Add(bestPos);
            }
        }
    }*/

    void update_swarms()
    {
        //string bins = "";
        // If there are multiple swarms
        if (swarms.Count > 1 || num_clusters > 1)
        {
            Debug.Log("Updating multiple swarms");
            // This is a bit of a mess currently
            int index = 0;
            // For each swarm, update based on that specific swarms bestfit and bestpos
            Debug.Log("Updating " + swarms.Count + " swarms, specifically");
            foreach (List<GameObject> L in swarms)
            {
                float sendFit = bestGfit[index];
                Vector3 sendPos = bestGPos[index];
                //bins += (L.Count + ". ");
                update_swarm(L, sendFit, sendPos, index);
                // update global best fit and pos if the swarm contains values that are better
/*                if(bestGfit[index] < bestGlobalFitness)
                {
                    bestGlobalFitness = bestGfit[index];
                    bestGlobalPosition = bestGPos[index];
                }*/
                index++;
            }
        }
        else
        {
            Debug.Log("Updating one swarm");
            update_swarm();
        }
        //Debug.Log("Particles in bins: " + bins);
    }

    void update_swarm(List<GameObject> swarm, float sentFit, Vector3 sentPos, int index)
    {
        Debug.Log("Begin swarm update " + swarm.Count);
        for (int i = 0; i < swarm.Count; ++i)
        {
            Color c = swarm[i].GetComponent<Renderer>().material.color;
            swarm[i].GetComponent<Renderer>().material.color = Color.black;
            float minV = swarm[i].GetComponent<PSOagent>().minV;
            float maxV = swarm[i].GetComponent<PSOagent>().maxV;
            Rigidbody currPR = swarm[i].GetComponent<Rigidbody>();

            r1 = ran.NextDouble();
            r2 = ran.NextDouble();

            double _vx = (w * currPR.velocity.x) +
                (c1 * r1 * (swarm[i].GetComponent<PSOagent>().bestPosition.x - swarm[i].transform.position.x)) +
                (c2 * r2 * (sentPos.x - swarm[i].transform.position.x));

            r1 = ran.NextDouble();
            r2 = ran.NextDouble();

            double _vz = (w * currPR.velocity.z) +
                (c1 * r1 * (sentPos.z - swarm[i].transform.position.z)) +
                (c2 * r2 * (sentPos.z - swarm[i].transform.position.z));

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

            Vector3 newV = new Vector3((float)_vx, 0, (float)_vz);
            swarm[i].GetComponent<Rigidbody>().velocity = newV;
            // clamp x and z min positions
            swarm[i].transform.position = swarm[i].transform.position + Time.deltaTime * newV;
            if (swarm[i].transform.position.x < minX)
                swarm[i].transform.position = new Vector3(minX, 0, swarm[i].transform.position.z);
            if (swarm[i].transform.position.y < minX)
                swarm[i].transform.position = new Vector3(swarm[i].transform.position.x, 0, minX);
            // clamp x and z max positions
            if (swarm[i].transform.position.x > maxX)
                swarm[i].transform.position = new Vector3(maxX, 0, swarm[i].transform.position.z);
            if (swarm[i].transform.position.y > maxX)
                swarm[i].transform.position = new Vector3(swarm[i].transform.position.x, 0, maxX);

            float newFitness = (float)ObjectiveFunction(swarm[i].transform.position);
            swarm[i].GetComponent<PSOagent>().fitness = newFitness;

            if (newFitness < swarm[i].GetComponent<PSOagent>().bestFitness)
            {
                swarm[i].GetComponent<PSOagent>().bestPosition = swarm[i].transform.position;
                swarm[i].GetComponent<PSOagent>().bestFitness = newFitness;
            }
            if (newFitness < sentFit)
            {
                sentFit = newFitness;
                sentPos = swarm[i].transform.position;
                bestGPos[index] = sentPos;
                bestGfit[index] = sentFit;
            }
            //swarm[i].GetComponent<Renderer>().material.color = c;
        }
        //Debug.Log("End swarm update");
    }

    // Updates each particle assuming they are in a single swarm
    void update_swarm()
    {
        for (int i = 0; i < particle.Length; ++i)
        {
            float minV = particle[i].GetComponent<PSOagent>().minV;
            float maxV = particle[i].GetComponent<PSOagent>().maxV;
            Rigidbody currPR = particle[i].GetComponent<Rigidbody>();

            r1 = ran.NextDouble();
            r2 = ran.NextDouble();

            double _vx = (w * currPR.velocity.x) +
                (c1 * r1 * (particle[i].GetComponent<PSOagent>().bestPosition.x - particle[i].transform.position.x)) +
                (c2 * r2 * (bestGlobalPosition.x - particle[i].transform.position.x));

            r1 = ran.NextDouble();
            r2 = ran.NextDouble();

            double _vz = (w * currPR.velocity.z) +
                (c1 * r1 * (bestGlobalPosition.z - particle[i].transform.position.z)) +
                (c2 * r2 * (bestGlobalPosition.z - particle[i].transform.position.z));

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

            Vector3 newV = new Vector3((float)_vx, 0, (float)_vz);
            particle[i].GetComponent<Rigidbody>().velocity = newV;
            // clamp x and z min positions
            particle[i].transform.position = particle[i].transform.position + Time.deltaTime * newV;
            if (particle[i].transform.position.x < minX)
                particle[i].transform.position = new Vector3(minX, 0, particle[i].transform.position.z);
            if (particle[i].transform.position.y < minX)
                particle[i].transform.position = new Vector3(particle[i].transform.position.x, 0, minX);
            // clamp x and z max positions
            if (particle[i].transform.position.x > maxX)
                particle[i].transform.position = new Vector3(maxX, 0, particle[i].transform.position.z);
            if (particle[i].transform.position.y > maxX)
                particle[i].transform.position = new Vector3(particle[i].transform.position.x, 0, maxX);

            float newFitness = (float)ObjectiveFunction(particle[i].transform.position);
            particle[i].GetComponent<PSOagent>().fitness = newFitness;

            if (newFitness < particle[i].GetComponent<PSOagent>().bestFitness)
            {
                particle[i].GetComponent<PSOagent>().bestPosition = particle[i].transform.position;
                particle[i].GetComponent<PSOagent>().bestFitness = newFitness;
            }
            if (newFitness < bestGlobalFitness)
            {
                bestGlobalPosition = particle[i].transform.position;
                bestGlobalFitness = newFitness;
            }
        }
    }

    void dynamic()
    {   // https://www.sciencedirect.com/science/article/pii/S2212017312005798 << this is literally what im doing, why is it "dynamic"??
        /* calculate average fitness
         * For each particle <
             * if fitness is less than average
             *      consider the particle (???)
             *      calculate fitness
             * if fitness value is new best
             *      set global fitness to new value
             *      set current bestfitness
         * >
         * choose particle with best fitness
         * for each particle
         *      calulcate velocities, update positions
         * do until max iterations
         */
    }
}

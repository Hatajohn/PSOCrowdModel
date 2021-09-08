using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Perception : MonoBehaviour
{
    //List<GameObject> seenObjects;
    public bool debug = false;
    GameObject cObj = null;
    GameObject cObj_g = null;
    public float minDist;
    public float minDist_g;

    // Start is called before the first frame update
    void Start()
    {
    }

    // Update is called once per frame
    void Update()
    {
        RaycastHit hit;
        Physics.Raycast(transform.position, transform.forward, out hit, 100);
        //arbitrary large numbers to force reset
        minDist = 99999;
        minDist_g = 99999;
        GameObject[] gameObjects = GameObject.FindGameObjectsWithTag("pedestrian");
        float angle;
        Vector3 targetDir;

        if (debug)
        {
            drawView(60.0f);
            Debug.DrawRay(this.transform.position, transform.forward * hit.distance, Color.yellow, 0.01f);
        }

            foreach (GameObject obj in gameObjects)
        {
            if (obj.transform != this.transform)
            {
                targetDir = obj.transform.position - this.transform.position;
                angle = Vector3.Angle(targetDir, this.transform.forward);
                // All GameObject in the FoV (120 degrees)
                if (angle < 60.0f)
                {
                    if(debug)
                        Debug.DrawRay(this.transform.position, targetDir, Color.red, 0.01f);
                    float dist = Vector3.Distance(this.transform.position, obj.transform.position);
                    if (dist < minDist)
                    {
                        //print("NEW MIN DISTANCE = " + dist);
                        minDist = dist;
                        cObj = obj;
                    }
                }
                // length of vector to get distance
                float gDist = (targetDir).sqrMagnitude;
                if (gDist < minDist_g)
                {
                    cObj_g = obj;
                    minDist_g = gDist;
                }
            }
        }
        // Closest GameObject within viewpoint
        if (cObj != null)
        {
            targetDir = cObj.transform.position - this.transform.position;
            angle = Vector3.Angle(targetDir, this.transform.forward);
            if(angle < 60.0f)
                if (debug)
                    Debug.DrawRay(this.transform.position, targetDir, Color.blue, 0.01f);
        }
        // Closest global GameObject
        if (cObj_g != null)
        {
            targetDir = cObj_g.transform.position - this.transform.position;
            /*
            if (debug)
                Debug.DrawRay(this.transform.position, targetDir, Color.green, 0.01f);
            */
        }

        /*        numSeen = gameObjects.Length;
                if (numSeen != numRemembered)
                {
                    print("Pedestrian count = " + numSeen);
                }
                //if(numFront != oldFront)
                //{
                    print("Visible in front of agent count = " + numFront);
                //}
                numRemembered = numSeen;*/
    }

    void drawView(float angle)
    {
        //Draws the FoV using a raycast 60 degrees left and right of the forward facing direction
        float a = angle * Mathf.Deg2Rad;
        Vector3 dirR = (transform.forward * Mathf.Cos(a) + transform.right * Mathf.Sin(a)).normalized;
        Vector3 dirL = (transform.forward * Mathf.Cos(a) - transform.right * Mathf.Sin(a)).normalized;

        RaycastHit rightEdge;
        Physics.Raycast(transform.position, dirR, out rightEdge, 100);
        Debug.DrawRay(this.transform.position, dirR * rightEdge.distance, Color.yellow, 0.01f);

        RaycastHit leftEdge;
        Physics.Raycast(transform.position, dirL, out leftEdge, 100);
        Debug.DrawRay(this.transform.position, dirL * leftEdge.distance, Color.yellow, 0.01f);
    }
}

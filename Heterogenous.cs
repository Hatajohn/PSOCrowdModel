using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;

public class Heterogenous : MonoBehaviour
{
    // Calibrate with wheelchair data
    public float min_rad = 0.381f;
    public float max_rad = 1.3f;
    public float mass = 70.0f;
    public float mass_offset = 10.0f;
    public float minSpeed = 4.0f;
    public float maxSpeed = 7.0f;
    public float maxAccel = 6.0f;
    public float minAccel = 6.0f;
    public float min_fov = 60.0f;
    public float max_fov = 60.0f;
    public float min_dist = 50.0f;
    public float max_dist = 50.0f;
    public bool dp = false;
    public int panic = 90;

    //15.8kg wheelchair
    // Start is called before the first frame update
    void Start()
    {
        // Radius
        float rad = Random.Range(min_rad, max_rad);
        transform.localScale = new Vector3(rad, 1, rad);
        //Separate values between dp and ndp
        if (!dp)
        {
            // ndp radius
            gameObject.GetComponent<NavMeshAgent>().radius = rad;
            // ndp speed
            gameObject.GetComponent<NavMeshAgent>().speed = Random.Range(minSpeed, maxSpeed);
            // ndp acceleration
            gameObject.GetComponent<NavMeshAgent>().acceleration = Random.Range(minAccel, maxAccel);
            // ndp turn speed
            gameObject.GetComponent<NavMeshAgent>().angularSpeed = 270.0f;

            // ndp mass
            gameObject.GetComponent<Rigidbody>().mass = Random.Range(mass - mass_offset, mass + mass_offset);
            // Collider radius
            gameObject.GetComponent<CapsuleCollider>().radius = rad + 0.05f;
        }
        else
        {
            //Fixed radius based on wheelchair
            gameObject.GetComponent<NavMeshAgent>().radius = 0.5334f;
            gameObject.GetComponent<NavMeshAgent>().speed = Random.Range(minSpeed, maxSpeed);
            gameObject.GetComponent<NavMeshAgent>().acceleration = Random.Range(minAccel, maxAccel) / 2.0f;
            gameObject.GetComponent<NavMeshAgent>().angularSpeed = 90.0f;

            // Extra mass from wheelchair based on
            float _mass = mass + Random.Range(15, 18);
            gameObject.GetComponent<Rigidbody>().mass = Random.Range(_mass - mass_offset, _mass + mass_offset);
            gameObject.GetComponent<CapsuleCollider>().radius = 0.5334f + 0.05f;
        }
        this.GetComponent<MoveTo>().setMaxView(Random.Range(min_fov, max_fov), Random.Range(min_dist, max_dist));
        this.GetComponent<MoveTo>().panic = (Random.Range(0, 100) < panic);
    }
}

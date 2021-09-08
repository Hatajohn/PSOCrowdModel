using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;

public class Shove : MonoBehaviour
{
    //public int collisions = 0;
    //NavMeshPath path;
    //float mag = 5.0f;
    private float speed = -1.0f;
    private float c_speed;
    private int collisions = 1;
    private void OnTriggerEnter(Collider c)
    {
        //Debug.LogError("Cool!");
        if (c.gameObject.tag == "pedestrian" || c.gameObject.tag == "Wall")
        {
            this.collisions += 1;
            if (this.speed == -1.0f)
            {
                this.speed = gameObject.GetComponent<NavMeshAgent>().speed;
                this.c_speed = speed;
            }
            if (this.c_speed > this.speed / 10)
            {
                c_speed /= collisions;
                this.gameObject.GetComponent<NavMeshAgent>().speed = c_speed;
            }
            /*collisions += 1;
            GameObject obj = collision.gameObject;
            NavMeshAgent b = obj.GetComponent<NavMeshAgent>();
            NavMeshAgent a = GetComponent<NavMeshAgent>();
            if (a.enabled && b.enabled)
            {
                a.enabled = false;
                b.enabled = false;
            }
            float other_mass = obj.GetComponent<Rigidbody>().mass;
            float mass = GetComponent<Rigidbody>().mass;
            Debug.Log("A and B : " + mass + ", " + other_mass);
            Vector3 _m = (mass * a.velocity) + (other_mass * b.velocity);
            Vector3 _v = _m / (mass + other_mass);
            Vector3 dir = (collision.gameObject.transform.position - transform.position).normalized;*/
            /*if ((mass * a.velocity).magnitude > (other_mass * b.velocity).magnitude)
            {   
                obj.GetComponent<Rigidbody>().velocity = new Vector3(-mag * _v.x, 0, -mag * _v.z);
                gameObject.GetComponent<Rigidbody>().velocity = new Vector3(mag * _v.x, 0, mag * _v.z);
                Debug.Log("Poosh 1 : " + _v.x + ", " + _v.z);
            }
            else
            {
                obj.GetComponent<Rigidbody>().velocity = new Vector3(mag * _v.x, 0, mag * _v.z);
                gameObject.GetComponent<Rigidbody>().velocity = new Vector3(mag * _v.x, 0, mag * _v.z);
                Debug.Log("Poosh 2 : " + _v.x + ", " + _v.z);
            }*/
            /*obj.GetComponent<Rigidbody>().velocity = new Vector3(mag * _v.x, 0, mag * _v.z);
            Debug.Log("Colliding!");*/
        }
    }

    private void OnTriggerExit(Collider c)
    {
        if (c.gameObject.tag == "pedestrian" || c.gameObject.tag == "Wall")
        {
            gameObject.GetComponent<NavMeshAgent>().speed = speed;
            //collision.gameObject.GetComponent<Rigidbody>().velocity = new Vector3(0,0,0);
            //collision.gameObject.GetComponent<NavMeshAgent>().enabled = true;
            //GetComponent<NavMeshAgent>().enabled = true;
            //GetComponent<NavMeshAgent>().enabled = true;
            //collision.gameObject.GetComponent<NavMeshAgent>().enabled = true;
            /*GameObject obj = collision.gameObject;
            NavMeshPath path = new NavMeshPath();
            Transform d = obj.GetComponent<MoveTo>().dest;
            NavMesh.CalculatePath(d.position, d.position, NavMesh.AllAreas, path);
            gameObject.GetComponent<NavMeshAgent>().velocity = v;
            obj.GetComponent<NavMeshAgent>().SetPath(path);*/
        }
    }
}

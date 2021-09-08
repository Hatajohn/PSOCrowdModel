using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class WallForce : MonoBehaviour
{
    public float force = 5.0f;
    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    private void OnCollisionEnter(Collision collision)
    {
        GameObject g = collision.gameObject;
        if (g.gameObject.tag.Equals("pedestrian"))
        {
            g.GetComponent<Rigidbody>().AddForce(collision.contacts[0].normal * force, ForceMode.VelocityChange);
            Debug.DrawRay(collision.contacts[0].point, collision.contacts[0].normal * 100, Random.ColorHSV(0f, 1f, 1f, 1f, 0.5f, 1f), 10f);
        }
    }
}

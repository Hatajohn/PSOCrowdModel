using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PreventCol : MonoBehaviour
{
    public bool inCol = false;
    float angle;
    private Vector3 _dir;
    public Vector3 dir;
    Vector3 normal;
    static Rigidbody r;
    private void Awake()
    {
        r = GetComponent<Rigidbody>();
    }
    // Start is called before the first frame update
    void Start()
    {
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    void OnCollisionEnter(Collision collision)
    {
        GameObject obj = collision.gameObject;
        if (obj.tag.Equals("Wall"))
        {
            inCol = true;
            foreach (var item in collision.contacts)
            {
                if (Vector3.Angle(-item.normal, r.velocity.normalized) < 90)
                {
                    //obj.GetComponent<Rigidbody>().velocity -= obj.GetComponent<Rigidbody>().velocity.magnitude * -item.normal;
                    _dir = Vector3.Cross(item.normal, Vector3.up).normalized;
                    angle = Mathf.Min(Vector3.Angle(_dir, r.velocity), Vector3.Angle(-_dir, r.velocity));
                    if(Vector3.Angle(_dir, r.velocity) < Vector3.Angle(-_dir, r.velocity)){
                        dir = _dir;
                    }
                    else
                    {
                        dir = -_dir;
                    }
                    Debug.DrawRay(transform.position, Mathf.Cos(angle) * r.velocity.normalized * 10, Color.magenta, 0.001f);
                    dir *= r.velocity.magnitude;
                    /*
                    Vector3 horizontal = Mathf.Cos(angle) * r.velocity;
                    r.velocity -= horizontal;*/
                    Debug.DrawRay(item.point, item.normal * 10, Color.green, 0.001f);
                }
                normal = -item.normal;
            }
        }
    }

    private void OnCollisionStay(Collision collision)
    {
        GameObject obj = collision.gameObject;
        if (obj.tag.Equals("Wall"))
        {
            //inCol = true;
            foreach (var item in collision.contacts)
            {
                if (Vector3.Angle(-item.normal, r.velocity.normalized) < 90)
                {
                    _dir = Vector3.Cross(item.normal, Vector3.up).normalized;
                    angle = Mathf.Min(Vector3.Angle(_dir, r.velocity), Vector3.Angle(-_dir, r.velocity));
                    if (Vector3.Angle(_dir, r.velocity) < Vector3.Angle(-_dir, r.velocity))
                    {
                        dir = _dir;
                    }
                    else
                    {
                        dir = -_dir;
                    }
                    Debug.DrawRay(transform.position, Mathf.Cos(angle) * r.velocity.normalized * 10, Color.magenta, 0.001f);
                    dir *= r.velocity.magnitude;
                    /*
                    Vector3 horizontal = Mathf.Cos(angle) * r.velocity;
                    r.velocity -= horizontal;*/
                    Debug.DrawRay(item.point, item.normal * 10, Color.green, 0.001f);
                }
            }
            r.AddForce(normal * 5, ForceMode.VelocityChange);
        }
    }

    private void OnCollisionExit(Collision collision)
    {
        GameObject obj = collision.gameObject;
        if (obj.tag.Equals("Wall"))
        {
            inCol = false;
        }
    }
}

using System.Collections;
using System.Collections.Generic;
using System;
using UnityEngine;

public class SpeedOnStart : MonoBehaviour
{
    public bool debug = false;

    UnityEngine.AI.NavMeshAgent me;
    Vector3 _pos;
    float dSpeed;
    float patience;
    float _r;
    // Start is called before the first frame update
    void Start()
    {
        me = this.GetComponent<UnityEngine.AI.NavMeshAgent>();
        me.speed = Math.Min(3.0f, UnityEngine.Random.value * 6.0f); 
        //me.speed = 5.0f;
        dSpeed = me.speed;
        me.radius = Math.Min(1.0f, UnityEngine.Random.value * 3.0f);
        //me.radius = 1.5f;
        //patience = Math.Min(0.2f, UnityEngine.Random.value);
        patience = 0.01f;
        _r = me.radius;
        _pos = this.transform.position;
    }

    // Update is called once per frame
    void Update()
    {
        Vector3 pos = this.transform.position;
        float v = Vector3.Magnitude(me.velocity);
        if (v < dSpeed/2 && me.radius > 0.5f)
        {
            me.radius = Math.Max(0.5f, me.radius - patience);
        }
        else
        {
            //reset the radius
            me.radius = Math.Min(_r, me.radius + patience);
        }

        if (debug)
        {
            Debug.Log(me.radius + ", " + v);
        }
    }
}

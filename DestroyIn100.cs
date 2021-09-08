using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class DestroyIn100 : MonoBehaviour
{
    GameObject p;
    private int time = 0;
    bool swapped = false;
    // Start is called before the first frame update
    void Start()
    {
        p = this.transform.parent.gameObject;
        if (p == null)
            Destroy(this);
    }

    // Update is called once per frame
    void Update()
    {
        if(!swapped && time >= 1500)
        {
            p.tag = "notPedestrian";
        }else
            time++;
    }
}

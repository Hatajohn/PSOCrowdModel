using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO;
using UnityEngine.SceneManagement;

public class PSORecorder : MonoBehaviour
{
    //Agent id maps to 
    Dictionary<int, Dictionary<float, Vector3>> data;
    // Start is called before the first frame update
    void Start()
    {
        data = new Dictionary<int, Dictionary<float, Vector3>>();
    }

    public void storeData(int id, float t, Vector3 pos)
    {
        Debug.LogWarning("LESSS GOOOOO" + id);
        //agent id, time stamp, position
        if (!data.ContainsKey(id))
            data.Add(id, new Dictionary<float, Vector3>());
        if(!data[id].ContainsKey(t))
            data[id].Add(t, pos);
    }

    public void saveFile(string filename)
    {
        StreamWriter writer = new StreamWriter("Assets/Resources/Data/"+filename, true);
        for (int key = 0; key < data.Count; key++)
        {
            writer.WriteLine("Agent " + key);
            string line = "";
            int i = 0;
            foreach(float f in data[key].Keys)
            {
                line += f + " : " + data[key][f];
                if (i < data[key].Count - 1)
                    line += ", ";
                i++;
            }
            writer.WriteLine(line);
        }

        SceneManager.LoadScene("Experimental");
    }
}

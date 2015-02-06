using UnityEngine;
using System.Collections;

public class AutoRotate : MonoBehaviour
{

    public Space space = Space.World;
    public Vector3 speed = Vector3.zero;

    // Update is called once per frame
    void Update()
    {
        transform.Rotate(speed * Time.deltaTime, space);
    }
}

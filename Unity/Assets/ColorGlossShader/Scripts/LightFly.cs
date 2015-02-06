using UnityEngine;
using System.Collections;

public enum Axis
{
    xy, xz, yz,
}
public class LightFly : MonoBehaviour
{

    public float range = 1f;
    public float speed = 1f;
    public Axis axis = Axis.xz;

    private Vector3 posOrg = Vector3.zero;
    private float angle = 0f;

    // Use this for initialization
    void Start()
    {
        posOrg = transform.position;
    }

    // Update is called once per frame
    void Update()
    {
        Vector3 pos = posOrg;
        angle += Time.deltaTime * speed;
        float rad = Mathf.Deg2Rad * angle;
        Vector2 offset = new Vector2(Mathf.Sin(rad) * range, Mathf.Cos(rad) * range);
        if (axis == Axis.xy)
        {
            pos.x += offset.x;
            pos.y += offset.y;
        }
        else if (axis == Axis.xz)
        {
            pos.x += offset.x;
            pos.z += offset.y;
        }
        else
        {
            pos.y += offset.x;
            pos.z += offset.y;
        }
        transform.position = pos;
    }
}

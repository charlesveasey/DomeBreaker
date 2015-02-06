using UnityEngine;
using System.Collections;

public class Shooter : MonoBehaviour {

	public Rigidbody projectile;
	public Transform shotPos;
	public float shotForce = 1000f;
	public float moveSpeed = 10f;
	public Transform target;
	public Transform domeCamera;
	private float rx = 0;
	private float ry = 0;


	void FixedUpdate () {

		Ray ray = Camera.main.ScreenPointToRay(Input.mousePosition);

		RaycastHit hit;
		
		if (Physics.Raycast (ray, out hit, 100f)) {
			//Debug.DrawLine (ray.origin, hit.point);
			//Debug.DrawLine( ray.origin, (ray.direction - Camera.main.transform.position) );
			//target.transform.position = new Vector3(ray.origin.x, ray.origin.y, target.transform.position.z);
		}

		float v = Input.GetAxis ("Horizontal") * Time.deltaTime * moveSpeed;
		float h = Input.GetAxis ("Vertical") * Time.deltaTime * moveSpeed;
	
		//target.transform.position = new Vector3 (transform.position.x, 0, transform.position.y);

		//target.Translate (new Vector3 (h, v, 0));

		//float px = target.localPosition.x;
		//float py = target.localPosition.y;

		//float rx = h;
		//float rz = map (py, -1f, 1f, 90, -90f);

		//shotPos.rotation.eulerAngles.x

		rx += h;
		ry += v;

		//shotPosP.localRotation = Quaternion.Euler(0, 0, ry);
		shotPos.localRotation = Quaternion.Euler(rx+270, ry+90, 0);
		//shotPos.Rotate (new Vector3 (0, 0, ry));

		if (Input.GetButtonUp ("Fire1")) {
			Rigidbody shot = Instantiate (projectile, shotPos.position, shotPos.rotation) as Rigidbody;	
			shot.AddForce(shotPos.forward * shotForce);
		}

	}

	
	float map(float s, float a1, float a2, float b1, float b2){
		return b1 + (s-a1)*(b2-b1)/(a2-a1);
	}

	Vector3 PolarToCartesian(Vector2 polar){
		Vector3 origin = new Vector3(0,0,1);
		//build a quaternion using euler angles for lat,lon
		Quaternion rotation = Quaternion.Euler(polar.x,polar.y,0);
		//transform our reference vector by the rotation. Easy-peasy!
		Vector3 point = rotation*origin;
		
		return point;
	}

}

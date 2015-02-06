using UnityEngine;
using System.Collections;
using System.Collections.Generic;

public class ProjectileCollide : MonoBehaviour {

	public bool hasCollided = false;
	public Texture texture;
	public Texture texture2;

	private int hitCnt = 0;
	private float time;
	private bool aBreak = false;
	private ShatterOnCollision shatterOnCollision;
	private ShatterTool shatterTool;
	private Rigidbody rigidbody;

	void Start(){
		shatterOnCollision = gameObject.GetComponent<ShatterOnCollision>();
		shatterTool = gameObject.GetComponent<ShatterTool>();
		rigidbody = gameObject.GetComponent<Rigidbody>();
		shatterOnCollision.enabled = false;
		shatterTool.enabled = false;
		rigidbody.constraints = RigidbodyConstraints.FreezeAll;
	}

	void OnCollisionEnter(Collision collision) {
		renderer.material.mainTexture = texture2;
		time = Time.unscaledTime;
		hasCollided = true;
		//OSCHandler.Instance.SendMessageToClient("SoundControl", "address/folder",50);
			List<object> values = new List<object>(); values.AddRange(new object[]{gameObject.tag, collision.transform.position.x, collision.transform.position.y, collision.transform.position.z}); 
		OSCHandler.Instance.SendMessageToClient ("SoundControl", "address/folder", values);
	}

	void Update(){
		if (hasCollided && Time.unscaledTime - time >= .5f) {
			hasCollided = false;
			AllowBreak ();
			//if (!audio.isPlaying)
			//audio.Play();
		}
	}

	public void AllowBreak(){
		print ("allow break");
		aBreak = true;
		shatterOnCollision.enabled = true;
		shatterTool.enabled = true;
		rigidbody.constraints = RigidbodyConstraints.None;
	}

}

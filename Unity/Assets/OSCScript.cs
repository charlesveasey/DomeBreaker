using UnityEngine;
using System.Collections;

public class OSCScript : MonoBehaviour {

	// Use this for initialization
	void Start () {
		OSCHandler.Instance.Init ();
	}
	
	// Update is called once per frame
	void Update () {
	}
}

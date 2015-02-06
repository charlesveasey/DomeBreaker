using UnityEngine;
using System.Collections;
using Holoville.HOTween;
using Holoville.HOTween.Plugins;

public class TweenVal : object {
	public float value;
}

public class Main : MonoBehaviour {
	
	// dome structure
	public GameObject dome;
	private Component[] ProjectileCollides;

	// start 
	void Start () {

		//Screen.SetResolution(2048, 2048, false);
		Screen.showCursor = false;
		ProjectileCollides = GetComponentsInChildren<ProjectileCollide>();
	
	}
		
	// update loop
	void Update () {

	/*
		foreach (ProjectileCollide pc in ProjectileCollides) {
			if (!pc.hasCollided){
				return;
			}
		}
		foreach (ProjectileCollide pc in ProjectileCollides) {
			pc.AllowBreak();
		}
*/
	}





}
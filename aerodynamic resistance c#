using UnityEngine;
using System.Collections;

// This class calculates the aerodynamic friction of a car, 
// and is mainly responsible for adjusting a cars top speed.
public class AerodynamicResistance : MonoBehaviour {

	// Friction coefficients along the X, Y and Z-Axes of the car.
	// A higher Z-Friction coefficient will make a car slower.
	public Vector3 coefficients = new Vector3(3.0f, 4.0f, 0.5f);

	void Update () {
		// Velocity in local space.
		Vector3 localVelo = transform.InverseTransformDirection(rigidbody.velocity);
		
		// Strip signs.
		Vector3 absLocalVelo = new Vector3( Mathf.Abs(localVelo.x), Mathf.Abs(localVelo.y), Mathf.Abs(localVelo.z));
		
		// Calculate and apply aerodynamic force.
		Vector3 airResistance = Vector3.Scale( Vector3.Scale(localVelo, absLocalVelo), -2*coefficients);
		rigidbody.AddForce( transform.TransformDirection( airResistance ) );
	}
}

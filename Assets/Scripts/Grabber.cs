using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.XR;
using UnityEngine.Experimental.Input;
using InputDevice = UnityEngine.XR.InputDevice;

public class Grabber : MonoBehaviour
{
    public XRNode inputSource;
    public Rigidbody anchor;

    public HashSet<Rigidbody> collidedBodies = new HashSet<Rigidbody>();
    private List<HingeJoint> joints = new List<HingeJoint>();

    private InputDevice device;

    private void Awake()
    {
        device = InputDevices.GetDeviceAtXRNode(XRNode.RightHand);
        Debug.Log("nuts");
    }

    // Update is called once per frame
    void Update()
    {
        

        if (Input.GetKeyDown(inputSource == XRNode.LeftHand ? KeyCode.JoystickButton14 : KeyCode.JoystickButton15))
        {

            Debug.Log($"Enter:");
            joints = collidedBodies.Select(b => b.gameObject.AddComponent<HingeJoint>()).ToList();
            foreach (var fixedJoint in joints)
            {
                fixedJoint.connectedBody = anchor;
            }
        }


        if (Input.GetKeyUp(inputSource == XRNode.LeftHand ? KeyCode.JoystickButton14 : KeyCode.JoystickButton15))
        {
            foreach (var joint in joints)
            {
                Destroy(joint);
            }
            joints.Clear();
        }
    }

    private void OnCollisionEnter(Collision other)
    {
        if (other.rigidbody)
        {
            device.SendHapticImpulse(0, 0.5f);
            collidedBodies.Add(other.rigidbody);
        }
    }

    private void OnCollisionExit(Collision other)
    {
        collidedBodies.Remove(other.rigidbody);
    }
}

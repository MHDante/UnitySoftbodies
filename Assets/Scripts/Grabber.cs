using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.SpatialTracking;
using UnityEngine.XR;
using InputDevice = UnityEngine.XR.InputDevice;

[RequireComponent(typeof(TrackedPoseDriver), typeof(Rigidbody))]
public class Grabber : MonoBehaviour
{

    public HashSet<Rigidbody> collidedBodies = new HashSet<Rigidbody>();
    private List<FixedJoint> joints = new List<FixedJoint>();

    private InputDevice device;
    private TrackedPoseDriver tpd;
    private Rigidbody anchor;

    private void Awake()
    {
        tpd = GetComponent<TrackedPoseDriver>();
        anchor = GetComponent<Rigidbody>();
        device = InputDevices.GetDeviceAtXRNode(XRNode.RightHand);
    }

    // Update is called once per frame
    void Update()
    {
        var butt = tpd.poseSource == TrackedPoseDriver.TrackedPose.LeftPose
            ? KeyCode.JoystickButton14
            : KeyCode.JoystickButton15;



        if (Input.GetKeyDown(butt))
        {

            joints = collidedBodies.Select(b => b.gameObject.AddComponent<FixedJoint>()).ToList();
            foreach (var fixedJoint in joints)
            {
                fixedJoint.connectedBody = anchor;
            }
        }


        if (Input.GetKeyUp(butt))
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

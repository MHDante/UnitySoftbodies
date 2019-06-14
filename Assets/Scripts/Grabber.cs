using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.SpatialTracking;
using UnityEngine.XR;
using InputDevice = UnityEngine.XR.InputDevice;

[RequireComponent(typeof(TrackedPoseDriver), typeof(Rigidbody))]
public class Grabber : MonoBehaviour
{

    public HashSet<Rigidbody> CollidedBodies = new HashSet<Rigidbody>();
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

    
    // ReSharper disable once UnusedMember.Local - Useful for debugging.
    void DrawPt(Vector3 gi, Color? color = null)
    {
        var c = color ?? Color.white;
        Debug.DrawLine(gi, gi + new Vector3(0, 0, .1f), c);
        Debug.DrawLine(gi, gi + new Vector3(0, .1f, 0), c);
        Debug.DrawLine(gi, gi + new Vector3(.1f, 0, 0), c);

    }

    // Update is called once per frame
    void Update()
    {
        
        var bounds = GetComponent<Collider>().bounds;

        DrawPt(bounds.min,Color.green);
        DrawPt(bounds.max,Color.red);

        var butt = tpd.poseSource == TrackedPoseDriver.TrackedPose.LeftPose
            ? KeyCode.JoystickButton14
            : KeyCode.JoystickButton15;

        if (Input.GetKeyDown(butt))
        {
            joints = CollidedBodies.Select(b => b.gameObject.AddComponent<FixedJoint>()).ToList();
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
            CollidedBodies.Add(other.rigidbody);
        }
    }

    private void OnCollisionExit(Collision other)
    {
        CollidedBodies.Remove(other.rigidbody);
    }
}

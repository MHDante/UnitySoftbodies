using UnityEngine;
using System.Collections;
using UnityEngine.EventSystems;

public class CameraMove : MonoBehaviour
{

    public float cameraSensitivity = 45;
    public float climbSpeed = 2;
    public float normalMoveSpeed = 5;
    public float slowMoveFactor = 0.5f;
    public float fastMoveFactor = 1.5f;

    private float rotationX = 0.0f;
    private float rotationY = 0.0f;

    void Start() { }

    void Update()
    {

        var t = transform;
        if (Input.GetMouseButton(1))
        {
            rotationX += Input.GetAxis("Mouse X") * cameraSensitivity * Time.deltaTime;
            rotationY += Input.GetAxis("Mouse Y") * cameraSensitivity * Time.deltaTime;
            rotationY = Mathf.Clamp(rotationY, -90, 90);
            
            transform.localRotation = Quaternion.AngleAxis(rotationX, Vector3.up) * Quaternion.AngleAxis(rotationY, Vector3.left);
        }

        if (Input.GetKey(KeyCode.LeftShift) || Input.GetKey(KeyCode.RightShift))
        {
            var position = t.position;
            position += t.forward * (normalMoveSpeed * fastMoveFactor) * Input.GetAxis("Vertical") *
                                  Time.deltaTime;
            position += transform.right * (normalMoveSpeed * fastMoveFactor) * Input.GetAxis("Horizontal") *
                                  Time.deltaTime;
            transform.position = position;
        }
        else if (Input.GetKey(KeyCode.LeftControl) || Input.GetKey(KeyCode.RightControl))
        {
            var position = t.position;
            position += t.forward * (normalMoveSpeed * slowMoveFactor) * Input.GetAxis("Vertical") *
                                  Time.deltaTime;
            position += transform.right * (normalMoveSpeed * slowMoveFactor) * Input.GetAxis("Horizontal") *
                                  Time.deltaTime;
            transform.position = position;
        }
        else
        {
            transform.position += transform.forward * normalMoveSpeed * Input.GetAxis("Vertical") * Time.deltaTime;
            transform.position += transform.right * normalMoveSpeed * Input.GetAxis("Horizontal") * Time.deltaTime;
        }


        if (Input.GetKey(KeyCode.Q)) { transform.position += transform.up * climbSpeed * Time.deltaTime; }
        if (Input.GetKey(KeyCode.E)) { transform.position -= transform.up * climbSpeed * Time.deltaTime; }


    }
}
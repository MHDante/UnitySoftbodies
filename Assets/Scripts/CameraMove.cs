using UnityEngine;

public class CameraMove : MonoBehaviour
{
    public float CameraSensitivity = 45;
    public float ClimbSpeed = 2;
    public float NormalMoveSpeed = 5;
    public float SlowMoveFactor = 0.5f;
    public float FastMoveFactor = 1.5f;

    private float rotationX = 0.0f;
    private float rotationY = 0.0f;
    

    void Update()
    {

        var t = transform;
        if (Input.GetMouseButton(1))
        {
            rotationX += Input.GetAxis("Mouse X") * CameraSensitivity * Time.deltaTime;
            rotationY += Input.GetAxis("Mouse Y") * CameraSensitivity * Time.deltaTime;
            rotationY = Mathf.Clamp(rotationY, -90, 90);
            
            transform.localRotation = Quaternion.AngleAxis(rotationX, Vector3.up) * Quaternion.AngleAxis(rotationY, Vector3.left);
        }

        if (Input.GetKey(KeyCode.LeftShift) || Input.GetKey(KeyCode.RightShift))
        {
            var position = t.position;
            position += t.forward * (NormalMoveSpeed * FastMoveFactor) * Input.GetAxis("Vertical") *
                                  Time.deltaTime;
            position += transform.right * (NormalMoveSpeed * FastMoveFactor) * Input.GetAxis("Horizontal") *
                                  Time.deltaTime;
            transform.position = position;
        }
        else if (Input.GetKey(KeyCode.LeftControl) || Input.GetKey(KeyCode.RightControl))
        {
            var position = t.position;
            position += t.forward * (NormalMoveSpeed * SlowMoveFactor) * Input.GetAxis("Vertical") *
                                  Time.deltaTime;
            position += transform.right * (NormalMoveSpeed * SlowMoveFactor) * Input.GetAxis("Horizontal") *
                                  Time.deltaTime;
            transform.position = position;
        }
        else
        {
            transform.position += transform.forward * NormalMoveSpeed * Input.GetAxis("Vertical") * Time.deltaTime;
            transform.position += transform.right * NormalMoveSpeed * Input.GetAxis("Horizontal") * Time.deltaTime;
        }


        if (Input.GetKey(KeyCode.Q)) { transform.position += transform.up * ClimbSpeed * Time.deltaTime; }
        if (Input.GetKey(KeyCode.E)) { transform.position -= transform.up * ClimbSpeed * Time.deltaTime; }


    }
}
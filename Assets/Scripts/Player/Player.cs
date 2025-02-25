using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Player : MonoBehaviour
{
    [Header("Cam settings")]
    [SerializeField] private Camera _cam;

    [Header("Movement settings")]
    [SerializeField] private float _movementSpeed;
    [SerializeField] private float _rotationSpeed;

    [Header("Jump")]
    [SerializeField] private Transform _jumpCheckPoint;
    [SerializeField] private float _jumpCheckerRadius = 0.3f;
    [SerializeField] private LayerMask _jumpLayer;
    [SerializeField] private float _jumpForce = 2f;

    private float _horizontalAngle;
    private float _verticalAngle;

    private bool _canSitInCar;

    private Rigidbody _rb;

    private void Awake()
    {
        _rb = GetComponent<Rigidbody>();
    }
    void Start()
    {
        Cursor.visible = false;
        Cursor.lockState = CursorLockMode.Locked;
    }

    void Update()
    {
        Movement();
        Rotation();
        Jump();
        SwitchState();
    }

    private void Movement()
    {
        float H = Input.GetAxis("Horizontal");
        float V = Input.GetAxis("Vertical");

        transform.position += (transform.forward * V + transform.right * H) * Time.deltaTime * _movementSpeed;
    }

    private void Rotation()
    {
        float mouseX = Input.GetAxis("Mouse X");
        float mouseY = Input.GetAxis("Mouse Y");

        _horizontalAngle += mouseX * _rotationSpeed * Time.deltaTime;
        _verticalAngle -= mouseY * _rotationSpeed * Time.deltaTime;

        _verticalAngle = Mathf.Clamp(_verticalAngle, -80f, 80f);

        Quaternion rotation = Quaternion.Euler(_verticalAngle, _horizontalAngle, 0);
        transform.rotation = Quaternion.Euler(0, _horizontalAngle, 0);
        _cam.transform.rotation = rotation;
    }

    private void Jump()
    {
        bool isGrounded = Physics.CheckSphere(_jumpCheckPoint.position, _jumpCheckerRadius, _jumpLayer);

        if(isGrounded && Input.GetKeyDown(KeyCode.Space))
        {
            _rb.AddForce(transform.up * _jumpForce, ForceMode.Impulse);
        }
    }

    private void OnDrawGizmos()
    {
        Gizmos.color = Color.yellow;
        Gizmos.DrawSphere(_jumpCheckPoint.position, _jumpCheckerRadius);
    }

    public void SwitchState()
    {
        if(Input.GetKeyDown(KeyCode.F) && _canSitInCar)
        {
            gameObject.SetActive(!gameObject.activeSelf);
            FindObjectOfType<Car>().SwitchState(_canSitInCar);
        }
    }

    private void OnTriggerEnter(Collider other)
    {
        Car car = other.GetComponent<Car>();
        if (car != null)
        {
            _canSitInCar = true;
        }
    }
    private void OnTriggerExit(Collider other)
    {
        Car car = other.GetComponent<Car>();
        if (car != null)
        {
            _canSitInCar = false;
        }
    }

}

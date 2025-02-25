using NWH.WheelController3D;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Car : MonoBehaviour
{
    [SerializeField] private GameObject _camera;
    private CarController _carController;
    private bool _inCar;
    public bool InCar => _inCar;
    private void Awake()
    {
        _carController = GetComponent<CarController>();
    }
    
    public void SwitchState(bool state)
    {
        _inCar = state;
        _carController.enabled = _inCar;
        _camera.SetActive(_inCar);
    }

}

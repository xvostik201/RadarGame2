using System;
using System.Collections;
using System.Collections.Generic;
using Unity.VisualScripting;
using UnityEngine;

public class Missle : MonoBehaviour
{
    [SerializeField] private float _rotationSpeed = 5f;

    [SerializeField] private float _velocitySpeed = 4f;

    [SerializeField] private float _damage = 25f;

    [SerializeField] private AudioClip _lauchClip;

    private Radar _radar;

    private Transform _target;
    private bool _isActive;

    private Rigidbody _rb;

    private MissleHolder _missleHolder;
    private ParticleSystem _particleSystem;
    private TrailRenderer _trail;
    private AudioSource _audioSource;

    private void Awake()
    {
        _trail = GetComponentInChildren<TrailRenderer>();
        _particleSystem = GetComponentInChildren<ParticleSystem>();
        _radar = FindObjectOfType<Radar>();
        _rb = GetComponent<Rigidbody>();
        _missleHolder = GetComponentInParent<MissleHolder>();
    }
    private void Start()
    {
        _trail.enabled = false;
    }
    void Update()
    {
        if(_audioSource != null) _audioSource.transform.position = transform.position;
        Movement();
        BlipOnRadar();
    }

    private void BlipOnRadar()
    {
        if (_radar != null && _target != null)
        {
            Vector2 radarPos = _radar.WorldToRadarPosition(transform.position);
            _radar.CreateOrUpdateMissleBlip(radarPos, this);
        }
    }

    private void Movement()
    {
        if (_isActive) transform.position += transform.forward * _velocitySpeed * Time.deltaTime;
        if(_target != null)
        {
            Vector3 direction = _target.transform.position - transform.position;

            Quaternion targetRotation = Quaternion.LookRotation(direction);

            Quaternion angle = Quaternion.RotateTowards(
                transform.rotation,
                targetRotation,
                _rotationSpeed * Time.deltaTime);

            transform.rotation = Quaternion.Euler(angle.eulerAngles.x, angle.eulerAngles.y, 0);
        }
    }

    public void LaunchMissle(Transform target)
    {
        _audioSource = AudioManager.Instance.PlaySound(_lauchClip, transform.position, 150, 1f);
        _target = target;
        _isActive = true;
        transform.parent = null;
        _trail.enabled = true;
        _particleSystem.Play();
        _missleHolder.DeleteMissleFromHolder(this);
    }

    private void OnCollisionEnter(Collision collision)
    {
        Plane plane = collision.gameObject.GetComponentInParent<Plane>();
        if(plane != null)
        {
            plane.TakeDamage(_damage);
        }
        Debug.Log("Взрыв");
        gameObject.SetActive(false);
        _radar.DeleteMissleBlip(this);
    }
}

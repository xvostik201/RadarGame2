using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public class MissleHolder : MonoBehaviour
{
    [SerializeField] private Transform[] _rotationMechanisms;
    [SerializeField] private float _animateAngle = -160f;
    [SerializeField] private float _animateTime = 0.7f;
    [SerializeField] private float _waitTime = 0.3f;
    [SerializeField] private AudioClip _mechanismClip;
    private Missle[] _missles;
    private int _currentRocketIndex = 0;
    private AudioSource[] _audioSources;


    void Start()
    {
        _audioSources = new AudioSource[_rotationMechanisms.Length];
        _missles = GetComponentsInChildren<Missle>();
    }
    private void Update()
    {
        for(int i = 0; i < _rotationMechanisms.Length; i++)
        {
            if(_audioSources[i] != null)
            {
                _audioSources[i].transform.position = _rotationMechanisms[i].transform.position;
            }
        }
    }
    public void DeleteMissleFromHolder(Missle missle)
    {
        for(int i = 0; i < _missles.Length; i++)
        {
            if(_missles[i] == missle)
            {
                _missles[i] = null;
            }
        }
    }
    public bool HaveMissle()
    {
        bool have = false;
        for(int i = 0; i < _missles.Length; i++)
        {
            if (_missles[i] != null)
            {
                have = true;
            }
        }
        return have;
    }
    public void ActivateAnimationAndLauchMissle(Transform target)
    {
        if(_currentRocketIndex < _missles.Length) StartCoroutine(AnimateRotationMechanism(_animateTime, _currentRocketIndex, _waitTime, target));
        else Debug.Log("Нет ракет в " + gameObject.name);
    }
    private IEnumerator AnimateRotationMechanism(float animateTime, int rocketIndex, float waitTime, Transform target)
    {
        _audioSources[rocketIndex] = AudioManager.Instance.PlaySound(_mechanismClip, _rotationMechanisms[rocketIndex].position, 45f, 1f);
        _currentRocketIndex++;
        float animateTimer = 0;
        Quaternion initialRotation = _rotationMechanisms[rocketIndex].localRotation;
        Quaternion endRotation = initialRotation * Quaternion.Euler(_animateAngle, 0,0);

        while (animateTimer < animateTime)
        {
            animateTimer += Time.deltaTime;
            float t = Mathf.Clamp01(animateTimer / animateTime);
            _rotationMechanisms[rocketIndex].localRotation = Quaternion.Slerp(initialRotation, endRotation, t);
            yield return null;
        }

        yield return new WaitForSeconds(waitTime);

        _missles[rocketIndex].LaunchMissle(target);
    }

}

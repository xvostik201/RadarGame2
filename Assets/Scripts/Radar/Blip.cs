using System.Collections;
using System.Collections.Generic;
using System.Runtime.CompilerServices;
using UnityEngine;
using UnityEngine.UI;

public class Blip : MonoBehaviour
{
    public enum TypeOfBlip
    {
        Rocket,
        Plane
    }

    public TypeOfBlip BlipType;

    [Header("Outline")]
    [SerializeField] private Image _blipOutlineImage;

    [SerializeField] private Image _mainImage;

    [Header("Sprites")]
    [SerializeField] private Sprite[] _allBlipSprites;

    private Plane _blipPlane;

    private float _timeToEnabled = 2f;

    private Coroutine _timerCoroutine;

    private Radar _radar;

    private bool _isTracked;


    private Missle _missle;
    public Plane GetPlane => _blipPlane;
    public bool IsTracked => _isTracked;
    public Missle GetMissle => _missle;

    


    void Awake()
    {
        _radar = FindObjectOfType<Radar>();
        _blipOutlineImage.enabled = false;
    }

    private void Start()
    {
        _mainImage.sprite = GetSprite();
    }

    private void OnMouseEnter()
    {
        if (BlipType != TypeOfBlip.Plane) return;
        _blipOutlineImage.enabled = true;
    }

    private void OnMouseDown()
    {
        if (BlipType != TypeOfBlip.Plane) return;
        _radar.GetWayInfo(this);
        _isTracked = true;
    }

    private void OnMouseExit()
    {
        if (BlipType != TypeOfBlip.Plane) return;
        _blipOutlineImage.enabled = false;
    }
    public void ActivateBlip()
    {
        gameObject.SetActive(true);

        if (_timerCoroutine != null)
        {
            StopCoroutine(_timerCoroutine);
        }

        _timerCoroutine = StartCoroutine(DisableAfterTime());
    }

    private Sprite GetSprite()
    {
        switch (BlipType)
        {
            case TypeOfBlip.Rocket:
                return _allBlipSprites[1];
            case TypeOfBlip.Plane:
                return _allBlipSprites[0];
            default:
                return null;
        }
    }

    private IEnumerator DisableAfterTime()
    {
        yield return new WaitForSeconds(_timeToEnabled);
        gameObject.SetActive(false);
    }

    public void SetTimer(float value)
    {
        _timeToEnabled = value;
    }

    public void SetMissle(Missle missle, TypeOfBlip typeOfBlip)
    {
        _missle = missle;
        BlipType = typeOfBlip;
    }

    public void SetBlipPlane(Plane plane, TypeOfBlip typeOfBlip)
    {
        _blipPlane = plane;
        BlipType = typeOfBlip;
    }

    public void DeactivadeTracking()
    {
        _isTracked = false;
    }


}

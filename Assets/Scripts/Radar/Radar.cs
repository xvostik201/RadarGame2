using System;
using System.Collections;
using System.Collections.Generic;
using TMPro;
using UnityEngine;
using UnityEngine.UI;

public class Radar : MonoBehaviour
{
    [Header("Radar Raycast Settings")]
    [SerializeField] private int _raysCount = 15;
    [SerializeField] private float[] _radarRanges;
    [SerializeField] private float _rayYStep = 1;
    [SerializeField] private float _radarYOffset = 15;

    private float _currentRadarRange;

    [Header("Radar In Game Settings")]
    [SerializeField] private Transform _radarRotationTower;
    [SerializeField] private Transform _radar;
    [SerializeField] private List<float> _radarRotationSpeeds;

    private float _currentRadarRotationSpeed;

    [Header("In game sound")]
    [SerializeField] private Transform _radarDynamic;
    [SerializeField, Range(0, 1)] private float _blipVolume;
    [SerializeField] private float _3dSoundMaxBlipDistance = 25f;

    [Header("UI Radar Settings")]
    [SerializeField] private RectTransform _radarMain;
    [SerializeField] private RectTransform _radarArrow;

    [SerializeField] private TMP_Text _radarRangeText;
    [SerializeField] private TMP_Text _rotationSpeedText;
    [SerializeField] private TMP_Text _radarModeText;

    [Header("Range buttons")]
    [SerializeField] private GameButton[] _rangeButton;
    [SerializeField] private TMP_Text[] _rangeButtonText;

    [Header("Speed buttons")]
    [SerializeField] private GameButton[] _rotationSpeedButton;
    [SerializeField] private TMP_Text[] _rotationSpeedButtonText;

    [Header("Mode buttons")]
    [SerializeField] private GameButton[] _moveSwitchButtons;

    [Header("UI blip")]
    [SerializeField] private GameObject _blipPrefab;
    [SerializeField] private float _blipLifetime = 1.5f;
    [SerializeField] private float _blipSizeFactor = 2f;
    [SerializeField] private float _blipMinSize = 0.1f;
    [SerializeField] private float _blipMaxSize = 1f;

    [Header("SFX")]
    [SerializeField] private AudioClip _blipSound;

    [Header("Blip set info")]
    [SerializeField] private GameObject _blipRadarPointImage;
    [SerializeField] private LineRenderer _wayLineRenderer;

    [Header("Area scanner")]
    [SerializeField] private float _testAngle = 45f;
    private float _currentAngle;

    private bool _isRotationMode = true;

    private bool _isReversed = false;

    [Header("Ground to air missle")]
    [SerializeField] private Transform _missleRotationTowerPoint;
    [SerializeField] private Transform[] _missleHoldersRotationTowerPoint;
    [SerializeField] private float _missleRotationTowerRotationSpeed;
    [SerializeField] private MissleHolder[] _missleHolders;
    [SerializeField] private GameButton _launchMissleButton;

    private HashSet<Plane> _trackedPlanes = new HashSet<Plane>();
    private List<GameObject> _createdBlips = new List<GameObject>();
    private List<GameObject> _createdWayPointBlip = new List<GameObject>();
    private List<GameObject> _createdMissleBlips = new List<GameObject>();

    private bool _movingTowardsMax;

    private void Awake()
    {
        _currentRadarRange = _radarRanges[0];
        _currentRadarRotationSpeed = _radarRotationSpeeds[0];

        UpdateText();
        SetButtons();
    }

    void Start()
    {
        _blipPrefab.GetComponent<Blip>().SetTimer(_blipLifetime);
    }

    void Update()
    {
        RotateAndScan();
        MissleTowerRotation();
    }

    private void RotateAndScan()
    {
        if (_isRotationMode)
        {
            float sign = _isReversed ? -1f : 1f;
            _radarRotationTower.Rotate(0, sign * _currentRadarRotationSpeed * Time.deltaTime, 0);
        }
        else
        {
            OscillateAroundAngle();
        }

        _radarArrow.localRotation = Quaternion.Euler(0, 0, -_radarRotationTower.eulerAngles.y);

        Vector3 pos = transform.position;
        Vector3 direction = _radarRotationTower.forward;

        HashSet<Plane> currentlyDetectedPlanes = new HashSet<Plane>();

        for (int i = 0; i < _raysCount; i++)
        {
            Vector3 newPos = new Vector3(pos.x, pos.y + (i * _rayYStep + _radarYOffset), pos.z);
            if (Physics.Raycast(newPos, direction, out RaycastHit hitInfo, _currentRadarRange))
            {
                Plane plane = hitInfo.collider.GetComponentInParent<Plane>();
                if (plane != null)
                {
                    currentlyDetectedPlanes.Add(plane);

                    if (!_trackedPlanes.Contains(plane))
                    {
                        Debug.Log("Attention!!! Plane!!! - pos " + WorldToRadarPosition(hitInfo.point));

                        Vector2 planePos = WorldToRadarPosition(hitInfo.point);
                        Vector2 objectSize = GetObjectSize(hitInfo.collider);

                        plane.Positions.Add(hitInfo.point);

                        CreateOrUpdateBlip(planePos, objectSize, plane);

                        _trackedPlanes.Add(plane);
                    }
                }
                Debug.DrawRay(newPos, direction * hitInfo.distance, Color.green, Time.deltaTime);
            }
            else
            {
                Debug.DrawRay(newPos, direction * _currentRadarRange, Color.red, Time.deltaTime);
            }
        }

        _trackedPlanes.IntersectWith(currentlyDetectedPlanes);
    }

    private void OscillateAroundAngle()
    {
        float currentY = _radarRotationTower.localEulerAngles.y;
        if (currentY > 180f) currentY -= 360f; 

        float minAngle = _currentAngle - _testAngle;
        float maxAngle = _currentAngle + _testAngle;

        float targetAngle = _movingTowardsMax ? maxAngle : minAngle;

        float step = Mathf.Abs(_currentRadarRotationSpeed) * Time.deltaTime;
        float newY = Mathf.MoveTowards(currentY, targetAngle, step);

        _radarRotationTower.localRotation = Quaternion.Euler(0f, newY, 0f);

        if (Mathf.Approximately(newY, targetAngle))
        {
            _movingTowardsMax = !_movingTowardsMax;
        }
    }

    private Vector2 GetObjectSize(Collider collider)
    {
        Bounds bounds = collider.bounds;
        Vector2 size = bounds.size;
        return size;
    }

    public Vector2 WorldToRadarPosition(Vector3 worldPosition)
    {
        Vector3 delta = worldPosition - _radar.position;

        float dx = delta.x;
        float dz = delta.z;

        float normalizedX = dx / _currentRadarRange;
        float normalizedY = dz / _currentRadarRange;

        float radarWidth = _radarMain.rect.width;
        float radarHeightUI = _radarMain.rect.height;

        Vector2 radarPos = new Vector2(normalizedX * (radarWidth / 2), normalizedY * (radarHeightUI / 2));

        return radarPos;
    }

    private void CreateOrUpdateBlip(Vector2 pos, Vector2 size, Plane plane)
    {
        GameObject blip = GetBlip(plane);

        blip.SetActive(true);
        blip.GetComponent<Blip>().ActivateBlip();

        RectTransform blipRect = blip.GetComponent<RectTransform>();
        blipRect.anchoredPosition = pos;

        float radarSizeFactor = Mathf.Max(_radarMain.rect.width, _radarMain.rect.height); 
        float scaleFactor = radarSizeFactor / (_currentRadarRange * 2);

        Vector2 blipSize = size * scaleFactor * _blipSizeFactor; 
        blipSize.x = Mathf.Clamp(blipSize.x, _blipMinSize, _blipMaxSize);
        blipSize.y = Mathf.Clamp(blipSize.y, _blipMinSize, _blipMaxSize);
        blipRect.sizeDelta = blipSize;

        AudioManager.Instance.PlaySound(_blipSound, _radarDynamic.position, _3dSoundMaxBlipDistance, _blipVolume);
        
        Blip blipScr = blip.GetComponent<Blip>();
        if (blipScr.IsTracked)
        {
            GetWayInfo(blipScr);
        }
    }


    private GameObject GetBlip(Plane plane)
    {
        foreach (GameObject blipObj in _createdBlips)
        {
            if (blipObj == null) continue;

            Blip blipScr = blipObj.GetComponent<Blip>();
            if (blipScr.GetPlane == plane)
            {
                return blipObj;
            }
        }
        GameObject newBlip = Instantiate(_blipPrefab, _radarMain);
        newBlip.GetComponent<Blip>().SetBlipPlane(plane, Blip.TypeOfBlip.Plane);
        _createdBlips.Add(newBlip);
        return newBlip;
    }

    private void UpdateRadarButton(int index, bool isRangeButton)
    {
        if (isRangeButton)
        {
            _currentRadarRange = _radarRanges[index];
        }
        else
        {
            _currentRadarRotationSpeed = _radarRotationSpeeds[index];
        }
        UpdateText();
    }

    private void UpdateText()
    {
        _radarRangeText.text = _currentRadarRange.ToString() + " M";
        _rotationSpeedText.text = _currentRadarRotationSpeed.ToString() + " DPS";
        _radarModeText.text = _isRotationMode ? "Вращение" : "Колебание";
    }
    public void CreateOrUpdateMissleBlip(Vector2 pos, Missle missle)
    {
        GameObject blip = GetMissleBlip(missle);

        blip.SetActive(true);
        blip.GetComponent<Blip>().ActivateBlip();

        RectTransform blipRect = blip.GetComponent<RectTransform>();
        blipRect.anchoredPosition = pos;
        blipRect.rotation = Quaternion.Euler(0, 0, -missle.transform.rotation.eulerAngles.y);

        float radarSizeFactor = Mathf.Max(_radarMain.rect.width, _radarMain.rect.height);
        float scaleFactor = radarSizeFactor / (_currentRadarRange * 2);

        Vector2 blipSize = new Vector2(0.3f, 0.3f) * scaleFactor * _blipSizeFactor; 
        blipSize.x = Mathf.Clamp(blipSize.x, _blipMinSize, _blipMaxSize);
        blipSize.y = Mathf.Clamp(blipSize.y, _blipMinSize, _blipMaxSize);
        blipRect.sizeDelta = blipSize;
    }


    private GameObject GetMissleBlip(Missle missle)
    {
        foreach (GameObject blipObj in _createdMissleBlips)
        {
            if (blipObj == null) continue;

            Blip blipScr = blipObj.GetComponent<Blip>();
            if (blipScr.GetMissle == missle)
            {
                return blipObj;
            }
        }

        GameObject newBlip = Instantiate(_blipPrefab, _radarMain);
        newBlip.GetComponent<Blip>().SetMissle(missle, Blip.TypeOfBlip.Rocket);
        _createdMissleBlips.Add(newBlip);
        return newBlip;
    }

    private void SetButtons()
    {
        for (int i = 0; i < _rangeButton.Length; i++)
        {
            int index = i;
            _rangeButton[i].OnButtonPressAction += () => UpdateRadarButton(index, true);
            _rangeButtonText[i].text = _radarRanges[i].ToString() + " M";
        }
        for (int i = 0; i < _rotationSpeedButton.Length; i++)
        {
            int index = i;
            if (i < _rotationSpeedButton.Length - 1)
            {
                _rotationSpeedButton[i].OnButtonPressAction += () => UpdateRadarButton(index, false);
                _rotationSpeedButtonText[i].text = _radarRotationSpeeds[i].ToString() + " DPS";
            }
            else
            {
                _rotationSpeedButton[i].OnButtonPressAction += InvertValue;
                _rotationSpeedButtonText[i].text = "Инвертировать значение";
            }
        }
        for (int i = 0; i < _moveSwitchButtons.Length; i++)
        {
            _moveSwitchButtons[i].OnButtonPressAction += SwitchMode;
        }
    }

    private void InvertValue()
    {
        _isReversed = !_isReversed;  

        _radarArrow.localScale = new Vector3(_radarArrow.localScale.x * -1, 1, 1);

        UpdateText();
    }

    private void SwitchMode()
    {
        _isRotationMode = !_isRotationMode;
        _radarModeText.text = _isRotationMode ? "Вращение" : "Колебание";

        if (!_isRotationMode)
        {
            float eulerY = _radarRotationTower.eulerAngles.y;
            if (eulerY > 180f) eulerY -= 360f;
            _currentAngle = eulerY;

            _movingTowardsMax = true;
        }
    }

    public void GetWayInfo(Blip blip)
    {
        foreach (GameObject blipObj in _createdBlips)
        {
            Blip oldBlip = blipObj.GetComponent<Blip>();
            if (oldBlip != blip)
            {
                oldBlip.DeactivadeTracking();
                foreach (var blipList in _createdWayPointBlip)
                {
                    Destroy(blipList);
                }
                _createdWayPointBlip.Clear();
            }
        }
        int existingCount = _createdWayPointBlip.Count;
        int newCount = blip.GetPlane.Positions.Count;

        int advancedPoints = newCount - existingCount;
        for (int k = 0; k < advancedPoints; k++)
        {
            GameObject blipWayRadarObj = Instantiate(_blipRadarPointImage, _radarMain);
            _createdWayPointBlip.Add(blipWayRadarObj);
        }

        for (int i = 0; i < newCount; i++)
        {
            RectTransform blipRect = _createdWayPointBlip[i].GetComponent<RectTransform>();
            blipRect.anchoredPosition = WorldToRadarPosition(blip.GetPlane.Positions[i]);
        }

        _wayLineRenderer.positionCount = newCount;
        for (int i = 0; i < newCount; i++)
        {
            _wayLineRenderer.SetPosition(i, _createdWayPointBlip[i].transform.position);
        }
        SetMissleTarget();
    }

    private void MissleTowerRotation()
    {
        if (IsPlaneTracker())
        {
            Vector3 direction = GetTrackedPlane().transform.position - _missleRotationTowerPoint.position;

            Quaternion targetRotation = Quaternion.LookRotation(new Vector3(direction.x, 0, direction.z));
            Quaternion angleY = Quaternion.RotateTowards(
                _missleRotationTowerPoint.rotation,
                targetRotation,
                _missleRotationTowerRotationSpeed * Time.deltaTime
            );
            _missleRotationTowerPoint.localRotation = Quaternion.Euler(0, angleY.eulerAngles.y, 0);

            float targetXAngle = Mathf.Atan2(-direction.y, direction.magnitude) * Mathf.Rad2Deg;

            Quaternion targetXRotation = Quaternion.Euler(targetXAngle, 0, 0);

            foreach (Transform holder in _missleHoldersRotationTowerPoint)
            {
                holder.localRotation = Quaternion.RotateTowards(
                    holder.localRotation,
                    targetXRotation,
                    _missleRotationTowerRotationSpeed * Time.deltaTime
                );
            }
        }
        else
        {

        }
    }


    private bool IsPlaneTracker()
    {
        foreach (GameObject blip in _createdBlips)
        {
            Blip blipScr = blip.GetComponent<Blip>();
            if (blipScr.IsTracked && blipScr.GetPlane != null)
            {
                return true; 
            }
        }
        return false;
    }

    private Plane GetTrackedPlane()
    {
        foreach (GameObject blip in _createdBlips)
        {
            Blip blipScr = blip.GetComponent<Blip>();
            if (blipScr.IsTracked)
            {
                return blipScr.GetPlane;
            }
        }
        return null;
    }

    private void SetMissleTarget()
    {
        _launchMissleButton.OnButtonPressAction -= FindAndLauchMissle;
        _launchMissleButton.OnButtonPressAction += FindAndLauchMissle;
    }

    private void FindAndLauchMissle()
    {
        if (GetTrackedPlane() == null) return;
        for(int i = 0; i < _missleHolders.Length; i++)
        {
            if (_missleHolders[i].HaveMissle())
            {
                _missleHolders[i].ActivateAnimationAndLauchMissle(GetTrackedPlane().transform);
                break;
            }
        }
    }

    public void DeleteMissleBlip(Missle missle)
    {
        if (_createdMissleBlips.Contains(missle.gameObject))
        {
            _createdMissleBlips.Remove(missle.gameObject);
        }
    }
    public void DeletePlaneBlip(Plane plane)
    {
        if (_createdBlips.Contains(plane.gameObject))
        {
            _createdBlips.Remove(plane.gameObject);
        }
    }

}

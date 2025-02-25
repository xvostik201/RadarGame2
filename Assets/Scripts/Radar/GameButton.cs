using System;
using System.Collections;
using UnityEngine;

public class GameButton : MonoBehaviour
{
    public enum AnimationDirection { X, Y, Z }

    [SerializeField] private AnimationDirection _direction;
    [SerializeField] private float _animationTowardDistance = 0.1f;
    [SerializeField] private float _timeToAnimation = 0.5f;

    [SerializeField] private Color _exitColor;
    [SerializeField] private Color _downColor = Color.green;
    [SerializeField] private Color _enterColor;

    private MeshRenderer _meshRenderer;

    private Vector3 _animationPoint;
    private bool isAnimating = false;

    public Action OnButtonPressAction;

    private void Awake()
    {
        _meshRenderer = GetComponent<MeshRenderer>();
    }
    private void Start()
    {
        Vector3 pos = transform.position;
        switch (_direction)
        {
            case AnimationDirection.X:
                _animationPoint = new Vector3(pos.x - _animationTowardDistance, pos.y, pos.z);
                break;
            case AnimationDirection.Y:
                _animationPoint = new Vector3(pos.x, pos.y - _animationTowardDistance, pos.z);
                break;
            case AnimationDirection.Z:
                _animationPoint = new Vector3(pos.x, pos.y, pos.z - _animationTowardDistance);
                break;
        }
    }

    private void OnMouseEnter()
    {
        _meshRenderer.material.color = _enterColor;
    }
    private void OnMouseDown()
    {
        if (!isAnimating)
        {
            StartCoroutine(StartAnimation());
            OnButtonPressAction?.Invoke();
            _meshRenderer.material.color = _downColor;
        }
    }
    private void OnMouseExit()
    {
        _meshRenderer.material.color = _exitColor;
    }

    private IEnumerator MoveTo(Vector3 start, Vector3 end)
    {
        float elapsedTime = 0f;
        while (elapsedTime < _timeToAnimation)
        {
            float t = elapsedTime / _timeToAnimation;
            transform.position = Vector3.Lerp(start, end, t);
            elapsedTime += Time.deltaTime;
            yield return null;
        }
        transform.position = end;
    }

    private IEnumerator StartAnimation()
    {
        isAnimating = true;

        Vector3 originalPos = transform.position;
        Vector3 targetPos = _animationPoint;

        yield return StartCoroutine(MoveTo(originalPos, targetPos));
        yield return StartCoroutine(MoveTo(targetPos, originalPos));

        isAnimating = false;
    }
}

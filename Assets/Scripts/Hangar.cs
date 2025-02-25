using System.Collections;
using UnityEngine;

public class Hangar : MonoBehaviour
{
    [SerializeField] private Transform _door;
    [SerializeField] private float _moveTowardPositionY = 5f;
    [SerializeField] private GameButton _doorButton;
    [SerializeField] private float _animateTime = 5f;

    private Vector3 _startPos;
    private Vector3 _endPos;
    private Coroutine _currentCoroutine;

    void Start()
    {
        _startPos = _door.position;
        _endPos = _startPos + _door.forward * _moveTowardPositionY;
        _doorButton.OnButtonPressAction += ToggleDoor;
    }

    private void ToggleDoor()
    {
        Vector3 targetPos = Vector3.Distance(_door.position, _startPos) < Vector3.Distance(_door.position, _endPos)
            ? _endPos
            : _startPos;

        if (_currentCoroutine != null)
        {
            StopCoroutine(_currentCoroutine);
        }
        _currentCoroutine = StartCoroutine(DoorAnimation(targetPos));
    }

    private IEnumerator DoorAnimation(Vector3 targetPos)
    {
        float fullDistance = Vector3.Distance(_startPos, _endPos);

        while (Vector3.Distance(_door.position, targetPos) > 0.01f)
        {
            float step = (fullDistance / _animateTime) * Time.deltaTime;
            _door.position = Vector3.MoveTowards(_door.position, targetPos, step);
            yield return null;
        }
        _door.position = targetPos;
        _currentCoroutine = null;
    }
}

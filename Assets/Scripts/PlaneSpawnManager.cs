using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PlaneSpawnManager : MonoBehaviour
{
    [SerializeField] private Vector3 _minSpawnPos;
    [SerializeField] private Vector3 _maxSpawnPos;
    [SerializeField] private Transform _testPlane;
    void Start()
    {
        TestSpawnPos();
    }

    private void TestSpawnPos()
    {
        Vector3 spawnPos = new Vector3(Random.Range(_minSpawnPos.x, _maxSpawnPos.x),
                    Random.Range(_minSpawnPos.y, _maxSpawnPos.y),
                    Random.Range(_minSpawnPos.z, _maxSpawnPos.z));
        _testPlane.position = spawnPos;
    }

    void Update()
    {
        if (Input.GetKeyDown(KeyCode.R))
        {
            TestSpawnPos();
        }
    }
}

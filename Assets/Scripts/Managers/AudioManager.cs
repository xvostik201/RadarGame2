using UnityEngine;
using System.Collections;
using System.Collections.Generic;

public class AudioManager : MonoBehaviour
{
    public static AudioManager Instance;

    [SerializeField]
    private int poolSize = 35; 

    private List<AudioSource> audioSourcePool;

    void Awake()
    {
        if (Instance == null)
        {
            Instance = this;
            DontDestroyOnLoad(gameObject);
            InitializeAudioPool();
        }
        else
        {
            Destroy(gameObject);
        }
    }

    private void InitializeAudioPool()
    {
        audioSourcePool = new List<AudioSource>();

        for (int i = 0; i < poolSize; i++)
        {
            GameObject audioObject = new GameObject("PooledAudioSource_" + i);
            AudioSource audioSource = audioObject.AddComponent<AudioSource>();

            audioSource.spatialBlend = 1.0f;     
            audioSource.spatialize = true;       
            audioSource.spatializePostEffects = true; 
            audioSource.rolloffMode = AudioRolloffMode.Linear; 
            audioSource.minDistance = 1f;       
            audioSource.maxDistance = 150f;      

            audioObject.transform.parent = transform;
            audioSourcePool.Add(audioSource);
            audioObject.SetActive(false);
        }
    }

    private AudioSource GetAvailableAudioSource()
    {
        foreach (var source in audioSourcePool)
        {
            if (!source.isPlaying)
            {
                source.gameObject.SetActive(true);
                return source;
            }
        }

        GameObject audioObject = new GameObject("PooledAudioSource_" + audioSourcePool.Count);
        AudioSource audioSource = audioObject.AddComponent<AudioSource>();

        audioSource.spatialBlend = 1.0f;
        audioSource.spatialize = true;
        audioSource.spatializePostEffects = true;
        audioSource.rolloffMode = AudioRolloffMode.Logarithmic;
        audioSource.minDistance = 1f;
        audioSource.maxDistance = 50f;

        audioObject.transform.parent = transform;
        audioSourcePool.Add(audioSource);
        return audioSource;
    }

    public AudioSource PlaySound(AudioClip clip, Vector3 position, float distance, float volume = 1.0f, bool loop = false)
    {
        AudioSource source = GetAvailableAudioSource();
        source.transform.position = position;
        source.clip = clip;
        source.volume = volume;
        source.loop = loop;
        source.maxDistance = distance;
        source.Play();

        if (!loop)
        {
            StartCoroutine(DisableAudioSourceAfterPlay(source));
        }

        return source; 
    }

    private IEnumerator DisableAudioSourceAfterPlay(AudioSource source)
    {
        yield return new WaitWhile(() => source.isPlaying);
        source.gameObject.SetActive(false);
    }

    public void StopSound(AudioSource source)
    {
        if (source != null)
        {
            source.Stop();
            source.gameObject.SetActive(false);
        }
    }
    public AudioSource PlaySound2D(AudioClip clip, float volume = 1.0f)
    {
        AudioSource source = GetAvailableAudioSource();
        source.clip = clip;                  
        source.volume = volume;
        source.transform.position = Camera.main.transform.position;
        source.Play();                       
        StartCoroutine(DisableAudioSourceAfterPlay(source));
        return source;
    }


    public void PlayMusic(AudioClip musicClip, float volume = 1.0f, bool loop = true)
    {
        AudioSource musicSource = GetComponent<AudioSource>();
        if (musicSource == null)
        {
            musicSource = gameObject.AddComponent<AudioSource>();
        }

        musicSource.clip = musicClip;
        musicSource.volume = volume;
        musicSource.loop = loop;
        musicSource.spatialBlend = 0.0f;
        musicSource.spatialize = false;
        musicSource.Play();
    }

    public void StopMusic()
    {
        AudioSource musicSource = GetComponent<AudioSource>();
        if (musicSource != null)
        {
            musicSource.Stop();
        }
    }
}

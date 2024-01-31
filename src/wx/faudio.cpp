#ifndef NO_FAUDIO

// Application
#include "wxvbam.h"
#include <stdio.h>

// Interface
#include "../common/SoundDriver.h"

// FAudio
#include <FAudio.h>
#include <pthread.h>
#include <string>
#include <vector>

// Internals
#include "../System.h" // for systemMessage()
#include "../gba/Globals.h"

int GetFADevices(FAudio* fa, wxArrayString* names, wxArrayString* ids,
    const wxString* match)
{
    uint32_t hr;
    uint32_t dev_count = 0;
    hr = FAudio_GetDeviceCount(fa, &dev_count);

    if (hr != 0) {
        wxLogError(_("FAudio: Enumerating devices failed!"));
        return true;
    } else {
        FAudioDeviceDetails dd;

        for (uint32_t i = 0; i < dev_count; i++) {
            hr = FAudio_GetDeviceDetails(fa, i, &dd);

            if (hr != 0) {
                continue;
            } else {
                if (ids) {
                    ids->push_back((wchar_t*) dd.DeviceID);
                    names->push_back((wchar_t*) dd.DisplayName);
                } else if (*match == wxString((wchar_t*) dd.DeviceID))
                    return i;
            }
        }
    }

    return -1;
}

bool GetFADevices(wxArrayString& names, wxArrayString& ids)
{
    uint32_t hr;
    FAudio* fa = NULL;
    uint32_t flags = 0;
#ifdef _DEBUG
    flags = FAUDIO_DEBUG_ENGINE;
#endif
    hr = FAudioCreate(&fa, flags, FAUDIO_DEFAULT_PROCESSOR);

    if (hr != 0) {
        wxLogError(_("The FAudio interface failed to initialize!"));
        return false;
    }

    GetFADevices(fa, &names, &ids, NULL);
    FAudio_Release(fa);
    return true;
}

static int FAGetDev(FAudio* fa)
{
    if (gopts.audio_dev.empty())
        return 0;
    else {
        int ret = GetFADevices(fa, NULL, NULL, &gopts.audio_dev);
        return ret < 0 ? 0 : ret;
    }
}

class FAudio_Output;

static void faudio_device_changed(FAudio_Output*);

class FAudio_Device_Notifier {
    std::wstring last_device;
    pthread_mutex_t lock;
    
    std::vector<FAudio_Output*> instances;

public:
    FAudio_Device_Notifier()
    {
        pthread_mutex_init(&lock, NULL);
    }

    ~FAudio_Device_Notifier()
    {
        pthread_mutex_destroy(&lock);
    }

    ULONG FAUDIOAPI AddRef()
    {
        return 1;
    }

    ULONG FAUDIOAPI Release()
    {
        return 1;
    }

    //uint32_t FAUDIOAPI QueryInterface or it should be, I still need to find a cross platform way to query devices.

    void OnDefaultDeviceChanged() // This is the callback that is called when the default device is changed. The XAudio2 driver leverages mmdeviceapi, but here we can't rely on that, so we need to find a method that work on all platforms.
    {

        pthread_mutex_lock(&lock);
        for (auto it = instances.begin(); it < instances.end(); ++it)
        {
            faudio_device_changed(*it);
        }
        pthread_mutex_unlock(&lock);
    }

    void do_register(FAudio_Output* p_instance)
    {
        pthread_mutex_lock(&lock);
        instances.push_back(p_instance);
        pthread_mutex_unlock(&lock);
    }

    void do_unregister(FAudio_Output* p_instance)
    {
        pthread_mutex_lock(&lock);

        for (auto it = instances.begin(); it < instances.end(); ++it) {
            if (*it == p_instance) {
                instances.erase(it);
                break;
            }
        }
        
        pthread_mutex_unlock(&lock);
    }

} g_notifier;

// Synchronization Event
class FAudio_BufferNotify : public FAudioVoiceCallback {
public:
    void *hBufferEndEvent;

    FAudio_BufferNotify()
    {
        hBufferEndEvent = NULL;
        //I'm still figuring out how to deal with pthreads, so I'm going to leave this empty for now.
        //hBufferEndEvent = pthread_cond_init();
    }
    ~FAudio_BufferNotify()
    {
        //I'm still figuring out how to deal with pthreads, so I'm going to leave this empty for now.
        //pthread_cond_destroy(hBufferEndEvent);
    }

    void OnBufferEnd(void* pBufferContext)
    {

    }
    void OnVoiceProcessingPassStart(uint32_t BytesRequired) {}
    void OnVoiceProcessingPassEnd() {}
    void OnStreamEnd() {}
    void OnBufferStart(void* pBufferContext) {}
    void OnLoopEnd(void* pBufferContext) {}
    void OnVoiceError(void* pBufferContext, uint32_t Error) {}
};
    
// Class Declaration
class FAudio_Output
    : public SoundDriver {
public:
    FAudio_Output();
    ~FAudio_Output();

    // Initialization
    bool init(long sampleRate);

    // Sound Data Feed
    void write(uint16_t* finalWave, int length);

    // Play Control
    void pause();
    void resume();
    void reset();
    void close();
    void device_change();

    // Configuration Changes
    void setThrottle(unsigned short throttle);

private:
    bool failed;
    bool initialized;
    bool playing;
    uint32_t freq;
    uint32_t bufferCount;
    uint8_t* buffers;
    int currentBuffer;
    int soundBufferLen;

    volatile bool device_changed;

    FAudio* faud;
    FAudioMasteringVoice* mVoice; // listener
    FAudioSourceVoice* sVoice; // sound source
    FAudioBuffer buf;
    FAudioVoiceState vState;
    FAudio_BufferNotify notify; // buffer end notification
};

// Class Implementation
FAudio_Output::FAudio_Output()
{
    failed = false;
    initialized = false;
    playing = false;
    freq = 0;
    bufferCount = gopts.audio_buffers;
    buffers = NULL;
    currentBuffer = 0;
    device_changed = false;
    faud = NULL;
    mVoice = NULL;
    sVoice = NULL;
    memset(&buf, NULL, sizeof(buf));
    memset(&vState, NULL, sizeof(vState));
    g_notifier.do_register(this);
}

FAudio_Output::~FAudio_Output()
{
    g_notifier.do_unregister(this);
    close();
}

void FAudio_Output::close()
{
    initialized = false;

    if (sVoice) {
        if (playing) {
            assert(FAudioSourceVoice_Stop(sVoice, 0, FAUDIO_COMMIT_NOW) == 0);
        }

        FAudioVoice_DestroyVoice(sVoice);
        sVoice = NULL;
    }

    if (buffers) {
        free(buffers);
        buffers = NULL;
    }

    if (mVoice) {
        FAudioVoice_DestroyVoice(mVoice);
        mVoice = NULL;
    }

    if (faud) {
        FAudio_Release(faud);
        faud = NULL;
    }
}

void FAudio_Output::device_change()
{
    device_changed = true;
}

bool FAudio_Output::init(long sampleRate)
{
    if (failed || initialized)
        return false;

    uint32_t hr;
    // Initialize FAudio
    uint32_t flags = 0;
    //#ifdef _DEBUG
    //	flags = FAUDIO_DEBUG_ENGINE;
    //#endif
    hr = FAudioCreate(&faud, flags, FAUDIO_DEFAULT_PROCESSOR);

    if (hr != 0) {
        wxLogError(_("The FAudio interface failed to initialize!"));
        failed = true;
        return false;
    }

    freq = sampleRate;
    // calculate the number of samples per frame first
    // then multiply it with the size of a sample frame (16 bit * stereo)
    soundBufferLen = (freq / 60) * 4;
    // create own buffers to store sound data because it must not be
    // manipulated while the voice plays from it
    buffers = (uint8_t*)malloc((bufferCount + 1) * soundBufferLen);
    // + 1 because we need one temporary buffer when all others are in use
    FAudioWaveFormatEx wfx;
    memset(&wfx, NULL, sizeof(wfx));
    wfx.wFormatTag = FAUDIO_FORMAT_PCM;
    wfx.nChannels = 2;
    wfx.nSamplesPerSec = freq;
    wfx.wBitsPerSample = 16;
    wfx.nBlockAlign = wfx.nChannels * (wfx.wBitsPerSample / 8);
    wfx.nAvgBytesPerSec = wfx.nSamplesPerSec * wfx.nBlockAlign;
    // create sound receiver
    hr = FAudio_CreateMasteringVoice(faud,
    &mVoice,
    FAUDIO_DEFAULT_CHANNELS,
    FAUDIO_DEFAULT_SAMPLERATE,
    0,
    FAGetDev(faud),
    NULL);

    if (hr != 0) {
        wxLogError(_("FAudio: Creating mastering voice failed!"));
        failed = true;
        return false;
    }

    // create sound emitter
    //This should be  FAudio_CreateSourceVoice()
    //hr = faud->CreateSourceVoice(&sVoice, &wfx, 0, 4.0f, &notify);
    hr = FAudio_CreateSourceVoice(faud, &sVoice, &wfx, 0, 4.0f, &notify);

    if (hr != 0) {
        wxLogError(_("FAudio: Creating source voice failed!"));
        failed = true;
        return false;
    }

    if (gopts.upmix) {
        // set up stereo upmixing
        FAudioDeviceDetails dd;
        memset(&dd, NULL, sizeof(dd));
        assert(FAudio_GetDeviceDetails(faud, 0, &dd) == 0);
        float* matrix = NULL;
        matrix = (float*)malloc(sizeof(float) * 2 * dd.OutputFormat.Format.nChannels);

        if (matrix == NULL)
            return false;

        bool matrixAvailable = true;

        switch (dd.OutputFormat.Format.nChannels) {
        case 4: // 4.0
            //Speaker \ Left Source           Right Source
            /*Front L*/ matrix[0] = 1.0000f;
            matrix[1] = 0.0000f;
            /*Front R*/ matrix[2] = 0.0000f;
            matrix[3] = 1.0000f;
            /*Back  L*/ matrix[4] = 1.0000f;
            matrix[5] = 0.0000f;
            /*Back  R*/ matrix[6] = 0.0000f;
            matrix[7] = 1.0000f;
            break;

        case 5: // 5.0
            //Speaker \ Left Source           Right Source
            /*Front L*/ matrix[0] = 1.0000f;
            matrix[1] = 0.0000f;
            /*Front R*/ matrix[2] = 0.0000f;
            matrix[3] = 1.0000f;
            /*Front C*/ matrix[4] = 0.7071f;
            matrix[5] = 0.7071f;
            /*Side  L*/ matrix[6] = 1.0000f;
            matrix[7] = 0.0000f;
            /*Side  R*/ matrix[8] = 0.0000f;
            matrix[9] = 1.0000f;
            break;

        case 6: // 5.1
            //Speaker \ Left Source           Right Source
            /*Front L*/ matrix[0] = 1.0000f;
            matrix[1] = 0.0000f;
            /*Front R*/ matrix[2] = 0.0000f;
            matrix[3] = 1.0000f;
            /*Front C*/ matrix[4] = 0.7071f;
            matrix[5] = 0.7071f;
            /*LFE    */ matrix[6] = 0.0000f;
            matrix[7] = 0.0000f;
            /*Side  L*/ matrix[8] = 1.0000f;
            matrix[9] = 0.0000f;
            /*Side  R*/ matrix[10] = 0.0000f;
            matrix[11] = 1.0000f;
            break;

        case 7: // 6.1
            //Speaker \ Left Source           Right Source
            /*Front L*/ matrix[0] = 1.0000f;
            matrix[1] = 0.0000f;
            /*Front R*/ matrix[2] = 0.0000f;
            matrix[3] = 1.0000f;
            /*Front C*/ matrix[4] = 0.7071f;
            matrix[5] = 0.7071f;
            /*LFE    */ matrix[6] = 0.0000f;
            matrix[7] = 0.0000f;
            /*Side  L*/ matrix[8] = 1.0000f;
            matrix[9] = 0.0000f;
            /*Side  R*/ matrix[10] = 0.0000f;
            matrix[11] = 1.0000f;
            /*Back  C*/ matrix[12] = 0.7071f;
            matrix[13] = 0.7071f;
            break;

        case 8: // 7.1
            //Speaker \ Left Source           Right Source
            /*Front L*/ matrix[0] = 1.0000f;
            matrix[1] = 0.0000f;
            /*Front R*/ matrix[2] = 0.0000f;
            matrix[3] = 1.0000f;
            /*Front C*/ matrix[4] = 0.7071f;
            matrix[5] = 0.7071f;
            /*LFE    */ matrix[6] = 0.0000f;
            matrix[7] = 0.0000f;
            /*Back  L*/ matrix[8] = 1.0000f;
            matrix[9] = 0.0000f;
            /*Back  R*/ matrix[10] = 0.0000f;
            matrix[11] = 1.0000f;
            /*Side  L*/ matrix[12] = 1.0000f;
            matrix[13] = 0.0000f;
            /*Side  R*/ matrix[14] = 0.0000f;
            matrix[15] = 1.0000f;
            break;

        default:
            matrixAvailable = false;
            break;
        }

        if (matrixAvailable) {
            hr = FAudioVoice_SetOutputMatrix(sVoice, NULL, 2, dd.OutputFormat.Format.nChannels, matrix, FAUDIO_DEFAULT_CHANNELS);
            assert(hr == 0);
        }

        free(matrix);
        matrix = NULL;
    }

    hr = FAudioSourceVoice_Start(sVoice, 0, FAUDIO_COMMIT_NOW);
    assert(hr == 0);
    playing = true;
    currentBuffer = 0;
    device_changed = false;
    initialized = true;
    return true;
}

void FAudio_Output::write(uint16_t* finalWave, int length)
{
    uint32_t flags = 0;
    if (!initialized || failed)
        return;

    while (true) {
        if (device_changed) {
            close();

            if (!init(freq))
                return;
        }

        FAudioSourceVoice_GetState(sVoice, &vState, flags);
        assert(vState.BuffersQueued <= bufferCount);

        if (vState.BuffersQueued < bufferCount) {
            if (vState.BuffersQueued == 0) {
                // buffers ran dry
                if (systemVerbose & VERBOSE_SOUNDOUTPUT) {
                    static unsigned int i = 0;
                    log("FAudio: Buffers were not refilled fast enough (i=%i)\n", i++);
                }
            }

            // there is at least one free buffer
            break;
        } //else {
            // the maximum number of buffers is currently queued
            if (!coreOptions.speedup && coreOptions.throttle && !gba_joybus_active) {
                // wait for one buffer to finish playing
                if (WaitForSingleObject(notify.hBufferEndEvent, 10000) == WAIT_TIMEOUT) {
                   device_changed = true;
                }
            } else {
                // drop current audio frame
                return;
            }
        }
    }

    // copy & protect the audio data in own memory area while playing it
    memcpy(&buffers[currentBuffer * soundBufferLen], finalWave, soundBufferLen);
    buf.AudioBytes = soundBufferLen;
    buf.pAudioData = &buffers[currentBuffer * soundBufferLen];
    currentBuffer++;
    currentBuffer %= (bufferCount + 1); // + 1 because we need one temporary buffer
    uint32_t hr = FAudioSourceVoice_SubmitSourceBuffer(sVoice, &buf, NULL);
    assert(hr == 0);
}

void FAudio_Output::pause()
{
    if (!initialized || failed)
        return;

    if (playing) {
        uint32_t hr = FAudioSourceVoice_Stop(sVoice, 0, FAUDIO_COMMIT_NOW);
        assert(hr == 0);
        playing = false;
    }
}

void FAudio_Output::resume()
{
    if (!initialized || failed)
        return;

    if (!playing) {
        uint32_t hr = FAudioSourceVoice_Start(sVoice, 0, FAUDIO_COMMIT_NOW);
        assert(hr == 0);
        playing = true;
    }
}

void FAudio_Output::reset()
{
    if (!initialized || failed)
        return;

    if (playing) {
        uint32_t hr = FAudioSourceVoice_Stop(sVoice, 0, FAUDIO_COMMIT_NOW);
        assert(hr == 0);
    }

    FAudioSourceVoice_FlushSourceBuffers(sVoice);
    FAudioSourceVoice_Start(sVoice, 0, FAUDIO_COMMIT_NOW);
    playing = true;
}

void FAudio_Output::setThrottle(unsigned short throttle_)
{
    if (!initialized || failed)
        return;

    if (throttle_ == 0)
        throttle_ = 100;

    uint32_t hr = FAudioSourceVoice_SetFrequencyRatio(sVoice, (float)throttle_ / 100.0f, FAUDIO_MAX_FILTER_FREQUENCY);
    assert(hr == 0);
}

void faudio_device_changed(FAudio_Output* instance)
{
    instance->device_change();
}

SoundDriver* newFAudio_Output()
{
    return new FAudio_Output();
}

#endif // #ifndef NO_FAUDIO

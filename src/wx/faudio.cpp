#ifndef NO_FAUDIO

// Application
#include "wxvbam.h"
#include <stdio.h>

// Interface
#include "../common/SoundDriver.h"

// FAudio
#include <FAudio.h>

#include <string>
#include <vector>

// Internals
#include "../System.h" // for systemMessage()
#include "../gba/Globals.h"

int GetFADevices(FAudio* fa, wxArrayString* names, wxArrayString* ids,
    const wxString* match)
{
    uint32_t dev_count = 0;

    if (FAudio_GetDeviceCount(fa, &dev_count) != R_OK) {
        wxLogError(_("FAudio: Enumerating devices failed!"));
        return true;
    } else {
        FAudioDeviceDetails dd;

        for (uint32_t i = 0; i < dev_count; i++) {
            if (FAudio_GetDeviceDetails(fa, i, &dd) != R_OK) {
                continue;
            } else {
                if (ids) {
                    ids->push_back((wchar_t*) dd.DeviceID); //FAudio is an interesting beast, but not that hard to adapt... once you get used to it, XAudio2 wouldn't need this, but FAudio declares FAudioDeviceDetails as int32_t
                    names->push_back((wchar_t*) dd.DisplayName);
                } else if (*match == dd.DeviceID)
                    return i;
            }
        }
    }

    return -1;
}

bool GetFADevices(wxArrayString& names, wxArrayString& ids)
{
    FAudio* fa = NULL;
    uint32_t flags = 0;
#ifdef _DEBUG
    flags = FAUDIO_DEBUG_ENGINE;
#endif

    if (FAudioCreate(&fa, flags, FAUDIO_DEFAULT_PROCESSOR) != R_OK) {
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
    std::vector<FAudio_Output*> instances;

public:
    FAudio_Device_Notifier() = default;
    ~FAudio_Device_Notifier() = default;

    void do_register(FAudio_Output* p_instance)
    {
        instances.push_back(p_instance);
    }

    void do_unregister(FAudio_Output* p_instance)
    {
        for (auto it = instances.begin(); it < instances.end(); ++it) {
            if (*it == p_instance) {
                instances.erase(it);
                break;
            }
        }
    }
    void OnDefaultDeviceChanged()
    {
        for (auto it = instances.begin(); it < instances.end(); ++it)
        {
            faudio_device_changed(*it);
        }
    }
} g_notifier;

// Synchronization Event
class FAudio_BufferNotify : public FAudioVoiceCallback {
public:
    void OnBufferEnd(void* pBufferContext) {}
    void OnVoiceProcessingPassStart(uint32_t BytesRequired) {}
    void OnVoiceProcessingPassEnd() {}
    void OnStreamEnd() {}
    void OnBufferStart(void* pBufferContext) {}
    void OnLoopEnd(void* pBufferContext) {}
    void OnVoiceError(void* pBufferContext, int Error) {}
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
            assert(FAudioSourceVoice_Stop(sVoice, 0, FAUDIO_COMMIT_NOW) == R_OK);
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

    // Initialize FAudio
    uint32_t flags = 0;
    //#ifdef _DEBUG
    //	flags = FAUDIO_DEBUG_ENGINE;
    //#endif

    if (FAudioCreate(&faud, flags, FAUDIO_DEFAULT_PROCESSOR) != R_OK) {
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

    if (FAudio_CreateMasteringVoice(faud, &mVoice, FAUDIO_DEFAULT_CHANNELS, FAUDIO_DEFAULT_SAMPLERATE, 0, FAGetDev(faud), NULL) != R_OK) {
        wxLogError(_("FAudio: Creating mastering voice failed!"));
        failed = true;
        return false;
    }

    // create sound emitter
    //This should be  FAudio_CreateSourceVoice()
    //hr = faud->CreateSourceVoice(&sVoice, &wfx, 0, 4.0f, &notify);

    if (FAudio_CreateSourceVoice(faud, &sVoice, (const FAudioWaveFormatEx*)&wfx, 0, 4.0f, &notify, NULL, NULL) != R_OK) {
        wxLogError(_("FAudio: Creating source voice failed!"));
        failed = true;
        return false;
    }

    if (gopts.upmix) {
        // set up stereo upmixing
        FAudioDeviceDetails dd;
        memset(&dd, NULL, sizeof(dd));
        assert(FAudio_GetDeviceDetails(faud, 0, &dd) == R_OK);
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
            assert(FAudioVoice_SetOutputMatrix(sVoice, NULL, 2, dd.OutputFormat.Format.nChannels, matrix, FAUDIO_DEFAULT_CHANNELS) == R_OK);
        }

        free(matrix);
        matrix = NULL;
    }

    assert(FAudioSourceVoice_Start(sVoice, 0, FAUDIO_COMMIT_NOW) == R_OK);
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
            //if (!coreOptions.speedup && coreOptions.throttle && !gba_joybus_active) {
                // wait for one buffer to finish playing
            //    if (WaitForSingleObject(notify.hBufferEndEvent, 10000) == WAIT_TIMEOUT) {
            //       device_changed = true;
            //    }
            //} else {
                // drop current audio frame
            //    return;
            //}
        //}
    }

    // copy & protect the audio data in own memory area while playing it
    memcpy(&buffers[currentBuffer * soundBufferLen], finalWave, soundBufferLen);
    buf.AudioBytes = soundBufferLen;
    buf.pAudioData = &buffers[currentBuffer * soundBufferLen];
    currentBuffer++;
    currentBuffer %= (bufferCount + 1); // + 1 because we need one temporary buffer
    assert(FAudioSourceVoice_SubmitSourceBuffer(sVoice, &buf, NULL) == R_OK);
}

void FAudio_Output::pause()
{
    if (!initialized || failed)
        return;

    if (playing) {
        assert(FAudioSourceVoice_Stop(sVoice, 0, FAUDIO_COMMIT_NOW) == R_OK);
        playing = false;
    }
}

void FAudio_Output::resume()
{
    if (!initialized || failed)
        return;

    if (!playing) {
        assert(FAudioSourceVoice_Start(sVoice, 0, FAUDIO_COMMIT_NOW) == R_OK);
        playing = true;
    }
}

void FAudio_Output::reset()
{
    if (!initialized || failed)
        return;

    if (playing) {
        assert(FAudioSourceVoice_Stop(sVoice, 0, FAUDIO_COMMIT_NOW) == R_OK);
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

    assert(FAudioSourceVoice_SetFrequencyRatio(sVoice, (float)throttle_ / 100.0f, FAUDIO_MAX_FILTER_FREQUENCY) == R_OK);
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

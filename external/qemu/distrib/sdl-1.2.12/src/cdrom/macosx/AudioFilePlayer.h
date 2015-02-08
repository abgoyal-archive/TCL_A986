
#include "SDL_config.h"

#ifndef __AudioFilePlayer_H__
#define __AudioFilePlayer_H__

#include <CoreServices/CoreServices.h>

#include <AudioUnit/AudioUnit.h>

#include "SDL_error.h"

const char* AudioFilePlayerErrorStr (OSStatus error);


typedef void (*AudioFilePlayNotifier)(void          *inRefCon,
                                    OSStatus        inStatus);

enum {
    kAudioFilePlayErr_FilePlayUnderrun = -10000,
    kAudioFilePlay_FileIsFinished = -10001,
    kAudioFilePlay_PlayerIsUninitialized = -10002
};


struct S_AudioFileManager;

#pragma mark __________ AudioFilePlayer
typedef struct S_AudioFilePlayer
{
/*public:*/
    int             (*SetDestination)(struct S_AudioFilePlayer *afp, AudioUnit *inDestUnit);
    void            (*SetNotifier)(struct S_AudioFilePlayer *afp, AudioFilePlayNotifier inNotifier, void *inRefCon);
    void            (*SetStartFrame)(struct S_AudioFilePlayer *afp, int frame); /* seek in the file */
    int             (*GetCurrentFrame)(struct S_AudioFilePlayer *afp); /* get the current frame position */
    void            (*SetStopFrame)(struct S_AudioFilePlayer *afp, int frame);   /* set limit in the file */
    int             (*Connect)(struct S_AudioFilePlayer *afp);
    void            (*Disconnect)(struct S_AudioFilePlayer *afp);
    void            (*DoNotification)(struct S_AudioFilePlayer *afp, OSStatus inError);
    int             (*IsConnected)(struct S_AudioFilePlayer *afp);
    AudioUnit       (*GetDestUnit)(struct S_AudioFilePlayer *afp);
    void            (*Print)(struct S_AudioFilePlayer *afp);

/*private:*/
    AudioUnit                       mPlayUnit;
    SInt16                          mForkRefNum;
    
    AudioUnitInputCallback          mInputCallback;

    AudioStreamBasicDescription     mFileDescription;
    
    int                             mConnected;
    
    struct S_AudioFileManager*      mAudioFileManager;
    
    AudioFilePlayNotifier           mNotifier;
    void*                           mRefCon;
    
    int                             mStartFrame;
    
#pragma mark __________ Private_Methods
    
    int          (*OpenFile)(struct S_AudioFilePlayer *afp, const FSRef *inRef, SInt64 *outFileSize);
} AudioFilePlayer;


AudioFilePlayer *new_AudioFilePlayer(const FSRef    *inFileRef);
void delete_AudioFilePlayer(AudioFilePlayer *afp);



#pragma mark __________ AudioFileManager
typedef struct S_AudioFileManager
{
/*public:*/
        /* this method should NOT be called by an object of this class
           as it is called by the parent's Disconnect() method */
    void                (*Disconnect)(struct S_AudioFileManager *afm);
    int                 (*DoConnect)(struct S_AudioFileManager *afm);
    OSStatus            (*Read)(struct S_AudioFileManager *afm, char *buffer, UInt32 *len);
    const char*         (*GetFileBuffer)(struct S_AudioFileManager *afm);
    const AudioFilePlayer *(*GetParent)(struct S_AudioFileManager *afm);
    void                (*SetPosition)(struct S_AudioFileManager *afm, SInt64 pos);  /* seek/rewind in the file */
    int                 (*GetByteCounter)(struct S_AudioFileManager *afm);  /* return actual bytes streamed to audio hardware */
    void                (*SetEndOfFile)(struct S_AudioFileManager *afm, SInt64 pos);  /* set the "EOF" (will behave just like it reached eof) */
   
/*protected:*/
    AudioFilePlayer*    mParent;
    SInt16              mForkRefNum;
    SInt64              mAudioDataOffset;
    
    char*               mFileBuffer;

    int                 mByteCounter;

    int                mReadFromFirstBuffer;
    int                mLockUnsuccessful;
    int                mIsEngaged;
    
    int                 mNumTimesAskedSinceFinished;


	void*               mTmpBuffer;
	UInt32              mBufferSize;
	UInt32              mBufferOffset;
/*public:*/
    UInt32              mChunkSize;
    SInt64              mFileLength;
    SInt64              mReadFilePosition;
    int                 mWriteToFirstBuffer;
    int                 mFinishedReadingData;

/*protected:*/
    OSStatus            (*Render)(struct S_AudioFileManager *afm, AudioBuffer *ioData);
    OSStatus            (*GetFileData)(struct S_AudioFileManager *afm, void** inOutData, UInt32 *inOutDataSize);
    void                (*AfterRender)(struct S_AudioFileManager *afm);

/*public:*/
    /*static*/
    OSStatus            (*FileInputProc)(void                             *inRefCon,
                                         AudioUnitRenderActionFlags      inActionFlags,
                                         const AudioTimeStamp            *inTimeStamp,
                                         UInt32                          inBusNumber,
                                         AudioBuffer                     *ioData);
} AudioFileManager;


AudioFileManager *new_AudioFileManager (AudioFilePlayer *inParent,
                      SInt16          inForkRefNum, 
                      SInt64          inFileLength,
                      UInt32          inChunkSize);
    
void delete_AudioFileManager(AudioFileManager *afm);

#endif


/*
* 
* Copyright (c) 2011, Ban the Rewind
* All rights reserved.
* 
* Redistribution and use in source and binary forms, with or 
* without modification, are permitted provided that the following 
* conditions are met:
* 
* Redistributions of source code must retain the above copyright 
* notice, this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright 
* notice, this list of conditions and the following disclaimer in 
* the documentation and/or other materials provided with the 
* distribution.
* 
* Neither the name of the Ban the Rewind nor the names of its 
* contributors may be used to endorse or promote products 
* derived from this software without specific prior written 
* permission.
* 
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE 
* COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, 
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, 
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF 
* ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
* 
*/

#pragma once

// Includes
#include <algorithm>
#include "boost/bind.hpp"
#include "boost/signals2.hpp"
#include "cinder/app/AppBasic.h"
#include <MMSystem.h>
#include <utility>
#include <vector>
#include <windows.h>

// Imports
using namespace std;

// Constants
#define BUFFER_COUNT					32
#define BUFFER_LENGTH					1024
#define MESSAGE_BUFFER_SIZE				256

// Define device query functions if WDK is not present
#ifndef DRV_RESERVED
#define DRV_RESERVED					0×0800
#endif
#ifndef DRV_QUERYDEVICEINTERFACE
#define DRV_QUERYDEVICEINTERFACE		(DRV_RESERVED + 12)
#endif
#ifndef DRV_QUERYDEVICEINTERFACESIZE
#define DRV_QUERYDEVICEINTERFACESIZE	(DRV_RESERVED + 13)
#endif

// Device map alias
typedef map<int32_t, string> DeviceList;

// Audio input for Windows
template<typename T>
class AudioInputT
{

public:

	// Header for WAV file
	typedef struct
	{
		char RIFF[4];
		DWORD bytes;
		char WAVE[4];
		char fmt[4];
		int siz_wf;
		WORD wFormatTag;
		WORD nChannels;
		DWORD nSamplesPerSec;
		DWORD nAvgBytesPerSec;
		WORD nBlockAlign;
		WORD wBitsPerSample;
		char data[4];
		DWORD pcmbytes;
	} WAVFILEHEADER;

private:

	// Callback alias
	typedef boost::signals2::connection Callback;
	typedef std::shared_ptr<Callback> CallbackRef;
	typedef map<int32_t, CallbackRef> CallbackList;

	// The object
	struct Obj
	{

		/****** CONSTRUCTORS ******/

		// Constructor
		Obj(int32_t bitsPerSample, int32_t sampleRate, int32_t channelCount) 
		{

			// Initialize flag
			mReceiving = false;

			// Set parameters
			mBitsPerSample = bitsPerSample;
			mChannelCount = channelCount;
			mSampleRate = sampleRate;

			// Set up PCM format
			mWavFormat.wFormatTag = WAVE_FORMAT_PCM;
			mWavFormat.nChannels = mChannelCount;
			mWavFormat.nSamplesPerSec =	mSampleRate;
			mWavFormat.nAvgBytesPerSec = mSampleRate * mChannelCount * sizeof(T);
			mWavFormat.nBlockAlign = mChannelCount * sizeof(T);
			mWavFormat.wBitsPerSample = mBitsPerSample;
			mWavFormat.cbSize = 0;

			// Initialize device list
			mDeviceId = 0;
			mDeviceCount = -1;
			mLocale = locale(""); // Uses system's default language for UTF encoding
			getDeviceList();

		}

		// Destructor
		~Obj()
		{

			// Stop
			if (mReceiving) 
				stop();

			// Clear vectors
			mCallbacks.clear();
			mHeaderBuffers.clear();
			mInputBuffers.clear();

			// Free resoures
			if (mBuffer != 0)
				delete [] mBuffer;
			if (mNormalBuffer != 0)
				delete [] mNormalBuffer;

		}



		/****** METHODS ******/

		// Gets error
		bool error()
		{

			// Error occurred
			if (mResultHnd)
			{

				// Report error and break
				memset(mErr, 0, MESSAGE_BUFFER_SIZE);
				waveInGetErrorTextA(mResultHnd, mErr, MESSAGE_BUFFER_SIZE);
				OutputDebugStringA(mErr);
				OutputDebugStringA("\n");
				return true;

			}

			// No error
			return false;

		}

		// Build and return device list
		DeviceList getDeviceList()
		{

			// Get device count
			int32_t deviceCount = (int32_t)waveInGetNumDevs();

			// Skip routine if device count hasn't changed
			if (mDeviceCount != deviceCount)
			{

				// Update device count
				mDeviceCount = deviceCount;

				// Build new list
				mDeviceList.clear();
				for (int32_t i = 0; i < mDeviceCount; i++)
				{

					// Get device
					WAVEINCAPS * mDevice = new WAVEINCAPS();
					waveInGetDevCaps((UINT_PTR)i, mDevice, sizeof(WAVEINCAPS));

					// Get device name
					memset(mDeviceName, 0, sizeof(mDeviceName));
					use_facet<ctype<wchar_t> >(mLocale).narrow(mDevice->szPname, mDevice->szPname + wcslen(mDevice->szPname), 'X', &mDeviceName[0]);

					// Add device to list
					mDeviceList.insert(make_pair(i, string(mDeviceName)));

					// Clean up
					delete mDevice;

				}

			}

			// Return list
			return mDeviceList;

		}

		// Returns data as -1 to 1 float
		float * getNormalizedData()
		{

			// Bail if there's no data to convert
			if (mBuffer == 0) 
				return 0;

			// Delete buffer if it's the wrong size
			if (mNormalBuffer != 0 && sizeof(mNormalBuffer) != mBufferSize)
			{
				delete [] mNormalBuffer;
				mNormalBuffer = 0;
			}

			// Create buffer, if needed
			if (mNormalBuffer == 0)
				mNormalBuffer = new float[mBufferSize];

			// Normalize data for analysis
			switch (mBitsPerSample)
			{
			case 8: 
				for (int32_t i = 0; i < mBufferSize; i++)
					mNormalBuffer[i] = (float)(mBuffer[i] / (1.0 * 0x80) - 1.0);
				break;
			case 16: 
				for (int32_t i = 0; i < mBufferSize; i++)
					mNormalBuffer[i] = (float)(mBuffer[i] / (1.0 * 0x8000));
				break;
			case 32: 
				for (int32_t i = 0; i < mBufferSize; i++)
					mNormalBuffer[i] = (float)(mBuffer[i] / (8.0 * 0x10000000));
				break;
			}

			// Return normalized buffer
			return mNormalBuffer;

		}

		// Receive message from multimedia API
		void receiveMessage(MSG message)
		{

			// Read message
			switch (message.message) 
			{

			case MM_WIM_DATA:

				// Check receiving flag
				if (mReceiving)
				{

					// Check for pointer to wave header
					if (((WAVEHDR *)message.lParam)->dwBytesRecorded) 
					{

						// Update buffer
						mBufferSize = (int32_t)((unsigned long)((WAVEHDR *)message.lParam)->dwBytesRecorded / sizeof(T));
						mBuffer = (T *)((WAVEHDR *)message.lParam)->lpData;

						// Execute callbacks
						mSignal(getNormalizedData(), mBufferSize);

					}

					// Re-use buffer
					waveInAddBuffer(mDeviceHnd, ((WAVEHDR *)message.lParam), sizeof(WAVEHDR));

				}
				else
				{

					// Mark buffer complete
					++mBuffersComplete;

				}

				break;
			case MM_WIM_OPEN:

				// Reset complete count if main thread is opening the device
				mBuffersComplete = 0;

				break;

			}

		}

		// Set device
		void setDevice(int32_t deviceID)
		{

			// New device ID must be less than the number of devices
			// and different from current ID
			if (deviceID >= 0 && deviceID < mDeviceCount && deviceID != mDeviceId)
			{

				// Stop input if we're currently receiving
				bool receiving = mReceiving;
				if (receiving) 
					stop();

				// Switch device ID 
				mDeviceId = deviceID;

				// Restart if we were receiving audio
				if (receiving) 
					start();

			}

		}

		// Start receiving and recording
		void start()
		{

			// Stop playback if we're already receiving
			if (mReceiving)
				stop();

			// Check recording flag
			if (!mReceiving)
			{

				// Set flag
				mReceiving = true;

				// Initialize buffers
				mBuffer = 0;
				mNormalBuffer = 0;

				// Start callback thread
				DWORD mThreadID;
				mWaveInThread = CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)AudioInputT<T>::Obj::waveInProc, (PVOID)this, 0, &mThreadID);
				if (!mWaveInThread) 
					return;
				CloseHandle(mWaveInThread);

				// Open input device
				mResultHnd = waveInOpen(&mDeviceHnd, mDeviceId, &mWavFormat, mThreadID, 0, CALLBACK_THREAD);
				if (error()) 
					return;

				// Prepare buffers
				mBuffersComplete = 0;
				for (int32_t i = 0; i < BUFFER_COUNT; i++) 
				{

					// Create buffers
					mHeaderBuffers.push_back(new T[BUFFER_LENGTH * sizeof(T)]);
					mInputBuffers.push_back(new WAVEHDR());

					// Set up header
					mInputBuffers[i]->dwBufferLength = BUFFER_LENGTH * sizeof(T);
					mInputBuffers[i]->lpData = (LPSTR)mHeaderBuffers[i];
					mInputBuffers[i]->dwBytesRecorded = 0;
					mInputBuffers[i]->dwUser = 0L;
					mInputBuffers[i]->dwFlags = 0L;
					mInputBuffers[i]->dwLoops = 0L;

					// Add buffer to input device
					mResultHnd = waveInPrepareHeader(mDeviceHnd, mInputBuffers[i], sizeof(WAVEHDR));
					if (error()) 
						return;
					mResultHnd = waveInAddBuffer(mDeviceHnd, mInputBuffers[i], sizeof(WAVEHDR));
					if (error()) 
						return;

				}

				// Start input
				mResultHnd = waveInStart(mDeviceHnd);
				if (error()) 
					mReceiving = false;

			}

		}

		// Stop recording
		void stop()
		{

			// Check receiving flag
			if (mReceiving)
			{

				// Turn off flags
				mReceiving = false;

				// Wait for every buffer to complete
				while (mBuffersComplete < BUFFER_COUNT) 
					Sleep(1);

				// Close input device
				mResultHnd = waveInReset((HWAVEIN)mDeviceId);
				if (error()) 
					return;
				mResultHnd = waveInClose((HWAVEIN)mDeviceId);
				if (error()) 
					return;

				// Release buffers
				for (int32_t i = 0; i < BUFFER_COUNT; i++) 
					mResultHnd = waveInUnprepareHeader((HWAVEIN)mDeviceId, (LPWAVEHDR)&(mInputBuffers[i]), sizeof(WAVEHDR));

			}

		}

		// Media API callback
		static DWORD WINAPI waveInProc(LPVOID arg)
		{

			// Get instance
			AudioInputT<T>::Obj * mInstance = (AudioInputT<T>::Obj *) arg;

			// Get message from thread
			MSG mMessage;

			// Bail if instance not available
			if (mInstance == NULL) 
				return 0;

			// Get message from thread
			while (GetMessage(&mMessage, 0, 0, 0)) 
				mInstance->receiveMessage(mMessage);

			// Return
			return 0;

		}


		/****** PROPERTIES ******/

		// Flags
		bool mReceiving;

		// The current audio buffer
		T * mBuffer;
		uint_least8_t mBuffersComplete;
		int32_t mBufferSize;

		// Buffer for analyzing audio
		float * mNormalBuffer;

		// Windows multimedia API
		HANDLE mWaveInThread;
		vector<T *> mHeaderBuffers;
		vector<WAVEHDR *> mInputBuffers;
		MMRESULT mResultHnd;
		WAVEFORMATEX mWavFormat;

		// WAV format
		int32_t mBitsPerSample;
		int32_t mChannelCount;
		int32_t mSampleRate;

		// Callback list
		boost::signals2::signal<void (float *, int32_t)> mSignal;
		CallbackList mCallbacks;

		// Device list
		HWAVEIN mDeviceHnd;
		int32_t mDeviceId;
		int32_t mDeviceCount;
		DeviceList mDeviceList;
		char mDeviceName[32];
		locale mLocale;

		// Error message buffer
		int_fast8_t mErr[MESSAGE_BUFFER_SIZE];

	};

	// Pointer to object
	std::shared_ptr<Obj> mObj;

public:

	// Constructors
	AudioInputT() : mObj(std::shared_ptr<Obj>(new Obj(sizeof(T) * 8, 44100, 2))) {}
	AudioInputT(int32_t sampleRate, int32_t channelCount) : mObj(std::shared_ptr<Obj>(new Obj(sizeof(T) * 8, sampleRate, channelCount))) {}
	~AudioInputT() { mObj.reset(); }

	// Methods
	int32_t getBitsPerSample() { return mObj->mBitsPerSample; }
	int32_t getChannelCount() { return mObj->mChannelCount; }
	T * getData() { return mObj->mBuffer; }
	int32_t getDataSize() { return mObj->mBufferSize; }
	int32_t getDeviceCount() { return mObj->mDeviceCount; }
	DeviceList getDeviceList() { return mObj->getDeviceList(); }
	float * getNormalizedData() { return mObj->mNormalBuffer; }
	int32_t getSampleRate() { return mObj->mSampleRate; }
	bool isReceiving() { return mObj->mReceiving; }
	void setDevice(int32_t deviceID) { mObj->setDevice(deviceID); }
	void start() { mObj->start(); }
	void stop() { mObj->stop(); }

	// Add callback
	template<typename U>
	int32_t addCallback(void (U::* callbackFunction)(float * data, int32_t size), U * callbackObject) 
	{

		// Determine return ID
		int32_t mCallbackID = mObj->mCallbacks.empty() ? 0 : mObj->mCallbacks.rbegin()->first + 1;

		// Create callback and add it to the list
		mObj->mCallbacks.insert(make_pair(mCallbackID, CallbackRef(new Callback(mObj->mSignal.connect(boost::function<void (float *, int32_t)>(boost::bind(callbackFunction, callbackObject, ::_1, ::_2)))))));

		// Return callback ID
		return mCallbackID;

	}

	// Removes callback
	void removeCallback(int32_t callbackID) 
	{

		// Disconnect the callback connection
		mObj->mCallbacks.find(callbackID)->second->disconnect();

		// Remove the callback from the list
		mObj->mCallbacks.erase(callbackID); 

	}

};

// Aliases
typedef AudioInputT<uint8_t> AudioInput8;
typedef AudioInputT<int16_t> AudioInput16;
typedef AudioInputT<uint32_t> AudioInput32;
typedef AudioInput16 AudioInput;

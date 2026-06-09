// jhcMpiCam.cpp : SM371 USB camera background image capture using OpenCV
//
// Written by Jonathan H. Connell, jconnell@alum.mit.edu
//
///////////////////////////////////////////////////////////////////////////
//
// Copyright 2025 Etaoin Systems
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//    http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// 
///////////////////////////////////////////////////////////////////////////

#include <unistd.h>

#include <jhcMpiCam.h>


///////////////////////////////////////////////////////////////////////////
//                      Creation and Initialization                      //
///////////////////////////////////////////////////////////////////////////

//= Destructor cleans up files and any allocated items.

jhcMpiCam::~jhcMpiCam ()
{
  Done();
}


//= Default constructor initializes certain values.

jhcMpiCam::jhcMpiCam ()
{
  ok = -1;
  run = 0;
}


///////////////////////////////////////////////////////////////////////////
//                              Main Functions                           //
///////////////////////////////////////////////////////////////////////////

//= Open connection to camera and start background acquisition thread.
// uses most recently added device (see: v4l2-ctl --list-devices)
// must do "sudo apt install uhubctl" for Raspberry Pi 4
// returns 1 if okay, 0 or negative for error

int jhcMpiCam::Start ()
{
  int i;

  // initialize rotating buffers
  fill = &c0;
  done = NULL;
  lock = NULL;
  fresh = 0;     

  // suppress warnings about bad sources (too heavy handed?)
  freopen("/dev/null", "w", stderr);

  // connect to V4L2 device with defaults 
  ok = 0;
  for (i = 0; i <= 4; i++)
    if (vcap.open(i, cv::CAP_V4L2))    // GStreamer complains
      if (vcap.isOpened())             // double check
        break;
  if (i > 4)                           // nothing worked
    return ok;            
 
  // launch receiver and pre-processor thread
  run = 1;
  pthread_create(&hoover, NULL, grab, (void *) this);
  ok = 1;
  return ok;
}
 

//= Get a pointer to the most recent RGB image from sensor.
// returns pixel buffer pointer, NULL if not ready or stream broken
// NOTE: only 8 fps if dark environment!

const unsigned char *jhcMpiCam::Color (int block)
{
  int wait = 0;

  // check if source is operational and new frame is ready
  if (ok <= 0)  
    return NULL;
  while (fresh <= 0)
  {
    if (block <= 0)                    // return immediately
      return NULL;
    if (wait++ > 500)                  // barf after 0.5 sec
      return NULL;
    usleep(1000);                      // 1 ms loop
  }

  // swap buffers to be sure output pointer remains valid
  pthread_mutex_lock(&data);
  lock = done;                         // mark as in-use
  fresh = 0;
  pthread_mutex_unlock(&data);
  return lock->data;
}


//= Stop background thread.

void jhcMpiCam::Done ()
{
  timespec one_sec;

  ok = -1;
  if (run <= 0)
    return;
  run = 0;     
  clock_gettime(CLOCK_REALTIME, &one_sec); 
  one_sec.tv_sec += 1; 
  pthread_timedjoin_np(hoover, 0, &one_sec);
  if (vcap.isOpened())
    vcap.release();                    // should not be needed
}


///////////////////////////////////////////////////////////////////////////
//                        Background Acquisition                         //
///////////////////////////////////////////////////////////////////////////

//= Background thread grabs each successive frame.

void *jhcMpiCam::grab (void *rgb)
{
  jhcMpiCam *me = (jhcMpiCam *) rgb;

  me->grab_loop();
  return NULL;
}


//= Continually receive frames from camera into best open buffer image.

void jhcMpiCam::grab_loop ()
{
  nice(-20);                                     // for better fps under load
  while (run > 0) 
  {
    // attempt to read next frame (blocks)
    if (!vcap.read(*fill))
      break;

    // shuffle output images
    pthread_mutex_lock(&data);
    done = fill;                                 // most recent complete
    fresh += 1;
    if (fill == &c0)
      fill = ((lock != &c1) ? &c1 : &c2);
    else if (fill == &c1)
      fill = ((lock != &c0) ? &c0 : &c2);
    else                                         // lock == c2             
      fill = ((lock != &c0) ? &c0 : &c1);
    pthread_mutex_unlock(&data);
  }
  ok = 0;                                        // stream ended
}


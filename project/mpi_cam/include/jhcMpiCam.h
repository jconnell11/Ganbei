// jhcMpiCam.h : SM371 USB camera background image capture using OpenCV
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

#pragma once

#include <pthread.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>


//= SM371 USB camera (MasterPi) background image capture using OpenCV.
// rotates through 3 element image buffers: fill, done, lock

class jhcMpiCam
{
// PRIVATE MEMBER VARIABLES
private:
  // capture interface and overall health
  cv::VideoCapture vcap;
  int ok;  

  // background receiver
  pthread_t hoover;
  pthread_mutex_t data;
  int run;

  // color images and status
  cv::Mat c0, c1, c2;
  cv::Mat *fill, *done, *lock;
  int fresh;


// PUBLIC MEMBER FUNCTIONS
public:
  // creation and initialization
  ~jhcMpiCam ();
  jhcMpiCam ();

  // main functions
  int Start ();
  const unsigned char *Color (int block =0);
  void Done ();


// PRIVATE MEMBER FUNCTIONS
private:
  // background thread functions
  static void *grab (void *rgb);
  void grab_loop ();


};


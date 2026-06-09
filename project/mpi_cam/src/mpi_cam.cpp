// mpi_cam.cpp : read frames from SM371 USB camera with OpenCV
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

#include <jhcMpiCam.h>


///////////////////////////////////////////////////////////////////////////
//                          Global Variables                             //
///////////////////////////////////////////////////////////////////////////

//= Instance of class with interface driver.

static jhcMpiCam rgb;


///////////////////////////////////////////////////////////////////////////
//                           Main Functions                              //
///////////////////////////////////////////////////////////////////////////

//= Open connection to camera and start background capture thread.
// returns 1 if okay, 0 or negative for error

extern "C" int cam_start ()
{
  return rgb.Start();
}


//= Get a pointer to the most recent color image from sensor.
// returns pixel buffer pointer, NULL if not ready or stream broken

extern "C" const unsigned char *cam_color (int block)
{
  return rgb.Color(block);
}


//= Stop background capture thread.

extern "C" void cam_done ()
{
  rgb.Done();
}

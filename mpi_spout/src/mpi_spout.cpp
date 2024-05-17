// mpi_spout.cpp : speech output with flashing lights for MasterPi robot
//
// Written by Jonathan H. Connell, jconnell@alum.mit.edu
//
///////////////////////////////////////////////////////////////////////////
//
// Copyright 2023-2024 Etaoin Systems
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

#include <jhcMpiSpout.h>


///////////////////////////////////////////////////////////////////////////
//                          Global Variables                             //
///////////////////////////////////////////////////////////////////////////

//= Text-to-speech audio player and LED display driver.

static jhcMpiSpout spout;


///////////////////////////////////////////////////////////////////////////
//                           Main Functions                              //
///////////////////////////////////////////////////////////////////////////

//= Create and configure Text-to-Speech system.
// if sh > 0 then shifts frequency more or less than 100%
// returns 1 if successful, 0 or negative for problem

extern "C" int mpi_start (int sh =0)
{
  return spout.Start(sh);
}


//= Convert text to audio and start playing (does not block).

extern "C" void mpi_say (const char *txt)
{
  spout.Say(txt);
}
  

//= Determine raw color code for sonar LEDs at the current time.
// returns pre-corrected 0xRRGGBB value, negative if not talking

extern "C" int mpi_mouth ()
{
  return spout.Mouth();
}


//= Cleanly shut down system.

extern "C" void mpi_done ()
{
  spout.Done();
}

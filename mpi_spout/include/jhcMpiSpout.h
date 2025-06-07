// jhcMpiSpout.h : speech output with flashing lights for MasterPi robot
//
// Written by Jonathan H. Connell, jconnell@alum.mit.edu
//
///////////////////////////////////////////////////////////////////////////
//
// Copyright 2023-2025 Etaoin Systems
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
#include <time.h>

#include <jhcFestTTS.h>


//= Speech output with flashing lights for MasterPi robot.

class jhcMpiSpout
{
// PRIVATE MEMBER VARIABLES
private:
  // phoneme color sequence
  static const int pmax = 200;    
  float off[pmax];
  int col[pmax];
  int np;

  // TTS engine and background thread
  jhcFestTTS tts;
  pthread_t build;
  pthread_mutex_t vars;
  int building;

  // lip sync timing
  timespec t0;
  int step;


// PUBLIC MEMBER FUNCTIONS
public:
  // creation and initialization
  ~jhcMpiSpout ();
  jhcMpiSpout ();
  int Start (int sh =0);

  // main functions
  void Emotion (int feel =6, int very =0);
  void Say (const char *txt);
  int Mouth ();
  void Done ();


// PRIVATE MEMBER FUNCTIONS
private:
  // main functions
  static void *lips (void *spout);

};

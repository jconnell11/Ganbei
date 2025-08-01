// jhcMpiSpout.cpp : speech output with flashing lights for MasterPi robot
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

#include <math.h>
#include <unistd.h>
#include <string.h>
#include <time.h>

#include <jhcFestTTS.h>

#include <jhcMpiSpout.h>


///////////////////////////////////////////////////////////////////////////
//                      Creation and Initialization                      //
///////////////////////////////////////////////////////////////////////////

//= Destructor cleans up files and any allocated items.

jhcMpiSpout::~jhcMpiSpout ()
{
  Done();
}


//= Default constructor initializes certain values.

jhcMpiSpout::jhcMpiSpout ()
{
  building = 0;
}


//= Create and configure Text-to-Speech system.
// if sh > 0 then shifts frequency more or less than 100%
// returns 1 if successful, 0 or negative for problem

int jhcMpiSpout::Start (int sh)
{
  if (sh > 0)
  {
    tts.shift = sh;
    tts.freq = (int)(0.01 * sh * 105);  
  }
  return tts.Start(100);
}


///////////////////////////////////////////////////////////////////////////
//                              Main Functions                           //
///////////////////////////////////////////////////////////////////////////

//= Set prosody for next utterance based on current emotion.
// feel: 0 bored, 1 happy, 2 sad, 3 angry, 4 scared, 5 excited
// use spout_emo(5, 0) for default neutral tone
// cf. Burkhardt + Sendlmeier 2000 "... Acoustical Correlates ..."

void jhcMpiSpout::Emotion (int feel, int very)
{
  float k = 1.0;                         // exaggeration

  if ((feel <= 0) && (very > 0))         // bored          
    tts.Prosody(-10, -25, -10, k);
  else if (feel <= 0)                    // tired         
    tts.Prosody(-5, -12, -5, k);

  else if ((feel == 1) && (very > 0))    // happy         
    tts.Prosody(20, 50, 0, k);
  else if (feel == 1)                    // pleased       
    tts.Prosody(10, 50, 0, k);

  else if ((feel == 2) && (very > 0))    // sad           
    tts.Prosody(-10, -10, -20, k);
  else if (feel == 2)                    // discontent    
    tts.Prosody(-5, -5, -10, k);

  else if ((feel == 3) && (very > 0))    // angry         
    tts.Prosody(-10, 50, 15, k);
  else if (feel == 3)                    // annoyed       
    tts.Prosody(-5, 25, 7, k);

  else if ((feel == 4) && (very > 0))    // scared        
    tts.Prosody(25, -25, 15, k);
  else if (feel == 4)                    // wary          
    tts.Prosody(12, -12, 7, k);

  else if (very > 0)                     // excited       
    tts.Prosody(10, 50, 20, k);
  else                                   // neutral
    tts.Prosody(0, 0, 0, k);
}


//= Convert text to audio and start playing (does not block).
// requests phoneme file then build lip sync in background 

void jhcMpiSpout::Say (const char *txt)
{
  timespec one_sec;

  tts.Prep(txt);            // causes tts.Poised() > 0
  if (building > 0)
  {
    clock_gettime(CLOCK_REALTIME, &one_sec); 
    one_sec.tv_sec += 1; 
    pthread_timedjoin_np(build, 0, &one_sec);
  }
  building = 1;
  pthread_create(&build, NULL, lips, (void *) this);
}


//= Build table of audio offset times and LED colors.

void *jhcMpiSpout::lips (void *spout)
{
  jhcMpiSpout *me = (jhcMpiSpout *) spout; 
  const char *ph;
  float t;

  // wait until phoneme file has been generated
  while (me->tts.Poised() <= 0)
    sleep(0.001);
  pthread_mutex_lock(&(me->vars));

  // convert phonemes to colors and cache time offsets
  me->np = 0;
  me->step = 0;
  while ((ph = me->tts.Phoneme(me->off[me->np])) != NULL)
  {
    // sock puppet scheme: white for vowels, black otherwise
    me->col[me->np] = 0;
    if ((strchr("aeiou", ph[0]) != NULL) &&
        (strchr("lmn", ph[1]) == NULL))
      me->col[me->np] = 0x502020;
    me->np += 1;
    if (me->np >= me->pmax)
      break;
  } 

  // start audio playback and record beginning time
  me->tts.Emit();
  clock_gettime(CLOCK_BOOTTIME, &(me->t0));
  pthread_mutex_unlock(&(me->vars));
  return NULL;
}


//= Determine raw color code for sonar LEDs at the current time.
// returns pre-corrected 0xRRGGBB value, negative if not talking

int jhcMpiSpout::Mouth () 
{
  timespec now;
  float play;
  int rgb, last = np - 1;

  // make sure audio is playing
  if (tts.Talking() <= 0)
    return -1;
  pthread_mutex_lock(&vars);
  
  // determine color for this time offset
  clock_gettime(CLOCK_BOOTTIME, &now);
  play = (now.tv_sec - t0.tv_sec) + 1.0e-9 * (now.tv_nsec - t0.tv_nsec); 
  while (play >= off[step])
  {
    if (step >= last)
      break;
    step++;
  }
  rgb = col[step]; 
  
  // return corrected color  
  pthread_mutex_unlock(&vars);
  return rgb;
}


//= Cleanly shut down system.

void jhcMpiSpout::Done ()
{
  timespec one_sec;

  if (building > 0)
  {
    clock_gettime(CLOCK_REALTIME, &one_sec); 
    one_sec.tv_sec += 1; 
    pthread_timedjoin_np(build, 0, &one_sec);
  }
  building = 0;
  tts.Done();
}

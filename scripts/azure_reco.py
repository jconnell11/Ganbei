#!/usr/bin/env python3
# encoding: utf-8

# =========================================================================
#
# azure_reco.py : Python wrapper for Microsoft Azure speech recognizer
#
# Written by Jonathan H. Connell, jconnell@alum.mit.edu
#
# =========================================================================
#
# Copyright 2024-2025 Etaoin Systems
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#    http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# 
# =========================================================================

import os, string, yaml, time
import azure.cognitiveservices.speech as az


# Python wrapper for Microsoft Azure speech recognizer
# assumes key and region are in a file in subdirectory "config"
# needs: sudo apt-get install libssl-dev libasound2
# needs: sudo pip3 install azure-cognitiveservices-speech

class AzureReco:

  # clear status variables
  def __init__(self):
    self.show = 0
    self.rc = -3
    self.heard = ""
    self.blob = ""
    self.read = 0
    self.ms = 0


  # connect to speech recognition engine using stored credentials
  # can optionally print out partial results as they become available
  # returns 1 if successful, 0 or negative for problem

  def Start(self, prog =0):

    # retrieve credentials from configuration file
    try:
      with open("config/azure_reco.yaml", 'r') as f:
        data = yaml.load(f, Loader=yaml.FullLoader)
    except:
      return -2
    key = data['sp_key']
    area = data['sp_reg']
    if key == None or area == None or len(key) != 32:
      return -1
    if not all(c in string.hexdigits for c in key):
      return -1

    # create speech recognizer and connect to microphone
    cfg = az.SpeechConfig(subscription=key, region=area)
    cfg.speech_recognition_language="en-US"
    cfg.set_profanity(az.ProfanityOption.Raw)
    cfg.set_property(az.PropertyId.Speech_SegmentationSilenceTimeoutMs, "500")
    mic = az.audio.AudioConfig(use_default_microphone=True)
    self.eng = az.SpeechRecognizer(speech_config=cfg, audio_config=mic)

    # add proper spellings of names from text file
    self.names = az.PhraseListGrammar.from_recognizer(self.eng) 
    with open("config/all_names.txt", 'r') as f:
      for line in f:
        self.names.addPhrase(line.strip())

    # hook up events to callbacks
    self.eng.session_started.connect(self.restore)
    self.eng.recognizing.connect(self.partial)
    self.eng.recognized.connect(self.sentence)
    self.eng.canceled.connect(self.error)
    self.eng.session_stopped.connect(self.dropped)

    # initialize variables
    self.show = prog
    self.rc = -2
    self.blob = ""
    self.read = 0

    # start processing speech and wait for ready status
    self.eng.start_continuous_recognition()
    for i in range(20):
      if self.Status() >= 0:
        return 1
      time.sleep(0.05)
    return 0


  # add some name to grammar to increase likelihood of correct spelling
  # can be called even when recognition is actively running

  def Name(self, person):
    self.names.addPhrase(person)


  # check to see if a new utterance is available
  # return: 2 new result, 1 speaking, 0 silence, negative for error

  def Status(self):
    return self.rc


  # get last utterance heard by the speech recognizer (changes status)

  def Heard(self):
    self.next_chunk()
    return self.heard


  # approximate time (ms) that utterance started before notification

  def Delay(self): 
    return self.ms


  # cleanly disconnect and exit speech recognition

  def Done(self):
    if self.rc > -3:
      self.eng.stop_continuous_recognition()
    self.rc = -3


  # -----------------------------------------------------------------------

  # use Inverse-Text-Normalized form to help break up long lexical results

  def next_chunk (self):

    # clear output then sanity check
    self.heard = ""
    if self.rc < 2:
      return

    # look for terminal punctuation
    last = len(self.blob) - 1
    scan = self.read
    while True:

      # find the first occurence of either ".", "!", or "?"
      end = self.blob.find(".", scan)
      if end < 0:
        end = last + 1
      ex = self.blob.find("!", scan)
      if ex >= 0:
        end = min(ex, end)
      qm = self.blob.find("?", scan)
      if qm >= 0:
        end = min(qm, end)
      if end > last:
        break;

      # ? and ! end immediately but ignore . inside word (e.g. "3.14")
      if self.blob[end] != '.':
        break
      if end < last and self.blob[end + 1] != ' ':
        scan = end + 1
        continue
       
      # possibly expand "Dr" to "drive" (not if "Dr. Jones")
      if end >= 2 and self.blob[end - 2 : end] == "Dr":
        if ((end < last and self.blob[end + 1] != ' ') or 
            (end < last - 1 and not (self.blob[end + 2 : end + 3].isupper() and 
              self.blob[end + 2] != 'F' and self.blob[end + 2] != 'B'))):

          # copy string without "Dr" then add replacement
          self.heard += self.blob[self.read : end - 2]
          self.heard += "drive"
          if end == last:              # end of blob
            self.heard += "."

          # keep scanning after any trailing space
          self.read = end + 1
          scan = self.read
          continue
  
      # check for allowable abbreviations
      for abbr in ["Dr", "Mr", "Ms", "Mrs", "Prof", "St"]:
        bk = len(abbr)
        if bk <= end:
          if self.blob[end - bk : end] == abbr:
            break
      else:
        break                            # true end found      
      scan = end + 1                 

    # copy rest of allowed portion of string
    if end > last:
      self.heard += self.blob[self.read : ]
    else:
      self.heard += self.blob[self.read : end + 1]

    # set up for remainder of blob (if any)
    self.read = end + 1
    if self.read < last:
      self.read += 1                   # skip space
    else:
      self.rc = 0


  # -----------------------------------------------------------------------
  # CALLBACKS

  # system connected properly

  def restore(self, evt):
    self.blob = ""
    self.read = 0
    self.rc = 0    


  # partial result received

  def partial(self, evt):
    if self.show > 0:
      print('  ' + evt.result.text + ' ...')
    self.rc = 1


  # full result received

  def sentence(self, evt):
    res = evt.result.text
    if res != "" and res != "Hey, Cortana.":     # quirk
      self.ms = int(0.0001 * evt.result.duration)
      self.blob = res
      self.read = 0
      self.rc = 2


  # recognition interrupted

  def error(self, evt):
    self.rc = -1


  # disconnected from server

  def dropped(self, evt):
    self.rc = -2


# =========================================================================

# simple continuous speech recognition

if __name__ == "__main__":
  reco = AzureReco()
  print('Initializing ...')
  if reco.Start(1) <= 0:
    print('  >>> Failed!')
  else: 
    print('--- Transcribing 20 seconds ---')
    for i in range(600):               # 20 secs
      rc = reco.Status()
      if rc < 0:
        break
      if rc == 2:
        print('Heard: ' + reco.Heard()) 
      time.sleep(0.033)
    reco.Done()
    print('--- Done ---')

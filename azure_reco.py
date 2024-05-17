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
# Copyright 2024 Etaoin Systems
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

import os, json, string, yaml, time
import azure.cognitiveservices.speech as az


# Python wrapper for Microsoft Azure speech recognizer
# assumes key and region are in a file in subdirectory "config"
# needs: sudo apt-get install libssl-dev libasound2
# needs: sudo pip3 install azure-cognitiveservices-speech

class AzureReco:

  # clear status variables
  def __init__(self):
    self.show = 0
    self.full = ''
    self.rc = -2

    self.add = 0


  # connect to speech recognition engine using stored credentials
  # can optionally print out partial results as they become available
  # returns 1 if successful, 0 or negative for problem

  def Start(self, prog =0):

    # retrieve credentials from configuration file
    path = os.path.dirname(os.path.realpath(__file__))
    try:
      with open(path + "/config/azure_reco.yaml", 'r') as f:
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
    cfg.output_format = az.OutputFormat.Detailed
    mic = az.audio.AudioConfig(use_default_microphone=True)
    self.eng = az.SpeechRecognizer(speech_config=cfg, audio_config=mic)

    # add proper spellings of names from text file
    names = az.PhraseListGrammar.from_recognizer(self.eng) 
    with open(path + "/config/all_names.txt", 'r') as f:
      for line in f:
        names.addPhrase(line.strip())

    # hook up events to callbacks
    self.eng.session_started.connect(self.restore)
    self.eng.recognizing.connect(self.partial)
    self.eng.recognized.connect(self.sentence)
    self.eng.canceled.connect(self.error)
    self.eng.session_stopped.connect(self.dropped)

    # initialize variables
    self.show = prog
    self.full = ''
    self.rc = -2

    # start processing speech and wait for ready status
    self.eng.start_continuous_recognition()
    for i in range(20):
      if self.Status() >= 0:
        return 1
      time.sleep(0.05)
    return 0


  # check to see if a new utterance is available
  # return: 2 new result, 1 speaking, 0 silence, negative for error

  def Status(self):
    val = self.rc
    if val > 0:
      self.rc = 0
    return val


  # get last complete utterance heard by the speech recognizer

  def Heard(self):
    return self.full


  # cleanly disconnect and exit speech recognition

  def Done(self):
    self.eng.stop_continuous_recognition()
    self.full = ''
    self.rc = -2


  # -----------------------------------------------------------------------

  # system connected properly

  def restore(self, evt):
    self.full = ''
    self.rc = 0    


  # partial result received

  def partial(self, evt):
    if self.show > 0:
      print('  ' + evt.result.text + ' ...')
    self.full = ''
    self.rc = 1


  # full result received

  def sentence(self, evt):
    detail = json.loads(evt.result.json)
    self.full = detail['NBest'][0]['Lexical']
    self.rc = 2

    if self.add <= 0:
      phrase_list_grammar.addPhrase("Sridhar")
      self.add = 1


  # recognition interrupted

  def error(self, evt):
    self.full = ''
    self.rc = -1


  # disconnected from server

  def dropped(self, evt):
    self.full = ''
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

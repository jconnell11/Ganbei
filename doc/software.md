## Software Installation

The easiest approach is just to copy a complete system image onto a new [SD card](https://www.amazon.com/dp/B0B7NVMBPL) (32/64GB microSDHX UHS-1). Note that this requires some disassembly of the Master Pi robot to get at the card slot. Contact me if you are interested in access to the image. Otherwise, there is a long list of downloads, substitutions, and configurations as listed below. Eventually there should be an installer script for most of this ...

### Raspian Bullseye

The first thing to do is upgrade to 64bit Raspbian Bullseye (Master Pi comes with 32bit Buster). First, copy off the entire /home/pi directory to an external machine using [RealVNC Viewer](https://www.realvnc.com/en/connect/download/viewer/windows/). This will preserve all the Hiwonder drivers. 

Next download Raspberry [Pi Imager](https://www.raspberrypi.com/software/) for Windows (or equivalent) and select "Legacy Bullseye 64 bit with desktop". Add customization for host, user, wifi, and ssh. Burn a new SD card using [Win32DiskImager](https://win32diskimager.org/) (or equivalent) and insert it into the slot at the front of the lower board. This method is generally much more robust than trying to do an in-place upgrade.

Power on then SSH via wifi into the machine using [PuTTY](https://www.putty.org/) (on Windows). Run the command "sudo raspi-config" then under "Interface Options" enable VNC and I2C. Reboot and the machine should be accessible via RealVNC instead. 

Finally, copy back all the files saved from the /home/pi directory.  

### Additions and Substitutions

Start by copying over the entire Git directory to /home/pi/Ganbei. This is the ALIA reasoning library, a bunch of configuration files of various types, and the Python interface code. It also contains a subproject for offline text-to-speech ([mpi_spout](../mpi_spout)).

Next, the directory [rpi_files](../rpi_files) contains a number of files to add or substitute for similarly named versions in various locations. You should copy items in the [MasterPi](../rpi_files/MasterPi) subdirectory to the same places under /home/pi/MasterPi. All the other subdirectories are condensed versions of the full pathnames where things belong. Note that the items in [usr_local_bin](../rpi_files/usr_local_bin) and [etc_systemd_system](../rpi_files/etc_systemd_system) will need to be marked as executable before being moved to /usr/local/bin and /etc/systemd/system, respectively.

### Configuration

Substitute new code for buttons on expansion board:

    sudo systemctl disable hw_button_scan.service
    sudo systemctl enable mpi_buttons.service

Allow root to use pulseaudio:

    sudo systemctl enable pulseaudio.service
    sudo adduser root pulse-access
    sudo nano /etc/rc.local
    add line: sudo systemctl --global disable pulseaudio.service pulseaudio.socket
    add line: pulseaudio --system &

Change startup sound (optional):

    sudo nano /etc/xdg/lxsession/LXDE-pi/autostart
    add at end: @/usr/bin/aplay /home/pi/Ganbei/R2D2_half.wav

Install TTS software and some system tools:

    sudo apt install festival festival-dev libasound2 soundstretch
    sudo apt install cmake python3-opencv screen

Install Python code checker and device drivers:

    sudo pip3 install --upgrade pip
    sudo pip3 install getch pyyaml pyflakes
    sudo pip3 install rpi.gpio smbus2 rpi_ws281x adafruit-circuitpython-bno08x

### Microsoft Azure

The system is default coded to use Microsoft Azure speech recognition, which is essentially __free__ for low intensity usage. However, you will need credentials to access this on-line service. Start by signing up [here](https://portal.azure.com/#create/Microsoft.CognitiveServicesSpeechServices) (possibly making a Microsoft account first) then select "Speech Services" and "+ Create". Finally, click "Manage keys" and modify local file [azure_reco.yaml](../config/azure_reco.yaml) with valid "Key" and "Location" strings.

---

May 2024 - Jonathan Connell - jconnell@alum.mit.edu



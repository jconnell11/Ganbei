# Software Installation

## Simple Method 

The easiest approach is just to copy a complete system image onto a new SD card (32/64GB microSDHX UHS-1). Note that this requires some disassembly of the Master Pi robot to get at the card slot. [Contact](mailto:jconnell@alum.mit.edu) me if you are interested in access to the image. 

---

## Complicated Method

If you are wary of image downloads, you can instead follow the long list of downloads, substitutions, and configurations listed below.

### Raspian Bullseye

The first thing to do is upgrade to 64bit Raspbian Bullseye (Master Pi comes with 32bit Buster). First, copy off the entire __/home/pi__ directory to an external machine using [RealVNC Viewer](https://www.realvnc.com/en/connect/download/viewer/windows/) or [SSHFS](https://github.com/evsar3/sshfs-win-manager). This will preserve all the Hiwonder drivers. 

Next download Raspberry [Pi Imager](https://www.raspberrypi.com/software/) for Windows (or equivalent) and select "Legacy Bullseye 64 bit with desktop". Add customization for host, user, wifi, and ssh. You should use your __robot name__ as the host name and "pi" as the user name. Burn a new SD card using [Win32DiskImager](https://win32diskimager.org/) (or equivalent) and insert it into the slot at the front of the lower board. This method is generally much more robust than trying to do an in-place upgrade.

Power on then SSH via wifi into the machine using [PuTTY](https://www.putty.org/) (on Windows). Run the command "sudo raspi-config" then under "Interface Options" enable VNC and I2C. Reboot and the machine should be accessible via RealVNC instead. 

Finally, copy back all the files saved from the /home/pi directory.  

### OpenCV with Neural Nets

The alia_vis component uses a neural net face recognition subsystem (not completely debugged yet ..) that assumes OpenCV can compute the forward pass. Unfortunately, the easy-to-install package for OpenCV does not include the correct support, so you need to clone and build the full system. Note that this takes __2+ hours__!

First, clone the source code:

    wget -O opencv.zip https://github.com/opencv/opencv/archive/4.x.zip
    wget -O opencv_contrib.zip https://github.com/opencv/opencv_contrib/archive/4.x.zip
    unzip opencv.zip
    unzip opencv_contrib.zip

Compile the whole system (very slow) and install:

    mkdir -p build && cd build
    cmake -DOPENCV_EXTRA_MODULES_PATH=../opencv_contrib-4.x/modules ../opencv-4.x
    cmake --build . -j $(nproc)
    sudo make install

Cleanup downloads:

    cd ~/
    rm *.zip

### Additions and Substitutions

Start by copying over the entire Git directory to /home/pi/Ganbei. This is the ALIA reasoning library, a bunch of configuration files of various types, and the Python interface code. It also contains a subproject for offline text-to-speech ([mpi_spout](../mpi_spout)) and for background color framegrabbing ([mpi_cam](../mpi_cam)). The source code for generating the Time-of-Flight driver can be downloaded separately from [tof_cam](https://github.com/jconnell11/tof_cam).

Next, the directory [rpi_files](../rpi_files) contains a number of files to add or substitute for similarly named versions in various locations. You should copy items in the [MasterPi](../rpi_files/MasterPi) subdirectory to the same places under /home/pi/MasterPi. All the other subdirectories are condensed versions of the full pathnames where things belong. Note that the items in [usr_local_bin](../rpi_files/usr_local_bin) and [etc_systemd_system](../rpi_files/etc_systemd_system) will need to be marked as executable before being moved to /usr/local/bin and /etc/systemd/system, respectively.

### System Configuration

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
    sudo apt install cmake screen uhubctl wmctrl libssl-dev

Install Python code checker, device drivers, and speech recognition:

    sudo pip3 install --upgrade pip
    sudo pip3 install getch pyyaml pyflakes
    sudo pip3 install rpi.gpio smbus2 rpi_ws281x adafruit-circuitpython-bno08x
    sudo pip3 install azure-cognitiveservices-speech

---

June 2025 - Jonathan Connell - jconnell@alum.mit.edu



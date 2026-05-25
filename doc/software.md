# Software Installation

## Simple Method 

The easiest approach is just to copy a complete system image onto a new [SD card](https://www.amazon.com/dp/B08GY9NYRM) (32GB microSDHC UHS-1). [Contact](mailto:jconnell@alum.mit.edu) me to access the download (free). Transfer the image to the SD card using [Win32DiskImager](https://win32diskimager.org/) (or equivalent) then insert the card into the slot at the front of the Pi processor board. Note that this requires some robot disassembly to get to.

---

## Complicated Method

If you are wary of bulk downloads, you can instead follow the list of piecemeal downloads, substitutions, and configurations listed below.

### Raspbian Bookworm

The first thing to do is make sure you are running 64bit Raspbian Bookworm (Master Pi currently ships with this). If so, you can __skip__ this particular section. 

Start by downloading the Raspberry [Pi Imager](https://www.raspberrypi.com/software/) for Windows (or equivalent) and fire it up. For device select "Pi 4", then for OS select "Raspberry Pi OS (other)" followed by "Raspberry Pi OS (Legacy, 64bit)". After this, insert a blank 32GB SDHC card and add customizations for host, user, wifi, and SSH. You should use your robot name (e.g. "Herbie") as the host name, "pi" as the user name, and enable SSH with password authentication. Finally, write the image to the card then plug it into the slot at the front of the Pi board. 

Power on the robot and SSH into the machine via wifi using [PuTTY](https://www.chiark.greenend.org.uk/~sgtatham/putty/latest.html) (on Windows). Run the command "sudo raspi-config" and under "Advanced Options / Wayland" select "X11", also under "Interface Options" enable both VNC and I2C. After reboot the machine should then be accessible via the more convenient RealVNC instead. 

### OpenCV with Neural Nets

The alia_vis component uses a neural net face recognition subsystem that assumes OpenCV can compute the forward pass. Unfortunately, the easy-to-install package for OpenCV does not include the correct support, so you need to clone and build the full system. Note that this can take __2+ hours__!

First, clone the source code:

    wget -O opencv.zip https://github.com/opencv/opencv/archive/4.x.zip
    wget -O opencv_contrib.zip https://github.com/opencv/opencv_contrib/archive/4.x.zip
    unzip opencv.zip
    unzip opencv_contrib.zip

Compile the whole system (very slow) then install it:

    mkdir -p build && cd build
    cmake -DOPENCV_EXTRA_MODULES_PATH=../opencv_contrib-4.x/modules ../opencv-4.x
    cmake --build . -j $(nproc)
    sudo make install

Cleanup the downloads at the end:

    cd ~/
    rm *.zip

### Additions and Substitutions

Start by copying over the entire Git directory to /home/pi/Ganbei. This is the ALIA reasoning library, a bunch of configuration files of various types, and the Python interface code. It also contains a subproject for offline text-to-speech ([mpi_spout](../mpi_spout)) and for background color framegrabbing ([mpi_cam](../mpi_cam)). The source code for generating the Time-of-Flight driver can be downloaded separately from [tof_cam](https://github.com/jconnell11/tof_cam) if you are interested. You do __not__ need to compile any of these projects since all the shared libraries are already included in the lib/ subdirectory.

Next, the directory [deb12_files](../deb12_files) contains a number of files to add or substitute for similarly named versions in various locations. The subdirectory names are condensed versions of the full pathnames where things belong so, for instance, home_pi should be copied to /home/pi. Note that the items in [usr_local_bin](../deb12_files/usr_local_bin) will need to be marked as executable before being moved to /usr/local/bin.

### System Configuration

Remove some Hiwonder services (if present):

    sudo systemctl disable hw_button_scan.service
    sudo systemctl disable masterpi.service
    sudo systemctl disable hw_find.service
    sudo systemctl disable hw_remote.service

Substitute new code for handling the buttons on the robot expansion board:

    sudo systemctl enable mpi_buttons.service
    sudo systemctl enable mpi_battery.service

Install local TTS software, some system tools, and Microsoft speech recognition:

    sudo apt install festival festival-dev soundstretch 
    sudo apt install pavucontrol uhubctl wmctrl screen
    pip install pyflakes getch azure-cognitiveservices-speech

---

May 2026 - Jonathan Connell - jconnell@alum.mit.edu



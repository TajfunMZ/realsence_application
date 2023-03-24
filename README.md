# realsence_application

Functions for the implementation of the realsence camera, using its rgb and depth options. Currently avaliable functions for the camera (small code adjustments may be necessery):
  - Find points of different color (color criterium has to be manualy set)
  - Align pointcloud with the found 'floor' or with the plain on which the selected points reside
  - Calculate volume within a selected box (can use colored markers or manualy selected points)
  - Calculate distance between different points (currently selected by color and clustering)


### Dependencies

  - Python >= 3.9
  - open3d >= 0.16
  - pyrealsense2
  - numpy
  - xlsxwriter
  - scipy
  
Optional:
  - *cv2*


### Environment & setup

I am using the tools listed below, but so long as the dependencies above are fullfiled any other alternative is acceptable. Italic items are optional.
 
 **Editor & extencions:**
 - visual studio code (with extencions)
    - python
    - *GitLens*
 - Anaconda


**Setup:**
  - Install vs code and its extencions
  - Install Anaconda
  - On Anaconda create a new environment and add the needed python libraries
  - Activate Anaconda in the terminal (open the terminal of the environment by clicking the play button of the environment in anaconda)
  - Add external libraries to the environment by running the following commands:
    - pip install open3d
    - pip install pyrealsense2
  
  - Add necessary files in your working drectory if necessary. File names:
    - configuration_files (saves and loads config data)
    - csv_results (saves measured data on normal measure run)
    - xls_results (saves measured data on xls measurment or on optimisation)
    - assets (saves and loads the pointclouds as .ply files)

You can check the included librares and its versions by running *conda list* in your terminal.
Unles specified otherwise, please keep your dependencies up to date.


### Use instructions

1. Connect the realsence camera to a usb port 3 or higher.
2. Open vs code and run the main file in python.
3. At need add variables after the file name as stated in the receaved message.
4. If necesarry add the necesarry files for the code to store data.

**Syntax**
.../main.py {programe mode} {file name} {other options seperated by space}

**Examples:**
.../main.py calibrate my_file_name  **or**  .../main.py calibrate my_file_name False False
  The first Bool defines if you want to also capture an rgb image for any color recognition.
  The second Bool defines if you whish to save the pointcloud to the assets folder

.../main.py measure my_save_and_config_name  **or**  .../main.py measure my_save_and_config_name 10 my_save_file_extencion
  The first Integer defines the number of measurments you whish to take (increase this for higher accuracy)
  The second String defines the extencion that will appear in the name of the saved excel. Keep in mind that the name length has a limit

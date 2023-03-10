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

You can check the included librares and its versions by running *conda list* in your terminal.
Unles specified otherwise, please keep your dependencies up to date.


### Use instructions

1. Connect the realsence camera to a usb port 3 or higher.
2. Open vs code and run the main file in python.
3. At need add variables after the file name as stated in the receaved message

**Syntax**
.../main.py {programe mode} {file name} {other options seperated by space}

**Examples:**
.../main.py calibrate file_name float(zero volume, if you want to set the volume of an empty object yourself) bool(auto detect color targets) bool(save pcd)
.../main.py measure file_name number_of_iterrations save_file_name_extencion(If you want more measurments with the same configuration file and a more recognisable name)

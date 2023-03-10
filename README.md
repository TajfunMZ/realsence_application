# realsence_application

Functions for the implementation of the realsence camera, using its rgb and depth options. Currently avaliable functions for the camera (small code adjustments may be necessery):
  - Find points of different color (color criterium has to be manualy set)
  - Align pointcloud with the found 'floor' or with the plain on which the selected points reside
  - Calculate volume within a selected box (can use colored markers or manualy selected points)
  - Calculate distance between different points (currently selected by color and clustering)


## Dependencies

  - Python >= 3.9
  - open3d >= 0.16
  - pyrealsense2
  - numpy
  - xlsxwriter
  - scipy
  
Optional:
  - *cv2*


## Environment & setup

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
  - Activate Anaconda in the terminal (can also be done by trying to run the code in vs code by clicking the run button that appears after adding the python extension)
  - Add external libraries to the environment by running the following commands:
    - pip install --user open3d
    - pip install --user pyrealsense2

You can check the included librares and its versions by running *conda list* in your terminal.
Unles specified otherwise, please keep your dependencies up to date.

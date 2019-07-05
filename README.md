# honours_thesis_WIP
Repository where I conducted most of the work for the thesis- repo migrated from Bitbucket. This is the project I did to fulfill my requirements for my undergraduate honours thesis.

This project requires PyBullet which can be installed in the following ways according to which Python version you are running.

For Python 2.x
```
(sudo) pip install PyBullet
```

For Python 3.x
```
pip3 install PyBullet
```

PyBullet's quickstart guide can be found [here](https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit#heading=h.2ye70wns7io3). 
Note that since PyBullet is a python wrapper for a C++ library some windows users might find that their compilers need to be updated before PyBullet can be installed.

To run a script:
```
python -i script_name
```
  A GUI should pop up where you will see the specified body fall into the world. The `-i` allows you to interact with the figure and the GUI once the script starts running.
 
There are many different xml specifications and scripts in this repo which represent my journey through exploring physics based animiation.
I first started by just having a cube fall into the world (fallingCube.py) I then progressed to ragdoll testing with luxo (test_ragdoll.py) and torque testing
(testTorque.py). I also did statue testing with the actual legs specification (statueTest.py) before moving on to trying to drive the legs towards a walking
motion (drivenLegs.py and drivenLegsMujoco.py). If you are not interested in the various interations I suggest you go to the repo containing the final version of my project
[here](https://github.com/kaitlinthachuk/honours_thesis). A full disscussion of the theory behind the project can be found [here](https://github.com/kaitlinthachuk/honours_thesis/blob/master/KaitlinThachuk_CPSC449Thesis.pdf).

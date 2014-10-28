
# README #

**ARTE** is a Matlab toolbox focussed on robotic manipulators, both serial and parallel mechanisms are included.

The main features of **ARTE** are:

* Simulate any industrial robot within a Matlab environment.
* Represent a visualize the Denavit-Hartenberg's system on the robot.
* 3D graphics of a great number of industrial robots are available.
* You can plot and observe the position, velocity and acceleration of the joint coordinates of the robot when a movement is performed.
* Also, the torques and forces at each joint can be plotted.
* A GUI teaching pendant is available to move and program the robots.
* The robots can be programmed and simulated ABB RAPID language. A step by step simulation of the programs can be carried out within the Matlab's editor and debugger.
* A Matlab to RAPID interpreter is included that allows to program the real robot using your Matlab code.
* New robot models can be easily included.
* Practical exercises are provided.

Created by [Arturo Gil](http://arvc.umh.es/personal/arturo/index.php?lang=en&vista=normal&dest=inicio&idp=arturo&type=per&ficha=on): arturo.gil@umh.es. [Miguel Hernández University, Spain.](http://www.umh.es)

ARTE is distributed under LGPL license.

* More information in: http://arvc.umh.es/arte

**INSTALL**
copy the directory containing all the .m files to a directory named arte3.3.X. To initialize the library, inside Matlab you should be placed in the directory where the library is installed:

```
#!matlab

>> pwd
ans =
/Users/arturo/Desktop/arte3.2.X
```


Next, in the Matlab command prompt type:

```
#!matlab

>> init_lib
```


To view all functions and demos type:

```
#!matlab

>>help Contents
```


If you want to have a fast view of the main capabilities of the library, type:

```
#!matlab

>> demos
```


**YOUTUBE CHANNEL**
http://www.youtube.com/playlist?list=PLClKgnzRFYe72qDYmj5CRpR9ICNnQehup

**ITUNES CHANNEL**
https://itunes.apple.com/es/itunes-u/robotica-umh1770-curso-2013/id681094508?mt=10

**PRACTICAL EXERCISES WITH THE LIBRARY**
http://arvc.umh.es/arte/index_en.html#practicals

**LIST OF ROBOTS SUPPORTED BY THE LIBRARY**
http://arvc.umh.es/arte/supported_robots.htmle/supported_robots.html}
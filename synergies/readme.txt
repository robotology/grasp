Welcome to the first release of the Software Synergies Grasper Module.

This module differs from other YARP modules as it intensively uses TypeSafeBottles, that is a type safe backward compatible extension of YARP bottles.

The module uses the bottles for parsing the config file, the required components are in the src folder.
It is possible (but not mandatory) to communicate with the module using typesafe bottles, the specifications and components are in the "common" folder. This self-contained folder contains all the required elements to use typesafe bottles in your own projects for communication or file parsing.

File description:
- app -- configuration files
- cmake -- YARP cmake file
- messages -- TypeSafeBottle release
- src -- header and implementation files
- CMakeLists.txt -- CMakeList file for the Grasper module
- DARWIN_ROOT.ini -- configuration file for YARP version
- mainpage.dox -- doxygen mainpage 
- doc -- documentation folder
- readme.txt -- this file

This module has been developed on YARP version 2.3.22 and has been tested on Linux and Windows.

For queries on this module please contact the authors Giuseppe Cotugno and Kris De Meyer (giuseppe.cotugno@kcl.ac.uk)

Have fun!


The development of this module and the TypeSafeBottles have been supported by EC FP7 DARWIN Project, Grant No. 270138. 
Website: http://darwin-project.eu/

Copyright 2014, King's College London

grasp
=====

A C++ [YARP](https://github.com/robotology/yarp) set of libraries and modules that provides the **iCub** robot with grasping capabilities.

## Installation

##### Dependencies
- [YARP](https://github.com/robotology/yarp)
- [iCub](https://github.com/robotology/icub-main)
- [icub-contrib-common](https://github.com/robotology/icub-contrib-common)
- [stereo-vision](https://github.com/robotology/stereo-vision)
- [segmentation](https://github.com/robotology/segmentation)
- [OpenCV](http://opencv.org/downloads.html)
- [PCL: Point Cloud Library] (http://pointclouds.org/)

#### Notes on Point Cloud Library
PCL has also a lot of dependencies. Most of them can be installed with the classic sudo apt-get install command. You are going to need:

- boost >= 1.4.6
- eigen >= 3.0
- flann >= 1.7.1
- vtk >= 5.6
- qhull 2009

We recommend to install `boost` and `eigen` from `sudo-apt get install`. `Flann` and `vtk` can be installed easily also from sources, but you can still install them using `sudo apt-get install`. The dependency `qhull` needs definitely to be installed from sources, as there is a problem between newer version of qhull and PCL >= 1.7.0, at least on Debian systems. For those who have Windows, it is usually enough to install the dependencies using binaries. Once you have all the dependencies, you can download the source code of `PCL 1.7.0` (or `PCL 1.7.1`), compile it and install it.

## Documentation

Online documentation is available here: [http://robotology.github.io/grasp](http://robotology.github.io/grasp).

## License

Material included here is Copyright of _iCub Facility - Istituto Italiano di Tecnologia_ and is released under the terms of the GPL v2.0 or later. See the file LICENSE for details.

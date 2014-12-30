sawRobotIO1394
==============

This SAW package contains code for interfacing with the Amp1394 QLA (Quad
Linear Amplifier) library.

Links
=====
 * License: http://github.com/jhu-cisst/cisst/blob/master/license.txt
 * JHU-LCSR software: http://jhu-lcsr.github.io/software/
 
Dependencies
============
 * Linux only
 * cisst libraries: https://github.com/jhu-cisst/cisst
 * QLA mechatronics: http://jhu-cisst.github.io/mechatronics

Building without cisst/saw
==========================

The core API ("osa" classes) can be built with or without CISST.
The core API has only a few dependencies on CISST, the largest of which being
cisstVector. If desired, the Eigen3 library can be used instead, by taking
advantage of Eigen's [customization
facilities](http://eigen.tuxfamily.org/dox-devel/TopicCustomizingEigen.html#ExtendingMatrixBase).
In this case, we are extending the `Eigen::Matrix` class by defining an
`EIGEN_MATRIX_PLUGIN` header file with some additional member functions which
make it compatiblw with cisstVector.

To build the core API without CISST, simply add the following definitions to a
CMakeLists.txt file which includes the appropriate source files:

```CMake
add_definitions(
  -DEIGEN_MATRIX_PLUGIN="sawRobotIO1394/cisstVectorEigenAddons.h"
  -DSAW_ROBOT_IO_1394_WO_CISST)
```

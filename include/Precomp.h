//#pragma once

#include "PPFMatch3D.h"
#include "ICP.h"
#include "PPFHelpers.h"

#include <string>
#include <cstdio>
#include <cstdlib>
#include <math.h>
#include <ctime>

#include <fstream>
#include <iostream>
#include <algorithm>

#if defined (_OPENMP)
#include<omp.h>
#endif

#include <sstream>  // flann dependency, needed in precomp now
#include "opencv2/flann/flann.hpp"

#include "c_utils.h"

#include <Eigen/Dense>



typedef Eigen::Vector3d Vec3d;
typedef Eigen::Vector3f Vec3f;
typedef Eigen::Vector4d Vec4d;
typedef Eigen::Matrix3d Matx33d;
typedef Eigen::Matrix4d Matx44d;


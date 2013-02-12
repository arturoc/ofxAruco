#ifndef _ArUco_DrawUtils_H_
#define _ArUco_DrawUtils_H_
#include "aruco.h"

namespace aruco
{
  /**\brief A set of functions to draw in opencv images
   */
  class CvDrawingUtils
  {
  public:
    
    static void draw3dAxis(cv::Mat &Image,Marker &m,CameraParameters &CP);
    
    static void draw3dCube(cv::Mat &Image,Marker &m,CameraParameters &CP);
    
    static void draw3dAxis(cv::Mat &Image,Board &m,CameraParameters &CP);
    
    static void draw3dCube(cv::Mat &Image,Board &m,CameraParameters &CP);

  };
};

#endif


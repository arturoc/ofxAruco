#ifndef _Aruco_Marker_H
#define _Aruco_Marker_H
#include <vector>
#include <iostream>
#include <opencv/cv.h>
#include "cameraparameters.h"
using namespace std;
namespace aruco {
/**\brief This class represents a marker. It is a vector of the fours corners ot the marker
 *
 */

class Marker: public std::vector<cv::Point2f>
{
public:
    //id of  the marker
    int id;
    //size of the markers sides in meters
    float ssize;
    //matrices of rotation and translation respect to the camera
    cv::Mat Rvec,Tvec;

    /**
     */
    Marker();
    /**
     */
    Marker(const Marker &M);
    /**
     */
    ~Marker() {}
    /**Indicates if this object is valid
     */
    bool isValid()const{return id!=-1 && size()==4;}

    /**Draws this marker in the input image
     */
    void draw(cv::Mat &in, cv::Scalar color, int lineWidth=1,bool writeId=true);

    /**Calculates the extrinsics (Rvec and Tvec) of the marker with respect to the camera
     * @param markerSize size of the marker side expressed in meters
     * @param CP parmeters of the camera
     */
    void calculateExtrinsics(float markerSize,const CameraParameters &CP)throw(cv::Exception);
    /**Calculates the extrinsics (Rvec and Tvec) of the marker with respect to the camera
     * @param markerSize size of the marker side expressed in meters
     * @param CameraMatrix matrix with camera parameters (fx,fy,cx,cy)
     * @param Distorsion matrix with distorsion parameters (k1,k2,p1,p2)
     */
    void calculateExtrinsics(float markerSize,cv::Mat  CameraMatrix,cv::Mat Distorsion=cv::Mat())throw(cv::Exception);
    
    /**Given the extrinsic camera parameters returns the GL_MODELVIEW matrix for opengl.
     * Setting this matrix, the reference coordinate system will be set in this marker
     */
    void glGetModelViewMatrix(  double modelview_matrix[16])throw(cv::Exception);

    /**
     */
    friend bool operator<(const Marker &M1,const Marker&M2)
    {
        return M1.id<M2.id;
    }
    /**
     */
    friend ostream & operator<<(ostream &str,const Marker &M)
    {
        str<<M.id<<"=";
        for (int i=0;i<4;i++)
            str<<"("<<M[i].x<< ","<<M[i].y<<") ";
        str<<"Txyz=";
        for (int i=0;i<3;i++)
            str<<M.Tvec.at<float>(i,0)<<" ";
        str<<"Rxyz=";
        for (int i=0;i<3;i++)
            str<<M.Rvec.at<float>(i,0)<<" ";

        return str;
    }
    
    /**
    * \brief Creates an ar marker with the id specified using a modified version of the hamming code.
    * 
    * There are a total of 5 rows of 5 cols. Each row encodes a total of 2 bits, so there are 2^10 bits:(0-1023).
    * 
    * The least significative bytes are first (from left-up to to right-bottom)
    * 
    * Example: the id = 110 (decimal) is be represented in binary as : 00 01 10 11 10.
    * 
    * Then, it will generate the following marker:
    * 
    * -# 1st row encodes 00: 1 0 0 0 0 : hex 0x10
    * -# 2nd row encodes 01: 1 0 1 1 1 : hex 0x17
    * -# 3nd row encodes 10: 0 1 0 0 1 : hex 0x09
    * -# 4th row encodes 11: 0 1 1 1 0 : hex 0x0e
    * -# 5th row encodes 10: 0 1 0 0 1 : hex 0x09
    * 
    * Note that : The first bit, is the inverse of the hamming parity. This avoids the 0 0 0 0 0 to be valid
    */
    static cv::Mat createMarkerImage(int id,int size) throw (cv::Exception);
    
    
private:
  void rotateXAxis(cv::Mat &rotation);

};

}
#endif

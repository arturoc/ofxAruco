#ifndef CV_ARUCO_H
#define CV_ARUCO_H
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cstdio>
#include <iostream>
#include "cameraparameters.h"
#include "marker.h"
using namespace std;

namespace aruco
{

/**\brief Main class for marker detection
 *
 */
class MarkerDetector
{
public:

    /**
     */
    MarkerDetector();

    /**
     */
    ~MarkerDetector();

    /**Detects the markers in the image passed
     *
     * If you provide information about the camera parameters and the size of the marker, then, the extrinsics of the markers are detected
     *
     * @param input input color image
     * @param detectedMarkers output vector with the markers detected
     * @param camMatrix intrinsic camera information.
     * @param distCoeff camera distorsion coefficient. If set Mat() if is assumed no camera distorion
     * @param markerSizeMeters size of the marker sides expressed in meters
     */
    void detect(cv::Mat &input,std::vector<Marker> &detectedMarkers,cv::Mat camMatrix=cv::Mat(),cv::Mat distCoeff=cv::Mat(),float markerSizeMeters=-1) throw (cv::Exception);
    /**Detects the markers in the image passed
     *
     * If you provide information about the camera parameters and the size of the marker, then, the extrinsics of the markers are detected
     *
     * @param input input color image
     * @param detectedMarkers output vector with the markers detected
     * @param camParams Camera parameters
     * @param markerSizeMeters size of the marker sides expressed in meters
     */
    void detect(cv::Mat &input,std::vector<Marker> &detectedMarkers, CameraParameters camParams=CameraParameters(),float markerSizeMeters=-1) throw (cv::Exception);

    /**This set the type of thresholding methods available
     */

    enum ThresholdMethods {FIXED_THRES,ADPT_THRES,CANNY};

    /**Sets the threshold method
     */
    void setThresholdMethod(ThresholdMethods m) {
        _thresMethod=m;
    }
    /**Returns the current threshold method
     */
    ThresholdMethods getThresholdMethod()const {
        return _thresMethod;
    }
    /**
     * Set the parameters of the threshold method
     * We are currently using the Adptive threshold ee opencv doc of adaptiveThreshold for more info
     *   @param param1: blockSize of the pixel neighborhood that is used to calculate a threshold value for the pixel
     *   @param param2: The constant subtracted from the mean or weighted mean
     */
    void setThresholdParams(double param1,double param2) {
        _thresParam1=param1;
        _thresParam2=param2;
    }
    /**
     * Set the parameters of the threshold method
     * We are currently using the Adptive threshold ee opencv doc of adaptiveThreshold for more info
     *   param1: blockSize of the pixel neighborhood that is used to calculate a threshold value for the pixel
     *   param2: The constant subtracted from the mean or weighted mean
     */
    void getThresholdParams(double &param1,double &param2)const {
        param1=_thresParam1;
        param2=_thresParam2;
    }


    /**Returns a reference to the internal image thresholded. It is for visualization purposes and to adjust manually
     * the parameters
     */
    cv::Mat & getThresholdedImage() {
        return thres;
    }


    /**Given the intrinsic camera parameters returns the GL_PROJECTION matrix for opengl.
     * PLease NOTE that when using OpenGL, it is assumed no camera distorsion! So, if it is not true, you should have
     * undistor image
     *
     * @param CamMatrix  arameters of the camera specified.
     * @param orgImgSize size of the original image
     * @param size of the image/window where to render (can be different from the real camera image). Please not that it must be related to CamMatrix
     * @param proj_matrix output projection matrix to give to opengl
     * @param gnear,gfar: visible rendering range
     * @param invert: indicates if the output projection matrix has to yield a horizontally inverted image because image data has not been stored in the order of glDrawPixels: bottom-to-top.
     */
    static void glGetProjectionMatrix( CameraParameters &  CamMatrix,cv::Size orgImgSize, cv::Size size,double proj_matrix[16],double gnear,double gfar,bool invert=false   )throw(cv::Exception);
private:

    cv::Mat grey,thres,thres2;
    std::vector<std::vector<cv::Point> > contours2;
    std::vector<cv::Vec4i> hierarchy2;
    //Threshold parameters
    double _thresParam1,_thresParam2;

    ThresholdMethods _thresMethod;
    /**Given the iput image with markers, creates an output image with it in the canonical position
     * @param in input image
     * @param out image with the marker
     * @param size of out
     * @param points 4 corners of the marker in the image in
     */
    void warp(cv::Mat &in,cv::Mat &out,cv::Size size, std::vector<cv::Point2f> points)throw (cv::Exception);

    int hammDistMarker(cv::Mat  bits);
    int mat2id(cv::Mat &bits);
    /**Correct errors in the markers
     */
    bool correctHammMarker(cv::Mat &bits);

    cv::Mat rotate(cv::Mat  in);

    /**
     * @pram nRotations number of 90deg rotations in clowise direction needed to set the marker in correct position
     */
    int getMarkerId(cv::Mat &in,int &nRotations);

    void drawApproxCurve(cv::Mat &in,std::vector<cv::Point>  &approxCurve ,cv::Scalar color);
    void drawContour(cv::Mat &in,std::vector<cv::Point>  &contour,cv::Scalar  );
    void thresHold(int method,cv::Mat &grey,cv::Mat &out);

    void drawAllContours(cv::Mat input);

    void draw(cv::Mat out,const std::vector<Marker> &markers );
    /**
     */
    bool isInto(cv::Mat &contour,std::vector<cv::Point2f> &b);
    /**
     */
    //bool isInto(vector<Point2f> &a,vector<Point2f> &b);
    /**
     */
    int perimeter(std::vector<cv::Point2f> &a);
    /**
     */
    template<typename T>
    void printMat(cv::Mat M,std::string info="")
    {

        std::cout<<info<<std::endl;
        for (int y=0;y<M.rows;y++)
        {
            for (int x=0;x<M.cols;x++)
            {
                if (sizeof(T)==1)
                    std::cout<<(int) M.at<T>(y,x)<<" ";
                else std::cout<<  M.at<T>(y,x)<<" ";
            }
            std::cout<<std::endl;
        }
    }
    /**
     */
    template<typename T>
    void printMat(CvMat   *M,std::string info="")
    {
        std::cout<<info<<std::endl;
        cv::Mat MM(M);
        for (int y=0;y<MM.rows;y++)
        {
            for (int x=0;x<MM.cols;x++)
            {
                if (sizeof(T)==1)
                    std::cout<<(int) MM.at<T>(y,x)<<" ";
                else std::cout<<  MM.at<T>(y,x)<<" ";
            }
            std::cout<<endl;
        }
    }

    //from ARToolKit

    static void argConvGLcpara2( double cparam[3][4], int width, int height, double gnear, double gfar, double m[16], bool invert )throw(cv::Exception);
    static int  arParamDecompMat( double source[3][4], double cpara[3][4], double trans[3][4] )throw(cv::Exception);
    static double norm( double a, double b, double c );
    static double dot(  double a1, double a2, double a3,
                        double b1, double b2, double b3 );

 

};




};
#endif

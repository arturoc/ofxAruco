#include "cameraparameters.h"
#include <fstream>
#include <opencv/cv.h>
using namespace std;
namespace aruco
{


CameraParameters::CameraParameters() {
    CameraMatrix=cv::Mat();
    Distorsion=cv::Mat();
    CamSize=cv::Size(-1,-1);
}
/**Creates the object from the info passed
 * @param cameraMatrix 3x3 matrix (fx 0 cx, 0 fy cy, 0 0 1)
 * @param distorsionCoeff 4x1 matrix (k1,k2,p1,p2)
 * @param size image size
 */
CameraParameters::CameraParameters(cv::Mat cameraMatrix,cv::Mat distorsionCoeff,cv::Size size) throw(cv::Exception) {
    if (cameraMatrix.rows!=3 || cameraMatrix.cols!=3)
        throw cv::Exception(9000,"invalid input cameraMatrix","CameraParameters::CameraParameters",__FILE__,__LINE__);
    CameraMatrix=cameraMatrix;
    if ( (distorsionCoeff.rows!=0 && distorsionCoeff.rows!=4)  ||( distorsionCoeff.cols!=0 && distorsionCoeff.cols!=1))
        throw cv::Exception(9000,"invalid input distorsionCoeff","CameraParameters::CameraParameters",__FILE__,__LINE__);
    Distorsion=distorsionCoeff;
    CamSize=size;
}
/**
 */
CameraParameters::CameraParameters(const CameraParameters &CI) {
    CI.CameraMatrix.copyTo(CameraMatrix);
    CI.Distorsion.copyTo(Distorsion);
    CamSize=CI.CamSize;
}
 
/**
*/
CameraParameters & CameraParameters::operator=(const CameraParameters &CI) {
    CI.CameraMatrix.copyTo(CameraMatrix);
    CI.Distorsion.copyTo(Distorsion);
    CamSize=CI.CamSize;
    return *this;
}
/**
*/
cv::Point3f CameraParameters::getCameraLocation(cv::Mat Rvec,cv::Mat Tvec)
{
    cv::Mat m33(3,3,CV_32FC1);
    cv::Rodrigues(Rvec, m33)  ;

    cv::Mat m44=cv::Mat::eye(4,4,CV_32FC1);
    for (int i=0;i<3;i++)
        for (int j=0;j<3;j++)
            m44.at<float>(i,j)=m33.at<float>(i,j);

    //now, add translation information
    for (int i=0;i<3;i++)
        m44.at<float>(i,3)=Tvec.at<float>(0,i);
    //invert the matrix
    m44.inv();
    return  cv::Point3f( m44.at<float>(0,0),m44.at<float>(0,1),m44.at<float>(0,2));

}

/**Reads the camera parameters from file
 */
void CameraParameters::readFromFile(string path)throw(cv::Exception)
{

    ifstream file(path.c_str());
    if (!file)  throw cv::Exception(9005,"could not open file:"+path,"CameraParameters::readFromFile",__FILE__,__LINE__);
//Create the matrices
    Distorsion=cv::Mat::zeros(4,1,CV_32FC1);
    CameraMatrix=cv::Mat::eye(3,3,CV_32FC1);
    char line[1024];
    while (!file.eof()) {
        file.getline(line,1024);
        char cmd[20];
        float fval;
        if ( sscanf(line,"%s = %f",cmd,&fval)==2) {
            string scmd(cmd);
            if (scmd=="fx") CameraMatrix.at<float>(0,0)=fval;
            else if (scmd=="cx") CameraMatrix.at<float>(0,2)=fval;
            else if (scmd=="fy") CameraMatrix.at<float>(1,1)=fval;
            else if (scmd=="cy") CameraMatrix.at<float>(1,2)=fval;
            else if (scmd=="k1") Distorsion.at<float>(0,0)=fval;
            else if (scmd=="k2") Distorsion.at<float>(1,0)=fval;
            else if (scmd=="p1") Distorsion.at<float>(2,0)=fval;
            else if (scmd=="p2") Distorsion.at<float>(3,0)=fval;
            else if (scmd=="width") CamSize.width=fval;
            else if (scmd=="height") CamSize.height=fval;
        }
    }
}
/**Saves this to a file
  */
void CameraParameters::saveToFile(string path)throw(cv::Exception)
{
    if (!isValid())  throw cv::Exception(9006,"invalid object","CameraParameters::saveToFile",__FILE__,__LINE__);
    ofstream file(path.c_str());
    if (!file)  throw cv::Exception(9006,"could not open file:"+path,"CameraParameters::saveToFile",__FILE__,__LINE__);
    file<<"# Aruco 1.0 CameraParameters"<<endl;
    file<<"fx = "<<CameraMatrix.at<float>(0,0)<<endl;
    file<<"cx = "<<CameraMatrix.at<float>(0,2)<<endl;
    file<<"fy = "<<CameraMatrix.at<float>(1,1)<<endl;
    file<<"cy = "<<CameraMatrix.at<float>(1,2)<<endl;
    file<<"k1 = "<<Distorsion.at<float>(0,0)<<endl;
    file<<"k2 = "<<Distorsion.at<float>(1,0)<<endl;
    file<<"p1 = "<<Distorsion.at<float>(2,0)<<endl;
    file<<"p2 = "<<Distorsion.at<float>(3,0)<<endl;
    file<<"width = "<<CamSize.width<<endl;
    file<<"height = "<<CamSize.height<<endl;
}

/**Adjust the parameters to the size of the image indicated
 */
void CameraParameters::resize(cv::Size size)throw(cv::Exception)
{
    if (!isValid())  throw cv::Exception(9007,"invalid object","CameraParameters::resize",__FILE__,__LINE__);
    if (size==CamSize) return;
    //now, read the camera size
    //resize the camera parameters to fit this image size
    float AxFactor= float(size.width)/ float(CamSize.width);
    float AyFactor= float(size.height)/ float(CamSize.height);
    CameraMatrix.at<float>(0,0)*=AxFactor;
    CameraMatrix.at<float>(0,2)*=AxFactor;
    CameraMatrix.at<float>(1,1)*=AyFactor;
    CameraMatrix.at<float>(1,2)*=AyFactor;
}

/****
 * 
 * 
 * 
 * 
 */
void CameraParameters::readFromXMLFile(string filePath)throw(cv::Exception)
{
    cv::FileStorage fs(filePath, cv::FileStorage::READ);
    int w=-1,h=-1;
    cv::Mat MCamera,MDist;
    
    fs["image_width"] >> w;
    fs["image_height"] >> h;
    fs["distortion_coefficients"] >> MDist;
    fs["camera_matrix"] >> MCamera;
    
    if (MCamera.cols==0 || MCamera.rows==0)throw cv::Exception(9007,"File :"+filePath+" does not contains valid camera matrix","CameraParameters::readFromXML",__FILE__,__LINE__);
    if (w==-1 || h==0)throw cv::Exception(9007,"File :"+filePath+" does not contains valid camera dimensions","CameraParameters::readFromXML",__FILE__,__LINE__);
    
    if (MCamera.type()!=CV_32FC1) MCamera.convertTo(CameraMatrix,CV_32FC1);
    else CameraMatrix=MCamera;
    if (MDist.type()!=CV_32FC1) MDist.convertTo(Distorsion,CV_32FC1);
    else Distorsion=MDist;
    CamSize.width=w;
    CamSize.height=h;
}
 
};

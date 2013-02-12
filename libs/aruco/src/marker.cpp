#include "marker.h"
#include <cstdio>
using namespace cv;
namespace aruco {
/**
 *
 */
Marker::Marker()
{
    id=-1;
    ssize=-1;
    Rvec.create(3,1,CV_32FC1);
    Tvec.create(3,1,CV_32FC1);
    for (int i=0;i<3;i++)
        Tvec.at<float>(i,0)=Rvec.at<float>(i,0)=-999999;
}
/**
 *
 */
Marker::Marker(const Marker &M):std::vector<cv::Point2f>(M)
{
    M.Rvec.copyTo(Rvec);
    M.Tvec.copyTo(Tvec);
    id=M.id;
    ssize=M.ssize;
}
/**
 *
*/
void Marker::glGetModelViewMatrix(   double modelview_matrix[16])throw(cv::Exception)
{
    //check if paremeters are valid
    bool invalid=false;
    for (int i=0;i<3 && !invalid ;i++)
    {
        if (Tvec.at<float>(i,0)!=-999999) invalid|=false;
        if (Rvec.at<float>(i,0)!=-999999) invalid|=false;
    }
    if (invalid) throw cv::Exception(9003,"extrinsic parameters are not set","Marker::getModelViewMatrix",__FILE__,__LINE__);
    Mat Rot(3,3,CV_32FC1),Jacob;
    Rodrigues(Rvec, Rot, Jacob);

    double para[3][4];
    for (int i=0;i<3;i++)
        for (int j=0;j<3;j++) para[i][j]=Rot.at<float>(i,j);
    //now, add the translation
    para[0][3]=Tvec.at<float>(0,0);
    para[1][3]=Tvec.at<float>(1,0);
    para[2][3]=Tvec.at<float>(2,0);
    double scale=1;

    modelview_matrix[0 + 0*4] = para[0][0];
    // R1C2
    modelview_matrix[0 + 1*4] = para[0][1];
    modelview_matrix[0 + 2*4] = para[0][2];
    modelview_matrix[0 + 3*4] = para[0][3];
    // R2
    modelview_matrix[1 + 0*4] = para[1][0];
    modelview_matrix[1 + 1*4] = para[1][1];
    modelview_matrix[1 + 2*4] = para[1][2];
    modelview_matrix[1 + 3*4] = para[1][3];
    // R3
    modelview_matrix[2 + 0*4] = -para[2][0];
    modelview_matrix[2 + 1*4] = -para[2][1];
    modelview_matrix[2 + 2*4] = -para[2][2];
    modelview_matrix[2 + 3*4] = -para[2][3];
    modelview_matrix[3 + 0*4] = 0.0;
    modelview_matrix[3 + 1*4] = 0.0;
    modelview_matrix[3 + 2*4] = 0.0;
    modelview_matrix[3 + 3*4] = 1.0;
    if (scale != 0.0)
    {
        modelview_matrix[12] *= scale;
        modelview_matrix[13] *= scale;
        modelview_matrix[14] *= scale;
    }


}


void Marker::draw(Mat &in, Scalar color, int lineWidth ,bool writeId)
{
    if (size()!=4) return;
    cv::line( in,(*this)[0],(*this)[1],color,lineWidth,CV_AA);
    cv::line( in,(*this)[1],(*this)[2],color,lineWidth,CV_AA);
    cv::line( in,(*this)[2],(*this)[3],color,lineWidth,CV_AA);
    cv::line( in,(*this)[3],(*this)[0],color,lineWidth,CV_AA);
    cv::rectangle( in,(*this)[0]-Point2f(2,2),(*this)[0]+Point2f(2,2),Scalar(0,0,255),lineWidth,CV_AA);
    cv::rectangle( in,(*this)[1]-Point2f(2,2),(*this)[1]+Point2f(2,2),Scalar(0,255,0),lineWidth,CV_AA);
    cv::rectangle( in,(*this)[2]-Point2f(2,2),(*this)[2]+Point2f(2,2),Scalar(255,0,0),lineWidth,CV_AA);
    if (writeId) {
        char cad[100];
        sprintf(cad,"id=%d",id);
        //determine the centroid
        Point cent(0,0);
        for (int i=0;i<4;i++)
        {
            cent.x+=(*this)[i].x;
            cent.y+=(*this)[i].y;
        }
        cent.x/=4.;
        cent.y/=4.;
        putText(in,cad, cent,FONT_HERSHEY_SIMPLEX, 0.5,  Scalar(255-color[0],255-color[1],255-color[2]),2);
    }
}

/**
 */
void Marker::calculateExtrinsics(float markerSize,const CameraParameters &CP)throw(cv::Exception)
{
    if (!CP.isValid()) throw cv::Exception(9004,"!CP.isValid(): invalid camera parameters. It is not possible to calculate extrinsics","calculateExtrinsics",__FILE__,__LINE__);
    calculateExtrinsics( markerSize,CP.CameraMatrix,CP.Distorsion);
}

/**
 */
void Marker::calculateExtrinsics(float markerSizeMeters,cv::Mat  camMatrix,cv::Mat distCoeff )throw(cv::Exception)
{
    if (!isValid()) throw cv::Exception(9004,"!isValid(): invalid marker. It is not possible to calculate extrinsics","calculateExtrinsics",__FILE__,__LINE__);
    if (markerSizeMeters<=0)throw cv::Exception(9004,"markerSize<=0: invalid markerSize","calculateExtrinsics",__FILE__,__LINE__);
    if ( camMatrix.rows==0 || camMatrix.cols==0) throw cv::Exception(9004,"CameraMatrix is empty","calculateExtrinsics",__FILE__,__LINE__);

    double halfSize=markerSizeMeters/2.;
    CvMat* objPoints=cvCreateMat(4,3,CV_32FC1);
    cvSet2D(objPoints,1,0,cvScalar(-halfSize));
    cvSet2D(objPoints,1,1,cvScalar(halfSize));
    cvSet2D(objPoints,1,2,cvScalar(0));
    cvSet2D(objPoints,2,0,cvScalar(halfSize));
    cvSet2D(objPoints,2,1,cvScalar(halfSize));
    cvSet2D(objPoints,2,2,cvScalar(0));
    cvSet2D(objPoints,3,0,cvScalar(halfSize));
    cvSet2D(objPoints,3,1,cvScalar(-halfSize));
    cvSet2D(objPoints,3,2,cvScalar(0));
    cvSet2D(objPoints,0,0,cvScalar(-halfSize));
    cvSet2D(objPoints,0,1,cvScalar(-halfSize));
    cvSet2D(objPoints,0,2,cvScalar(0));

    CvMat *imagePoints=cvCreateMat(4,2,CV_32FC1);
    CvMat cvCamMatrix=camMatrix;
    CvMat cvDistCoeffs;
    Mat zeros=Mat::zeros(4,1,CV_32FC1);
    if (distCoeff.rows>=4)  cvDistCoeffs=distCoeff;
    else  cvDistCoeffs=zeros;

    //Set image points from the marker
    for (int c=0;c<4;c++)
    {
        cvSet2D( imagePoints,c,0,cvScalar((*this)[c%4].x));
        cvSet2D( imagePoints,c,1,cvScalar((*this)[c%4].y));
    }
    CvMat cvRvec=Rvec;
    CvMat cvTvec=Tvec;
    cvFindExtrinsicCameraParams2(objPoints, imagePoints, &cvCamMatrix, &cvDistCoeffs,&cvRvec,&cvTvec);
    //rotate the X axis so that Y is perpendicular to the marker plane
    rotateXAxis(Rvec);
    ssize=markerSizeMeters;

    cvReleaseMat(&objPoints);
    cvReleaseMat(&imagePoints);
}


/**
*/

void Marker::rotateXAxis(Mat &rotation)
{
    cv::Mat R(3,3,CV_32FC1);
    Rodrigues(rotation, R);
    //create a rotation matrix for x axis
    cv::Mat RX=cv::Mat::eye(3,3,CV_32FC1);
    float angleRad=M_PI/2;
    RX.at<float>(1,1)=cos(angleRad);
    RX.at<float>(1,2)=-sin(angleRad);
    RX.at<float>(2,1)=sin(angleRad);
    RX.at<float>(2,2)=cos(angleRad);
    //now multiply
    R=R*RX;
    //finally, the the rodrigues back
    Rodrigues(R,rotation);
}

/**
*/
Mat Marker::createMarkerImage(int id,int size) throw (cv::Exception)
{
    if (id>=1024) throw cv::Exception(9004,"id>=1024","createMarker",__FILE__,__LINE__);
    Mat marker(size,size, CV_8UC1);
    marker.setTo(Scalar(0));
    //for each line, create
    int swidth=size/7;
    int ids[4]={0x10,0x17,0x09,0x0e};
    for (int y=0;y<5;y++) {
        int index=(id>>2*(4-y)) & 0x0003;
        int val=ids[index];
        for (int x=0;x<5;x++) {
            Mat roi=marker(Rect((x+1)* swidth,(y+1)* swidth,swidth,swidth));
            if ( ( val>>(4-x) ) & 0x0001 ) roi.setTo(Scalar(255));
            else roi.setTo(Scalar(0));
        }
    }
    return marker;
}
}

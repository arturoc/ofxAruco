#include "cvdrawingutils.h"

namespace aruco {
/****
 *
 *
 *
 ****/
void CvDrawingUtils::draw3dAxis(cv::Mat &Image,Marker &m,CameraParameters &CP)
{

    Mat objectPoints (4,3,CV_32FC1);
    objectPoints.at<float>(0,0)=0;
    objectPoints.at<float>(0,1)=0;
    objectPoints.at<float>(0,2)=0;
    objectPoints.at<float>(1,0)=m.ssize;
    objectPoints.at<float>(1,1)=0;
    objectPoints.at<float>(1,2)=0;
    objectPoints.at<float>(2,0)=0;
    objectPoints.at<float>(2,1)=m.ssize;
    objectPoints.at<float>(2,2)=0;
    objectPoints.at<float>(3,0)=0;
    objectPoints.at<float>(3,1)=0;
    objectPoints.at<float>(3,2)=m.ssize;

    vector<Point2f> imagePoints;
    cv::projectPoints( objectPoints, m.Rvec,m.Tvec, CP.CameraMatrix,CP.Distorsion,   imagePoints);
//draw lines of different colours
    cv::line(Image,imagePoints[0],imagePoints[1],Scalar(0,0,255),1,CV_AA);
    cv::line(Image,imagePoints[0],imagePoints[2],Scalar(0,255,0),1,CV_AA);
    cv::line(Image,imagePoints[0],imagePoints[3],Scalar(255,0,0),1,CV_AA);
    putText(Image,"x", imagePoints[1],FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0,0,255),2);
    putText(Image,"y", imagePoints[2],FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0,255,0),2);
    putText(Image,"z", imagePoints[3],FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255,0,0),2);
}

/****
 *
 *
 *
 ****/
void CvDrawingUtils::draw3dCube(cv::Mat &Image,Marker &m,CameraParameters &CP)
{
    Mat objectPoints (8,3,CV_32FC1);
    double halfSize=m.ssize/2;
    objectPoints.at<float>(0,0)=-halfSize;
    objectPoints.at<float>(0,1)=0;
    objectPoints.at<float>(0,2)=-halfSize;
    objectPoints.at<float>(1,0)=halfSize;
    objectPoints.at<float>(1,1)=0;
    objectPoints.at<float>(1,2)=-halfSize;
    objectPoints.at<float>(2,0)=halfSize;
    objectPoints.at<float>(2,1)=0;
    objectPoints.at<float>(2,2)=halfSize;
    objectPoints.at<float>(3,0)=-halfSize;
    objectPoints.at<float>(3,1)=0;
    objectPoints.at<float>(3,2)=halfSize;

    objectPoints.at<float>(4,0)=-halfSize;
    objectPoints.at<float>(4,1)=m.ssize;
    objectPoints.at<float>(4,2)=-halfSize;
    objectPoints.at<float>(5,0)=halfSize;
    objectPoints.at<float>(5,1)=m.ssize;
    objectPoints.at<float>(5,2)=-halfSize;
    objectPoints.at<float>(6,0)=halfSize;
    objectPoints.at<float>(6,1)=m.ssize;
    objectPoints.at<float>(6,2)=halfSize;
    objectPoints.at<float>(7,0)=-halfSize;
    objectPoints.at<float>(7,1)=m.ssize;
    objectPoints.at<float>(7,2)=halfSize;

    vector<Point2f> imagePoints;
    projectPoints( objectPoints, m.Rvec,m.Tvec,  CP.CameraMatrix,CP.Distorsion,   imagePoints);
//draw lines of different colours
    for (int i=0;i<4;i++)
        cv::line(Image,imagePoints[i],imagePoints[(i+1)%4],Scalar(0,0,255),1,CV_AA);

    for (int i=0;i<4;i++)
        cv::line(Image,imagePoints[i+4],imagePoints[4+(i+1)%4],Scalar(0,0,255),1,CV_AA);

    for (int i=0;i<4;i++)
        cv::line(Image,imagePoints[i],imagePoints[i+4],Scalar(0,0,255),1,CV_AA);

}


/****
 *
 *
 *
 ****/
void CvDrawingUtils::draw3dAxis(cv::Mat &Image,Board &B,CameraParameters &CP)
{
Mat objectPoints (4,3,CV_32FC1);
objectPoints.at<float>(0,0)=0;objectPoints.at<float>(0,1)=0;objectPoints.at<float>(0,2)=0;
objectPoints.at<float>(1,0)=2*B[0].ssize;objectPoints.at<float>(1,1)=0;objectPoints.at<float>(1,2)=0;
objectPoints.at<float>(2,0)=0;objectPoints.at<float>(2,1)=2*B[0].ssize;objectPoints.at<float>(2,2)=0;
objectPoints.at<float>(3,0)=0;objectPoints.at<float>(3,1)=0;objectPoints.at<float>(3,2)=2*B[0].ssize;

vector<Point2f> imagePoints;
projectPoints( objectPoints, B.Rvec,B.Tvec, CP.CameraMatrix, CP.Distorsion,   imagePoints);
//draw lines of different colours
cv::line(Image,imagePoints[0],imagePoints[1],Scalar(0,0,255),2,CV_AA);
cv::line(Image,imagePoints[0],imagePoints[2],Scalar(0,255,0),2,CV_AA);
cv::line(Image,imagePoints[0],imagePoints[3],Scalar(255,0,0),2,CV_AA);

putText(Image,"X", imagePoints[1],FONT_HERSHEY_SIMPLEX, 1, Scalar(0,0,255),2);
putText(Image,"Y", imagePoints[2],FONT_HERSHEY_SIMPLEX, 1, Scalar(0,255,0),2);
putText(Image,"Z", imagePoints[3],FONT_HERSHEY_SIMPLEX, 1, Scalar(255,0,0),2);
}


/****
 *
 *
 *
 ****/
void CvDrawingUtils::draw3dCube(cv::Mat &Image,Board &B,CameraParameters &CP)
{

float cubeSize=B[0].ssize;
float txz=-cubeSize/2;
Mat objectPoints (8,3,CV_32FC1);
objectPoints.at<float>(0,0)=txz;objectPoints.at<float>(0,1)=0;objectPoints.at<float>(0,2)=txz;
objectPoints.at<float>(1,0)=txz+cubeSize;objectPoints.at<float>(1,1)=0;objectPoints.at<float>(1,2)=txz;
objectPoints.at<float>(2,0)=txz+cubeSize;objectPoints.at<float>(2,1)=cubeSize;objectPoints.at<float>(2,2)=txz;
objectPoints.at<float>(3,0)=txz;objectPoints.at<float>(3,1)=cubeSize;objectPoints.at<float>(3,2)=txz;

objectPoints.at<float>(4,0)=txz;objectPoints.at<float>(4,1)=0;objectPoints.at<float>(4,2)=txz+cubeSize;
objectPoints.at<float>(5,0)=txz+cubeSize;objectPoints.at<float>(5,1)=0;objectPoints.at<float>(5,2)=txz+cubeSize;
objectPoints.at<float>(6,0)=txz+cubeSize;objectPoints.at<float>(6,1)=cubeSize;objectPoints.at<float>(6,2)=txz+cubeSize;
objectPoints.at<float>(7,0)=txz;objectPoints.at<float>(7,1)=cubeSize;objectPoints.at<float>(7,2)=txz+cubeSize;

vector<Point2f> imagePoints;
projectPoints( objectPoints,B.Rvec,B.Tvec, CP.CameraMatrix, CP.Distorsion,   imagePoints);
//draw lines of different colours
for(int i=0;i<4;i++)
  cv::line(Image,imagePoints[i],imagePoints[(i+1)%4],Scalar(0,0,255),1,CV_AA);

for(int i=0;i<4;i++)
  cv::line(Image,imagePoints[i+4],imagePoints[4+(i+1)%4],Scalar(0,0,255),1,CV_AA);

for(int i=0;i<4;i++)
  cv::line(Image,imagePoints[i],imagePoints[i+4],Scalar(0,0,255),1,CV_AA);
}

}

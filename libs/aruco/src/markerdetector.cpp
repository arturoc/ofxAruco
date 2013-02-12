#include "markerdetector.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <iostream>
#include <fstream>

using namespace std;
using namespace cv;

namespace aruco
{
/************************************
 *
 *
 *
 *
 ************************************/
MarkerDetector::MarkerDetector()
{
    _thresMethod=ADPT_THRES;
    _thresParam1=_thresParam2=7;
}
/************************************
 *
 *
 *
 *
 ************************************/

MarkerDetector::~MarkerDetector()
{

}

void MarkerDetector::detect(cv::Mat &input,std::vector<Marker> &detectedMarkers, CameraParameters camParams ,float markerSizeMeters ) throw (cv::Exception)
{
    detect( input, detectedMarkers,camParams.CameraMatrix ,camParams.Distorsion,  markerSizeMeters );

}

/************************************
 *
 *
 *
 *
 ************************************/
void MarkerDetector::detect(Mat &input,vector<Marker> &detectedMarkers,Mat camMatrix ,Mat distCoeff ,float markerSizeMeters ) throw (cv::Exception)
{
    //it must be a 3 channel image
    if (input.type()!=CV_8UC3)  throw cv::Exception(9000,"input.type()!=CV_8UC3","MarkerDetector::detect",__FILE__,__LINE__);
    vector<Marker> MarkerCanditates;
    detectedMarkers.clear();


    ///Do threshold the image and detect contours
    cv::cvtColor(input,grey,CV_BGR2GRAY);
    thresHold(_thresMethod,grey,thres);
    //pass a copy to findContours because the function modifies it
    thres.copyTo(thres2);
    cv::findContours( thres2 , contours2, hierarchy2,CV_RETR_TREE, CV_CHAIN_APPROX_NONE );
    vector<Point>  approxCurve;
    ///for each contour, analyze if it is a paralelepiped likely to be the marker
    for (unsigned int i=0;i<contours2.size();i++)
    {


        //check it is a possible element by first checking is has enough points
        if (contours2[i].size()>(unsigned int)(input.cols /5))
        {
            //approximate to a poligon
            approxPolyDP(  Mat  (contours2[i]),approxCurve , double(contours2[i].size())*0.05 , true);
            // 				drawApproxCurve(copy,approxCurve,Scalar(0,0,255));
            //check that the poligon has 4 points
            if (approxCurve.size()==4)
            {

//  	   drawContour(input,contours2[i],Scalar(255,0,225));
//  		  namedWindow("input");
//  		imshow("input",input);
//  	 	waitKey(0);
                //and is convex
                if (isContourConvex(Mat  (approxCurve)))
                {
// 					      drawApproxCurve(input,approxCurve,Scalar(255,0,255));
// 						//ensure that the   distace between consecutive points is large enough
                    float minDist=1e10;
                    for (int i=0;i<4;i++)
                    {
                        float d= sqrt( (approxCurve[i].x-approxCurve[(i+1)%4].x)*(approxCurve[i].x-approxCurve[(i+1)%4].x) +
                                       (approxCurve[i].y-approxCurve[(i+1)%4].y)*(approxCurve[i].y-approxCurve[(i+1)%4].y));
                        // 		norm(Mat(approxCurve[i]),Mat(approxCurve[(i+1)%4]));
                        if (d<minDist) minDist=d;
                    }
                    //check that distance is not very small
                    if (minDist>10)
                    {
                        //add the points
                        // 	      cout<<"ADDED"<<endl;
                        MarkerCanditates.push_back(Marker());
                        for (int i=0;i<4;i++)
                        {
                            MarkerCanditates.back().push_back( Point2f(approxCurve[i].x,approxCurve[i].y));
                        }
                    }
                }
            }
        }
    }
// 		 		  namedWindow("input");
//  		imshow("input",input);
//  						waitKey(0);
    ///sort the points in anti-clockwise order
    for (unsigned int i=0;i<MarkerCanditates.size();i++)
    {

        //trace a line between the first and second point.
        //if the thrid point is at the right side, then the points are anti-clockwise
        double dx1 = MarkerCanditates[i][1].x - MarkerCanditates[i][0].x;
        double dy1 =  MarkerCanditates[i][1].y - MarkerCanditates[i][0].y;
        double dx2 = MarkerCanditates[i][2].x - MarkerCanditates[i][0].x;
        double dy2 = MarkerCanditates[i][2].y - MarkerCanditates[i][0].y;
        double o = (dx1*dy2)-(dy1*dx2);

        if (o  < 0.0)		 //if the third point is in the left side, then sort in anti-clockwise order
        {
            swap(MarkerCanditates[i][1],MarkerCanditates[i][3]);

        }
    }
    /// remove these elements whise corners are too close to each other
    //first detect candidates

    vector<pair<int,int>  > TooNearCandidates;
    for (unsigned int i=0;i<MarkerCanditates.size();i++)
    {
        // 	cout<<"Marker i="<<i<<MarkerCanditates[i]<<endl;
        //calculate the average distance of each corner to the nearest corner of the other marker candidate
        for (unsigned int j=i+1;j<MarkerCanditates.size();j++)
        {
            float dist=0;
            for (int c=0;c<4;c++)
                dist+= sqrt(  (MarkerCanditates[i][c].x-MarkerCanditates[j][c].x)*(MarkerCanditates[i][c].x-MarkerCanditates[j][c].x)+(MarkerCanditates[i][c].y-MarkerCanditates[j][c].y)*(MarkerCanditates[i][c].y-MarkerCanditates[j][c].y));
            dist/=4;
            //if distance is too small
            if (dist< 10) {
                TooNearCandidates.push_back(pair<int,int>(i,j));
            }
        }
    }
    //mark for removal the element of  the pair with smaller perimeter
    vector<bool> toRemove (MarkerCanditates.size());
    for (unsigned int i=0;i<toRemove.size();i++) toRemove[i]=false;

    for (unsigned int i=0;i<TooNearCandidates.size();i++) {
        if ( perimeter(MarkerCanditates[TooNearCandidates[i].first ])>perimeter(MarkerCanditates[ TooNearCandidates[i].second] ))
            toRemove[TooNearCandidates[i].second]=true;
        else toRemove[TooNearCandidates[i].first]=true;
    }

    ///identify the markers
    for (unsigned int i=0;i<MarkerCanditates.size();i++)
    {
        if (!toRemove[i])
        {
//             MarkerCanditates[i].draw(input,cv::Scalar(0,255,0),1,false);
            //Find proyective homography
            Mat canonicalMarker;
            warp(input,canonicalMarker,Size(50,50),MarkerCanditates[i]);
            int nRotations;
            int id=getMarkerId(canonicalMarker,nRotations);
            if (id!=-1)
            {
                detectedMarkers.push_back(MarkerCanditates[i]);
                detectedMarkers.back().id=id;
                //sort the points so that they are always in the same order no matter the camera orientation
                std::rotate(detectedMarkers.back().begin(),detectedMarkers.back().begin()+4-nRotations,detectedMarkers.back().end());
            }
        }
    }
    //namedWindow("input");
    //imshow("input",input);
    //				waitKey(0);
    ///refine using subpixel accuracy the  corners
    if (detectedMarkers.size()>0)
    {
        vector<Point2f> Corners;
        for (unsigned int i=0;i<detectedMarkers.size();i++)
            for (int c=0;c<4;c++)
                Corners.push_back(detectedMarkers[i][c]);
        cornerSubPix(grey, Corners,cvSize(3,3), cvSize(-1,-1)   ,cvTermCriteria ( CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,15,0.05 ));
        //copy back
        for (unsigned int i=0;i<detectedMarkers.size();i++)
            for (int c=0;c<4;c++)     detectedMarkers[i][c]=Corners[i*4+c];
    }
    //sort by id
    std::sort(detectedMarkers.begin(),detectedMarkers.end());
    //there might be still the case that a marker is detected twice because of the double border indicated earlier,
    //detect and remove these cases
    toRemove.resize(detectedMarkers.size());
    for (unsigned int i=0;i<detectedMarkers.size();i++) toRemove[i]=false;

    for (int i=0;i<int(detectedMarkers.size())-1;i++) {
        if (detectedMarkers[i].id==detectedMarkers[i+1].id && !toRemove[i+1] ) {
            //deletes the one with smaller perimeter
            if (perimeter(detectedMarkers[i])>perimeter(detectedMarkers[i+1])) toRemove[i+1]=true;
            else toRemove[i]=true;
        }
    }
    //now, remove
    vector<Marker>::iterator it=detectedMarkers.begin();
    for (unsigned int i=0;i<toRemove.size();i++) {
        if (toRemove[i]) it=detectedMarkers.erase(it);
        else it++;
    }
    ///detect the position of detected markers if desired
    
    if (camMatrix.rows!=0  && markerSizeMeters>0)
    {
         for (unsigned int i=0;i<detectedMarkers.size();i++)
	      detectedMarkers[i].calculateExtrinsics(markerSizeMeters,camMatrix,distCoeff); 
    }


}

/************************************
 *
 *
 *
 *
 ************************************/
void MarkerDetector::thresHold(int method,Mat &grey,Mat &out)
{
    switch (method)
    {
    case FIXED_THRES:
        cv::threshold(grey, out, _thresParam1,255, CV_THRESH_BINARY_INV );
        break;
    case ADPT_THRES://currently, this is the best method
        cv::adaptiveThreshold(grey,out,255,ADAPTIVE_THRESH_GAUSSIAN_C,THRESH_BINARY_INV,_thresParam1,_thresParam2);
// 				cv::erode(out,grey,Mat());
// 				grey.copyTo(out);
        break;
    case CANNY:
    {
        //this should be the best method, and generally it is.
        //However, some times there are small holes in the marker contour that makes
        //the contour detector not to find it properly
        //if there is a missing pixel
        cv::Canny(grey, out, 10, 220);
        //I've tried a closing but it add many more points that some
        //times makes this even worse
// 			  Mat aux;
// 			  cv::morphologyEx(thres,aux,MORPH_CLOSE,Mat());
// 			  out=aux;
    }break;
    }
}
/************************************
 *
 *
 *
 *
 ************************************/
void MarkerDetector::drawAllContours(Mat input)
{
    drawContours( input,  contours2, -1,Scalar(255,0,255));
}

/************************************
 *
 *
 *
 *
 ************************************/
void MarkerDetector:: drawContour(Mat &in,vector<Point>  &contour,Scalar color  )
{
    for (unsigned int i=0;i<contour.size();i++)
    {
        cv::rectangle(in,contour[i],contour[i],color);
    }
}

void  MarkerDetector:: drawApproxCurve(Mat &in,vector<Point>  &contour,Scalar color  )
{
    for (unsigned int i=0;i<contour.size();i++)
    {
        cv::line( in,contour[i],contour[(i+1)%contour.size()],color);
    }
}
/************************************
 *
 *
 *
 *
 ************************************/

void MarkerDetector::draw(Mat out,const vector<Marker> &markers )
{
    for (unsigned int i=0;i<markers.size();i++)
    {
        cv::line( out,markers[i][0],markers[i][1],cvScalar(255,0,0),2,CV_AA);
        cv::line( out,markers[i][1],markers[i][2],cvScalar(255,0,0),2,CV_AA);
        cv::line( out,markers[i][2],markers[i][3],cvScalar(255,0,0),2,CV_AA);
        cv::line( out,markers[i][3],markers[i][0],cvScalar(255,0,0),2,CV_AA);
    }
}
/************************************
 *
 *
 *
 *
 ************************************/

void MarkerDetector::warp(Mat &in,Mat &out,Size size, vector<Point2f> points)throw (cv::Exception)
{

    if (points.size()!=4)    throw cv::Exception(9001,"point.size()!=4","PerpectiveWarper::warp",__FILE__,__LINE__);
    //obtain the perspective transform
    Point2f  pointsRes[4],pointsIn[4];
    for (int i=0;i<4;i++) pointsIn[i]=points[i];
    pointsRes[0]=(Point2f(0,0));
    pointsRes[1]= Point2f(size.width-1,0);
    pointsRes[2]= Point2f(size.width-1,size.height-1);
    pointsRes[3]= Point2f(0,size.height-1);
    //        cout<<pointsIn[0].x<< " "<<pointsIn[0].y<<
    Mat M=getPerspectiveTransform(pointsIn,pointsRes);
    cv::warpPerspective(in, out,  M, size);

}
/************************************
 *
 *
 *
 *
 ************************************/
int MarkerDetector::hammDistMarker(Mat  bits)
{
    int ids[4][5]=
    {
        {
            1,0,0,0,0
        }
        ,
        {
            1,0,1,1,1
        }
        ,
        {
            0,1,0,0,1
        }
        ,
        {
            0, 1, 1, 1, 0
        }
    };
    int dist=0;

    for (int y=0;y<5;y++)
    {
        int minSum=1e5;
        //hamming distance to each possible word
        for (int p=0;p<4;p++)
        {
            int sum=0;
            //now, count
            for (int x=0;x<5;x++)
                sum+=  bits.at<uchar>(y,x) == ids[p][x]?0:1;
            if (minSum>sum) minSum=sum;
        }
        //do the and
        dist+=minSum;
    }

    return dist;
}
/************************************
 *
 *
 *
 *
 ************************************/

int MarkerDetector::mat2id(Mat &bits)
{
    int val=0;
    for (int y=0;y<5;y++)
    {
        val<<=1;
        if ( bits.at<uchar>(y,1)) val|=1;
        val<<=1;
        if ( bits.at<uchar>(y,3)) val|=1;
    }
    return val;
}
/************************************
 *
 *
 *
 *
 ************************************/
bool MarkerDetector::correctHammMarker(Mat &bits)
{
    //detect this lines with errors
    bool errors[4];
    int ids[4][5]=
    {
        {
            0,0,0,0,0
        }
        ,
        {
            0,0,1,1,1
        }
        ,
        {
            1,1,0,0,1
        }
        ,
        {
            1, 1, 1, 1, 0
        }
    };

    for (int y=0;y<5;y++)
    {
        int minSum=1e5;
        //hamming distance to each possible word
        for (int p=0;p<4;p++)
        {
            int sum=0;
            //now, count
            for (int x=0;x<5;x++)
                sum+=  bits.at<uchar>(y,x) == ids[p][x]?0:1;
            if (minSum>sum) minSum=sum;
        }
        if (minSum!=0) errors[y]=true;
        else errors[y]=false;
    }

    return true;
}

/************************************
 *
 *
 *
 *
 ************************************/
Mat MarkerDetector::rotate(Mat  in)
{
    Mat out;
    in.copyTo(out);
    for (int i=0;i<in.rows;i++)
    {
        for (int j=0;j<in.cols;j++)
        {
            out.at<uchar>(i,j)=in.at<uchar>(in.cols-j-1,i);
        }
    }
    return out;
}

/************************************
 *
 *
 *
 *
 ************************************/
int MarkerDetector::getMarkerId(Mat &in,int &nRotations)
{
    assert(in.rows==in.cols);
    Mat grey;
    if ( in.type()==CV_8UC1) grey=in;
    else cv::cvtColor(in,grey,CV_BGR2GRAY);
    //threshold image
    threshold(grey, grey,125, 255, THRESH_BINARY|THRESH_OTSU);



    //   namedWindow("m");
    //   imshow("m",in);
    /*
          namedWindow("m2");
     imshow("m2",grey); */

    //Markers  are divided in 7x7 regions, of which the inner 5x5 belongs to marker info
    //the external border shoould be entirely black

    int swidth=in.rows/7;
    for (int y=0;y<7;y++)
    {
        int inc=6;
        if (y==0 || y==6) inc=1;//for first and last row, check the whole border
        for (int x=0;x<7;x+=inc)
        {
            int Xstart=(x)*(swidth);
            int Ystart=(y)*(swidth);
            Mat square=grey(Rect(Xstart,Ystart,swidth,swidth));
            int nZ=countNonZero(square);
            if (nZ> (swidth*swidth) /2) {
                return -1;//can not be a marker because the border element is not black!
            }
        }
    }

    //now,
    vector<int> markerInfo(5);
    Mat _bits=Mat::zeros(5,5,CV_8UC1);
    //get information(for each inner square, determine if it is  black or white)

    for (int y=0;y<5;y++)
    {

        for (int x=0;x<5;x++)
        {
            int Xstart=(x+1)*(swidth);
            int Ystart=(y+1)*(swidth);
            Mat square=grey(Rect(Xstart,Ystart,swidth,swidth));
            int nZ=countNonZero(square);
            if (nZ> (swidth*swidth) /2)  _bits.at<uchar>( y,x)=1;
        }
    }
// 		printMat<uchar>( _bits,"or mat");

    //checkl all possible rotations
    Mat _bitsFlip;
    Mat Rotations[4];
    Rotations[0]=_bits;
    int dists[4];
    dists[0]=hammDistMarker( Rotations[0]) ;
    pair<int,int> minDist( dists[0],0);
    for (int i=1;i<4;i++)
    {
        //rotate
        Rotations[i]=rotate(Rotations[i-1]);
        //get the hamming distance to the nearest possible word
        dists[i]=hammDistMarker( Rotations[i]) ;
        if (dists[i]<minDist.first)
        {
            minDist.first=  dists[i];
            minDist.second=i;
        }
    }
// 		        printMat<uchar>( Rotations [ minDist.second]);
// 		 	cout<<"MinDist="<<minDist.first<<" "<<minDist.second<<endl;

    nRotations=minDist.second;
    if (minDist.first!=0)	 //FUTURE WORK: correct if any error
        return -1;
    else return mat2id(Rotations [ minDist.second]);

    ////!!Add another bit to avoid 1 bit errors that are frequent
    /*	  if (! correctHammMarker(Rotations [ minDist.second] )) return -1;
    	  cout<<"ID="<<mat2id(Rotations [ minDist.second])<<endl;
    	  waitKey(0);
    	   return mat2id(Rotations [ minDist.second]);*/
}
/************************************
 *
 *
 *
 *

bool MarkerDetector::isInto(vector<Point2f> &a,vector<Point2f> &b)
{//NOT TESTED
	 	  CvMat  contour(b.size(),1,CV_32FC2);
		  float *ptr=contour.ptr<float>(0);
		  for(unsigned int i=0;i<a.size();i++){
			*(ptr++)=b[i].x;
			*(ptr++)=b[i].y;
		}
	for(unsigned int i=0;i<b.size();i++)
		if ( pointPolygonTest( contour,b[i],false)>0) return true;
	return false;
}
*/
/************************************
 *
 *
 *
 *
 ************************************/
bool MarkerDetector::isInto(Mat &contour,vector<Point2f> &b)
{

    for (unsigned int i=0;i<b.size();i++)
        if ( pointPolygonTest( contour,b[i],false)>0) return true;
    return false;
}
/************************************
 *
 *
 *
 *
 ************************************/
int MarkerDetector:: perimeter(vector<Point2f> &a)
{
    int sum=0;
    for (unsigned int i=0;i<a.size();i++) {
        int i2=(i+1)%a.size();
        sum+= sqrt ( (a[i].x-a[i2].x)*(a[i].x-a[i2].x)+(a[i].y-a[i2].y)*(a[i].y-a[i2].y) ) ;
    }
    return sum;
}
/************************************
 *
 *
 *
 *
 ************************************/
// 	bool MarkerDetector::isInto(vector<Point2f> &a,vector<Point2f> &b)
// 	{
// 				Mat contour(1,a.size(),CV_32FC2);
// 				float *ptr=(float *)a.ptr(0);
// 				for(unsigned int p=0;p<detectedMarkers[i].size();p++)
// 				{
// 					*ptr++=a[p].x;
// 					*ptr++=a[p].y;
// 				}
// 		for(unsigned int i=0;i<b.size();i++)
// 			if ( pointPolygonTest( contour,b[i],false)>0) return true;
// 		return false;
// 	}

/**
 *
 */
void MarkerDetector::glGetProjectionMatrix(CameraParameters & CP,Size orgImgSize,Size size,double proj_matrix[16],double gnear,double gfar,bool invert )throw(cv::Exception)
{
      if (CP.isValid()==false) throw cv::Exception(9100,"invalid camera parameters","MarkerDetector::glGetProjectionMatrix",__FILE__,__LINE__);
    //Deterime the rsized info
    double Ax=double(size.width)/double(orgImgSize.width);
    double Ay=double(size.height)/double(orgImgSize.height);
    double _fx=CP.CameraMatrix.at<float>(0,0)*Ax;
    double _cx=CP.CameraMatrix.at<float>(0,2)*Ax;
    double _fy=CP.CameraMatrix.at<float>(1,1)*Ay;
    double _cy=CP.CameraMatrix.at<float>(1,2)*Ay;
    double cparam[3][4] =
    {
        {
            _fx,  0,  _cx,  0
        },
        {0,          _fy,  _cy, 0},
        {0,      0,      1,      0}
    };

    argConvGLcpara2( cparam, size.width, size.height, gnear, gfar, proj_matrix, invert );

}

void MarkerDetector::argConvGLcpara2( double cparam[3][4], int width, int height, double gnear, double gfar, double m[16], bool invert )throw(cv::Exception)
{

    double   icpara[3][4];
    double   trans[3][4];
    double   p[3][3], q[4][4];
    int      i, j;

    cparam[0][2] *= -1.0;
    cparam[1][2] *= -1.0;
    cparam[2][2] *= -1.0;

    if ( arParamDecompMat(cparam, icpara, trans) < 0 )
        throw cv::Exception(9002,"parameter error","MarkerDetector::argConvGLcpara2",__FILE__,__LINE__);

    for ( i = 0; i < 3; i++ )
    {
        for ( j = 0; j < 3; j++ )
        {
            p[i][j] = icpara[i][j] / icpara[2][2];
        }
    }
    q[0][0] = (2.0 * p[0][0] / width);
    q[0][1] = (2.0 * p[0][1] / width);
    q[0][2] = ((2.0 * p[0][2] / width)  - 1.0);
    q[0][3] = 0.0;

    q[1][0] = 0.0;
    q[1][1] = (2.0 * p[1][1] / height);
    q[1][2] = ((2.0 * p[1][2] / height) - 1.0);
    q[1][3] = 0.0;

    q[2][0] = 0.0;
    q[2][1] = 0.0;
    q[2][2] = (gfar + gnear)/(gfar - gnear);
    q[2][3] = -2.0 * gfar * gnear / (gfar - gnear);

    q[3][0] = 0.0;
    q[3][1] = 0.0;
    q[3][2] = 1.0;
    q[3][3] = 0.0;

    for ( i = 0; i < 4; i++ )
    {
        for ( j = 0; j < 3; j++ )
        {
            m[i+j*4] = q[i][0] * trans[0][j]
                       + q[i][1] * trans[1][j]
                       + q[i][2] * trans[2][j];
        }
        m[i+3*4] = q[i][0] * trans[0][3]
                   + q[i][1] * trans[1][3]
                   + q[i][2] * trans[2][3]
                   + q[i][3];
    }

    if (!invert)
    {
        m[13]=-m[13] ;
        m[1]=-m[1];
        m[5]=-m[5];
        m[9]=-m[9];
    }

}

int  MarkerDetector::arParamDecompMat( double source[3][4], double cpara[3][4], double trans[3][4] )throw(cv::Exception)
{
    int       r, c;
    double    Cpara[3][4];
    double    rem1, rem2, rem3;

    if ( source[2][3] >= 0 )
    {
        for ( r = 0; r < 3; r++ )
        {
            for ( c = 0; c < 4; c++ )
            {
                Cpara[r][c] = source[r][c];
            }
        }
    }
    else
    {
        for ( r = 0; r < 3; r++ )
        {
            for ( c = 0; c < 4; c++ )
            {
                Cpara[r][c] = -(source[r][c]);
            }
        }
    }

    for ( r = 0; r < 3; r++ )
    {
        for ( c = 0; c < 4; c++ )
        {
            cpara[r][c] = 0.0;
        }
    }
    cpara[2][2] = norm( Cpara[2][0], Cpara[2][1], Cpara[2][2] );
    trans[2][0] = Cpara[2][0] / cpara[2][2];
    trans[2][1] = Cpara[2][1] / cpara[2][2];
    trans[2][2] = Cpara[2][2] / cpara[2][2];
    trans[2][3] = Cpara[2][3] / cpara[2][2];

    cpara[1][2] = dot( trans[2][0], trans[2][1], trans[2][2],
                       Cpara[1][0], Cpara[1][1], Cpara[1][2] );
    rem1 = Cpara[1][0] - cpara[1][2] * trans[2][0];
    rem2 = Cpara[1][1] - cpara[1][2] * trans[2][1];
    rem3 = Cpara[1][2] - cpara[1][2] * trans[2][2];
    cpara[1][1] = norm( rem1, rem2, rem3 );
    trans[1][0] = rem1 / cpara[1][1];
    trans[1][1] = rem2 / cpara[1][1];
    trans[1][2] = rem3 / cpara[1][1];

    cpara[0][2] = dot( trans[2][0], trans[2][1], trans[2][2],
                       Cpara[0][0], Cpara[0][1], Cpara[0][2] );
    cpara[0][1] = dot( trans[1][0], trans[1][1], trans[1][2],
                       Cpara[0][0], Cpara[0][1], Cpara[0][2] );
    rem1 = Cpara[0][0] - cpara[0][1]*trans[1][0] - cpara[0][2]*trans[2][0];
    rem2 = Cpara[0][1] - cpara[0][1]*trans[1][1] - cpara[0][2]*trans[2][1];
    rem3 = Cpara[0][2] - cpara[0][1]*trans[1][2] - cpara[0][2]*trans[2][2];
    cpara[0][0] = norm( rem1, rem2, rem3 );
    trans[0][0] = rem1 / cpara[0][0];
    trans[0][1] = rem2 / cpara[0][0];
    trans[0][2] = rem3 / cpara[0][0];

    trans[1][3] = (Cpara[1][3] - cpara[1][2]*trans[2][3]) / cpara[1][1];
    trans[0][3] = (Cpara[0][3] - cpara[0][1]*trans[1][3]
                   - cpara[0][2]*trans[2][3]) / cpara[0][0];

    for ( r = 0; r < 3; r++ )
    {
        for ( c = 0; c < 3; c++ )
        {
            cpara[r][c] /= cpara[2][2];
        }
    }

    return 0;
}

double MarkerDetector::norm( double a, double b, double c )
{
    return( sqrt( a*a + b*b + c*c ) );
}

double MarkerDetector::dot( double a1, double a2, double a3,
                              double b1, double b2, double b3 )
{
    return( a1 * b1 + a2 * b2 + a3 * b3 );
}

/*
 void MarkerDetector::getExtrinsicsParams(double m_modelview[16], guimage::ExtrinsicParams  &OutExt)throw(cv::Exception)
 {
  CvMat* inMatrix=cvCreateMat(3,3,CV_32FC1);
  CvMat* vectorRotation=cvCreateMat(1,3,CV_32FC1);

  cvSet2D( inMatrix , 0,0, cvScalar(m_modelview[0 + 0*4] ) );
  cvSet2D( inMatrix , 0,1, cvScalar(m_modelview[0 + 1*4] ) );
  cvSet2D( inMatrix , 0,2, cvScalar(m_modelview[0 + 2*4] ) );
  cvSet2D( inMatrix , 1,0, cvScalar(m_modelview[1 + 0*4] ) );
  cvSet2D( inMatrix , 1,1, cvScalar(m_modelview[1 + 1*4] ) );
  cvSet2D( inMatrix , 1,2, cvScalar(m_modelview[1 + 2*4] ) );
  cvSet2D( inMatrix , 2,0, cvScalar(-m_modelview[2 + 0*4] ) );
  cvSet2D( inMatrix , 2,1, cvScalar(-m_modelview[2 + 1*4] ) );
  cvSet2D( inMatrix , 2,2, cvScalar(-m_modelview[2 + 2*4] ) );

  cvRodrigues2(inMatrix,vectorRotation) ;
  OutExt.setParams(m_modelview[0 + 3*4],m_modelview[1 + 3*4],-m_modelview[2 + 3*4],cvGet2D(vectorRotation,0,0).val[0],cvGet2D(vectorRotation,0,1).val[0],cvGet2D(vectorRotation,0,2).val[0]);

  cvReleaseMat(&inMatrix);
  cvReleaseMat(&vectorRotation);

 }*/


 

};

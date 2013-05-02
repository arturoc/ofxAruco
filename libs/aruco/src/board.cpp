#include "board.h"
#include <fstream>
using namespace std;
using namespace cv;
namespace aruco {

	cv::Mat Board::createBoardImage( cv::Size gridSize,int MarkerSize,int MarkerDistance,unsigned int FirstMarkerID, BoardConfiguration& TInfo  ) throw (cv::Exception)
{

    vector<vector<int> > MarkersIds;
//     vector<vector<int> > *TheMarkersIds=NULL;

    /*srand(time(NULL));
    TInfo._markersId.create(gridSize,CV_32SC1);
    int nMarkers=gridSize.height*gridSize.width;
    if (FirstMarkerID+nMarkers>=1024)
        throw cv::Exception(9189,"Creation of board impies a marker with an impossible id:","aruco::createBoard",__FILE__,__LINE__);
    unsigned int idp=0;
    for (  int i=0;i<gridSize.height;i++)
        for (  int j=0;j<gridSize.width;j++,idp++)
            TInfo._markersId.at<int>(i,j)=FirstMarkerID+idp; //number in the range [0,1023]




    TInfo._markerSizePix=MarkerSize;
    TInfo._markerDistancePix=MarkerDistance;*/

    int sizeY=gridSize.height*MarkerSize+(gridSize.height-1)*MarkerDistance;
    int sizeX=gridSize.width*MarkerSize+(gridSize.width-1)*MarkerDistance;

    Mat tableImage(sizeY,sizeX,CV_8UC1);
    tableImage.setTo(Scalar(255));
    for (int y=0;y<gridSize.height;y++)
        for (int x=0;x<gridSize.width;x++) {
            Mat subrect(tableImage,cv::Rect( x*(MarkerDistance+MarkerSize),y*(MarkerDistance+MarkerSize),MarkerSize,MarkerSize));
            Mat marker=Marker::createMarkerImage( TInfo._markersId.at<int>(y,x),MarkerSize);
            marker.copyTo(subrect);
        }

    return tableImage;
}
/**
*
*
*/
BoardConfiguration::BoardConfiguration()
{
    _markerSizePix=_markerDistancePix=-1;
}
/**
*
*
*/
BoardConfiguration::BoardConfiguration(const BoardConfiguration  &T)
{
    _markersId=T._markersId;
    _markerSizePix=T._markerSizePix;
    _markerDistancePix=T._markerDistancePix;

}
/**
*
*
*/
void BoardConfiguration::saveToFile(string sfile)throw (cv::Exception)
{
    ofstream file(sfile.c_str());
    if (!file) throw cv::Exception(9190,"File could not be opened for writing :"+sfile,"BoardConfiguration::saveToFile",__FILE__,__LINE__);
    file<<"ArucoBoard 1.0"<<endl;
    file<<_markersId.size().height<<" "<<_markersId.size().width<<" ";
    for (  int i=0;i<_markersId.size().height;i++) {
        for (  int j=0;j<_markersId.size().width;j++)
            file<<_markersId.at<int>(i,j)<<" ";
    }
    file<<endl;
    file<< _markerSizePix<<" "<<_markerDistancePix<<" "<<endl;

}
/**
*
*
*/
void BoardConfiguration::readFromFile(string sfile)throw (cv::Exception)
{
    ifstream file(sfile.c_str());
    if (!file)  throw cv::Exception(9191,"File could not be opened fir reading :"+sfile,"BoardConfiguration::readFromFile",__FILE__,__LINE__);
    string sig,ver;
    file>>sig>>ver;
    if (sig!="ArucoBoard") throw cv::Exception(9191,"Invalid file type :"+sfile,"BoardConfiguration::readFromFile",__FILE__,__LINE__);
    if (ver!="1.0") throw cv::Exception(9191,"Invalid file version :"+sfile,"BoardConfiguration::readFromFile",__FILE__,__LINE__);
	
    file>>size.height>>size.width;//read size
    _markersId.create(size,CV_32SC1);
    for (  int i=0;i<size.height;i++)
        for (  int j=0;j<size.width;j++)
            file>>_markersId.at<int>(i,j);

    file>> _markerSizePix>>_markerDistancePix;
}

}

/*
 * ofxAruco.h
 *
 *  Created on: 02/10/2011
 *      Author: arturo
 */

#ifndef OFXARUCO_H_
#define OFXARUCO_H_

#include "ofConstants.h"
#include "ofPixels.h"
#include "ofMatrix4x4.h"
#include "ofThread.h"

#include "boarddetector.h"
#include "markerdetector.h"
#include "Poco/Condition.h"

class ofxAruco: public ofThread {
public:
	ofxAruco();
    
	void setThreaded(bool threaded); // defaults to true
	void setup(string calibrationFile,float w, float h, string boardConfig="", float markerSize=.15);
	void setupXML(string calibrationXML,float w, float h, string boardConfig="", float markerSize=.15);
    
    void addBoardConf(string boardConfig="");
    
	void detectMarkers(ofPixels & pixels);
	void detectBoards(ofPixels & pixels);
    
	void draw();
    
	vector<aruco::Marker> & getMarkers();
    //    bgraf
    //	aruco::Board & getBoard();
    vector<aruco::Board> & getBoards();
	float getBoardProbability();
    vector<float> getBoardProbabilities();
    
	int getNumMarkers();
	int getNumBoards();
    
	void begin(int marker);
    //    bgraf
    //	void beginBoard();
    void beginBoard(int boardnum);
	void end();
    
	ofMatrix4x4 getProjectionMatrix();
	ofMatrix4x4 getModelViewMatrix(int marker);
	ofMatrix4x4 getModelViewMatrixBoard(int board);
    
    // bgraf (int board added)
	ofVec3f getBoardTranslation(int board);
	ofQuaternion getBoardRotation(int board);
    
	void getBoardImage(ofPixels & pixels);
	void getMarkerImage(int markerID, int size, ofPixels & pixels);
	void getThresholdImage(ofPixels & pixels);
    
	double getThresholdParam1();
	double getThresholdParam2();
	void setThresholdParams(double param1, double param2);
	void setThresholdMethod(aruco::MarkerDetector::ThresholdMethods method);
    
    // bgraf
    //	aruco::BoardConfiguration & getBoardConfig();
    vector<aruco::BoardConfiguration> & getBoardConfigs();
    
private:
	void threadedFunction();
	void findMarkers(ofPixels & pixels);
    // bgraf
    //	void findBoard(ofPixels & pixels);
    void findBoards(ofPixels & pixels);
    
	ofPixels frontPixels, backPixels, intraPixels;
	bool newDetectMarkers, newDetectBoard;
	Poco::Condition condition;
    
	aruco::MarkerDetector detector;
	aruco::BoardDetector boardDetector;
    
	struct TrackedMarker{
		aruco::Marker marker;
		int age;
	};
	vector<aruco::Marker> markers,backMarkers,intraMarkers;
	vector<TrackedMarker> prevMarkers;
	int maxAge;
    // bgraf
    //	aruco::Board board;
    //	aruco::BoardConfiguration boardConfig;
    //    float boardProbability;
    vector<aruco::Board> boards;
    vector<aruco::BoardConfiguration> boardConfigs;
	vector<float> boardProbabilities;
    
	aruco::CameraParameters camParams;
    
	float markerSize;
    
	cv::Size size;
    
	double projMatrix[16];
	float projfMatrix[16];
	ofMatrix4x4 ofprojMatrix;
    
    
	aruco::Marker * findMarker(int id);
	TrackedMarker * findTrackedMarker(int id);
	bool threaded;
	bool foundMarkers,foundBoard;
};

#endif /* OFXARUCO_H_ */

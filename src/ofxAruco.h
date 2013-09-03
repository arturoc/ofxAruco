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

	void detectMarkers(ofPixels & pixels);
	void detectBoard(ofPixels & pixels);

	void draw();

	vector<aruco::Marker> & getMarkers();
	aruco::Board & getBoard();
	float getBoardProbability();

	int getNumMarkers();
	int getNumBoards();

	void begin(int marker);
	void beginBoard();
	void end();

	ofMatrix4x4 getProjectionMatrix();
	ofMatrix4x4 getModelViewMatrix(int marker);
	ofMatrix4x4 getModelViewMatrixBoard();

	ofVec3f getBoardTranslation();
	ofQuaternion getBoardRotation();

	void getBoardImage(ofPixels & pixels);
	void getMarkerImage(int markerID, int size, ofPixels & pixels);
	void getThresholdImage(ofPixels & pixels);

	double getThresholdParam1();
	double getThresholdParam2();
	void setThresholdParams(double param1, double param2);
	void setThresholdMethod(aruco::MarkerDetector::ThresholdMethods method);

	aruco::BoardConfiguration & getBoardConfig();

private:
	void threadedFunction();
	void findMarkers(ofPixels & pixels);
	void findBoard(ofPixels & pixels);

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
	aruco::Board board;
	aruco::BoardConfiguration boardConfig;
	float boardProbability;
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

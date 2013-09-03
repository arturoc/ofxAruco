/*
 * ofxAruco.cpp
 *
 *  Created on: 02/10/2011
 *      Author: arturo
 */

#include "ofxCv.h"
#include "ofxAruco.h"

ofxAruco::ofxAruco()
:threaded(true)
,newDetectMarkers(false)
,newDetectBoard(false){

}

void ofxAruco::setThreaded(bool _threaded){
	threaded = _threaded;
}

void ofxAruco::setup(string calibrationFile,float w, float h, string boardConfigFile, float _markerSize){
	size.width = w;
	size.height = h;
	markerSize = _markerSize;
	detector.setThresholdMethod(aruco::MarkerDetector::ADPT_THRES);

	camParams.readFromFile(ofToDataPath(calibrationFile));
	camParams.resize(cv::Size(w,h));

	maxAge = 7;

	double projMatrix[16];
	aruco::MarkerDetector::glGetProjectionMatrix(camParams,size,size,projMatrix,0.05,10,false);

	for(int i=0;i<16;i++){
		ofprojMatrix.getPtr()[i]=projMatrix[i];
	}

	if(boardConfigFile!=""){
		boardConfig.readFromFile(ofToDataPath(boardConfigFile));
	}

	if(threaded) startThread();
}

void ofxAruco::setupXML(string calibrationXML,float w, float h, string boardConfigFile, float _markerSize){
	size.width = w;
	size.height = h;
	markerSize = _markerSize;
	detector.setThresholdMethod(aruco::MarkerDetector::ADPT_THRES);

	camParams.readFromXMLFile(ofToDataPath(calibrationXML));
	camParams.resize(cv::Size(w,h));

	aruco::MarkerDetector::glGetProjectionMatrix(camParams,size,size,projMatrix,0.05,10,false);

	for(int i=0;i<16;i++){
		ofprojMatrix.getPtr()[i]=projMatrix[i];
	}

	if(boardConfigFile!=""){
		boardConfig.readFromFile(ofToDataPath(boardConfigFile));
	}

	if(threaded) startThread();
}

aruco::BoardConfiguration & ofxAruco::getBoardConfig(){
	return boardConfig;
}

aruco::Marker * ofxAruco::findMarker(int id){
	for(size_t i=0;i<backMarkers.size();i++){
		if(backMarkers[i].idMarker==id){
			return &backMarkers[i];
		}
	}
	return NULL;
}


ofxAruco::TrackedMarker * ofxAruco::findTrackedMarker(int id){
	for(size_t i=0;i<prevMarkers.size();i++){
		if(prevMarkers[i].marker.idMarker==id){
			return &prevMarkers[i];
		}
	}
	return NULL;
}

void ofxAruco::detectMarkers(ofPixels & pixels){
	if(!threaded){
		findMarkers(pixels);
	}else{
		lock();
		frontPixels = pixels;
		newDetectMarkers = true;

		if(foundMarkers){
			swap(markers,intraMarkers);
			foundMarkers = false;
		}
		condition.signal();
		unlock();
	}
}

void ofxAruco::detectBoard(ofPixels & pixels){
	if(!threaded){
		findMarkers(pixels);
	}else{
		lock();
		frontPixels = pixels;
		newDetectBoard = true;

		if(foundMarkers){
			swap(markers,intraMarkers);
			foundMarkers = false;
		}
		condition.signal();
		unlock();
	}
}


void ofxAruco::findMarkers(ofPixels & pixels){
	cv::Mat mat = ofxCv::toCv(pixels);
	detector.detect(mat,backMarkers,camParams,markerSize);

	vector<vector<TrackedMarker>::iterator > toDelete;
	vector<aruco::Marker > toAdd;
	for(size_t i=0;i<prevMarkers.size();i++){
		if(prevMarkers[i].age>maxAge){
			toDelete.push_back(prevMarkers.begin()+i);
			continue;
		}
		aruco::Marker * prev = findMarker(prevMarkers[i].marker.idMarker);
		if(!prev){
			prevMarkers[i].age++;
			toAdd.push_back(prevMarkers[i].marker);
		}else{
			prevMarkers[i].age = 0;
		}
	}

	for(size_t i=0;i<toDelete.size();i++){
		prevMarkers.erase(toDelete[i]);
	}

	for(size_t i=0;i<backMarkers.size();i++){
		TrackedMarker * marker = findTrackedMarker(backMarkers[i].idMarker);
		if(!marker){
			TrackedMarker tracked = {backMarkers[i],0};
			prevMarkers.push_back(tracked);
		}else{
			marker->marker = backMarkers[i];
		}
	}

	for(size_t i=0;i<toAdd.size();i++){
		backMarkers.push_back(toAdd[i]);
	}

	if(threaded){
		lock();
		swap(backMarkers,intraMarkers);
		foundMarkers = true;
		unlock();
	}else{
		swap(backMarkers,markers);
	}

}

void ofxAruco::findBoard(ofPixels & pixels){
	findMarkers(pixels);
	boardProbability = boardDetector.detect(backMarkers,boardConfig,board,camParams,markerSize);

	if(threaded){
		lock();
		foundBoard = true;
		unlock();
	}
}

void ofxAruco::draw(){
	glMatrixMode( GL_PROJECTION );
	glPushMatrix();

	aruco::MarkerDetector::glGetProjectionMatrix(camParams,size,size,projMatrix,0.05,10,false);

	for(int i=0;i<16;i++){
		ofprojMatrix.getPtr()[i]=projMatrix[i];
	}

	glLoadIdentity();
	glLoadMatrixf(ofprojMatrix.getPtr());

	glMatrixMode( GL_MODELVIEW );
	glPushMatrix();

	for(int i=0;i<(int)markers.size();i++){
		double matrix[16];
		float matrixf[16];
		markers[i].glGetModelViewMatrix(matrix);
		for(int i=0;i<16;i++){
			matrixf[i] = matrix[i];
		}

		glLoadIdentity();
		glLoadMatrixf(matrixf);
		ofDrawAxis(markerSize);
	}

	glPopMatrix();

	glMatrixMode( GL_PROJECTION );
	glPopMatrix();

	glMatrixMode( GL_MODELVIEW );
}

vector<aruco::Marker> & ofxAruco::getMarkers(){
	return markers;
}

aruco::Board & ofxAruco::getBoard(){
	return board;
}

int ofxAruco::getNumMarkers(){
	return markers.size();
}

float ofxAruco::getBoardProbability(){
	return boardProbability;
}

void ofxAruco::begin(int marker){
	glMatrixMode( GL_PROJECTION );
	glPushMatrix();

	aruco::MarkerDetector::glGetProjectionMatrix(camParams,size,size,projMatrix,0.05,10,false);

	for(int i=0;i<16;i++){
		ofprojMatrix.getPtr()[i]=projMatrix[i];
	}

	glLoadIdentity();
	glLoadMatrixf(ofprojMatrix.getPtr());



	double matrix[16];
	float matrixf[16];
	markers[marker].glGetModelViewMatrix(matrix);
	for(int i=0;i<16;i++){
		matrixf[i] = matrix[i];
	}

	glMatrixMode( GL_MODELVIEW );
	glPushMatrix();

	glLoadIdentity();
	glLoadMatrixf(matrixf);
}

void ofxAruco::end(){
	glMatrixMode( GL_MODELVIEW );
	glPopMatrix();

	glMatrixMode( GL_PROJECTION );
	glPopMatrix();

	glMatrixMode( GL_MODELVIEW );
}

void ofxAruco::beginBoard(){
	glMatrixMode( GL_PROJECTION );
	glPushMatrix();

	aruco::MarkerDetector::glGetProjectionMatrix(camParams,size,size,projMatrix,0.05,10,false);

	for(int i=0;i<16;i++){
		ofprojMatrix.getPtr()[i]=projMatrix[i];
	}

	glLoadIdentity();
	glLoadMatrixf(ofprojMatrix.getPtr());


	double matrix[16];
	float matrixf[16];
	board.glGetModelViewMatrix(matrix);
	for(int i=0;i<16;i++){
		matrixf[i] = matrix[i];
	}

	glMatrixMode( GL_MODELVIEW );
	glPushMatrix();

	glLoadMatrixf(matrixf);
}

ofMatrix4x4 ofxAruco::getProjectionMatrix(){
	aruco::MarkerDetector::glGetProjectionMatrix(camParams,size,size,projMatrix,0.05,10,false);
	for(int i=0;i<16;i++){
		ofprojMatrix.getPtr()[i]=projMatrix[i];
	}
	return ofprojMatrix;
}

ofMatrix4x4 ofxAruco::getModelViewMatrix(int marker){
	double matrix[16];
	ofMatrix4x4 modelview;
	markers[marker].glGetModelViewMatrix(matrix);

	for(int i=0;i<16;i++){
		modelview.getPtr()[i]=matrix[i];
	}

	return modelview;
}

ofMatrix4x4 ofxAruco::getModelViewMatrixBoard(){
	double matrix[16];
	ofMatrix4x4 modelview;
	board.glGetModelViewMatrix(matrix);

	for(int i=0;i<16;i++){
		modelview.getPtr()[i]=matrix[i];
	}

	return modelview;
}

ofVec3f ofxAruco::getBoardTranslation(){
	return ofVec3f (board.Tvec.at<float>(0,0),
					board.Tvec.at<float>(1,0),
					-board.Tvec.at<float>(2,0));
}

ofQuaternion ofxAruco::getBoardRotation(){
	ofMatrix4x4 mv = ofMatrix4x4::getInverseOf(ofMatrix4x4::getTransposedOf(getModelViewMatrixBoard()));
	ofVec3f t;
	ofQuaternion q;
	ofQuaternion so;
	ofVec3f s;
	mv.decompose(t,q,s,so);
	return q;
}

void ofxAruco::getBoardImage(ofPixels & pixels){
	cv::Mat m = aruco::Board::createBoardImage(boardConfig.size,boardConfig._markerSizePix,boardConfig._markerDistancePix,0,boardConfig);
	pixels.setFromPixels(m.data,m.cols,m.rows,OF_IMAGE_GRAYSCALE);
}

void ofxAruco::getMarkerImage(int markerID, int size, ofPixels & pixels){
	cv::Mat m = aruco::Marker::createMarkerImage(markerID,size);
	pixels.setFromPixels(m.data,size,size,OF_IMAGE_GRAYSCALE);
}

void ofxAruco::getThresholdImage(ofPixels & pixels){
	cv::Mat m = detector.getThresholdedImage();
	pixels.setFromPixels(m.data,m.cols,m.rows,OF_IMAGE_GRAYSCALE);
}

double ofxAruco::getThresholdParam1(){
	double param1, param2;
	detector.getThresholdParams(param1,param2);
	return param1;
}

double ofxAruco::getThresholdParam2(){
	double param1, param2;
	detector.getThresholdParams(param1,param2);
	return param1;
}

void ofxAruco::setThresholdParams(double param1, double param2){
	detector.setThresholdParams(param1,param2);
}

void ofxAruco::setThresholdMethod(aruco::MarkerDetector::ThresholdMethods method){
	detector.setThresholdMethod(method);
}

void ofxAruco::threadedFunction(){
	while(isThreadRunning()){
		lock();
		if(!newDetectBoard && !newDetectMarkers) condition.wait(mutex);
		bool detectMarkers = false;
		bool detectBoard = false;
		if(newDetectMarkers || newDetectBoard){
			swap(frontPixels, backPixels);
			detectMarkers = newDetectMarkers;
			detectBoard = newDetectBoard;

			newDetectMarkers = false;
			newDetectBoard = false;
		}
		unlock();

		if(detectMarkers){
			findMarkers(backPixels);
		}
		if(detectBoard){
			findBoard(backPixels);
		}
	}
}

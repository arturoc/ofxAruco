#include "ofApp.h"
#include "ofxCv.h"
#include "ofBitmapFont.h"

void drawMarker(float size, const ofColor & color){
	ofDrawAxis(size);
	ofPushMatrix();
		// move up from the center by size*.5
		// to draw a box centered at that point
		ofTranslate(0,size*0.5,0);
		ofFill();
		ofSetColor(color,50);
		ofDrawBox(size);
		ofNoFill();
		ofSetColor(color);
		ofDrawBox(size);
	ofPopMatrix();
}

//--------------------------------------------------------------
void ofApp::setup(){
	ofSetWindowTitle("ofxAruco - Highly Reliable Markers");
	ofSetVerticalSync(true);

	//setup cam parameters
	string cameraIntrinsics = "intrinsics.yml";
	string markerFile = "marker.xml";

	//setup video
	grabber.listDevices();
	grabber.setDeviceID(1);
	grabber.initGrabber(1920,1080);
	video = &grabber;
	
	
	//load marker
	aruco.setUseHighlyReliableMarker(markerFile);
	
	//init 
	aruco.setThreaded(true);
	aruco.setupXML(cameraIntrinsics, video->getWidth(), video->getHeight());

	showMarkers = true;
	ofEnableAlphaBlending();

}

//--------------------------------------------------------------
void ofApp::update(){
	video->update();
	if(video->isFrameNew()){
		aruco.detectBoards(video->getPixels());
	}
}

//--------------------------------------------------------------
void ofApp::draw(){
	ofSetColor(255);
	video->draw(0,0);

	if(showMarkers){
		for(int i=0;i<aruco.getNumMarkers();i++){
			aruco.begin(i);
			drawMarker(0.15,ofColor::white);
			aruco.end();
		}
	}


	for (auto& m: aruco.getMarkers()) {
		cout << m.getArea() << endl;
	}

	ofSetColor(255);
	ofDrawBitmapString("markers detected: " + ofToString(aruco.getNumMarkers()),20,20);
	ofDrawBitmapString("fps " + ofToString(ofGetFrameRate()),20,40);
	ofDrawBitmapString("m toggles markers",20,60);


}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
	if(key=='m') showMarkers = !showMarkers;
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){

}

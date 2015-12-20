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
		ofBox(size);
		ofNoFill();
		ofSetColor(color);
		ofBox(size);
	ofPopMatrix();
}

//--------------------------------------------------------------
void ofApp::setup(){
	ofSetVerticalSync(true);
	useVideo = false;
	string boardName = "boardConfiguration.yml";

	if(useVideo){
		player.loadMovie("videoboard.mp4");
		player.play();
		video = &player;
	}else{
    //grabber.setDeviceID(1);
		grabber.initGrabber(640,480);
		video = &grabber;
	}

	//aruco.setThreaded(false);
	aruco.setup("intrinsics.int", video->getWidth(), video->getHeight(), boardName);
	aruco.getBoardImage(board.getPixelsRef());
	board.update();

	showMarkers = true;
	showBoard = true;
	showBoardImage = false;

	ofEnableAlphaBlending();

	ofPixels pixels;
	ofBitmapStringGetTextureRef().readToPixels(pixels);
	ofSaveImage(pixels,"font.bmp");
}

//--------------------------------------------------------------
void ofApp::update(){
	video->update();
	if(video->isFrameNew()){
		aruco.detectBoards(video->getPixelsRef());
	}
}

//--------------------------------------------------------------
void ofApp::draw(){
	ofSetColor(255);
	video->draw(0,0);

	//aruco.draw();

	if(showMarkers){
		for(int i=0;i<aruco.getNumMarkers();i++){
			aruco.begin(i);
			drawMarker(0.15,ofColor::white);
			aruco.end();
		}
	}


	if(showBoard && aruco.getBoardProbability()>0.03){
		for(int i=0;i<aruco.getNumBoards();i++){
			aruco.beginBoard(i);
			drawMarker(.5,ofColor::red);
			aruco.end();
		}
	}
	

	ofSetColor(255);
	if(showBoardImage){
    	board.draw(ofGetWidth()-320,0,320,320*float(board.getHeight())/float(board.getWidth()));
    }
	ofDrawBitmapString("markers detected: " + ofToString(aruco.getNumMarkers()),20,20);
	ofDrawBitmapString("fps " + ofToString(ofGetFrameRate()),20,40);
	ofDrawBitmapString("m toggles markers",20,60);
	ofDrawBitmapString("b toggles board",20,80);
	ofDrawBitmapString("i toggles board image",20,100);
	ofDrawBitmapString("s saves board image",20,120);
	ofDrawBitmapString("0-9 saves marker image",20,140);
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
	if(key=='m') showMarkers = !showMarkers;
	if(key=='b') showBoard = !showBoard;
	if(key=='i') showBoardImage = !showBoardImage;
	if(key=='s') board.saveImage("boardimage.png");
	if(key>='0' && key<='9'){
	    // there's 1024 different markers
	    int markerID = key - '0';
	    aruco.getMarkerImage(markerID,240,marker);
	    marker.saveImage("marker"+ofToString(markerID)+".png");
	}
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

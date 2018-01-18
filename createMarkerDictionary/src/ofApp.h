#pragma once

#include "ofMain.h"
#include "ofxAruco.h"
#include "highlyreliablemarkers.h"

class ofApp : public ofBaseApp{

	public:
		void setup();
		void update();
		void draw();

		void keyPressed  (int key);
		void keyReleased(int key);
		void mouseMoved(int x, int y );
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void windowResized(int w, int h);
		void dragEvent(ofDragInfo dragInfo);
		void gotMessage(ofMessage msg);
		
		void createMarker(unsigned int dictSize, unsigned int n);
		void createMarkerImages(aruco::Dictionary& dictionary);

private:
	aruco::Dictionary dictionary;

	int dictionarySize{ 32 };
	int markerResolution{ 4 };
	int tau;

	int state{ 0 };
	bool stateDrawn{ false };
};

#include "ofApp.h"
#include "ofxCv.h"
#include "ofBitmapFont.h"


typedef std::vector< int > Word;

class MarkerGenerator {

private:
	int _nTransitions;
	std::vector< int > _transitionsWeigth;
	int _totalWeigth;
	int _n;

public:
	MarkerGenerator(int n) {
		_n = n;
		_nTransitions = n - 1;
		_transitionsWeigth.resize(_nTransitions);
		_totalWeigth = 0;
		for (int i = 0; i < _nTransitions; i++) {
			_transitionsWeigth[i] = i;
			_totalWeigth += i;
		}
	}

	aruco::MarkerCode generateMarker() {

		aruco::MarkerCode emptyMarker(_n);

		for (int w = 0; w < _n; w++) {
			Word currentWord(_n, 0);
			int randomNum = rand() % _totalWeigth;
			int currentNTransitions = _nTransitions - 1;
			for (int k = 0; k < _nTransitions; k++) {
				if (_transitionsWeigth[k] > randomNum) {
					currentNTransitions = k;
					break;
				}
			}
			std::vector< int > transitionsIndexes(_nTransitions);
			for (int i = 0; i < _nTransitions; i++)
				transitionsIndexes[i] = i;
			std::random_shuffle(transitionsIndexes.begin(), transitionsIndexes.end());

			std::vector< int > selectedIndexes;
			for (int k = 0; k < currentNTransitions; k++)
				selectedIndexes.push_back(transitionsIndexes[k]);
			std::sort(selectedIndexes.begin(), selectedIndexes.end());
			int currBit = rand() % 2;
			int currSelectedIndexesIdx = 0;
			for (int k = 0; k < _n; k++) {
				currentWord[k] = currBit;
				if (currSelectedIndexesIdx < selectedIndexes.size() && k == selectedIndexes[currSelectedIndexesIdx]) {
					currBit = 1 - currBit;
					currSelectedIndexesIdx++;
				}
			}

			for (int k = 0; k < _n; k++)
				emptyMarker.set(w * _n + k, bool(currentWord[k]), false);
		}

		return emptyMarker;
	}
};


//--------------------------------------------------------------
void ofApp::setup(){
	ofSetWindowTitle("Highly Reliable Marker Creator");
	ofEnableAlphaBlending();


}

//--------------------------------------------------------------
void ofApp::update(){
	ofBackground(40);

	if (state == 1 && stateDrawn)
	{
		createMarker(dictionarySize, markerResolution);
		createMarkerImages(dictionary);
		state = 2;
	}
}

//--------------------------------------------------------------
void ofApp::draw(){
	ofSetColor(200);
	if (state == 0)
	{
		ofDrawBitmapString("Choose your parameters and press 'g' to generate marker", 15, 30);
		ofDrawBitmapString("Number of markers : " + ofToString(dictionarySize) + "   -/+ (i/o)", 15, 60);
		ofDrawBitmapString("Marker Resolution in px : " + ofToString(markerResolution) + "   -/+ (k/l)", 15, 80);

		int w = 300;
		ofPushMatrix();
		ofTranslate((ofGetWidth() - w) / 2, 120);
		ofSetColor(255);
		ofDrawRectangle(0, 0, w, w);
		int d = w / (markerResolution + 4);
		ofTranslate(d, d);
		ofSetColor(0);
		ofDrawRectangle(0, 0, w - 2 * d, w - 2 * d);
		ofTranslate(d, d);

		for (int x = 0; x < markerResolution; x++)
		{
			for (int y = 0; y < markerResolution; y++)
			{
				if ((x%5 + y%2) % 3) ofSetColor(0);
				else ofSetColor(255);
				ofDrawRectangle(d*x, d*y, d, d);
			}
		}
		ofPopMatrix();
	}
	else if(state == 1){
		ofDrawBitmapString("Markers are beeing created. This could take a while.", 15, 80);
		ofDrawBitmapString("Have a look at the console window to see the progess.", 15, 100);
		stateDrawn = true;
	}
	else {
		ofDrawBitmapString("Marker creation has finished!", 15, 80);
		ofDrawBitmapString("The dictionary is 'marker.xml' and the images can be", 15, 100);
		ofDrawBitmapString("found in marker/", 15, 120);
	}
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
	if (key == 'i') {
		dictionarySize--;
		if (dictionarySize == 0) dictionarySize++;
	}
	if (key == 'o') {
		dictionarySize++;
	}
	if (key == 'k') {
		markerResolution--;
		if (markerResolution == 1) markerResolution++;
	}
		
	if (key == 'l') markerResolution++;
	if (key == 'g') {
		state = 1;
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

void ofApp::createMarker(unsigned int dictSize, unsigned int n)
{
	aruco::Dictionary D;

	unsigned int tau = 2 * ((4 * ((n * n) / 4)) / 3);
	std::cout << "Tau: " << tau << std::endl;

	srand(time(NULL));

	MarkerGenerator MG(n);

	const int MAX_UNPRODUCTIVE_ITERATIONS = 100000;
	int currentMaxUnproductiveIterations = MAX_UNPRODUCTIVE_ITERATIONS;

	unsigned int countUnproductive = 0;
	while (D.size() < dictSize) {

		aruco::MarkerCode candidate;
		candidate = MG.generateMarker();

		if (candidate.selfDistance() >= tau && D.distance(candidate) >= tau) {
			D.push_back(candidate);
			std::cout << "Accepted Marker " << D.size() << "/" << dictSize << std::endl;
			countUnproductive = 0;
		} else {
			countUnproductive++;
			if (countUnproductive == currentMaxUnproductiveIterations) {
				tau--;
				countUnproductive = 0;
				std::cout << "Reducing Tau to: " << tau << std::endl;
				if (tau == 0) {
					std::cerr << "Error: Tau=0. Small marker size for too high number of markers. Stop" << std::endl;
					break;
				}
				if (D.size() >= 2)
					currentMaxUnproductiveIterations = MAX_UNPRODUCTIVE_ITERATIONS;
				else
					currentMaxUnproductiveIterations = MAX_UNPRODUCTIVE_ITERATIONS / 15;
			}
		}
	}

	D.tau0 = tau;
	D.toFile("data/marker.xml");

	dictionary = D;
}

void ofApp::createMarkerImages(aruco::Dictionary & dictionary)
{
	int nMarkers = dictionary.size();
	int markerSize = dictionary[0].n();
	int tau0 = dictionary.tau0;

	int pixelSize = 75;

	ofFbo fbo;
	int markerDim = pixelSize * (markerSize + 4);
	fbo.allocate(markerDim, markerDim);

	for (unsigned int i = 0; i < nMarkers; i++) {
		fbo.begin();
		ofClear(255);
		ofDrawRectangle(0, 0, fbo.getWidth(), fbo.getHeight());
		ofPushMatrix();
		ofTranslate(pixelSize, pixelSize);
		ofSetColor(0);
		int dimRect = pixelSize * (markerSize + 2);
		ofDrawRectangle(0, 0, dimRect, dimRect);
		ofTranslate(pixelSize, pixelSize);
		for (int x = 0; x < markerSize; x++)
		{
			for (int y = 0; y < markerSize; y++)
			{
				if (dictionary[i].get(y*markerSize + x))
					ofSetColor(255);
				else
					ofSetColor(0);
				ofDrawRectangle(x*pixelSize, y*pixelSize, pixelSize, pixelSize);
			}
		}
		ofPopMatrix();
		fbo.end();

		//save fbo to file
		ofPixels p;
		fbo.readToPixels(p);
		ofImage img;
		img.setFromPixels(p);
		img.save("marker/marker_" + ofToString(i) + ".png");
	}

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){

}

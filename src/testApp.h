#pragma once

#include "ofMain.h"
#include "ofxCv.h"
extern "C" {
#include "lsd.h"
}

class testApp : public ofBaseApp{

	public:
		void setup();
		void update();
		void draw();

		void keyPressed(int key);
		void keyReleased(int key);
		void mouseMoved(int x, int y );
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void windowResized(int w, int h);
		void dragEvent(ofDragInfo dragInfo);
		void gotMessage(ofMessage msg);

		ofImage my_image;
		ofImage my_img_gray;
		//EXIF
		float focal_length;
		string cam_model;
		float sensor_width;

		bool resize_image;
		ofVec2f center;
		ofMatrix3x3 K;

        typedef std::pair<double,int> mypair;
        bool comparator ( const mypair& l, const mypair& r);

        ofMesh mesh;
        vector<vector<double> > L; //Line Pairs Detected
};

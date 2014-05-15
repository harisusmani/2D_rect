#include "ofMain.h"
#include "testApp.h"

//========================================================================
int main( ){
	ofSetupOpenGL(1024,768,OF_WINDOW);
    //ofSetupOpenGL(1024,2046,OF_WINDOW); //Window Size for a Portrait Image
	ofRunApp(new testApp());

}

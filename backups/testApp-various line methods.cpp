#include "testApp.h"

using namespace ofxCv;
using namespace cv;


typedef std::pair<double,int> mypair;
bool comparator ( const mypair& l, const mypair& r){
    return l.first < r.first;
    }

//--------------------------------------------------------------
void testApp::setup(){
    //ofSetVerticalSync(true);
	my_image.loadImage("dc_1.jpg");

	//Import EXIF Data, Find a Good Library//
	focal_length=22;
	cam_model="Canon EOS M";
	sensor_width=22.3;
	//EXIF Data Completed//

    resize_image=1; //To Enable or Disable Resize

	//Rescale Image to be Max in 1000px
	if (resize_image && max(my_image.width,my_image.height)>1000)
    {
        float s=float(1000)/max(my_image.width,my_image.height);
        my_image.resize(floor(my_image.width*s),floor(my_image.height*s));
    }

    float f=focal_length*(float)max(my_image.width,my_image.height)/sensor_width;
    K.set(f,0.0,0.0,0.0,f,0.0,0.0,0.0,1.0);
    center.set(float(my_image.width)/2,float(my_image.height)/2);
    cout << K;

    my_img_gray=my_image;
    my_img_gray.setImageType(OF_IMAGE_GRAYSCALE);

    //Converting Image to Image Double//
    image_double dub_image;
    ntuple_list lsd_out;
    unsigned int w=my_img_gray.width;
    unsigned int h=my_img_gray.height;
    unsigned char * imgP=my_img_gray.getPixels();

    int verbose=0;

    // LSD parameters start
    double scale = 0.8;       // Scale the image by Gaussian filter to 'scale'.
    double sigma_scale = 0.6; // Sigma for Gaussian filter is computed as sigma = sigma_scale/scale.
    double quant = 2.0;       // Bound to the quantization error on the gradient norm.
    double ang_th = 22.5;     // Gradient angle tolerance in degrees.
    double eps = 0.0;         // Detection threshold, -log10(NFA).
    double density_th = 0.7;  // Minimal density of region points in rectangle.
    int n_bins = 1024;        // Number of bins in pseudo-ordering of gradient modulus.
    double max_grad = 255.0;  // Gradient modulus in the highest bin. The default value corresponds to the highest
                              // gradient modulus on images with gray levels in [0,255].
    // LSD parameters end

    dub_image = new_image_double(w,h);
    cout << "\n--------\nInput data being written to image buffer \n";
    for(int j=0;j<(w*h);j++){
        dub_image->data[j] = imgP[j];
        if (verbose){
            cout << " " << dub_image->data[j];
        }
    }
    // Call LSD //
    lsd_out = LineSegmentDetection( dub_image, scale, sigma_scale, quant, ang_th, eps,
                               density_th, n_bins, max_grad, NULL );
    cout << "LSD has done it's thing!\n";
    cout << "Number of Lines: "<< lsd_out->size << "Number of Dimensions: " << lsd_out->dim << "\n";

    cout << lsd_out->values[0] << " " << lsd_out->values[1] <<" " <<  lsd_out->values[2] <<" " <<  lsd_out->values[3] <<" " <<  lsd_out->values[4] <<" " <<  lsd_out->values[5] << "\n";

    //SORTING WHICH RETURNS INDEX TOO!
    //http://stackoverflow.com/questions/1577475/c-sorting-and-keeping-track-of-indexes
    vector<mypair> line_lengths;
    double distance;
    for(int j=0;j<(lsd_out->size);j=j+5){
        distance=sqrt(pow(lsd_out->values[j+2]-lsd_out->values[j],2) + pow(lsd_out->values[j+3]-lsd_out->values[j+1],2));
        if (distance> 300) {
                cout << distance << " ";
        }
        line2.circle(lsd_out->values[j+2],lsd_out->values[j+3],4);
        //line2.lineTo(lsd_out->values[j], lsd_out->values[j+1]);
        line2.circle(lsd_out->values[j], lsd_out->values[j+1],4);
        line.addVertex(ofPoint(lsd_out->values[j+2], lsd_out->values[j+3]));
        line.addVertex(ofPoint(lsd_out->values[j], lsd_out->values[j+1]));
        line_lengths.push_back(make_pair(distance,j));

        mesh.setMode(OF_PRIMITIVE_LINES);
        mesh.enableColors();
        ofVec3f first(lsd_out->values[j+2],lsd_out->values[j+3],0.0);
        ofVec3f second(lsd_out->values[j],lsd_out->values[j+1],0.0);
        mesh.addVertex(first);
        mesh.addColor(ofFloatColor(1.0, 0.0, 0.0));
        mesh.addVertex(second);
        mesh.addColor(ofFloatColor(1.0, 0.0, 0.0));

    }
    sort(line_lengths.begin(),line_lengths.end());
    reverse(line_lengths.begin(), line_lengths.end());

    // line_lengths.push_back(make_pair(5.0,0)); //,<3,1>,<9,3>];
    //line_lengths.push_back(make_pair(3.0,1));
    //line_lengths.push_back(make_pair(9.0,2));
    //sort( line_lengths.begin(), line_lengths.end());
    cout << line_lengths[0].first << " " << line_lengths[0].second << " " << line_lengths[1].first << " " << line_lengths[1].second;



    //vector<vector<double> > L;
    //L.resize(4);
    //for (int i = 0; i < 4; ++i)
    //array2D[i].resize(lsd_out->size);

    line.addVertex(ofPoint(0, 0));
    line.addVertex(ofPoint(100, 0));
    line.addVertex(ofPoint(50, 100));
    line.addVertex(ofPoint(0, 0));
    line.close();

}

//--------------------------------------------------------------
void testApp::update(){

}

//--------------------------------------------------------------
void testApp::draw(){
    //ofSetColor(255);
	//my_image.draw(0, 0);
    my_img_gray.draw(0, 0);
	//line.draw();
    //line2.draw();
	mesh.draw();
}

//--------------------------------------------------------------
void testApp::keyPressed(int key){

}

//--------------------------------------------------------------
void testApp::keyReleased(int key){

}

//--------------------------------------------------------------
void testApp::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void testApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void testApp::mousePressed(int x, int y, int button){

}

//--------------------------------------------------------------
void testApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void testApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void testApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void testApp::dragEvent(ofDragInfo dragInfo){

}

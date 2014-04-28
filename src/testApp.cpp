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

    bool verbose=0;

    dub_image = new_image_double(w,h);
    double px=0;
    cout << "\n--------\nInput data being written to image buffer \n";
    for(int j=0;j<(w*h);j++){
        px=imgP[j];
        dub_image->data[j] = px;
        if (verbose){
            cout << " " << dub_image->data[j];
        }
    }
    // Call LSD //
    lsd_out = LineSegmentDetection( dub_image, scale, sigma_scale, quant, ang_th, eps,
                               density_th, n_bins, max_grad, NULL );
    cout << "LSD has done it's thing!\n";
    cout << "Number of Lines: "<< lsd_out->size << "Number of Dimensions: " << lsd_out->dim << "\n";

    if (verbose)
    {
    cout << "LSD Values: " << lsd_out->values[0] << " " << lsd_out->values[1] <<" " <<  lsd_out->values[2] <<" " <<  lsd_out->values[3] <<" " <<  lsd_out->values[4] <<" " <<  lsd_out->values[5] << "\n";
    }

    //SORTING WHICH RETURNS INDEX TOO!
    //http://stackoverflow.com/questions/1577475/c-sorting-and-keeping-track-of-indexes
    vector<mypair> line_lengths;
    //double distance;
    double sqd_distance;

    mesh.setMode(OF_PRIMITIVE_LINES);
    mesh.enableColors();

    ofVec3f first(0.0,0.0,0.0);
    ofVec3f second(0.0,0.0,0.0);
    double x1,x2,y1,y2;

    for(int j=0;j<(lsd_out->size*lsd_out->dim);j=j+5){
        x1=lsd_out->values[j];
        y1=lsd_out->values[j+1];
        x2=lsd_out->values[j+2];
        y2=lsd_out->values[j+3];
        //distance=sqrt(pow(lsd_out->values[j+2]-lsd_out->values[j],2) + pow(lsd_out->values[j+3]-lsd_out->values[j+1],2));
        sqd_distance=(x2-x1)*(x2-x1) + (y2-y1)*(y2-y1);

        line_lengths.push_back(make_pair(sqd_distance,j));

        //To Draw as Primitive Lines //
        first.set(x2,y2,0.0);
        second.set(x1,y1,0.0);
        mesh.addVertex(first);
        mesh.addColor(ofFloatColor(1.0, 0.0, 0.0));
        mesh.addVertex(second);
        mesh.addColor(ofFloatColor(1.0, 0.0, 0.0));
        //Lines Added, will be drawn //
    }
    sort(line_lengths.begin(),line_lengths.end());
    reverse(line_lengths.begin(), line_lengths.end());

    cout << line_lengths[0].first << " " << line_lengths[0].second << " " << line_lengths[1].first << " " << line_lengths[1].second << "\n";

    unsigned int maxlines=700;
    //unsigned int max2=floor(lsd_out->size*0.7);
    unsigned int no_of_lines=min(lsd_out->size,maxlines);
    //Store these Lines pairs in a Matrix, in descending order of Distance
    cout << "Number of Lines: " << no_of_lines << "\n";
    L.resize(4); //Height
    for (int i = 0; i < 4; ++i){
        L[i].resize(no_of_lines);
    }
    for (int j=0; j<no_of_lines; j++){
        L[0][j] = lsd_out->values[line_lengths[j].second];
        L[1][j] = lsd_out->values[(line_lengths[j].second)+1];
        L[2][j] = lsd_out->values[(line_lengths[j].second)+2];
        L[3][j] = lsd_out->values[(line_lengths[j].second)+3];
    }

    /*
    cout << "Testing Values in L, C1: " << L[0][0] << " " << L[1][0] << " " << L[2][0] << " " << L[3][0];
    cout << "Testing Values in L, C2: " << L[0][1] << " " << L[1][1] << " " << L[2][1] << " " << L[3][1];
        first.set(L[0][0],L[1][0],0.0);
        second.set(L[2][0],L[3][0],0.0);
        mesh.addVertex(first);
        mesh.addColor(ofFloatColor(0.0, 1.0, 0.0));
        mesh.addVertex(second);
        mesh.addColor(ofFloatColor(0.0, 1.0, 0.0));

        first.set(L[0][1],L[1][1],0.0);
        second.set(L[2][1],L[3][1],0.0);
        mesh.addVertex(first);
        mesh.addColor(ofFloatColor(0.0, 1.0, 0.0));
        mesh.addVertex(second);
        mesh.addColor(ofFloatColor(0.0, 1.0, 0.0)); */

    //GAP FILLING
    double athresh=2;
    double dthresh=1;  //dthresh times the length of the two segments will be allowed as gap to be filled

    //LINE EXTENSION

    //Finding Adjacent Lines
    double athreshadj=10;

    vector<vector<bool> > adj; //Line x Line Inf Matrix Initialization, adj
    adj.resize(no_of_lines); //Height
    for (int i = 0; i < no_of_lines; ++i){
        adj[i].resize(no_of_lines);
        for (int j = 0; j < no_of_lines; ++j){
            //adj[i][j]=1.0/0.0;
            adj[i][j]=0;
        }
    }

    ofVec2f v1,v2,x;
    athreshadj=abs(cos((athreshadj*PI)/180.0));
    for (int i = 0; i < no_of_lines; ++i){
        for (int j = i+1; j < no_of_lines; ++j){ //Everyline infront
            v1.set(L[0][i]-L[2][i],L[1][i]-L[3][i]);
            v2.set(L[0][j]-L[2][j],L[1][j]-L[3][j]);
            v1.normalize();
            v2.normalize();
            if (abs(v1.dot(v2))<athreshadj) //acos(v1.dot(v2)) //So Angle is greater!
            {
               x=solveLinearSys(L[0][i]-L[2][i],-L[0][j]+L[2][j],L[1][i]-L[3][i],-L[1][j]+L[3][j],-L[2][i]+L[2][j],-L[3][i]+L[3][j]);
               if (not isinf(x.x) and not isinf(x.y))
               {
                   adj[i][j]=(x.x>=-DBL_EPSILON) && (x.x<=1+DBL_EPSILON) && (x.y>=-DBL_EPSILON) && (x.y<=1+DBL_EPSILON);
                   adj[j][i]=adj[i][j] || adj[j][i];
                   if (adj[i][j])
                   {
                      cout << "i=" << i <<"  j=" << j << "\n";
                   }
               }
            }
        }
    }

    /*double test=1.0/0.0;
    cout << test << "\n";
    cout << isinf(test); */
}

//--------------------------------------------------------------
void testApp::update(){

}

//--------------------------------------------------------------
void testApp::draw(){
    ofSetColor(255);
	//my_image.draw(0, 0);
    my_img_gray.draw(0, 0);
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

ofVec2f testApp::solveLinearSys(double a11,double a12,double a21,double a22,double b1,double b2){
    ofVec2f out;
    double det=(a11*a22)-(a12*a21);
    out.set((a22*b1-a12*b2)/det,(-a21*b1+a11*b2)/det);
    return out;
}

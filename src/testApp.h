#pragma once

#include "ofMain.h"
#include "ofxCv.h"
#include "ofxCvImage.h"

extern "C" {
#include "lsd.h"
}

#include "dlib/optimization.h"

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

		ofVec2f solveLinearSys(double a11,double a12,double a21,double a22,double b1,double b2);

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

        //For Solver Example
        typedef dlib::matrix<double,0,1> column_vector;

        column_vector fitFunc4Lines(std::vector<std::vector<double> > L_vec,unsigned int r1,unsigned int r2,unsigned int r3,unsigned int r4, float f);
        void distFunc(std::vector<std::vector<double> >L_vec,testApp::column_vector modelX,float f,double thresh,std::vector<int> & arIn,std::vector<int> & acIn);

};

class cost_function
{
public:

    cost_function (std::vector<std::vector<double> > L_vec, double r1, double r2, double r3, double r4, double f)
    { //RUN ALL INITIALIZATIONS HERE!

        L = (cv::Mat_<double>(3,4) << L_vec[0][r1], L_vec[0][r2], L_vec[0][r3], L_vec[0][r4],
                                  L_vec[1][r1], L_vec[1][r2], L_vec[1][r3], L_vec[1][r4],
                                  L_vec[2][r1], L_vec[2][r2], L_vec[2][r3], L_vec[2][r4]);

        K_mat = (cv::Mat_<double>(3,3)<< f,0.0,0.0,0.0,f,0.0,0.0,0.0,1.0);

    }

    double operator() ( const testApp::column_vector& arg) const //THIS WILL BE CALLED REPEATREDLY!
    {
        //MAKE ROTATION MATRIX
        ofMatrix4x4 R=ofMatrix4x4::newRotationMatrix(arg(0)*180.0/PI, ofVec3f(-1, 0, 0), arg(1)*180.0/PI, ofVec3f(0, -1, 0), 0, ofVec3f(0, 0, -1));
        double m[3][3] = {{R(0,0), R(0,1), R(0,2)}, {R(1,0), R(1,1), R(1,2)}, {R(2,0), R(2,1), R(2,2)}};
        cv::Mat R_mat = cv::Mat(3, 3, CV_64F, m);

        cv::Mat K_c=K_mat.clone();
        K_c=K_c.inv();
        R_mat=R_mat.t();
        cv::Mat Hinv=K_mat*R_mat*K_c;

        Hinv=Hinv.t();
        cv::Mat Lp=Hinv*L;
        Lp.resize(2);

        cv::Mat Lp_t=Lp.clone();
        Lp_t=Lp_t.t();

        cv::Mat C=Lp_t*Lp;
        //cout << C << endl;
        C=abs(C);

        double total_cost=C.at<double>(2,0)+C.at<double>(3,1)+C.at<double>(0,2)+C.at<double>(1,3);
        //cout << "Total Cost: " << total_cost << endl;
        return(total_cost);
    }

private:
    cv::Mat L;
    cv::Mat K_mat;
};

#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <iostream>
#include <ctype.h>
#include <math.h>
#include <fstream>

using namespace cv;
using namespace std;


Point2f point;
float focus = 2.6;
float x1,x2,x3,x4,y11,y2,y3,y4;
float geom( float z, float x, float y, int i)
{
	float theta, r, phi;
	r = sqrt(pow(z,2)+pow(x,2)+pow(y,2));
	theta = acos(z/r);
	
//	cout << "theta["<< i << "] = " << theta << endl;
	
	return( theta);
}
float vect(float x1, float x2, float x3, float x4,float y1, float y2, float y3, float y4)
{
	float ax,ay,bx,by,beta;
	ax = x3 - x1; 
	ay = y3 - y1;
	bx = x4 - x2;
	by = y4 - y2;
	
	beta = acos((ax*bx+ay*by)/(sqrt(pow(ax,2)+pow(ay,2))+sqrt(pow(bx,2)+pow(by,2))));
	
//	cout << beta;
	return (beta);
}
void find(float ax, float ay, float bx, float by, float cx, float cy, float dx, float dy)
{
	float coordx[4], coordy[4], imax[3];
	bool g = true;
	int j[3];
	coordx[0] = ax; coordy[0] = ay;
	coordx[1] = bx; coordy[1] = by;
	coordx[2] = cx; coordy[2] = cy;
	coordx[3] = dx; coordy[4] = dy;
	imax[0] = pow(bx-ax,2) + pow(by-ay,2);
	imax[1] = pow(cx-ax,2) + pow(cy-ax,2);
	imax[2] = pow(dx-ax,2) + pow(dy-ay,2);

	
	//for (int i = 0; i < 3; i++)
		//cout << imax[i] << endl;

	while (g)
	{
		g = false;
	
		for (int i = 0; i < 3; i++)
		{
			if(imax[i] < imax[i+1])
			{
				swap(imax[i],imax[i+1]);			
				g = true;		
			}
		}
	}
	//for (int i = 0; i < 3; i++)
		//cout << "_" << imax[i] << endl;
}


int main( int argc, char** argv )
{
//    ofstream out;
//    out.open("data.txt");
  //  out.close;
 
    VideoCapture cap;
    TermCriteria termcrit(TermCriteria::COUNT|TermCriteria::EPS,20,0.03);
    Size subPixWinSize(10,10), winSize(31,31);

    const int MAX_COUNT = 500;
    bool needToInit = false;
   
    if( argc == 1 || (argc == 2 && strlen(argv[1]) == 1 && isdigit(argv[1][0])))
        cap.open(argc == 2 ? argv[1][0] - '0' : 0);
    else if( argc == 2 )
        cap.open(argv[1]);

    if( !cap.isOpened() )
    {
        cout << "Could not initialize capturing...\n";
        return 0;
    }

    namedWindow( "LK Demo", 1 );
   
    Mat gray, prevGray, image, frame;
    vector<Point2f> points[2];

    for(;;)
    {
        cap >> frame;
        if( frame.empty() )
            break;

        frame.copyTo(image);
        cvtColor(image, gray, COLOR_BGR2GRAY);

   
        if( needToInit )
        {
            // automatic initialization
            goodFeaturesToTrack(gray, points[1], MAX_COUNT, 0.1, 100, Mat(), 3, 0, 0.04);
            cornerSubPix(gray, points[1], subPixWinSize, Size(-1,-1), termcrit);
        }
        else if( !points[0].empty() )
        {
            vector<uchar> status;
            vector<float> err;
            if(prevGray.empty())
                gray.copyTo(prevGray);
            calcOpticalFlowPyrLK(prevGray, gray, points[0], points[1], status, err, winSize,
                                 3, termcrit, 0, 0.001);
            size_t i, k;
            for( i = k = 0; i < points[1].size(); i++ )
            {
                points[1][k++] = points[1][i];
                circle( image, points[1][i], 3, Scalar(0,255,0), -1, 8);
//		cout << "x[" << i << "] = "  << points[0][i].x << " y[" << i << "] = "  << points[0][i].y  << endl;
//                float th = geom(focus, points[1][i].x, points[1][i].y, i);
		
// 		line( image, points[1][0], points[1][3], Scalar(255,0,0),3,8);
//		line( image, points[1][1], points[1][2], Scalar(255,0,0),3,8);  
	    }
		
            points[1].resize(k);
	    find(7,3,4,5,7,3,7,1);
//	    x1 = find(points[1][0].x,points[1][1].x,points[1][2].x,points[1][3].x,0);
//	    x2 = find(points[1][0].x,points[1][1].x,points[1][2].x,points[1][3].x,2);
//	    x3 = find(points[1][0].x,points[1][1].x,points[1][2].x,points[1][3].x,1);
//	    x4 = find(points[1][0].x,points[1][1].x,points[1][2].x,points[1][3].x,3);
//
//	    y11 = find(points[1][0].y,points[1][1].y,points[1][2].y,points[1][3].y,0);
//            y2 = find(points[1][0].y,points[1][1].y,points[1][2].y,points[1][3].y,1);
//            y3 = find(points[1][0].y,points[1][1].y,points[1][2].y,points[1][3].y,3);
//            y4 = find(points[1][0].y,points[1][1].y,points[1][2].y,points[1][3].y,2);
	}
        needToInit = false;
        imshow("LK Demo", image);
//	cout << x1 << endl;
        char c = (char)waitKey(10);
        if( c == 27 )
            break;
        switch(c)
	{
	case 'r':
            needToInit = true;
            break;
        case 'c':
            points[0].clear();
            points[1].clear();
            break;
	}
        std::swap(points[1], points[0]);
        cv::swap(prevGray, gray);
    }
  //    cout << x1 <<", " << y11 << endl;
  //    cout << x2 <<", " << y2 << endl;
  //    cout << x3 <<", " << y3 << endl;
  //    cout << x4 <<", " << y4 << endl;
      return 0;
}

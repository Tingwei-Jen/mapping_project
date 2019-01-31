#include "utils.h"
#include <iostream>
#include <string> 
#include <fstream>
#include <sstream>

using namespace cv;
using namespace std;

Utils::Utils()
{
    cout<<"Construct Utils!!"<<endl;
}

cv::Mat Utils::quaternion2RotM(double x, double y, double z, double w)
{
    double xx = x*x;
    double yy = y*y;
    double zz = z*z;

    double wx = w*x;
    double wy = w*y;
    double wz = w*z;

    double xy = x*y;
    double xz = x*z;
    double yz = y*z;

    Mat R = ( Mat_<double> ( 3,3 ) <<
                ( 1 - 2*yy - 2*zz ), ( 2*xy - 2*wz ), ( 2*xz + 2*wy ),
                ( 2*xy + 2*wz ), ( 1 - 2*xx - 2*zz ), ( 2*yz - 2*wx ),
                ( 2*xz - 2*wy ), ( 2*yz + 2*wx ), ( 1 - 2*xx - 2*yy ));

    return R;
}


void Utils::readCsv(std::string path, int n_data, vector<Mat>& T_w_cs, vector<string>& imagenames)
{
    std::vector<double> timesec;
    std::vector<double> timensec;
    std::vector<double> posx;
    std::vector<double> posy; 
    std::vector<double> posz; 
    std::vector<double> qx;
    std::vector<double> qy;
    std::vector<double> qz; 
    std::vector<double> qw;

    ifstream file(path);

    for(int row = 0; row < n_data+1; ++row)
    {
        std::string line;
        std::getline(file, line);
        if ( !file.good() ) 
            break;

        if(row >=1)                               //捨棄title
        {
            std::stringstream iss(line);

            for (int col = 0; col < 13; ++col)
            {
                std::string val;
                std::getline(iss, val, ',');
                if ( !iss.good() ) 
                    break;

                std::stringstream convertor(val);
                double value;
                convertor >> value;

                if(col==2)
                    timesec.push_back(value);
                else if(col==3)
                    timensec.push_back(value);
                else if(col==6)
                    posx.push_back(value);
                else if (col==7)
                    posy.push_back(value);
                else if (col==8)
                    posz.push_back(value);
                else if (col==9)
                    qx.push_back(value);
                else if (col==10)
                    qy.push_back(value);
                else if (col==11)
                    qz.push_back(value);
                else if (col==12)
                    qw.push_back(value);
            }
        }
    }

    for(int i=0; i<n_data; i++)
    {
        timensec[i] = timensec[i]/1000000000;
        timesec[i] = timesec[i] + timensec[i];
        string name =  std::to_string(timesec[i]).substr(0,13);
        imagenames.push_back(name);
    }
    
    for(int i=0; i<n_data; i++)
    {
        cv::Mat R = quaternion2RotM(qx[i], qy[i], qz[i], qw[i]);
        Mat T = (Mat_<double> (4,4) <<
            R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2), posx[i],
            R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2), posy[i],
            R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2), posz[i],
                            0,                 0,                 0,       1
        );

        T_w_cs.push_back(T);
    }
}
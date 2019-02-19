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
        string name =  std::to_string(timesec[i]).substr(0,12);
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

void Utils::readMonitorVertex(std::vector<std::vector<Point2d>>& vertexss)
{

    vertexss.clear();

    vector<Point2d> vertexs25;
    vertexs25.push_back(Point2d(534, 349));    //left top
    vertexs25.push_back(Point2d(855, 369));    //right top
    vertexs25.push_back(Point2d(520, 566));    //left buttom
    vertexs25.push_back(Point2d(846, 563));    //right buttom
    vertexss.push_back(vertexs25);

    vector<Point2d> vertexs26;
    vertexs26.push_back(Point2d(526, 349));    //left top
    vertexs26.push_back(Point2d(850, 364));    //right top
    vertexs26.push_back(Point2d(513, 566));    //left buttom
    vertexs26.push_back(Point2d(842, 560));    //right buttom
    vertexss.push_back(vertexs26);

    vector<Point2d> vertexs27;
    vertexs27.push_back(Point2d(517, 345));    //left top
    vertexs27.push_back(Point2d(849, 359));    //right top
    vertexs27.push_back(Point2d(510, 568));    //left buttom
    vertexs27.push_back(Point2d(841, 557));    //right buttom
    vertexss.push_back(vertexs27);

    vector<Point2d> vertexs28;
    vertexs28.push_back(Point2d(512, 346));    //left top
    vertexs28.push_back(Point2d(846, 356));    //right top
    vertexs28.push_back(Point2d(506, 570));    //left buttom
    vertexs28.push_back(Point2d(840, 556));    //right buttom
    vertexss.push_back(vertexs28);

    vector<Point2d> vertexs29;
    vertexs29.push_back(Point2d(505, 345));    //left top
    vertexs29.push_back(Point2d(844, 355));    //right top
    vertexs29.push_back(Point2d(501, 573));    //left buttom
    vertexs29.push_back(Point2d(839, 556));    //right buttom
    vertexss.push_back(vertexs29);

    vector<Point2d> vertexs30;
    vertexs30.push_back(Point2d(501, 343));    //left top
    vertexs30.push_back(Point2d(843, 356));    //right top
    vertexs30.push_back(Point2d(496, 576));    //left buttom
    vertexs30.push_back(Point2d(838, 558));    //right buttom
    vertexss.push_back(vertexs30);

    vector<Point2d> vertexs31;
    vertexs31.push_back(Point2d(500, 344));    //left top
    vertexs31.push_back(Point2d(846, 357));    //right top
    vertexs31.push_back(Point2d(496, 582));    //left buttom
    vertexs31.push_back(Point2d(842, 562));    //right buttom
    vertexss.push_back(vertexs31);

    vector<Point2d> vertexs32;
    vertexs32.push_back(Point2d(500, 350));    //left top
    vertexs32.push_back(Point2d(852, 358));    //right top
    vertexs32.push_back(Point2d(494, 587));    //left buttom
    vertexs32.push_back(Point2d(846, 567));    //right buttom
    vertexss.push_back(vertexs32);

    vector<Point2d> vertexs33;
    vertexs33.push_back(Point2d(499, 341));    //left top
    vertexs33.push_back(Point2d(857, 353));    //right top
    vertexs33.push_back(Point2d(495, 585));    //left buttom
    vertexs33.push_back(Point2d(852, 565));    //right buttom
    vertexss.push_back(vertexs33);

    vector<Point2d> vertexs34;
    vertexs34.push_back(Point2d(500, 340));    //left top
    vertexs34.push_back(Point2d(861, 351));    //right top
    vertexs34.push_back(Point2d(494, 587));    //left buttom
    vertexs34.push_back(Point2d(857, 566));    //right buttom
    vertexss.push_back(vertexs34);

    vector<Point2d> vertexs35;
    vertexs35.push_back(Point2d(499, 339));    //left top
    vertexs35.push_back(Point2d(869, 349));    //right top
    vertexs35.push_back(Point2d(495, 590));    //left buttom
    vertexs35.push_back(Point2d(865, 569));    //right buttom
    vertexss.push_back(vertexs35);

    vector<Point2d> vertexs36;
    vertexs36.push_back(Point2d(503, 333));    //left top
    vertexs36.push_back(Point2d(878, 348));    //right top
    vertexs36.push_back(Point2d(496, 591));    //left buttom
    vertexs36.push_back(Point2d(873, 571));    //right buttom
    vertexss.push_back(vertexs36);

    vector<Point2d> vertexs37;
    vertexs37.push_back(Point2d(506, 330));    //left top
    vertexs37.push_back(Point2d(888, 348));    //right top
    vertexs37.push_back(Point2d(497, 594));    //left buttom
    vertexs37.push_back(Point2d(881, 576));    //right buttom
    vertexss.push_back(vertexs37);

    vector<Point2d> vertexs38;
    vertexs38.push_back(Point2d(504, 325));    //left top
    vertexs38.push_back(Point2d(894, 349));    //right top
    vertexs38.push_back(Point2d(493, 595));    //left buttom
    vertexs38.push_back(Point2d(886, 581));    //right buttom
    vertexss.push_back(vertexs38);

    vector<Point2d> vertexs39;
    vertexs39.push_back(Point2d(500, 315));    //left top
    vertexs39.push_back(Point2d(898, 343));    //right top
    vertexs39.push_back(Point2d(487, 591));    //left buttom
    vertexs39.push_back(Point2d(888, 579));    //right buttom
    vertexss.push_back(vertexs39);

    vector<Point2d> vertexs40;
    vertexs40.push_back(Point2d(496, 300));    //left top
    vertexs40.push_back(Point2d(903, 333));    //right top
    vertexs40.push_back(Point2d(482, 582));    //left buttom
    vertexs40.push_back(Point2d(889, 573));    //right buttom
    vertexss.push_back(vertexs40);

    vector<Point2d> vertexs41;
    vertexs41.push_back(Point2d(495, 287));    //left top
    vertexs41.push_back(Point2d(910, 322));    //right top
    vertexs41.push_back(Point2d(480, 575));    //left buttom
    vertexs41.push_back(Point2d(895, 566));    //right buttom
    vertexss.push_back(vertexs41);

    vector<Point2d> vertexs42;
    vertexs42.push_back(Point2d(496, 275));    //left top
    vertexs42.push_back(Point2d(921, 311));    //right top
    vertexs42.push_back(Point2d(482, 570));    //left buttom
    vertexs42.push_back(Point2d(905, 561));    //right buttom
    vertexss.push_back(vertexs42);

    vector<Point2d> vertexs43;
    vertexs43.push_back(Point2d(494, 262));    //left top
    vertexs43.push_back(Point2d(928, 301));    //right top
    vertexs43.push_back(Point2d(480, 564));    //left buttom
    vertexs43.push_back(Point2d(910, 554));    //right buttom
    vertexss.push_back(vertexs43);

    vector<Point2d> vertexs44;
    vertexs44.push_back(Point2d(488, 248));    //left top
    vertexs44.push_back(Point2d(931, 293));    //right top
    vertexs44.push_back(Point2d(471, 558));    //left buttom
    vertexs44.push_back(Point2d(910, 552));    //right buttom
    vertexss.push_back(vertexs44);

    vector<Point2d> vertexs45;
    vertexs45.push_back(Point2d(483, 237));    //left top
    vertexs45.push_back(Point2d(934, 287));    //right top
    vertexs45.push_back(Point2d(463, 555));    //left buttom
    vertexs45.push_back(Point2d(911, 552));    //right buttom
    vertexss.push_back(vertexs45);

    vector<Point2d> vertexs46;
    vertexs46.push_back(Point2d(474, 227));    //left top
    vertexs46.push_back(Point2d(934, 283));    //right top
    vertexs46.push_back(Point2d(454, 553));    //left buttom
    vertexs46.push_back(Point2d(909, 550));    //right buttom
    vertexss.push_back(vertexs46);

    vector<Point2d> vertexs47;
    vertexs47.push_back(Point2d(467, 222));    //left top
    vertexs47.push_back(Point2d(937, 279));    //right top
    vertexs47.push_back(Point2d(446, 557));    //left buttom
    vertexs47.push_back(Point2d(912, 553));    //right buttom
    vertexss.push_back(vertexs47);

    vector<Point2d> vertexs48;
    vertexs48.push_back(Point2d(461, 223));    //left top
    vertexs48.push_back(Point2d(939, 282));    //right top
    vertexs48.push_back(Point2d(438, 567));    //left buttom
    vertexs48.push_back(Point2d(917, 561));    //right buttom
    vertexss.push_back(vertexs48);

    vector<Point2d> vertexs49;
    vertexs49.push_back(Point2d(453, 222));    //left top
    vertexs49.push_back(Point2d(939, 288));    //right top
    vertexs49.push_back(Point2d(426, 575));    //left buttom
    vertexs49.push_back(Point2d(916, 573));    //right buttom
    vertexss.push_back(vertexs49);

    vector<Point2d> vertexs50;
    vertexs50.push_back(Point2d(448, 222));    //left top
    vertexs50.push_back(Point2d(943, 291));    //right top
    vertexs50.push_back(Point2d(418, 585));    //left buttom
    vertexs50.push_back(Point2d(921, 582));    //right buttom
    vertexss.push_back(vertexs50);

    vector<Point2d> vertexs51;
    vertexs51.push_back(Point2d(442, 217));    //left top
    vertexs51.push_back(Point2d(946, 293));    //right top
    vertexs51.push_back(Point2d(408, 589));    //left buttom
    vertexs51.push_back(Point2d(924, 590));    //right buttom
    vertexss.push_back(vertexs51);

    vector<Point2d> vertexs52;
    vertexs52.push_back(Point2d(435, 209));    //left top
    vertexs52.push_back(Point2d(947, 294));    //right top
    vertexs52.push_back(Point2d(394, 593));    //left buttom
    vertexs52.push_back(Point2d(922, 597));    //right buttom
    vertexss.push_back(vertexs52);

    vector<Point2d> vertexs53;
    vertexs53.push_back(Point2d(428, 203));    //left top
    vertexs53.push_back(Point2d(950, 295));    //right top
    vertexs53.push_back(Point2d(383, 598));    //left buttom
    vertexs53.push_back(Point2d(925, 604));    //right buttom
    vertexss.push_back(vertexs53);

    vector<Point2d> vertexs54;
    vertexs54.push_back(Point2d(421, 189));    //left top
    vertexs54.push_back(Point2d(953, 291));    //right top
    vertexs54.push_back(Point2d(372, 595));    //left buttom
    vertexs54.push_back(Point2d(925, 604));    //right buttom
    vertexss.push_back(vertexs54);

    vector<Point2d> vertexs55;
    vertexs55.push_back(Point2d(416, 168));    //left top
    vertexs55.push_back(Point2d(959, 283));    //right top
    vertexs55.push_back(Point2d(363, 583));    //left buttom
    vertexs55.push_back(Point2d(929, 601));    //right buttom
    vertexss.push_back(vertexs55);

    vector<Point2d> vertexs56;
    vertexs56.push_back(Point2d(418, 139));    //left top
    vertexs56.push_back(Point2d(973, 264));    //right top
    vertexs56.push_back(Point2d(359, 566));    //left buttom
    vertexs56.push_back(Point2d(933, 589));    //right buttom
    vertexss.push_back(vertexs56);

    vector<Point2d> vertexs57;
    vertexs57.push_back(Point2d(417, 116));    //left top
    vertexs57.push_back(Point2d(979, 255)) ;    //right top
    vertexs57.push_back(Point2d(355, 555));    //left buttom
    vertexs57.push_back(Point2d(938, 583));    //right buttom
    vertexss.push_back(vertexs57);

    vector<Point2d> vertexs58;
    vertexs58.push_back(Point2d(407, 103));    //left top
    vertexs58.push_back(Point2d(976, 251));    //right top
    vertexs58.push_back(Point2d(341, 552));    //left buttom
    vertexs58.push_back(Point2d(939, 582));    //right buttom
    vertexss.push_back(vertexs58);

    vector<Point2d> vertexs59;
    vertexs59.push_back(Point2d(395, 87));    //left top
    vertexs59.push_back(Point2d(976, 238));    //right top
    vertexs59.push_back(Point2d(331, 549));    //left buttom
    vertexs59.push_back(Point2d(940, 574));    //right buttom
    vertexss.push_back(vertexs59);

    vector<Point2d> vertexs60;
    vertexs60.push_back(Point2d(385, 82));    //left top
    vertexs60.push_back(Point2d(978, 237));    //right top
    vertexs60.push_back(Point2d(321, 555));    //left buttom
    vertexs60.push_back(Point2d(940, 578));    //right buttom
    vertexss.push_back(vertexs60);

    vector<Point2d> vertexs61;
    vertexs61.push_back(Point2d(380, 75));    //left top
    vertexs61.push_back(Point2d(980, 235));    //right top
    vertexs61.push_back(Point2d(312, 561));    //left buttom
    vertexs61.push_back(Point2d(944, 581));    //right buttom
    vertexss.push_back(vertexs61);

    vector<Point2d> vertexs62;
    vertexs62.push_back(Point2d(371, 68));    //left top
    vertexs62.push_back(Point2d(980, 234));    //right top
    vertexs62.push_back(Point2d(301, 567));    //left buttom
    vertexs62.push_back(Point2d(946, 586));    //right buttom
    vertexss.push_back(vertexs62);

    vector<Point2d> vertexs63;
    vertexs63.push_back(Point2d(362, 56));    //left top
    vertexs63.push_back(Point2d(982, 230));    //right top
    vertexs63.push_back(Point2d(293, 567));    //left buttom
    vertexs63.push_back(Point2d(949, 586));    //right buttom
    vertexss.push_back(vertexs63);

    vector<Point2d> vertexs64;
    vertexs64.push_back(Point2d(357, 47));    //left top
    vertexs64.push_back(Point2d(984, 228));    //right top
    vertexs64.push_back(Point2d(280, 569));    //left buttom
    vertexs64.push_back(Point2d(951, 588));    //right buttom
    vertexss.push_back(vertexs64);

}

void Utils::readLandMarkVertex(std::vector<std::vector<Point2d>>& vertexss)
{

    vertexss.clear();

    vector<Point2d> vertexs0;
    vertexs0.push_back(Point2d(474, 455));    //left top
    vertexs0.push_back(Point2d(581, 459));    //right top
    vertexs0.push_back(Point2d(467, 527));    //left buttom
    vertexs0.push_back(Point2d(577, 531));    //right buttom
    vertexss.push_back(vertexs0);

    vector<Point2d> vertexs1;
    vertexs1.push_back(Point2d(466, 459));    //left top
    vertexs1.push_back(Point2d(575, 463));    //right top
    vertexs1.push_back(Point2d(460, 532));    //left buttom
    vertexs1.push_back(Point2d(571, 536));    //right buttom
    vertexss.push_back(vertexs1);

    vector<Point2d> vertexs2;
    vertexs2.push_back(Point2d(451, 461));    //left top
    vertexs2.push_back(Point2d(563, 464));    //right top
    vertexs2.push_back(Point2d(446, 536));    //left buttom
    vertexs2.push_back(Point2d(559, 538));    //right buttom
    vertexss.push_back(vertexs2);    

    vector<Point2d> vertexs3;
    vertexs3.push_back(Point2d(436, 459));    //left top
    vertexs3.push_back(Point2d(549, 462));    //right top
    vertexs3.push_back(Point2d(430, 536));    //left buttom
    vertexs3.push_back(Point2d(546, 537));    //right buttom
    vertexss.push_back(vertexs3);

    vector<Point2d> vertexs4;
    vertexs4.push_back(Point2d(421, 458));    //left top
    vertexs4.push_back(Point2d(537, 461));    //right top
    vertexs4.push_back(Point2d(415, 537));    //left buttom
    vertexs4.push_back(Point2d(534, 539));    //right buttom
    vertexss.push_back(vertexs4);

    vector<Point2d> vertexs5;
    vertexs5.push_back(Point2d(404, 455));    //left top
    vertexs5.push_back(Point2d(523, 458));    //right top
    vertexs5.push_back(Point2d(398, 536));    //left buttom
    vertexs5.push_back(Point2d(521, 538));    //right buttom
    vertexss.push_back(vertexs5);

    vector<Point2d> vertexs6;
    vertexs6.push_back(Point2d(389, 446));    //left top
    vertexs6.push_back(Point2d(512, 449));    //right top
    vertexs6.push_back(Point2d(382, 529));    //left buttom
    vertexs6.push_back(Point2d(509, 531));    //right buttom
    vertexss.push_back(vertexs6);

    vector<Point2d> vertexs7;
    vertexs7.push_back(Point2d(382, 440));    //left top
    vertexs7.push_back(Point2d(510, 445));    //right top
    vertexs7.push_back(Point2d(374, 527));    //left buttom
    vertexs7.push_back(Point2d(506, 530));    //right buttom
    vertexss.push_back(vertexs7);

    vector<Point2d> vertexs8;
    vertexs8.push_back(Point2d(364, 438));    //left top
    vertexs8.push_back(Point2d(497, 445));    //right top
    vertexs8.push_back(Point2d(354, 528));    //left buttom
    vertexs8.push_back(Point2d(492, 532));    //right buttom
    vertexss.push_back(vertexs8);

    vector<Point2d> vertexs9;
    vertexs9.push_back(Point2d(371, 430));    //left top
    vertexs9.push_back(Point2d(508, 438));    //right top
    vertexs9.push_back(Point2d(359, 523));    //left buttom
    vertexs9.push_back(Point2d(502, 529));    //right buttom
    vertexss.push_back(vertexs9);

    vector<Point2d> vertexs10;
    vertexs10.push_back(Point2d(380, 429));    //left top
    vertexs10.push_back(Point2d(520, 438));    //right top
    vertexs10.push_back(Point2d(368, 524));    //left buttom
    vertexs10.push_back(Point2d(513, 532));    //right buttom
    vertexss.push_back(vertexs10);

    vector<Point2d> vertexs11;
    vertexs11.push_back(Point2d(362, 433));    //left top
    vertexs11.push_back(Point2d(508, 441));    //right top
    vertexs11.push_back(Point2d(349, 533));    //left buttom
    vertexs11.push_back(Point2d(501, 539));    //right buttom
    vertexss.push_back(vertexs11);

    vector<Point2d> vertexs12;
    vertexs12.push_back(Point2d(320, 439));    //left top
    vertexs12.push_back(Point2d(476, 445));    //right top
    vertexs12.push_back(Point2d(308, 544));    //left buttom
    vertexs12.push_back(Point2d(470, 548));    //right buttom
    vertexss.push_back(vertexs12);       

    vector<Point2d> vertexs13;
    vertexs13.push_back(Point2d(278, 437));    //left top
    vertexs13.push_back(Point2d(444, 443));    //right top
    vertexs13.push_back(Point2d(264, 549));    //left buttom
    vertexs13.push_back(Point2d(437, 551));    //right buttom
    vertexss.push_back(vertexs13);

    vector<Point2d> vertexs14;
    vertexs14.push_back(Point2d(229, 421));    //left top
    vertexs14.push_back(Point2d(408, 429));    //right top
    vertexs14.push_back(Point2d(212, 540));    //left buttom
    vertexs14.push_back(Point2d(399, 543));    //right buttom
    vertexss.push_back(vertexs14);

    vector<Point2d> vertexs15;
    vertexs15.push_back(Point2d(180, 442));    //left top
    vertexs15.push_back(Point2d(374, 446));    //right top
    vertexs15.push_back(Point2d(160, 571));    //left buttom
    vertexs15.push_back(Point2d(364, 568));    //right buttom
    vertexss.push_back(vertexs15);

    vector<Point2d> vertexs16;
    vertexs16.push_back(Point2d(126, 453));    //left top
    vertexs16.push_back(Point2d(338, 453));    //right top
    vertexs16.push_back(Point2d(105, 592));    //left buttom
    vertexs16.push_back(Point2d(329, 584));    //right buttom
    vertexss.push_back(vertexs16);

    vector<Point2d> vertexs17;
    vertexs17.push_back(Point2d(73, 435));    //left top
    vertexs17.push_back(Point2d(305, 436));    //right top
    vertexs17.push_back(Point2d(49, 584));    //left buttom
    vertexs17.push_back(Point2d(295, 574));    //right buttom
    vertexss.push_back(vertexs17);
}
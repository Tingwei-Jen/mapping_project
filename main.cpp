#include "mapping.h"
#include "utils.h"

#include <iostream>
#include <string> 

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>

using namespace std;
using namespace cv;
using namespace cv::xfeatures2d;

void test()
{

    Mat K = ( Mat_<double> ( 3,3 ) << 4.616e+02, 0, 3.630e+02, 0, 4.603e+02, 2.481e+02, 0, 0, 1 );
    Mat distCoeffs= ( Mat_<double> ( 1,4 ) << -2.917e-01, 8.228e-02, 5.333e-05, -1.578e-04);
    
    int n_data = 70;
    int n_train = 50;
    int n_test = 20;
    int ref_index = 0;
    vector<Mat> T_w_cs;
    vector<string> imagenames;
    Utils *utils = new Utils();
    utils->readCsv("../data/2019-01-18-19-16-50-vins_estimator-camera_pose.csv", n_data, T_w_cs, imagenames);
    
    int width = 752;
    int height = 480;
    int border = 20;
    Mapping *mapping = new Mapping(K, width, height, border);

    //reference
    Mat ref = imread("../data/images/"+imagenames[ref_index]+".png", 0);    //gray_scale
    Mat refUndistorted;
    undistort(ref, refUndistorted, K, distCoeffs);
    Mat T_w_ref = T_w_cs[ref_index];
    vector<Point2d> pts_ref;
    Point2d pt_ref1 = Point2f(511, 247);
    Point2d pt_ref2 = Point2f(576, 325);
    pts_ref.push_back(pt_ref1);
    pts_ref.push_back(pt_ref2);

    //draw pts_ref
    Mat plot_ref;
	cvtColor(refUndistorted, plot_ref, CV_GRAY2BGR);
    for(int i=0; i<pts_ref.size(); i++)
        circle(plot_ref, pts_ref[i], 3, Scalar(255,0,0), -1);
    imshow("plot_ref",plot_ref);
    
    //current
    int window_size = 6;
    bool plot = false;
    vector<double> depth_mu(pts_ref.size(), 3); 
    vector<double> depth_cov(pts_ref.size(), 3); 
    for(int i=ref_index+1; i<(ref_index+n_train); i++)
    {
        //read image and its pose
        Mat cur = imread("../data/images/"+imagenames[i]+".png", 0);    //gray_scale
        Mat curUndistorted;
        undistort(cur, curUndistorted, K, distCoeffs);
        Mat T_w_cur = T_w_cs[i];
        
        //draw cur
        Mat plot_cur;
	    cvtColor(curUndistorted, plot_cur, CV_GRAY2BGR);
        
        //ensure there is long enough dist between camera centers
        double dist;
        mapping->computeDistOfCenters(T_w_ref,T_w_cur,dist);
        if(dist<0.1)
            continue;

        for(int j=0; j<pts_ref.size(); j++)
        {   
            //find possible locations of pt_cur
            std::vector<Point2d> pts_cur_temp;
            mapping->epipolarSegment(T_w_ref, T_w_cur, pts_ref[j], depth_mu[j], sqrt(depth_cov[j]), pts_cur_temp);
            
            if(plot)
            {
                for(int k=0; k<pts_cur_temp.size(); k++)
                    cv::circle(plot_cur, pts_cur_temp[k], 1, Scalar(255,0,0), -1);
            }

            //decide the best one
            Point2d best_pt_cur;
            bool ret = mapping->epipolarSearch(refUndistorted, curUndistorted, pts_ref[j], pts_cur_temp, window_size, best_pt_cur);
            if(!ret)
                continue;
            
            if(plot)
            {
                cv::circle(plot_cur, best_pt_cur, 3, Scalar(255,0,255), -1);
            }

            //gaussian fusion
            mapping->updateDepthFilter(T_w_ref, T_w_cur, pts_ref[j], best_pt_cur, depth_mu[j], depth_cov[j]);    
        }   
        
        if(plot)
        {
            // //for helping check location of epipolar line
            // vector<Point3d> lines_cur;
            // mapping->computeEpipolarLines(pts_ref, T_w_ref, T_w_cur, lines_cur);

            // for(int line=0; line<lines_cur.size(); line++)
            // {
            //     double a = lines_cur[line].x;
            //     double b = lines_cur[line].y;
            //     double c = lines_cur[line].z;
            //     cv::line(plot_cur, Point(0,-c/b), Point(width, -(c+a*width)/b), Scalar(0,0,255));
            // }

            imshow("plot_cur",plot_cur);
            waitKey(0);
        }
    }

    //testing////////////////
    vector<double> pts_error(pts_ref.size(), 0.0); 
    vector<int> n_detected(pts_ref.size(), 0);
    for(int i=(ref_index+n_train); i<(ref_index+n_train+n_test); i++)
    {
        Mat cur = imread("../data/images/"+imagenames[i]+".png", 0);    //gray_scale
        Mat curUndistorted;
        undistort(cur, curUndistorted, K, distCoeffs);
        Mat T_w_cur = T_w_cs[i];

        //draw
        Mat plot_cur;
	    cvtColor(curUndistorted, plot_cur, CV_GRAY2BGR);
    
        for(int j=0; j<pts_ref.size(); j++)
        {
            std::vector<Point2d> pts_cur_temp;
            mapping->epipolarSegment(T_w_ref, T_w_cur, pts_ref[j], depth_mu[j], sqrt(depth_cov[j]), pts_cur_temp);
 
            //decide the best one
            Point2d best_pt_cur;
            bool ret = mapping->epipolarSearch(refUndistorted, curUndistorted, pts_ref[j], pts_cur_temp, window_size, best_pt_cur);
            if(!ret)
                continue;
            
            Point2d pt_cur_repro;
            mapping->computeReprojection(T_w_ref, T_w_cur, pts_ref[j], depth_mu[j], pt_cur_repro);
            Point2d repro_error = best_pt_cur - pt_cur_repro;
            pts_error[j] += norm(repro_error);
            n_detected[j] += 1;

            if(plot)
            {
                cv::circle(plot_cur, best_pt_cur, 3, Scalar(255,0,255), -1);
                cv::circle(plot_cur, pt_cur_repro, 3, Scalar(0,255,255), -1);
            }
        }

        if(plot)
        {
            imshow("plot_cur", plot_cur);
            waitKey(0);
        }
    }

    for(int i=0; i<pts_ref.size(); i++)
    {
        pts_error[i] /= n_detected[i];
        cout<<"average error: "<<pts_error[i]<<"  number of pts detected: "<<n_detected[i]<<endl;
    }
}

void test2()
{
    //parameter
    Mat K = ( Mat_<double> ( 3,3 ) << 4.616e+02, 0, 3.630e+02, 0, 4.603e+02, 2.481e+02, 0, 0, 1 );
    Mat distCoeffs= ( Mat_<double> ( 1,4 ) << -2.917e-01, 8.228e-02, 5.333e-05, -1.578e-04);
    
    //read data
    int n_data = 1000;
    int n_train = 30;
    int n_test = 70;
    int ref_index = 0;
    vector<Mat> T_w_cs;
    vector<string> imagenames;
    Utils *utils = new Utils();
    utils->readCsv("../data/2019-01-18-19-16-50-vins_estimator-camera_pose.csv", n_data, T_w_cs, imagenames);
    
    //feature matching 
    Ptr<SURF> detector = SURF::create( 400 );
    FlannBasedMatcher matcher;

    //reference
    Mat ref = imread("../data/images/"+imagenames[ref_index]+".png", 0);    //gray_scale
    Mat refUndistorted;
    undistort(ref, refUndistorted, K, distCoeffs);
    Mat T_w_ref = T_w_cs[ref_index];

    //detect features in ref
  	vector<KeyPoint> keypoints_ref_all;
    detector->detect(refUndistorted, keypoints_ref_all);
    
    //specific region
    vector<KeyPoint> keypoints_ref;
    for(int i=0; i<keypoints_ref_all.size(); i++)
    {
        if(keypoints_ref_all[i].pt.x>450 && keypoints_ref_all[i].pt.x<740 && keypoints_ref_all[i].pt.y>290 && keypoints_ref_all[i].pt.y<450)
            keypoints_ref.push_back(keypoints_ref_all[i]);
    }
    
    //descriptor in reference frame
    Mat descriptors_ref;
    detector->compute(refUndistorted, keypoints_ref, descriptors_ref);
    
    //draw ref and its features
  	Mat img_keypoints_ref;
  	drawKeypoints( refUndistorted, keypoints_ref, img_keypoints_ref, Scalar::all(-1), DrawMatchesFlags::DEFAULT );
    imshow("img_keypoints_ref",img_keypoints_ref);

    //draw ref
    Mat plot_ref;
    cvtColor(refUndistorted, plot_ref, CV_GRAY2BGR);

    int width = 752;
    int height = 480;
    int border = 20;
    Mapping *mapping = new Mapping(K, width, height, border);
    
    //training/////////////////////////////
    vector<double> depth_mu(keypoints_ref.size(), 3); 
    vector<double> depth_cov(keypoints_ref.size(), 3); 
    vector<int> n_detected(keypoints_ref.size(), 0);
    for(int index=ref_index+1; index<(ref_index+n_train); index++)
    {
        //read image and its pose
        Mat cur = imread("../data/images/"+imagenames[index]+".png", 0);    //gray_scale
        Mat curUndistorted;
        undistort(cur, curUndistorted, K, distCoeffs);
        Mat T_w_cur = T_w_cs[index];
        
        //draw cur
        Mat plot_cur;
	    cvtColor(curUndistorted, plot_cur, CV_GRAY2BGR);

        //ensure there is long enough dist between camera centers
        double dist;
        mapping->computeDistOfCenters(T_w_ref,T_w_cur,dist);
        if(dist<0.1)
            continue;

        //keypoints and descriptors in current image
        vector<KeyPoint> keypoints_cur_all;
        Mat descriptors_cur;
        detector->detect(curUndistorted, keypoints_cur_all);       
        detector->compute(curUndistorted, keypoints_cur_all, descriptors_cur);
        
        //matching
        vector< DMatch > matches;
        matcher.match( descriptors_ref, descriptors_cur, matches );
    
        //find good matches according distance between keypoints.
        double max_dist = 0; 
        double min_dist = 100;
        for( int i = 0; i < descriptors_ref.rows; i++ ){ 
            double dist = matches[i].distance;
            if( dist < min_dist ) min_dist = dist;
            if( dist > max_dist ) max_dist = dist;
        }

        vector< DMatch > good_matches;
        for( int i = 0; i < descriptors_ref.rows; i++ )
        {
            if( matches[i].distance <= max(2*min_dist, 0.02) ){ 
                good_matches.push_back( matches[i]); 
            }
        }

        //epipolar constraint
        vector< DMatch > good_matches_epipolar;
        for( int i = 0; i < good_matches.size(); i++ )
        {
    	    Point2d pt_ref = keypoints_ref[ good_matches[i].queryIdx ].pt;
    	    Point2d pt_cur = keypoints_cur_all[ good_matches[i].trainIdx ].pt;

            int idx = good_matches[i].queryIdx; 

            // distant judgement
            Point3d line_cur;
            mapping->computeEpipolarLine(pt_ref, T_w_ref, T_w_cur, line_cur);
            double dist;
            mapping->distPoint2Line(pt_cur, line_cur, dist);

            if(dist>5.0)
                continue;

            // another direction
            vector<Point2d> pts_cur_temp; 
            mapping->epipolarSegment(T_w_ref, T_w_cur, pt_ref, depth_mu[idx], sqrt(depth_cov[idx]), pts_cur_temp);
            Point2d pt_cur_temp_start = pts_cur_temp[0];
            Point2d pt_cur_temp_end = pts_cur_temp[pts_cur_temp.size()-1];
            Point2d pt_cur_temp_mid = 0.5*(pt_cur_temp_start+pt_cur_temp_end);

            double half_length = 0.5*norm(pt_cur_temp_start-pt_cur_temp_end);

            Point3d verticle_line;
            mapping->verticalLine(line_cur, pt_cur_temp_mid, verticle_line);

            double dist_v;
            mapping->distPoint2Line(pt_cur, verticle_line, dist_v);

            if(dist_v > (half_length+15))
                 continue;


            // if(idx==106)
            // {
            //     cv::circle(plot_cur, pt_cur_temp_start, 4, Scalar(0,255,0), -1);
            //     cv::circle(plot_cur, pt_cur_temp_end, 4, Scalar(0,255,0), -1);
            //     cv::circle(plot_cur, pt_cur_temp_mid, 4, Scalar(0,255,0), -1);

            //     double a1 = line_cur.x;
            //     double b1 = line_cur.y;
            //     double c1 = line_cur.z;
            //     cv::line(plot_cur, Point(0,-c1/b1), Point(width, -(c1+a1*width)/b1), Scalar(0,0,255));

            //     cv::circle(plot_cur, pt_cur, 4, Scalar(255,0,0), -1);
            //     cv::circle(plot_ref, pt_ref, 4, Scalar(255,0,0), -1);

            //     double a2 = verticle_line.x;
            //     double b2 = verticle_line.y;
            //     double c2 = verticle_line.z;
            //     cv::line(plot_cur, Point(0,-c2/b2), Point(width, -(c2+a2*width)/b2), Scalar(0,0,255));
            // }

            good_matches_epipolar.push_back(good_matches[i]);
        }

        //update depthfilter
        for( int i = 0; i < good_matches_epipolar.size(); i++ )
        {
            Point2d pt_ref = keypoints_ref[ good_matches_epipolar[i].queryIdx ].pt;
    	    Point2d pt_cur = keypoints_cur_all[ good_matches_epipolar[i].trainIdx ].pt;

            int idx = good_matches_epipolar[i].queryIdx;
            n_detected[idx]++;


            mapping->updateDepthFilter(T_w_ref, T_w_cur, pt_ref, pt_cur, depth_mu[idx], depth_cov[idx]); 

            // if(idx==106)
            // {
            //     cout<<"idx: "<<idx<<"  "<<pt_ref<<"  depth:  "<<depth_mu[idx]<<"   cov:  "<<depth_cov[idx]<<endl;
            //     cv::circle(plot_ref, pt_ref, 3, Scalar(255,0,0), -1);
            //     cv::circle(plot_cur, pt_cur, 3, Scalar(255,0,0), -1);

            //     imshow("plot_ref", plot_ref );
            //     imshow("plot_cur", plot_cur );
            //     waitKey(0);

            // }
        }    

    }//training/////////////////////////////   
    

    //result
    for(int i=0; i<keypoints_ref.size(); i++)
    {
        if(n_detected[i]!=0)
        {
            //cout<<i<<"   depth: "<<depth_mu[i]<<"   n_frame deteted: "<<n_detected[i]<<endl;
            //cv::circle(plot_ref, keypoints_ref[i].pt, 3, Scalar(255,0,255), -1);
        }
    }
   // imshow("plot_ref_after_train", plot_ref);


    //testing
    vector<double> pts_error(keypoints_ref.size(), 0.0); 
    vector<int> n_detected_test(keypoints_ref.size(), 0); 
    for(int index=(ref_index+n_train); index<(ref_index+n_train+n_test); index++)
    {
        //read image and its pose
        Mat cur = imread("../data/images/"+imagenames[index]+".png", 0);    //gray_scale
        Mat curUndistorted;
        undistort(cur, curUndistorted, K, distCoeffs);
        Mat T_w_cur = T_w_cs[index];

        //draw cur
        Mat plot_cur;
	    cvtColor(curUndistorted, plot_cur, CV_GRAY2BGR);

        //ensure there is long enough dist between camera centers
        double dist;
        mapping->computeDistOfCenters(T_w_ref,T_w_cur,dist);
        if(dist<0.1)
            continue;

        //keypoints and descriptors in current image
        vector<KeyPoint> keypoints_cur_all;
        Mat descriptors_cur;
        detector->detect(curUndistorted, keypoints_cur_all);       
        detector->compute(curUndistorted, keypoints_cur_all, descriptors_cur);

        //matching
        vector< DMatch > matches;
        matcher.match( descriptors_ref, descriptors_cur, matches );

        //find good matches according distance between keypoints.
        double max_dist = 0; 
        double min_dist = 100;
        for( int i = 0; i < descriptors_ref.rows; i++ ){ 
            double dist = matches[i].distance;
            if( dist < min_dist ) min_dist = dist;
            if( dist > max_dist ) max_dist = dist;
        }

        vector< DMatch > good_matches;
        for( int i = 0; i < descriptors_ref.rows; i++ )
        {
            if( matches[i].distance <= max(2*min_dist, 0.02) ){ 
                good_matches.push_back( matches[i]); 
            }
        }

        //epipolar constraint
        vector< DMatch > good_matches_epipolar;
        for( int i = 0; i < good_matches.size(); i++ )
        {
    	    Point2d pt_ref = keypoints_ref[ good_matches[i].queryIdx ].pt;
    	    Point2d pt_cur = keypoints_cur_all[ good_matches[i].trainIdx ].pt;

            int idx = good_matches[i].queryIdx; 

            // distant judgement
            Point3d line_cur;
            mapping->computeEpipolarLine(pt_ref, T_w_ref, T_w_cur, line_cur);
            double dist;
            mapping->distPoint2Line(pt_cur, line_cur, dist);

            if(dist>5.0)
                continue;

            // another direction
            vector<Point2d> pts_cur_temp; 
            mapping->epipolarSegment(T_w_ref, T_w_cur, pt_ref, depth_mu[idx], sqrt(depth_cov[idx]), pts_cur_temp);
            Point2d pt_cur_temp_start = pts_cur_temp[0];
            Point2d pt_cur_temp_end = pts_cur_temp[pts_cur_temp.size()-1];
            Point2d pt_cur_temp_mid = 0.5*(pt_cur_temp_start+pt_cur_temp_end);

            double half_length = 0.5*norm(pt_cur_temp_start-pt_cur_temp_end);

            Point3d verticle_line;
            mapping->verticalLine(line_cur, pt_cur_temp_mid, verticle_line);

            double dist_v;
            mapping->distPoint2Line(pt_cur, verticle_line, dist_v);

            if(dist_v > (half_length+15))
                 continue;

            // cv::circle(plot_cur, pt_cur_temp_start, 3, Scalar(255,255,0), -1);
            // cv::circle(plot_cur, pt_cur_temp_end, 3, Scalar(255,255,0), -1);
            // cv::circle(plot_cur, pt_cur_temp_mid, 3, Scalar(255,255,0), -1);

            // double a1 = line_cur.x;
            // double b1 = line_cur.y;
            // double c1 = line_cur.z;
            // cv::line(plot_cur, Point(0,-c1/b1), Point(width, -(c1+a1*width)/b1), Scalar(0,0,255));

            // double a2 = verticle_line.x;
            // double b2 = verticle_line.y;
            // double c2 = verticle_line.z;
            // cv::line(plot_cur, Point(0,-c2/b2), Point(width, -(c2+a2*width)/b2), Scalar(0,0,255));
            
            // Mat plot_ref_test;
	        // cvtColor(refUndistorted, plot_ref_test, CV_GRAY2BGR);
            // cv::circle(plot_ref_test, pt_ref, 3, Scalar(255,0,0), -1);
            // cv::circle(plot_cur, pt_cur, 3, Scalar(0,0,255), -1);

            // imshow("pt_ref", plot_ref_test );
            // imshow("plot_cur", plot_cur );
            // waitKey(0);

            good_matches_epipolar.push_back(good_matches[i]);
        }




        //compute reprojection error
        for( int i = 0; i < good_matches_epipolar.size(); i++ )
        {
            Point2d pt_ref = keypoints_ref[ good_matches_epipolar[i].queryIdx ].pt;
    	    Point2d pt_cur = keypoints_cur_all[ good_matches_epipolar[i].trainIdx ].pt;

            int idx = good_matches_epipolar[i].queryIdx;
           
            if(n_detected[idx]!=0)  //seed with depth 
            {
                if(idx==106)
                {
                    depth_mu[idx] = 0.51901;
                    


                    Point2d pt_cur_repro;
                    mapping->computeReprojection(T_w_ref, T_w_cur, pt_ref, depth_mu[idx], pt_cur_repro);
                    Point2d repro_error = pt_cur - pt_cur_repro;
                    //draw ref
                    Mat plot_ref_;
                    cvtColor(refUndistorted, plot_ref_, CV_GRAY2BGR);
                    cv::circle(plot_ref_, pt_ref, 3, Scalar(255,0,0), -1);
                    cv::circle(plot_cur, pt_cur_repro, 3, Scalar(0,0,255), -1);
                    cv::circle(plot_cur, pt_cur, 3, Scalar(255,0,0), -1);
                    imshow("plot_ref", plot_ref_);
                    imshow("repro", plot_cur);
                    
                    cout<<"idx: "<<idx<<"depth: "<<depth_mu[idx]<<"   "<<depth_cov[idx]<<"   "<<n_detected[idx]<<endl;
                    cout<<"error: "<<norm(repro_error)<<endl;
                    waitKey(0);


                }

                Point2d pt_cur_repro;
                mapping->computeReprojection(T_w_ref, T_w_cur, pt_ref, depth_mu[idx], pt_cur_repro);
                Point2d repro_error = pt_cur - pt_cur_repro;
                
                //draw ref
                Mat plot_ref_;
	            cvtColor(refUndistorted, plot_ref_, CV_GRAY2BGR);
                cv::circle(plot_ref_, pt_ref, 3, Scalar(255,0,0), -1);
                cv::circle(plot_cur, pt_cur_repro, 3, Scalar(0,0,255), -1);
                cv::circle(plot_cur, pt_cur, 3, Scalar(255,0,0), -1);
                imshow("plot_ref", plot_ref_);
                imshow("repro", plot_cur);
                
                cout<<"idx: "<<idx<<"depth: "<<depth_mu[idx]<<"   "<<depth_cov[idx]<<"   "<<n_detected[idx]<<endl;
                
                waitKey(0);
                pts_error[idx] += norm(repro_error);
                n_detected_test[idx] += 1;
            }
        }
    }

    for(int i=0; i<keypoints_ref.size(); i++)
    {
        if(n_detected_test[i]!=0)
        {
            pts_error[i] /= n_detected_test[i];
            cout<<"i: "<<i<<" average error: "<<pts_error[i]<<"  number of pts test detected: "<<n_detected_test[i]<<endl;
        }
           
       
    }

    waitKey(0);
}

int main()
{
    //test();
    test2();
    return 0;
}

// name priciple
// T_w_ref:        transformation matrix from world to reference frame
// pt_ref:         point in reference frame
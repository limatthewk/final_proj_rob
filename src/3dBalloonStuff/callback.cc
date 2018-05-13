
#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>

#include "callback.h"
#include "sensorParams.h"
#include <math.h>
#include <Eigen/Dense>


/*
* Process a single image and return a vector of observations.  Observations are
* defined by the struct BalloonInfo. Look how BalloonInfo is defined in
* include/callback.h
*/

using namespace std;
using namespace cv;

const std::vector<BalloonInfo> processImage(const cv::Mat& img) {
    /* Sensor params in: sensorParams */
    /* Example: cv::Mat cameraMatrixMat = sensorParams.camera_matrix; */
    /* Look at the available sensor params in include/sensorParams.h */

    int radiusThreshold = 50;

    Mat r_mask2,r_mask1,r_mask3;
    vector<vector<Point>> r_cnts, r_approx;

    Mat b_mask2,b_mask1,b_mask3,b_mask4;
    vector<vector<Point>> b_cnts,b2_cnts;

    Mat r_output, b_output;

    float r_r, b_r;
    Point2f r_c, b_c; 


    inRange(img, Scalar(0,0,170), Scalar(140,140,255), r_mask1);
    erode(r_mask1, r_mask2, Mat(),Point(-1,-1),2);
    dilate(r_mask2, r_mask3, Mat(),Point(-1,-1),2);

    findContours(r_mask3, r_cnts, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    int r_max_area = 0;

    vector<Point2f>r_center( r_cnts.size() );
    vector<float>r_radius( r_cnts.size() );
    int r_contours[r_cnts.size()];


    inRange(img, Scalar(170,0,0), Scalar(255, 150, 5), b_mask1);
    erode(b_mask1, b_mask2, Mat(),Point(-1,-1),2);
    dilate(b_mask2, b_mask3, Mat(),Point(-1,-1),2);


    inRange(img, Scalar(230,0,0), Scalar(255, 170, 5), b_mask4);



    findContours(b_mask3, b_cnts, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    findContours(b_mask4, b2_cnts, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    int b_max_area = 0;

    int b_contours[b_cnts.size()];
    int b2_contours[b2_cnts.size()];


    vector<Point2f>b_center( b_cnts.size()+b2_cnts.size() );
    vector<float>b_radius( b_cnts.size()+b2_cnts.size() );


    for( size_t i = 0; i<r_cnts.size();i++ ) {
        minEnclosingCircle( r_cnts[i], r_center[i], r_radius[i] );

        double r_area = contourArea(r_cnts[i]);

        if( (r_area > r_max_area) && (r_area > 10000) && (r_radius[i] > radiusThreshold)){
            r_max_area = r_area;
            r_r = r_radius[i];
            r_c = r_center[i];
        }

}

    for( size_t i = 0; i<b_cnts.size();i++ ) {
        minEnclosingCircle( b_cnts[i], b_center[i], b_radius[i] );

        double b_area = contourArea(b_cnts[i]);



        if( (b_area > b_max_area) && (b_area > 10000) && (b_radius[i] > radiusThreshold)){
            b_max_area = b_area;
            b_r = b_radius[i];
            b_c = b_center[i];


        }
}

    for( size_t i = 0; i<b2_cnts.size();i++ ) {
        minEnclosingCircle( b2_cnts[i], b_center[b_cnts.size()+i], b_radius[b_cnts.size()+i] );

        double b_area = contourArea(b2_cnts[i]);


        if( (b_area > b_max_area) && (b_area > 10000) && (b_radius[b_cnts.size()+i] > radiusThreshold)){
            b_max_area = b_area;
            b_r = b_radius[b_cnts.size()+i];
            b_c = b_center[b_cnts.size()+i];
        }
}



    // Populate vector
    std::vector<BalloonInfo> balloonInfo;
    

    // Enums defined in header file
    Color RED = red;
    Color BLUE = blue;

    balloonInfo.push_back(BalloonInfo());
    balloonInfo[0].color = RED;
    balloonInfo.push_back(BalloonInfo());
    balloonInfo[1].color = BLUE;
    
    if (r_r > .0001 && r_cnts.size() > 0) {
            balloonInfo[0].balloonLocation << r_c.x,r_c.y,0;
            balloonInfo[0].balloonRadius = r_r;

}
    else{
        balloonInfo[0].balloonLocation << -1,-1,0;
        balloonInfo[0].balloonRadius = -1;


    }

    if (b_r > .0001 && b_cnts.size() > 0) {
            balloonInfo[1].balloonLocation << b_c.x,b_c.y,0;
            balloonInfo[1].balloonRadius = b_r;
        }
    else{
        balloonInfo[1].balloonLocation << -1,-1,0;
        balloonInfo[1].balloonRadius = -1;


    }


    return balloonInfo;
}

/*
* Take a database of image observations and estimate the 3D position of the balloons
*/

Eigen::MatrixXd ecef2enu(Eigen::Vector3d p_ecef){
    //----- Define WGS-84 Earth parameters
    float aa = 6378137;
    float bb = 6356752.31425;
    float ee = sqrt((pow(aa,2) - pow(bb,2))/pow(aa,2));
    float ep = sqrt((pow(aa,2) - pow(bb,2))/pow(bb,2));

    //----- Convert to (phi,lambda,h) geodetic coordinates
    float x = p_ecef[0];
    float y = p_ecef[1];
    float z = p_ecef[2];

    float lambda = atan2(y,x);
    float p = sqrt(x*x + y*y);
    float theta = atan2(z*aa, p*bb);
    float phi = atan2(z + ep*ep*bb*pow(sin(theta),3),
                    p - ee*ee*aa*pow(cos(theta),3));


    Eigen::MatrixXd Renu_ecef(3,3);

    //----- Form the rotation matrix
    Renu_ecef(0,0) = -sin(lambda);
    Renu_ecef(0,1) = cos(lambda);
    Renu_ecef(0,2) = 0;
    Renu_ecef(1,0) = -sin(phi)*cos(lambda);
    Renu_ecef(1,1) = -sin(phi)*sin(lambda);
    Renu_ecef(1,2) = cos(phi);
    Renu_ecef(2,0) = cos(phi)*cos(lambda);
    Renu_ecef(2,1) = cos(phi)*sin(lambda);
    Renu_ecef(2,2) = sin(phi);


    return Renu_ecef;

}



Eigen::MatrixXd euler2dcm(Eigen::Vector3d euler){
// 3-1-2 rotation
    float cPhi = cos(euler[0]);
    float cThe = cos(euler[1]);
    float cPsi = cos(euler[2]);
    float sPhi = sin(euler[0]);
    float sThe = sin(euler[1]);
    float sPsi = sin(euler[2]);

    Eigen::MatrixXd r_bw(3,3);

    r_bw(0,0) = cPsi*cThe - sPhi*sPsi*sThe;
    r_bw(0,1) = cThe*sPsi + cPsi*sPhi*sThe;
    r_bw(0,2) = -cPhi*sThe;
    r_bw(1,0) = -cPhi*sPsi;
    r_bw(1,1) = cPhi*cPsi;
    r_bw(1,2) = sPhi;
    r_bw(2,0) = cPsi*sThe + cThe*sPhi*sPsi;
    r_bw(2,1) = sPsi*sThe - cPsi*cThe*sPhi;
    r_bw(2,2) = cPhi*cThe;

    return r_bw;


}



Eigen::MatrixXd euler2dcm321(Eigen::Vector3d euler){
// 3-1-2 rotation
    float cPhi = cos(euler[0]);
    float cThe = cos(euler[1]);
    float cPsi = cos(euler[2]);
    float sPhi = sin(euler[0]);
    float sThe = sin(euler[1]);
    float sPsi = sin(euler[2]);

    Eigen::MatrixXd r_bw(3,3);

    r_bw(0,0) = cThe*cPsi;
    r_bw(0,1) = cThe*sPsi;
    r_bw(0,2) = -sThe;
    r_bw(1,0) = sPhi*sThe*cPsi-cPhi*sPsi;
    r_bw(1,1) = sPhi*sThe*sPsi+cPhi*cPsi;
    r_bw(1,2) = sPhi*cThe;
    r_bw(2,0) = cPhi*sThe*cPsi+sPhi*sPsi;
    r_bw(2,1) = cPhi*sThe*sPsi-sPhi*cPsi;
    r_bw(2,2) = cPhi*cThe;

    return r_bw;


}




Eigen::Vector3d elAzToUnitEnu(float el,float az){
    Eigen::Vector3d u;
    float cEN = cos(el);
    u(0,0) = cEN*sin(az);
    u(1,0) = cEN*cos(az);
    u(2,0) = sin(el);

    return u;

}


Eigen::Vector3d ecef2lla(Eigen::Vector3d p_ecef){
    float aa = 6378137;
    float bb = 6356752.31425;
    float e = sqrt((pow(aa,2) - pow(bb,2))/pow(aa,2));
    float ep = sqrt((pow(aa,2) - pow(bb,2))/pow(bb,2));
    float x = p_ecef[0];
    float y = p_ecef[1];
    float z = p_ecef[2];
    float lon = atan2(y,x);
    float p = sqrt(pow(x,2)+pow(y,2));
    float theta = atan2(z*aa,p*bb);
    float lat = atan2(z+pow(ep,2)*bb*pow(sin(theta),3), p-pow(ep,2)*aa*pow(cos(theta),3));
    float N = aa/sqrt(1-pow(e,2)*pow(sin(lat),2));
    float alt = p/cos(lat)-N;

    Eigen::Vector3d p_lla;
    p_lla << lat, lon, alt;
    return p_lla;

}







const std::map<Color, Eigen::Vector3d> estimatePosition(const std::vector<Observation>& database) {
    /* Sensor params in: sensorParams */
    /* Example: cv::Mat cameraMatrixMat = sensorParams.camera_matrix; */
    /* Look at the available sensor params in include/sensorParams.h */




    float ps = 2*pow(10,-6); //pixel size
    //float sigma2c = 20;
    int N = database.size();
    //std::cout << N << " images" << std::endl << endl;

    int q = 0;

    Eigen::MatrixXd RCB, RBI, RCI, Pc;
    Eigen::Matrix3d K;
    Eigen::Vector3d RCB_e, rcI, rcB, rI, rpB,t, rp, rpG, rpI;
    Eigen::Vector2d xctilde;
    float xtilde, ytilde;
    Eigen::Vector3d riG = Eigen::Vector3d(-742015.08, -5462218.80, 3198013.75);
    //Eigen::Vector3d riG = Eigen::Vector3d(-742018.3187986395, -5462218.0363918105, 3198014.2988005267);
    Eigen::MatrixXd RLG = ecef2enu(riG);
    Eigen::Vector3d rrG = Eigen::Vector3d(-741990.536, -5462227.638, 3198019.45);


    rcB << 0.1159,-0.0004,-0.0435;
    rpB << 0.1013, -0.0004 , 0.0472;
    K << 1691.0*ps, 0, 1914.0*ps,
        0, 1697.0*ps, 1074.0*ps,
        0, 0, 1*ps;

    //cout << K << endl<< endl;


    RCB_e << 120*M_PI/180, 0, M_PI/2;
    RCB = euler2dcm(RCB_e);



    Eigen::MatrixXd H_r(2*N,4), H_b(2*N,4);
    H_r = Eigen::MatrixXd::Zero(2*N,4);
    H_b = Eigen::MatrixXd::Zero(2*N,4);
    int i_r = 0, i_b = 0;
    int N_red = 0;
    int N_blue = 0;




    // Example code
    for(const Observation& obs: database) {


        Eigen::Vector3d euler_angles;

        float el = obs.quad_att[0];
        float az = obs.quad_att[1];
        float roll = 0;

        euler_angles << 0,el,(az-M_PI/2);

        //cout << euler_angles << endl << endl;
        RBI = euler2dcm321(euler_angles);
        //cout << RBI << endl << endl;

        //cout << "Attitude: " << obs.quad_att.transpose() << endl << endl;

        //cout << "Euler angles: " << euler_angles.transpose() << endl << endl;

        //cout << "RBI: " << RBI << endl << endl;

        RCI = RCB*RBI;
        //cout << "RCI: " << RCI << endl << endl;
        rI = RLG*(obs.quad_pos - riG) - RBI.transpose()*rpB;
        //cout << "rI: " << rI.transpose() << endl << endl;

/*
        rpI = rI + RBI.transpose()*rpB;
        rpG = RLG.transpose() * rpI + riG;
        rpG = obs.quad_pos;
        rp = rpG - rrG;
        rcI = RLG*(rp + rrG - riG) + RBI.transpose()*(rcB - rpB);
*/
        rcI = rI + RBI.transpose()*rcB;
        //cout << "rcI: " << rcI.transpose() << endl << endl;
        t = -RCI * rcI;
        Eigen::MatrixXd rcit(RCI.rows(), RCI.cols()+t.cols());
        rcit << RCI, t;
        //cout << "rcit: " << rcit << endl << endl;
        //cout << "t: " << t << endl << endl;
        //cout << "K: " << K << endl << endl;
        Pc = K*rcit;
        //cout << "Pc: " << Pc << endl << endl;
        int q = 0;


        for(const BalloonInfo& info: obs.info_vec) {


            xctilde << info.balloonLocation.head(2);
            xtilde = xctilde(0);
            ytilde = xctilde(1);


            if (xctilde(0) != -1 && xctilde(1) != -1){
                if (q == 0){
                    //cout << "xctilde: " << xctilde.transpose() << endl << endl;
                    //cout << "pc: " << Pc << endl << endl;
                    H_r.row(i_r) = xtilde*Pc.row(2) - Pc.row(0);
                    H_r.row(i_r+1)= ytilde*Pc.row(2) - Pc.row(1);
                    i_r = i_r +2;
                    N_red++;
                    //cout << "N Red" << N_red << endl << endl;
                }
                
                else if (q == 1){
                    H_b.row(i_b) = xtilde*Pc.row(2) - Pc.row(0);
                    H_b.row(i_b+1)= ytilde*Pc.row(2) - Pc.row(1);
                    i_b = i_b +2;
                    N_blue++;
                    //cout << "N Blue" << N_blue << endl << endl;
                }
            }

            if (q == 0){
                q = 1;
            }
        }
    }

    cout << "rI: " << rI.transpose() << endl << endl;

    Eigen::MatrixXd Hr_r(2*N,3), Hr_b(2*N,3), z_r(2*N,1), z_b(2*N,1);

    Hr_r.col(0) = H_r.col(0);
    Hr_r.col(1) = H_r.col(1);
    Hr_r.col(2) = H_r.col(2);

    Hr_b.col(0) = H_b.col(0);
    Hr_b.col(1) = H_b.col(1);
    Hr_b.col(2) = H_b.col(2);

    z_r = -H_r.col(3);
    z_b = -H_b.col(3);

    //cout << H_r << endl << endl;
    //cout << Hr_r << endl << endl;
    //cout << z_r << endl << endl;

/*
    Eigen::MatrixXd Hr_r(4,3), z_r(4,1);

    Hr_r << -0.0040, 0.0005, -0.0001,
            0.0001, 0.0009, -0.0044,
           -0.0040, 0.0005, -0.0001,
            0.0001, 0.0010, -0.0044;

    z_r << -0.0001,
           -0.0023,
           -0.0001,
           -0.0022;
*/


/////////
    /*
    using namespace Eigen;

    MatrixXd C, V1;
    Vector3d V2;
    double V3;
    C.setRandom(27,18);
    JacobiSVD<MatrixXd> svd( H_r, ComputeFullU | ComputeFullV);

    if (svd.matrixV().size()==16){
        V1 = svd.matrixV().col(3);
        V2 << V1(0),V1(1),V1(2);
        V3 = V1(3);
        //cout << V2 << endl << endl;
        //cout << V3 << endl << endl;
        Eigen::MatrixXd r_XrHat2 = V2/V3;
        cout << r_XrHat2.transpose() << endl << endl;

    }


*/
////////////




    Eigen::MatrixXd r_XrHat = (Hr_r.transpose()*Hr_r).inverse()*Hr_r.transpose()*z_r;

    Eigen::MatrixXd b_XrHat = (Hr_b.transpose()*Hr_b).inverse()*Hr_b.transpose()*z_b;

    // Example return
    std::map<Color, Eigen::Vector3d> balloon_positions;
    balloon_positions[red] = r_XrHat;
    balloon_positions[blue] = b_XrHat;
    return balloon_positions;
}

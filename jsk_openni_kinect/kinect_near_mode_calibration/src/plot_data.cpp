#include <stdio.h>
#include <string.h>
#include <iostream>
#include <iomanip>
#include <unistd.h> // getopt
#include <cstdlib>

#include <math.h>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <Eigen/Core>
#include <Eigen/LU>

using namespace cv;
using namespace Eigen;
using namespace std;

// image size
#define ROWS 480
#define COLS 640

#define SHIFT_SCALE 0.125

// Pixel offset from IR image to depth image
cv::Point2f ir_depth_offset = cv::Point2f(-4, -3);

void undistort_nearest(Mat img, Mat &imgRect, Mat camMatrix, Mat distCoeffs) {
  Mat src = img, &dst = imgRect;
  dst.create( src.size(), src.type() );
  int stripe_size0 = std::min(std::max(1, (1 << 12) / std::max(src.cols, 1)), src.rows);
  Mat map1(stripe_size0, src.cols, CV_16SC2), map2(stripe_size0, src.cols, CV_16UC1);
  Mat_<double> A, Ar, I = Mat_<double>::eye(3,3);
  camMatrix.convertTo(A, CV_64F);
  A.copyTo(Ar);

  double v0 = Ar(1, 2);
  for( int y = 0; y < src.rows; y += stripe_size0 )
    {
      int stripe_size = std::min( stripe_size0, src.rows - y );
      Ar(1, 2) = v0 - y;
      Mat map1_part = map1.rowRange(0, stripe_size),
	    map2_part = map2.rowRange(0, stripe_size),
	    dst_part = dst.rowRange(y, y + stripe_size);

      initUndistortRectifyMap( A, distCoeffs, I, Ar, Size(src.cols, stripe_size),
                               map1_part.type(), map1_part, map2_part );
      remap( src, dst_part, map1_part, map2_part, INTER_NEAREST, BORDER_CONSTANT );
    }
}

int main (int argc, char **argv){
  // checkerboard pattern
  int ccols = 0;
  int crows = 0;
  // cell size
  double csize = 0.0;

  char *fdir = NULL;

  opterr = 0;
  int c;
  while ((c = getopt(argc, argv, "r:c:s:")) != -1)
    {
      switch (c)
        {
        case 'r':
          crows = atoi(optarg);
          break;
        case 'c':
          ccols = atoi(optarg);
          break;
        case 's':
          csize = atof(optarg);
          break;
        }
    }

  if (optind < argc)
    fdir = argv[optind];

  if (crows == 0 || ccols == 0 || csize == 0.0 || fdir == NULL)
    {
      printf("Must give the checkerboard dimensions and data directory.\n"
             "Usage:\n"
             "%s -r ROWS -c COLS -s SQUARE_SIZE my_data_dir\n", argv[0]);
      return 1;
    }
  
  // construct the planar pattern
  vector<Point3f> pat;
  for (int i=0; i<ccols; i++)
    for (int j=0; j<crows; j++)
      pat.push_back(Point3f(i*csize,j*csize,0));
  
  int x_num = 160, y_num = 120;
  double x_step = ((ccols-1)*csize)/x_num;
  double y_step = ((crows-1)*csize)/y_num;

  vector<Point3f> on_chessboard_points;
  for (int i=0; i<160; i++)
    for (int j=0; j<120; j++)
      on_chessboard_points.push_back(Point3f( (x_step * i), (y_step * j), 0));
  

  // read in images, set up feature points and patterns
  vector<vector<Point3f> > pats;
  vector<vector<Point2f> > points;
  
  vector<vector<Point3f> > on_chessboard_points_vec;

  int fnum = 0;

  // sample data
  // double shift_offset = 1088.6594;
  // double baseline = 0.07219;
  double shift_offset = 1052.8855;
  double baseline = 0.06831;  
  double fx = 369.652255;
  
  while (1)
    {
      char chess_fname[1024];
      sprintf(chess_fname,"%s/img_ir_%02d.png",fdir,fnum);
      Mat img = imread(chess_fname, -1);
      if(img.data == NULL) break;
      //
      vector<cv::Point2f> corners;
      bool ret = cv::findChessboardCorners(img,Size(crows,ccols),corners);
      if (ret)
        printf("Found corners in image %s\n",chess_fname);
      else {
        printf("*** Didn't find corners in image %s\n",chess_fname);
        return 1;
      }

      cv::cornerSubPix(img, corners, Size(5,5), Size(-1,-1),
                       TermCriteria(TermCriteria::MAX_ITER+TermCriteria::EPS, 30, 0.1));

      for (unsigned int i = 0; i < corners.size(); ++i)
        corners[i] += ir_depth_offset;
      
      pats.push_back(pat);
      points.push_back(corners);

      on_chessboard_points_vec.push_back(on_chessboard_points);

      fnum++;
    }

  // Monocular calibration of depth camera
  Mat camMatrix;
  Mat distCoeffs;
  vector<Mat> rvecs;
  vector<Mat> tvecs;
  double rp_err;
  // Currently assuming zero distortion
  rp_err = calibrateCamera(pats, points, Size(COLS,ROWS), camMatrix, distCoeffs,
                           rvecs, tvecs,
                           CV_CALIB_FIX_K3 | 
                           //CV_CALIB_FIX_K2 | 
                           //CV_CALIB_FIX_K1 | 
                           CV_CALIB_ZERO_TANGENT_DIST //|
                           //CV_CALIB_FIX_PRINCIPAL_POINT |
                           //CV_CALIB_FIX_ASPECT_RATIO
                           );

  printf("\nCalibration results:\n");

  // print camera matrix
  printf("\nCamera matrix\n");
  double *dptr = camMatrix.ptr<double>(0);
  for (int i=0; i<3; i++)
    {
      for (int j=0; j<3; j++)
        printf("%f ",*dptr++);
      printf("\n");
    }
  //printf("\nAssuming zero distortion\n");
  dptr = distCoeffs.ptr<double>(0);
  printf("\nDistortion coefficients:\n"
         "k1: %f\n"
         "k2: %f\n"
         "t1: %f\n"
         "t2: %f\n"
         "k3: %f\n", dptr[0], dptr[1], dptr[2], dptr[3], dptr[4]);
  
  printf("\nReprojection error = %f\n\n", rp_err);

  fnum = 0;

  FILE *disp_f = fopen("disp_vs_true.gp", "w");
  FILE *z_f = fopen("z_vs_true.gp", "w");

  FILE *disp_chess_f = fopen("on_chess_disp_vs_true.gp", "w");
  FILE *z_chess_f = fopen("on_chess_z_vs_true.gp", "w");
  
  while (1)
    {
      // Load raw depth readings
      char fname[1024];
      sprintf(fname,"%s/img_depth_%02d.png",fdir,fnum);
      Mat img_depth = imread(fname,-1);
      if (img_depth.data == NULL) break; // no data, not read, break out
      Mat img_depth_rect;
      undistort_nearest(img_depth, img_depth_rect, camMatrix, distCoeffs);

      // Get corner points and extrinsic parameters
      const cv::Mat pattern(pats[fnum]); // 3-channel matrix view of vector<Point3f>
      const cv::Mat on_pattern_pts(on_chessboard_points_vec[fnum]); // 3-channel matrix view of vector<Point3f>

      vector<Point2f> &corners = points[fnum];
      cv::Mat rvec = rvecs[fnum];
      cv::Mat tvec = tvecs[fnum];
      cv::Mat rot3x3;
      cv::Rodrigues(rvec, rot3x3);

      // Transform object points into camera coordinates using (rvec, tvec)
      cv::Mat world_points;
      cv::Mat on_chess_world_points;
      cv::Mat xfm(3, 4, cv::DataType<double>::type);
      cv::Mat xfm_rot = xfm.colRange(0,3);
      cv::Mat xfm_trans = xfm.col(3);
      rot3x3.copyTo(xfm_rot);
      tvec.reshape(1,3).copyTo(xfm_trans);
      cv::transform(pattern, world_points, xfm);
      cv::transform(on_pattern_pts, on_chess_world_points, xfm);

      vector<Point2f> on_chess_pts;
      projectPoints(on_pattern_pts, rvec, tvec, camMatrix, distCoeffs, on_chess_pts);
 
      for (unsigned int j = 0; j < corners.size(); ++j) {
        double Z = world_points.at<cv::Vec3f>(j)[2];   // actual depth
        //Z = norm(world_points.at<cv::Vec3f>(j));
        double r = img_depth_rect.at<uint16_t>(corners[j]); // sensor reading
        
        float disparity = SHIFT_SCALE * ( shift_offset - r);
        float z = fx * baseline / disparity;
        if ( (0 < r) && (2047.0 < r) ){
          fprintf(disp_f, "%lf %lf\n", disparity, Z);
          fprintf(z_f, "%lf %lf\n", z, Z);
        }
      }

      for (unsigned int j = 0; j < on_chess_pts.size(); ++j) {
        double Z = on_chess_world_points.at<cv::Vec3f>(j)[2];   // actual depth
        double r = img_depth_rect.at<uint16_t>(on_chess_pts[j]); // sensor reading        
        float disparity = SHIFT_SCALE * ( shift_offset - r);
        float z = fx * baseline / disparity;
        if ( (0 < r) && (r < 2047.0) ){
          fprintf(disp_chess_f, "%lf %lf\n", disparity, Z);
          fprintf(z_chess_f, "%lf %lf\n", z, Z);
        }
      }

      fnum++;
    }
  return 0;
}

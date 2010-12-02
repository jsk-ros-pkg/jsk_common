===================================================
NM33-UVCT(ロボット版)用LinuxSDK・サンプルプログラム
===================================================

NM33とは？
----------
オプト株式会社_ が開発した 全方位画像の取得が可能な超小型の魚眼レンズカメラ(
http://www.optnagano.co.jp/products/nm33.html) で，以下の特徴があります．

- 3cmx4cmの超小型軽量ボディに魚眼レンズを搭載
- ロボットなど組込み用途に最適、USBバスパワー対応
- 高速画像処理によりカメラ内部で魚眼映像を展開し出力
- UVC（USB Video Class）対応モデルを新規開発

NM33-UVCT(ロボット版）とは？
----------------------------
ロボットアプリケーション向けNM33で，
その特徴は全方位画像中の複数注視点画像の取得が可能であり，
物理的には一つのカメラですが，あたかも4つのカメラからの画像が取得でき
るような仕掛けがカメラの本体ハードウェアの中に組み込まれています．

複数注視画像の取得機能は少子高齢化社会と人を支援するＩＲＴ研究機構での
富士通研究所との共同研究の中で，オプト株式会社_ と
東京大学情報システム工学研究室_ が開発したものです．


NM33-UVCT(ロボット版)仕様書
---------------------------
NM33-UVCT(ロボット版)のコマンド仕様書が必要な方は オプト株式会社_ にご相談ください．


LinuxSDK・サンプルプログラム
----------------------------

NM33-UVCT(ロボット版)のLinuxSDKおよびサンプルプログラムを公開します．
ダウンロードは以下の様におこないます．::

 $ svn co https://jsk-ros-pkg.svn.sourceforge.net/svnroot/jsk-ros-pkg/trunk/opt_camera/src/opt_nm33_uvc
 $ cd opt_nm33_uvc

次にダウンロードしたopt_nm33_uvcデレクトリで ::

 $ make

とすることで init_xu_register と opt_nm33_viewer という2つの実行ファイルが
生成されます．コンピュータを立ち上げて初めてカメラを利用する場合は，ま
ず ::

 $ sudo ./init_xu_register

として初期設定を行う必要があります．その後::

 $ ./opt_nm33_viewer

として，NM33（ロボット版）を使ったサンプルプログラムを実行することが出来ます．

サンプルプログラムは以下の様になっており，OpenCVを使ったことがある人な
ら簡単に使える形になっています．::

  #include "opt_nm33_camera.h"
  #include <highgui.h>
  
    int main(int argc, char *argv[]) {
    int camera_index = 1;
    OptNM3xCamera *camera;
    IplImage *frame;
    CvMat subframe;
    int count = 0;
  
    camera = new OptNM3xCamera(camera_index);
    camera->setSmallHemisphere(1);
    camera->setLocationAbsolute(0, 0, 0, 0,   0);
    camera->setLocationAbsolute(1, 0, 0, 0,  40);
    camera->setLocationAbsolute(2, 0, 0, 0, 120);
  
    cvNamedWindow("Opt NM33 Camera 0", CV_WINDOW_AUTOSIZE);
    cvNamedWindow("Opt NM33 Camera 1", CV_WINDOW_AUTOSIZE);
    cvNamedWindow("Opt NM33 Camera 2", CV_WINDOW_AUTOSIZE);
    cvNamedWindow("Opt NM33 Camera 3", CV_WINDOW_AUTOSIZE);
  
    //cvSetMouseCallback ("Opt NM33 Camera 0", on_mouse);
  
    while (1)  {
      frame = camera->queryFrame();
  
      camera->getOmniImage(frame, subframe);
      cvShowImage ("Opt NM33 Camera 0", &subframe);
  
      camera->getWideImage(frame, subframe);
      cvShowImage ("Opt NM33 Camera 1", &subframe);
  
      camera->getMiddleImage(frame, subframe);
      cvShowImage ("Opt NM33 Camera 2", &subframe);
  
      camera->getNarrowImage(frame, subframe);
      cvShowImage ("Opt NM33 Camera 3", &subframe);
  
      float f = (float)count/20.0;
      camera->setLocationAbsolute(2, 180*sin(f), 45*sin(f+M_PI/3)+45, 180*sin(f+M_PI/2), 50*sin(f*2)+100);
  
      char c = cvWaitKey (2);
      if (c == '\x1b')
        break;
      count++;
    }
  }


注意
----

- このページは情報提供のためだけに作成されており，東京大学情報システム工学研究室並びにオプト株式会社は内容の正確性，有用性について保証いたしません．
- 閲覧者がこのページの情報を利用または利用しなかったことよる一切の損失についての責任は負いません．

.. _東京大学情報システム工学研究室: http://www.jsk.t.u-tokyo.ac.jp/
.. _オプト株式会社: http://www.optnagano.co.jp/


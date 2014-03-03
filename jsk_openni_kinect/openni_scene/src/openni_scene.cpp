// openni_scene.cpp
#include <ros/ros.h>
#include <ros/package.h>

#include <XnOpenNI.h>
#include <XnCppWrapper.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#if USE_PCL_AS_PCL_MSGS
#include <pcl/PointIndices.h>
#define POINT_INDICES_TYPE pcl::PointIndices
#else
#include <pcl_msgs/PointIndices.h>
#define POINT_INDICES_TYPE pcl_msgs::PointIndices
#endif

using std::string;
xn::Context g_Context;
xn::SceneAnalyzer g_SceneAnalyzer;

//
std::vector<ros::Publisher> pub_index;

// copy from openni.h
#define MAX(a,b) ((a)>(b)?(a):(b))
#define AVG(a,b) (((int)(a) + (int)(b)) >> 1)
#define AVG3(a,b,c) (((int)(a) + (int)(b) + (int)(c)) / 3)
#define AVG4(a,b,c,d) (((int)(a) + (int)(b) + (int)(c) + (int)(d)) >> 2)

#define CHECK_RC(nRetVal, what)                                         \
  if (nRetVal != XN_STATUS_OK){                                         \
    printf ("%s failed: %s\n", what, xnGetStatusString (nRetVal));      \
    return nRetVal;                                                     \
  }

unsigned int nLabel = 0;
std::vector< std::vector<int > > nIndices;

// COPY from OpenNIDriver::publishXYZRGBPointCloud
void createScenePointCloud (const xn::SceneMetaData& smd)
{
  const XnLabel* pLabels = smd.Data();
  nLabel = 0;
  nIndices.clear();
  for (unsigned int i = 0; i < smd.XRes()*smd.YRes();i++)
  {
    nLabel = MAX(nLabel, pLabels[i]);
  }
  nIndices.resize(nLabel);
  ROS_INFO("found %d people", nLabel);

  for (unsigned int i = 0; i < smd.XRes()*smd.YRes();i++)
  {
    // (push i (elt indexes pLabels[i]))
    if ( pLabels[i] <= 0) continue;
    nIndices[pLabels[i]-1].push_back(i);
  }
}

int main (int argc, char **argv)
{
  ros::init (argc, argv, "openni_scene");
  ros::NodeHandle nh("~");

  //pub_indices_ = nh.advertise<openni_scene::IndicesArray> ("indices", 4);

  // Initialize OpenNI
  string configFilename = ros::package::getPath ("openni_scene") + "/openni_scene.xml";
  xn::ScriptNode scriptNode;
  XnStatus nRetVal = g_Context.InitFromXmlFile (configFilename.c_str (), scriptNode);
  CHECK_RC (nRetVal, "InitFromXml");

  nRetVal = g_Context.FindExistingNode (XN_NODE_TYPE_SCENE, g_SceneAnalyzer);
  CHECK_RC (nRetVal, "Find scene analyzer");

  // Initialization done. Start generating
  nRetVal = g_Context.StartGeneratingAll ();
  CHECK_RC (nRetVal, "StartGenerating");

  while (ros::ok ())
  {
    g_Context.WaitAndUpdateAll ();
    ros::Time time = ros::Time::now();

    xn::SceneMetaData sceneMD;
    g_SceneAnalyzer.GetMetaData( sceneMD );

    createScenePointCloud ( sceneMD );

    // publish
#define max(a,b) (((a)<(b))?(b):(a))
    for (unsigned int i = 0; i < max(nLabel, pub_index.size()); i++) {

      if ( i >= nLabel ) {
        if ( pub_index[i] != NULL ) {
          pub_index[i].shutdown ();
        }
        continue;
      }

      POINT_INDICES_TYPE msg;
      msg.header.frame_id = "openni_depth_frame";
      //msg.header.stamp = time;
      msg.indices = nIndices[i];
      std::string topic = "indices" + string(1, '0' + i);

      ROS_INFO_STREAM(" index " << i << "/" << nLabel << " " << ((i<nLabel)?nIndices[i].size():-1) << "    " << pub_index.size() );
      if ( i < pub_index.size() ) {
        fprintf(stderr, "%p %d\n", &(pub_index[i]), pub_index[i] == NULL );
      }

      if ( i < pub_index.size() ) {
        if ( pub_index[i] == NULL ) {
          if ( nIndices[i].size() > 0 ) { // advertise
            ROS_INFO_STREAM( " 1 " << i << " " << pub_index.size());
            pub_index[i] = nh.advertise<POINT_INDICES_TYPE>(topic, 10);
            pub_index[i].publish(msg);
          } else { // do nothing
          }
        } else { //
          if ( nIndices[i].size() > 0 ) { // publish
            ROS_INFO_STREAM( " 3 " << i << " " << pub_index.size());
            pub_index[i].publish(msg);
          } else { // unadvertise
            pub_index[i].shutdown ();
          }
        }
      } else { // i >_ pub_index.size() )
        pub_index.push_back( nh.advertise<POINT_INDICES_TYPE>(topic, 10) );
        if ( nIndices[i].size() > 0 ) { // advertise
          ROS_INFO_STREAM( " 2 " << i << " " << pub_index.size());
          pub_index[i].publish(msg);
        } else { // do nothing
          pub_index[i].shutdown ();
        }
      }

    }
  }
  g_Context.Release ();
  return 0;
}

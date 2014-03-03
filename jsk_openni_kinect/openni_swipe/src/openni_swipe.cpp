// ros
#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/String.h>

ros::Publisher swipe_pub;
ros::Publisher swipe_status_pub;
//

//NITE
#include <XnOpenNI.h>
#include <XnCppWrapper.h>
#include <XnVHandPointContext.h>
#include <XnVSessionManager.h>
#include <XnVSwipeDetector.h>
#include <XnVSteadyDetector.h>
#include <XnVPushDetector.h>
#include <XnVWaveDetector.h>

// NITE objects
XnVSessionManager* g_pSessionManager = NULL;
XnVSwipeDetector*  g_pSwipeDetector;
XnVPushDetector*   g_pPushDetector;
XnVWaveDetector*   g_pWaveDetector;
XnVSteadyDetector* g_pSteadyDetector;

typedef enum
{
    IN_SESSION,
    NOT_IN_SESSION,
    QUICK_REFOCUS
} SessionState;
SessionState g_SessionState = NOT_IN_SESSION;

xn::Context g_Context;

void Publish_String(std::string str){
  std_msgs::String msg;
  msg.data = str;
  ROS_INFO((str + " published").c_str());
  swipe_pub.publish(msg);
}

// Swipe detector
void XN_CALLBACK_TYPE Swipe_SwipeUp(XnFloat fVelocity, XnFloat fAngle, void* cxt)
{
  Publish_String("Up");
}

void XN_CALLBACK_TYPE Swipe_SwipeDown(XnFloat fVelocity, XnFloat fAngle, void* cxt)
{
  Publish_String("Down");
}

void XN_CALLBACK_TYPE Swipe_SwipeLeft(XnFloat fVelocity, XnFloat fAngle, void* cxt)
{
  Publish_String("Left");
}

void XN_CALLBACK_TYPE Swipe_SwipeRight(XnFloat fVelocity, XnFloat fAngle, void* cxt)
{
  Publish_String("Right");
}

// Steady detector
void XN_CALLBACK_TYPE Steady_OnSteady(XnUInt32 nId, XnFloat fVelocity, void* cxt)
{
  Publish_String("Standby");
}

// Push detector
void XN_CALLBACK_TYPE Push_OnPush(XnFloat fVelocity, XnFloat fAngle, void* cxt)
{
  Publish_String("Push");
}

void CleanupExit()
{
  if (NULL != g_pSessionManager) {
    delete g_pSessionManager;
    g_pSessionManager = NULL;
  }

  delete g_pSwipeDetector;
  delete g_pSteadyDetector;

  g_Context.Shutdown();

  exit (1);
}

void XN_CALLBACK_TYPE SessionStart(const XnPoint3D& pFocus, void* UserCxt)
{
  ROS_INFO("hand gesture detection start");
  g_SessionState = IN_SESSION;
}

void XN_CALLBACK_TYPE SessionEnd(void* UserCxt)
{
  ROS_INFO("hand gesture detection end");
  g_SessionState = NOT_IN_SESSION;
}

void XN_CALLBACK_TYPE NoHands (void *UserCxt)
{
  printf ("Quick refocus\n");
  g_SessionState = QUICK_REFOCUS;
}

#define CHECK_RC(rc, what)                                      \
  if (rc != XN_STATUS_OK)                                       \
    {                                                           \
      printf("%s failed: %s\n", what, xnGetStatusString(rc));   \
      return rc;                                                \
    }

#define CHECK_ERRORS(rc, errors, what)          \
  if (rc == XN_STATUS_NO_NODE_PRESENT)          \
    {                                           \
      XnChar strError[1024];                    \
      errors.ToString(strError, 1024);          \
      printf("%s\n", strError);                 \
      return (rc);                              \
    }

int main(int argc, char **argv)
{
  ros::init (argc, argv, "openni_swipe");
  ros::NodeHandle nh;

  swipe_pub = nh.advertise<std_msgs::String>("swipe",10);
  swipe_status_pub = nh.advertise<std_msgs::UInt8>("swipe/status",10);

  XnStatus rc = XN_STATUS_OK;
  xn::EnumerationErrors errors;
  std::string configfile = ros::package::getPath ("openni_swipe") + "/openni_swipe.xml";

  rc = g_Context.InitFromXmlFile(configfile.c_str());
  CHECK_ERRORS(rc, errors, "InitFromXmlFile");
  CHECK_RC(rc, "InitFromXml");

  // Create and initialize point tracker
  g_pSessionManager = new XnVSessionManager;
  rc = g_pSessionManager->Initialize(&g_Context, "Wave", "RaiseHand");
  if (rc != XN_STATUS_OK)
	{
      printf("Couldn't initialize the Session Manager: %s\n", xnGetStatusString(rc));
      delete g_pSessionManager;
      return rc;
	}
  g_pSessionManager->RegisterSession(NULL, &SessionStart, &SessionEnd);

  g_pSwipeDetector = new XnVSwipeDetector;
  g_pSteadyDetector = new XnVSteadyDetector;
  g_pPushDetector = new XnVPushDetector;
  g_pWaveDetector = new XnVWaveDetector;

  // Swipe
  g_pSwipeDetector->RegisterSwipeUp(NULL, &Swipe_SwipeUp);
  g_pSwipeDetector->RegisterSwipeDown(NULL, &Swipe_SwipeDown);
  g_pSwipeDetector->RegisterSwipeLeft(NULL, &Swipe_SwipeLeft);
  g_pSwipeDetector->RegisterSwipeRight(NULL, &Swipe_SwipeRight);
  // Steady
  g_pSteadyDetector->RegisterSteady(NULL, &Steady_OnSteady);
  // Push
  g_pPushDetector->RegisterPush(NULL, &Push_OnPush);

  // Connect flow manager to the point tracker
  g_pSessionManager->AddListener(g_pSwipeDetector);
  g_pSessionManager->AddListener(g_pSteadyDetector);
  g_pSessionManager->AddListener(g_pPushDetector);

  g_Context.StartGeneratingAll();
  ros::Rate r (30);
  while (ros::ok ()){
    // Read next available data
    //g_Context.WaitOneUpdateAll(g_DepthGenerator);
    g_Context.WaitAndUpdateAll();

    // Process the data
    g_pSessionManager->Update(&g_Context);

    switch (g_SessionState)
    {
    case IN_SESSION:
	break;
    case NOT_IN_SESSION:
	ROS_INFO ("Perform wave gestures to track hand");
	break;
    case QUICK_REFOCUS:
	ROS_INFO ("Raise your hand for it to be identified");
	break;
    }
    std_msgs::UInt8 msg;
    msg.data = g_SessionState;
    swipe_status_pub.publish(msg);

    // ros sleep
    r.sleep();
  }
  CleanupExit();
}

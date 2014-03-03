// openni_pointer.cpp
#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/UInt8.h>
#include <tf/transform_broadcaster.h>

#include <XnOpenNI.h>
#include <XnCppWrapper.h>
#include <XnHash.h>
#include <XnLog.h>

// Header for NITE
#include <XnVNite.h>
#include <XnVPointControl.h>
using std::string;
xn::Context g_Context;
xn::DepthGenerator g_DepthGenerator;
xn::HandsGenerator g_HandsGenerator;

// NITE objects
XnVSessionManager *g_pSessionManager;
XnVFlowRouter *g_pFlowRouter;

//
ros::Publisher hand_point_pub_;
ros::Publisher hand_point_status_pub_;

typedef enum
{
    IN_SESSION,
    NOT_IN_SESSION,
    QUICK_REFOCUS
} SessionState;

#define CHECK_RC(nRetVal, what)                                         \
  if (nRetVal != XN_STATUS_OK){                                         \
    printf ("%s failed: %s\n", what, xnGetStatusString (nRetVal));      \
    return nRetVal;                                                     \
  }

SessionState g_SessionState = NOT_IN_SESSION;

// Callback for when the focus is in progress
void XN_CALLBACK_TYPE FocusProgress (const XnChar * strFocus, const XnPoint3D & ptPosition, XnFloat fProgress,
                                     void *UserCxt)
{
    //printf("Focus progress: %s @(%f,%f,%f): %f\n", strFocus, ptPosition.X, ptPosition.Y, ptPosition.Z, fProgress);
}

// callback for session start
void XN_CALLBACK_TYPE SessionStarting (const XnPoint3D & ptPosition, void *UserCxt)
{
    printf ("Session start: (%f,%f,%f)\n", ptPosition.X, ptPosition.Y, ptPosition.Z);
    g_SessionState = IN_SESSION;
}

// Callback for session end
void XN_CALLBACK_TYPE SessionEnding (void *UserCxt)
{
    printf ("Session end\n");
    g_SessionState = NOT_IN_SESSION;
}

void XN_CALLBACK_TYPE NoHands (void *UserCxt)
{
    printf ("Quick refocus\n");
    g_SessionState = QUICK_REFOCUS;
}

class XnVPointPublisher : public XnVPointControl
{
public:
    XnVPointPublisher(xn::DepthGenerator depthGenerator): XnVPointControl("XnVPointPublisher"), m_DepthGenerator(depthGenerator)
    {
    }

    virtual ~XnVPointPublisher()
    {
        std::map<XnUInt32, std::vector<XnPoint3D> >::iterator iter;
        for (iter = m_Position.begin(); iter != m_Position.end(); ++iter)
        {
            iter->second.clear();
        }
        m_Position.clear();
    }

    /**
     * Handle creation of a new point
     */
    void OnPointCreate(const XnVHandPointContext* cxt)
    {
        printf("** %d\n", cxt->nID);
        m_Position[cxt->nID].resize(2);
        OnPointUpdate(cxt);
    }

    // Handle destruction of an existing hand
    void OnPointDestroy(XnUInt32 nID)
    {
        // No need for the history buffer
        m_Position.erase(nID);
    }
    /**
     * Handle new position of an existing point
     */
    void OnPointUpdate(const XnVHandPointContext* cxt)
    {
        // positions are kept in projective coordinates, since they are only used for drawing
        XnPoint3D ptPosition(cxt->ptPosition);
        XnPoint3D ptProjective(cxt->ptPosition);
        m_DepthGenerator.ConvertRealWorldToProjective(1, &ptProjective, &ptProjective);
        printf ("%d (%f,%f,%f) ", cxt->nID, cxt->ptPosition.X, cxt->ptPosition.Y, cxt->ptPosition.Z);
        printf (" -> (%f,%f,%f)\n", ptProjective.X, ptProjective.Y, ptProjective.Z);

        // Add new position to the history buffer
        m_Position[cxt->nID][0]=ptPosition;
        m_Position[cxt->nID][1]=ptProjective;
    }


    /**
     * Handle a new message.
     * Calls other callbacks for each point, then draw the depth map (if needed) and the points
     */
    void Update(XnVMessage* pMessage) {
        // PointControl's Update calls all callbacks for each hand
        XnVPointControl::Update(pMessage);

        string frame_id("openni_depth_frame");
        string child_frame_id("hand_position");

        // get first hand
        if ( m_Position.size() > 0 ) {
            XnUInt32 Id = m_Position.begin()->first;
            XnPoint3D Position = m_Position.begin()->second[0];
            XnPoint3D Projection = m_Position.begin()->second[1];

            static tf::TransformBroadcaster br;
            tf::Transform transform;
            transform.setOrigin(tf::Vector3(Position.X/1000.0, Position.Y/1000.0, Position.Z/1000.0));
            transform.setRotation(tf::Quaternion(0,0,0,1));
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), frame_id, child_frame_id));

            geometry_msgs::PointStamped msg;
            msg.point.x = Projection.X;
            msg.point.y = Projection.Y;
            msg.header.stamp = ros::Time::now();
            hand_point_pub_.publish(msg);
        }

        std::map<XnUInt32, std::vector<XnPoint3D> >::const_iterator PointIterator;
        // Go over each existing hand
        for (PointIterator = m_Position.begin();
             PointIterator != m_Position.end();
             ++PointIterator)
        {
            XnUInt32 Id = PointIterator->first;
            string child_frame_id("hand_position");
            child_frame_id = child_frame_id + ((char)('0'+Id));

            // Go over all previous positions of current hand
            XnPoint3D Position = PointIterator->second[0];
            XnPoint3D Projection = PointIterator->second[1];

            static tf::TransformBroadcaster br;
            tf::Transform transform;
            transform.setOrigin(tf::Vector3(Position.X/1000.0, Position.Y/1000.0, Position.Z/1000.0));
            transform.setRotation(tf::Quaternion(0,0,0,1));
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), frame_id, child_frame_id));
        }
    }

protected:
    // Source of the depth map
    xn::DepthGenerator m_DepthGenerator;
    // position per hand
    std::map<XnUInt32, std::vector<XnPoint3D> > m_Position;
};

// the publisher
XnVPointPublisher* g_pPublisher;

int main (int argc, char **argv)
{
    ros::init (argc, argv, "openni_pointer");
    ros::NodeHandle nh;

    hand_point_pub_ = nh.advertise<geometry_msgs::PointStamped>("hand_position",10);
    hand_point_status_pub_ = nh.advertise<std_msgs::UInt8>("hand_position/status",10);

    // Initialize OpenNI
    string configFilename = ros::package::getPath ("openni_pointer") + "/openni_pointer.xml";
    XnStatus nRetVal = g_Context.InitFromXmlFile (configFilename.c_str ());
    CHECK_RC (nRetVal, "InitFromXml");
    nRetVal = g_Context.FindExistingNode (XN_NODE_TYPE_DEPTH, g_DepthGenerator);
    CHECK_RC (nRetVal, "Find depth generator");
    nRetVal = g_Context.FindExistingNode (XN_NODE_TYPE_HANDS, g_HandsGenerator);
    CHECK_RC (nRetVal, "Find hands generator");

    // Create Nite objects
    g_pSessionManager = new XnVSessionManager;
    nRetVal = g_pSessionManager->Initialize (&g_Context, "Click,Wave", "RaiseHand");
    CHECK_RC (nRetVal, "SessionManager::Initialize");
    g_pSessionManager->RegisterSession (NULL, SessionStarting, SessionEnding, FocusProgress);

    g_pPublisher = new XnVPointPublisher(g_DepthGenerator);
    g_pFlowRouter = new XnVFlowRouter;
    g_pFlowRouter->SetActive(g_pPublisher);

    //g_pFlowRouter->SetActive(g_pDrawer);
    g_pSessionManager->AddListener (g_pFlowRouter);

    g_pPublisher->RegisterNoPoints(NULL, NoHands);

    // Initialization done. Start generating
    nRetVal = g_Context.StartGeneratingAll ();
    CHECK_RC (nRetVal, "StartGenerating");
    ros::Rate r (30);
    while (ros::ok ())
    {
        g_Context.WaitAndUpdateAll ();

        // Update NITE tree
        g_pSessionManager->Update (&g_Context);
        switch (g_SessionState)

        {
        case IN_SESSION:
            ROS_INFO ("Tracking hands");
            break;
        case NOT_IN_SESSION:
            ROS_INFO ("Perform click or wave gestures to track hand");
            break;
        case QUICK_REFOCUS:
            ROS_INFO ("Raise your hand for it to be identified, or perform click or wave gestures");
            break;
        }
	std_msgs::UInt8 msg;
	msg.data = g_SessionState;
	hand_point_status_pub_.publish(msg);

        //publishTransforms();
        r.sleep ();
    }
    g_Context.Shutdown ();
    return 0;
}

#!/usr/bin/env python                                                                                              
import imp  ## for rosbuild                                                                                       
try:
    imp.find_module('rostwitter')
except:
    import roslib; roslib.load_manifest('rostwitter')

import rospy
import yaml,sys
import re, os
import twitter

from std_msgs.msg import String

global Api, CKEY, CSECRET, AKEY, ASECRET

def tweet(dat):
    global Api
    message = dat.data
    rospy.loginfo(rospy.get_name() + " sending %s", message)
    # search word start from / and end with {.jpeg,.jpg,.png,.gif}                                                 
    m = re.search('/\S+\.(jpeg|jpg|png|gif)', message)
    if m :
        filename = m.group(0)
        message = re.sub(filename,"",message)
        if os.path.exists(filename):
            ##rospy.logdebug(rospy.get_name() + " tweet %s with file %s", message, filename)                       
            Api.PostMedia(message, filename)
            return

    ## seg faults if message is longer than 140 byte ???                           Api.PostUpdate(message[0:140])
    ##rospy.logdebug(rospy.get_name() + " tweet %s", sub_msg)                                                      
    return

def load_oauth_settings():
    global CKEY, CSECRET, AKEY, ASECRET
    account_info = rospy.get_param('account_info', '/var/lib/robot/account.yaml')

    try:
        key = yaml.load(open(account_info))
        CKEY = key['CKEY']
        CSECRET = key['CSECRET']
        AKEY = key['AKEY']
        ASECRET = key['ASECRET']
    except IOError as e:
        rospy.logerr('"/var/lib/robot/%s" not found'%account_info)
        rospy.logerr("$ rosrun rostwitter get_access_token.py")
        rospy.logerr("cat /var/lib/robot/%s <<EOF"%account_info)
        rospy.logerr("CKEY: xxx")
        rospy.logerr("CSECRET: xxx")
        rospy.logerr("AKEY: xxx")
        rospy.logerr("ASECRET: xxx")
        rospy.logerr("EOF")
        sys.exit(-1)

if __name__ == '__main__':
    global Api
    rospy.init_node('rostwitter', anonymous=True)
    load_oauth_settings()
    Api = twitter.Api(consumer_key=CKEY,
                      consumer_secret=CSECRET,
                      access_token_key=AKEY,
                      access_token_secret=ASECRET)
    rospy.Subscriber("tweet", String, tweet)
    rospy.spin()





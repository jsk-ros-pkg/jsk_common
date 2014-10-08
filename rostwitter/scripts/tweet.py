#!/usr/bin/env python                                                                                              
import imp  ## for rosbuild                                                                                       
try:
    imp.find_module('rostwitter')
    # # append oauth package path
    # import rospkg
    # rp = rospkg.RosPack()
    # rostwitter_path = rp.get_path('rostwitter')
    # rostwitter_path
    # import sys, os
    # sys.path.append(os.path.join(rostwitter_path, "lib", "python2.7", 
    #                              "dist-packages"))
except:
    import roslib; roslib.load_manifest('rostwitter')

import rospy
import yaml,sys
import re, os
from io import BytesIO
from StringIO import StringIO

from std_msgs.msg import String

global Api, CKEY, CSECRET, AKEY, ASECRET

import requests
import oauth2 as oauth
import base64
import json as simplejson

# https://raw.githubusercontent.com/bear/python-twitter/v1.1/twitter.py

class twitter(object):
    def __init__(self,
                 consumer_key=None,
                 consumer_secret=None,
                 access_token_key=None,
                 access_token_secret=None):
        self._consumer_key        = consumer_key
        self._consumer_secret     = consumer_secret
        self._access_token_key    = access_token_key
        self._access_token_secret = access_token_secret

        self._signature_method_plaintext = oauth.SignatureMethod_PLAINTEXT()
        self._signature_method_hmac_sha1 = oauth.SignatureMethod_HMAC_SHA1()

        self._oauth_token    = oauth.Token(key=access_token_key, secret=access_token_secret)
        self._oauth_consumer = oauth.Consumer(key=consumer_key, secret=consumer_secret)

    def _RequestUrl(self, url, verb, data=None):
        req = oauth.Request.from_consumer_and_token(self._oauth_consumer,
                                                    token=self._oauth_token,
                                                    http_method=verb,
                                                    http_url=url)
        req.sign_request(self._signature_method_hmac_sha1, self._oauth_consumer, self._oauth_token)

        headers = req.to_header()

        if verb == 'POST':
            return requests.post(
                url,
                headers=headers,
                files=data
                )
        if verb == 'GET':
            url = self._BuildUrl(url, extra_params=data)
            return requests.get(
                url,
                headers=headers
                )
        return 0  # if not a POST or GET request

    def PostUpdate(self, status):
        url = 'https://api.twitter.com/1.1/statuses/update.json'

        data = {'status': StringIO(status)}
        json = self._RequestUrl(url, 'POST', data=data)
        data = simplejson.loads(json.content)
        if 'error' in data:
            raise Exception(data)
        return data

    def PostMedia(self, status, media):
        url = 'https://api.twitter.com/1.1/statuses/update_with_media.json'

        data = {'status': StringIO(status)}
        data['media[]'] = open(str(media), 'rb')
        json = self._RequestUrl(url, 'POST', data=data)
        data = simplejson.loads(json.content)
        if 'errors' in data:
            raise Exception(data)
        return data

def tweet(dat):
    global Api
    message = dat.data
    rospy.loginfo(rospy.get_name() + " sending %s", message)
    # search word start from / and end with {.jpeg,.jpg,.png,.gif}                                                 
    m = re.search('/\S+\.(jpeg|jpg|png|gif)', message)
    if m:
        filename = m.group(0)
        message = re.sub(filename,"",message)
        if os.path.exists(filename):
            ##rospy.logdebug(rospy.get_name() + " tweet %s with file %s", message, filename)                       
            ret = Api.PostMedia(message, filename)
            #ret = Api.PostUpdate(message)
    else:
        ret = Api.PostUpdate(message[0:140])
    ## seg faults if message is longer than 140 byte ???                           
        
    rospy.loginfo(rospy.get_name() + " receiving %s", ret)                                                      
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
        rospy.logerr('"%s" not found'%account_info)
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
    Api = twitter(consumer_key=CKEY,
                  consumer_secret=CSECRET,
                  access_token_key=AKEY,
                  access_token_secret=ASECRET)
    rospy.Subscriber("tweet", String, tweet)
    rospy.spin()





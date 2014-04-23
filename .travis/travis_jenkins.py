#!/usr/bin/env python

import jenkins
import urllib
import urllib2
import json
import time

from os import environ as env

print "TRAVIS_BRANCH",  env['TRAVIS_BRANCH']
print "TRAVIS_BUILD_DIR",env['TRAVIS_BUILD_DIR']
print "TRAVIS_BUILD_ID",env['TRAVIS_BUILD_ID']
print "TRAVIS_BUILD_NUMBER",env['TRAVIS_BUILD_NUMBER']
print "TRAVIS_COMMIT",env['TRAVIS_COMMIT']
print "TRAVIS_COMMIT_RANGE",env['TRAVIS_COMMIT_RANGE']
print "TRAVIS_JOB_ID",env['TRAVIS_JOB_ID']
print "TRAVIS_JOB_NUMBER",env['TRAVIS_JOB_NUMBER']
print "TRAVIS_PULL_REQUEST",env['TRAVIS_PULL_REQUEST']
print "TRAVIS_SECURE_ENV_VARS",env['TRAVIS_SECURE_ENV_VARS']
print "TRAVIS_REPO_SLUG",env['TRAVIS_REPO_SLUG']

#u = urllib2.Request(j.build_job_url('trusty-travis') , "TRAVIS_BRANCH=%s&TRAVIS_COMMIT=%s&TRAVIS_REPO_SLUG=%s"%(env['TRAVIS_BRANCH'],env['TRAVIS_COMMIT'],env['TRAVIS_REPO_SLUG']))
#print "TRAVIS_BRANCH=%s&TRAVIS_COMMIT=%s&TRAVIS_REPO_SLUG=%s&ROS_DISTRO=%s&ROSWS=%s&BUILDER=%s&USE_DEB=%s"%(env['TRAVIS_BRANCH'],env['TRAVIS_COMMIT'],env['TRAVIS_REPO_SLUG'],env['ROS_DISTRO'],env['ROSWS'],env['BUILDER'],env['USE_DEB'])
#u = j.jenkins_open(urllib2.Request('http://jenkins.jsk.imi.i.u-tokyo.ac.jp:8080/job/trusty-travis/buildWithParameters',"TRAVIS_BRANCH=%s&TRAVIS_COMMIT=%s&TRAVIS_REPO_SLUG=%s&ROS_DISTRO=%s&ROSWS=%s&BUILDER=%s&USE_DEB=%s"%(env['TRAVIS_BRANCH'],env['TRAVIS_COMMIT'],env['TRAVIS_REPO_SLUG'],env['ROS_DISTRO'],env['ROSWS'],env['BUILDER'],env['USE_DEB'])))
#print u

# this over rides existing methods
BUILD_INFO = 'job/%(name)s/%(number)d/api/json?depth=0'
class Jenkins2(jenkins.Jenkins):

    def build_job(self, name, parameters=None, token=None):
        if not self.job_exists(name):
            raise jenkins.JenkinsException('no such job[%s]'%(name))
        if token:
            parameters['token'] = token
        print self.build_job_url(name), urllib.urlencode(parameters)
        return self.jenkins_open(urllib2.Request(self.build_job_url(name, {'foo':'bar'}), urllib.urlencode(parameters)))

    def get_build_info(self, name, number):
        '''
        Get build information dictionary.

        :param name: Job name, ``str``
        :param name: Build number, ``int``
        :returns: dictionary of build information, ``dict``

        Example::

            >>> next_build_number = j.get_job_info('build_name')['next_build_number']
            >>> output = j.build_job('build_'+kwargs['vcs_server_type'], params)
            >>> sleep(10)
            >>> build_info = j.get_build_info('build_name', next_build_number)
            >>> print(build_info)
            {u'building': False, u'changeSet': {u'items': [{u'date': u'2011-12-19T18:01:52.540557Z', u'msg': u'test', u'revision': 66, u'user': u'unknown', u'paths': [{u'editType': u'edit', u'file': u'/branches/demo/index.html'}]}], u'kind': u'svn', u'revisions': [{u'module': u'http://eaas-svn01.i3.level3.com/eaas', u'revision': 66}]}, u'builtOn': u'', u'description': None, u'artifacts': [{u'relativePath': u'dist/eaas-87-2011-12-19_18-01-57.war', u'displayPath': u'eaas-87-2011-12-19_18-01-57.war', u'fileName': u'eaas-87-2011-12-19_18-01-57.war'}, {u'relativePath': u'dist/eaas-87-2011-12-19_18-01-57.war.zip', u'displayPath': u'eaas-87-2011-12-19_18-01-57.war.zip', u'fileName': u'eaas-87-2011-12-19_18-01-57.war.zip'}], u'timestamp': 1324317717000, u'number': 87, u'actions': [{u'parameters': [{u'name': u'SERVICE_NAME', u'value': u'eaas'}, {u'name': u'PROJECT_NAME', u'value': u'demo'}]}, {u'causes': [{u'userName': u'anonymous', u'shortDescription': u'Started by user anonymous'}]}, {}, {}, {}], u'id': u'2011-12-19_18-01-57', u'keepLog': False, u'url': u'http://eaas-jenkins01.i3.level3.com:9080/job/build_war/87/', u'culprits': [{u'absoluteUrl': u'http://eaas-jenkins01.i3.level3.com:9080/user/unknown', u'fullName': u'unknown'}], u'result': u'SUCCESS', u'duration': 8826, u'fullDisplayName': u'build_war #87'}
        '''
        try:
            response = self.jenkins_open(urllib2.Request(
                self.server + BUILD_INFO % locals()))
            if response:
                return json.loads(response)
            else:
                raise jenkins.JenkinsException('job[%s] number[%d] does not exist'
                                       % (name, number))
        except urllib2.HTTPError:
            raise jenkins.JenkinsException('job[%s] number[%d] does not exist'
                                   % (name, number))
        except ValueError:
            raise jenkins.JenkinsException(
                'Could not parse JSON info for job[%s] number[%d]'
                % (name, number)
            )

## start from here
j = Jenkins2('http://jenkins.jsk.imi.i.u-tokyo.ac.jp:8080/')
next_build_number = j.get_job_info('trusty-travis')['nextBuildNumber']
j.build_job('trusty-travis', {'TRAVIS_BRANCH': env['TRAVIS_BRANCH'], 'TRAVIS_COMMIT': env['TRAVIS_COMMIT'], 'TRAVIS_REPO_SLUG': env['TRAVIS_REPO_SLUG'], 'ROS_DISTRO': env['ROS_DISTRO'], 'ROSWS': env['ROSWS'], 'BUILDER': env['BUILDER'], 'USE_DEB':env['USE_DEB']})

building = True
while building == True :
    time.sleep(10)
    info = j.get_build_info('trusty-travis',next_build_number)
    building = info['building']
    result = info['result']
    print info['url'], "building..",building, "result...",result

if result == "SUCCESS" :
    exit(0)
else:
    exit(1)



import os.path as osp
import signal
import subprocess

from nodelet.srv import NodeletList
import psutil
import rosgraph
import rosnode
import rospy


try:
    from xmlrpc.client import ServerProxy
except ImportError:
    from xmlrpclib import ServerProxy

from jsk_tools.process_utils import search_pid_from_process_cmd


def kill_nodelets(manager_name, manager_namespace, nodelet_list=None,
                  child_process=None, wait_duration=3.0):
    """Kill nodelets and restart nodelet manager.

    To respawn nodelets, we assumes respawn=true is set.
    """
    if nodelet_list is None:
        nodelet_list = rospy.ServiceProxy(
            osp.join(manager_namespace, manager_name, 'list'),
            NodeletList)().nodelets

    if child_process is None:
        pid = search_pid_from_process_cmd('__name:=' + manager_name)
        if pid is not None:
            rospy.loginfo('Found nodelet manager process ({})'.format(pid))
            process = psutil.Process(pid)
            if process.is_running():
                process.send_signal(signal.SIGINT)
            else:
                raise Exception('manager process is not running')
            # wait until manager is dead
            start = rospy.Time.now()
            while psutil.pid_exists(pid) and (
                    rospy.Time.now() - start < rospy.Duration(wait_duration)):
                rospy.sleep(0.1)
            if process.is_running():
                process.send_signal(signal.SIGKILL)
        else:
            rospy.loginfo('Not found nodelet manager process.')
    else:
        # manager process is launching by respawner as its child process
        try:
            rospy.loginfo('kill child process {}').format(child_process)
            child_process.terminate()
            child_process.wait()
        except Exception as e:
            rospy.logwarn('{}'.format(str(e)))

    # restart nodelet manager
    respawn_manager_cmd = ["rosrun", "nodelet",
                           "nodelet", "manager",
                           "__name:=" + manager_name,
                           "__ns:=" + manager_namespace]
    child_process = subprocess.Popen(respawn_manager_cmd)
    name = manager_name + '/respawner'
    # Kill nodelet. This assumes respawn=true is set.
    master = rosgraph.Master(name)
    for nodelet in nodelet_list:
        try:
            pid = ServerProxy(rosnode.get_api_uri(
                master, nodelet, skip_cache=True)).getPid(name)[2]
        except TypeError:
            rospy.logwarn("Failed killing nodelet: %s", nodelet)
            continue
        rospy.loginfo("Killing nodelet: %s(%d)", nodelet, pid)
        process = psutil.Process(pid)
        process.send_signal(signal.SIGKILL)
    return child_process

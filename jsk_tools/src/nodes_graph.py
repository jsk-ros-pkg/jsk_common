#!/usr/bin/env python

import roslib; roslib.load_manifest('jsk_tools')
roslib.load_manifest('rosnode')
roslib.load_manifest('rxgraph')

import os
import subprocess

import socket
import xmlrpclib

import rxgraph
import rosnode


class NodeGraph:
    def __init__(self, username=None):
        # overwrite the function for generating dotcode for nodes
        rxgraph.dotcode._generate_node_dotcode = self.generate_node_dotcode

        self.user = username
        pkglist = (roslib.rospack.rospackexec(['list'])).split('\n')
        self.pkginfo_local = [pkg.split(' ') for pkg in pkglist]
        self.pkginfo = {}    # hostname -> {pkgname -> pkgpath}
        self.pkginfo[os.environ['ROS_IP']] = self.pkginfo_local
        self.pkginfo[os.environ['ROS_HOSTNAME']] = self.pkginfo_local
        self.nodepkgmap = {} # nodename -> pkgname
        self.update_nodepkgmap()

    def add_pkginfo_remote(self, hostname):
        if hostname in self.pkginfo.keys():
            return None
        ssh_target = self.user + '@' + hostname if self.user else hostname
        cmd = ['ssh', ssh_target, 'rospack', 'list']
        pkglist = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE).communicate()[0]
        if pkglist == '':
            # TODO: get rospack list from remote host (difficult?)
            self.pkginfo[hostname] = self.pkginfo_local
            return None
        pkglist = pkglist.split('\n')
        self.pkginfo[hostname] = [pkg.split(' ') for pkg in pkglist]

    # get pkgname from nodename
    def getnodepkg1(self, node_name):
        socket.setdefaulttimeout(1.0)
        master = roslib.scriptutil.get_master()
        node_api = rosnode.get_api_uri(master, node_name)
        host = node_api.split(':')[1].lstrip('/')
        node = xmlrpclib.ServerProxy(node_api)
        try:
            pid = rosnode._succeed(node.getPid(rosnode.ID))
        except socket.error:
            print("Communication with [%s=%s] failed!"%(node_name,node_api))
            return ['']

        cmd = ['ps','ww','--pid',str(pid)]
        if os.environ['ROS_IP'] != host and os.environ['ROS_HOSTNAME'] != host:
            ssh_target = self.user + '@' + host if self.user else hostname
            cmd = ['ssh', ssh_target] + cmd
            self.add_pkginfo_remote(host)

        ps = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE).communicate()[0]
        res = ps[ps.find('/'):]
        node_args = res.split(' ')
        parsed = node_args[0].split('/')
        pkgnames = [pkg[0] for pkg in self.pkginfo[host]]
        result = [dirname for dirname in parsed if (dirname in pkgnames)]
        if len(result) == 0:
            result = ['']
        return result

    def update_nodepkgmap(self):
        master = roslib.scriptutil.get_master()
        nodes = rosnode.get_node_names()
        for n in nodes:
            self.nodepkgmap[n] = self.getnodepkg1(n)[0]

    def run(self):
        try:
            import wx
            app = wx.App()
            frame = rxgraph.impl.init_frame()
            updater = rxgraph.impl.init_updater(frame, output_file='/tmp/graph.dot')
            updater.start()
            frame.Show()
            app.MainLoop()
        except KeyboardInterrupt:
            pass
        finally:
            rxgraph.impl.set_shutdown(True)

    def get_package_image_url(self, package):
        path = None
        for p in self.pkginfo_local:
            if p[0] == package:
                path = p[1]
        if not path:
            return None
        buff = []
        for root, dirs, files in os.walk(path):
            for fname in files:
                if fname[-3:] in ['jpg','png','gif']:
                    buff += ['%s/%s' % (root,fname)]
        imgurl = buff[0] # how to get the best image ?
        return imgurl

    # generate dot code
    def generate_node_dotcode(self, node, g, quiet):
        safename = rxgraph.dotcode.safe_dotcode_name(node)
        pkgname = ''
        pkgpath = ''
        if node in self.nodepkgmap.keys():
            pkgname = self.nodepkgmap[node]
            for p in self.pkginfo_local:
                if p[0] == pkgname:
                    pkgpath = p[1]
                    break
        # set node color and shape
        style = 'color="black", shape="ellipse"'
        label = '"%s"' % node
        if 'jsk-ros-pkg' in pkgpath:
            style = 'color="red", shape="ellipse", style="filled", fillcolor="red"'
            imgurl = None # self.get_package_image_url(pkgname)
            if imgurl:
                style = style.replace('ellipse','plaintext')
                label = '<<table><tr><td><img src="%s"/></td></tr></table>>' % imgurl
        elif 'rtm' in pkgpath:
            style = 'color="orange", shape="ellipse", style="filled", fillcolor="orange"'
        return '  %s [%s, label=%s, URL="node:%s"];' % (safename, style, label, node)


if __name__=='__main__':
    obj = NodeGraph()
    # print(obj.nodepkgmap)
    obj.run()
    subprocess.call(['dot', '-Tpng', '-o', '/tmp/graph.png', '/tmp/graph.dot'])

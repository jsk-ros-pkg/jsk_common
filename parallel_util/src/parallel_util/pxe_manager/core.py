#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Copyright (C) 2011 Ryohei Ueda
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#     http://www.apache.org/licenses/LICENSE-2.0
# 
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import xml.sax.saxutils
import logging
from .template import *
import random
from socket import gethostbyaddr
import BaseHTTPServer
import cgi
import sqlite3
import socket
import binascii
import os
import ping                     # easy_install ping
from subprocess import check_call
from string import Template

import parallel_util

LOGGER_FILE_FORMAT = '[%(asctime)s] %(name)s [%(levelname)s] %(message)s'
LOGGER_CONSOLE_FORMAT = '>> %(message)s'

def ping_host(hostname):
    ping_ret = ping.quiet_ping(hostname, timeout=0.1)
    return ping_ret[0] == 0     # verify return core is 0

def create_logger(log_file_name):
    logger = logging.getLogger("pxe")
    fh = logging.FileHandler(log_file_name)
    fh.setLevel(logging.INFO)
    ch = logging.StreamHandler()
    ch.setLevel(logging.INFO)
    fh.setFormatter(logging.Formatter(LOGGER_FILE_FORMAT))
    ch.setFormatter(logging.Formatter(LOGGER_CONSOLE_FORMAT))
    logger.addHandler(fh)
    logger.addHandler(ch)
    logger.setLevel(logging.INFO)
    return logger

def send_wol_magick_packet(macs, ipaddr, port):
    "http://www.emptypage.jp/gadgets/wol.html"
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    for mac in macs:
        for sep in ':-':
            if sep in mac:
                mac = ''.join([x.rjust(2, '0') for x in mac.split(sep)])
                break
        mac = mac.rjust(12, '0')
        p = '\xff' * 6 + binascii.unhexlify(mac) * 16
        s.sendto(p, (ipaddr, port))
    s.close()
    
def open_db(db):
    logger = logging.getLogger("pxe")
    logger.info("opening %s as DB" % (db))
    if os.path.exists(db):
        con = sqlite3.connect(db)
    else:
        con = sqlite3.connect(db)
        con.execute(DB_CREATE_TABLE_SQL)
    return con

def delete_host(con, host):
    template = Template(DEL_HOST_SQL)
    sql = template.substitute({"hostname": host})
    con.execute(sql)
    con.commit()

def add_host(con, host, ip, mac, root):
    template = Template(ADD_HOST_SQL)
    sql = template.substitute({"hostname": host,
                               "ip": ip,
                               "macaddress": mac,
                               "root": root})
    con.execute(sql)
    con.commit()

def all_hosts(con):
    sql_result = con.execute(ALL_HOSTS_SQL)
    result = {}
    for row in sql_result:
        result[row[0]] = {"ip": row[1], "macaddress": row[2], "root": row[3]}
    return result

def find_by_ip(con, ip):
    template = Template(FIND_IP_SQL)
    sql_str = template.substitute({"ip": ip})
    sql_result = con.execute(sql_str)
    for row in sql_result:
        return {"ip": row[1], "macaddress": row[2], "root": row[3]}
    return False

def find_by_hostname(con, hostname):
    template = Template(FIND_HOST_SQL)
    sql_str = template.substitute({"hostname": hostname})
    sql_result = con.execute(sql_str)
    for row in sql_result:
        return {"ip": row[1], "macaddress": row[2], "root": row[3]}
    raise "cannot find %s" % (hostname)

def delete_machine(hostname, db):
    logger = logging.getLogger("pxe")
    con = open_db(db)
    logger.info("deleteing a machine:: %s" % hostname)
    delete_host(con, hostname)
    con.close()
    
def add_machine(hostname, mac, ip, root, db):
    logger = logging.getLogger("pxe")
    con = open_db(db)
    logger.info("adding a machine:: hostname - %s, mac - %s, ip - %s, root - %s" % (hostname, mac, ip, root))
    add_host(con, hostname, ip, mac, root)
    con.close()
    
def generate_dhcp(db,
                  prefix_dhcp_file,
                  subnet,
                  netmask,
                  dhcp_range_start,
                  dhcp_range_stop,
                  broadcast,
                  dns_ip,
                  domain_name,
                  gateway,
                  pxe_server,
                  pxe_filename,
                  overwrite_dhcp,
                  dhcp_conf_file,
                  restart_dhcp):
    logger = logging.getLogger("pxe")
    logger.info("generating dhcpd.conf")
    con = open_db(db)
    machines = all_hosts(con)
    generated_string = []
    # generate host section
    for (hostname, ip_mac) in machines.items():
        ip = ip_mac["ip"]
        mac = ip_mac["macaddress"]
        template = Template(DHCP_HOST_TMPL)
        host_str = template.substitute({"hostname": hostname,
                                        "ip": ip,
                                        "mac": mac})
        generated_string.append(host_str)
    if prefix_dhcp_file:
        logger.info("adding %s as prefix" % (prefix_dhcp_file))
        f = open(prefix_dhcp_file)
        prefix_str = "".join(f.readlines())
    else:
        prefix_str = ""
    # generate dhcp subnet section
    dhcp_subnet_tmpl = Template(DHCP_SUBNET_TMPL)
    replace_dict = {"subnet": subnet,
                    "netmask": netmask,
                    "dhcp_range_start": dhcp_range_start,
                    "dhcp_range_stop": dhcp_range_stop,
                    "broadcast": broadcast,
                    "dns_ip": dns_ip,
                    "domain_name": domain_name,
                    "gateway": gateway,
                    "pxe_server": pxe_server,
                    "pxe_filename": pxe_filename,
                    "hosts": "\n".join(generated_string)}
    dhcp_subnet_str = dhcp_subnet_tmpl.substitute(replace_dict)
    if overwrite_dhcp:
        logger.info("overwriting %s" % (dhcp_conf_file))
        path = dhcp_conf_file
        f = open(path, "w")
        f.write(prefix_str + dhcp_subnet_str)
        f.close()
        if restart_dhcp:
            logger.info("restarting dhcp")
            check_call(["sudo", "/etc/init.d/dhcp3-server", "restart"])
    else:
        print(prefix_str + dhcp_subnet_str)

def print_machine_list(db):
    logger = logging.getLogger("pxe")
    con = open_db(db)
    machines = all_hosts(con)
    for hostname, ip_mac in machines.items():
        ip = ip_mac["ip"]
        mac = ip_mac["macaddress"]
        root = ip_mac["root"]
        print("""%s:
  ip: %s
  mac: %s
  root: %s
""" % (hostname, ip, mac, root))

def wake_on_lan(hostname, port, broadcast, db):
    logger = logging.getLogger("pxe")
    con = open_db(db)
    logger.info("wake on lan %s" % (hostname))
    mac = find_by_hostname(con, hostname)["macaddress"]
    send_wol_magick_packet([mac], broadcast, port)

def chroot_command(chroot_dir, *args):
    command = ["sudo", "chroot", chroot_dir]
    command.extend(*args)
    return check_call(command)
    
def generate_pxe_template_filesystem(template_dir):
    logger = logging.getLogger("pxe")
    logger.info("generating template filesyste")
    try:
        check_call(["sudo", "debootstrap", "lucid", template_dir])
    except:
        # remove template_dir
        logger.info("removing template dir")
        check_call(["sudo", "rm", "-rf", template_dir])
        raise


def copy_template_filesystem(template_dir, target_dir, apt_sources):
    logger = logging.getLogger("pxe")
    logger.info("copying filesystem")
    check_call(["sudo", "cp", "-ax", template_dir, target_dir])
    logger.info("copying etc/apt/sources.list")
    check_call(["sudo", "cp", apt_sources,
                os.path.join(target_dir, "etc", "apt", "sources.list")])
    return

class ChrootEnvironment():
    def __init__(self, target_dir):
        self.target_dir = target_dir
    def __enter__(self):
        target_dir = self.target_dir
        check_call(["sudo", "mount", "-o", "bind", "/dev/",
                    os.path.join(target_dir, "dev")])
        chroot_command(target_dir,
                       "mount -t proc none /proc".split())
        chroot_command(target_dir,
                       "mount -t sysfs none /sys".split())
        chroot_command(target_dir,
                       "mount -t devpts none /dev/pts".split())
    def __exit__(self, type, value, traceback):
        target_dir = self.target_dir
        chroot_command(target_dir,
                       "umount -lf /proc".split())
        chroot_command(target_dir,
                       "umount -lf /sys".split())
        chroot_command(target_dir,
                       "umount -lf /dev/pts".split())
        check_call(["sudo", "umount", "-lf", os.path.join(target_dir, "dev")])
 

def install_apt_packages(target_dir):
    env = ChrootEnvironment(target_dir)
    with env:
        logger = logging.getLogger("pxe")
        logger.info("installing base apt packages")
        chroot_command(target_dir, ["apt-get", "update"])
        chroot_command(target_dir, ["apt-get", "install", "wget"])
        chroot_command(target_dir, ["sh", "-c",
                                    "wget -q https://www.ubuntulinux.jp/ubuntu-ja-archive-keyring.gpg -O- | apt-key add -"])
        chroot_command(target_dir, ["sh", "-c",
                                    "wget -q https://www.ubuntulinux.jp/ubuntu-jp-ppa-keyring.gpg -O- | apt-key add -"])
        chroot_command(target_dir, ["sh", "-c",
                                    "wget https://www.ubuntulinux.jp/sources.list.d/lucid.list -O /etc/apt/sources.list.d/ubuntu-ja.list"])
        chroot_command(target_dir, ["apt-get", "update"])
        chroot_command(target_dir,
                       ["apt-get", "install", "--force-yes", "-y"] + APT_PACKAGES.split())
        logger.info("installing ros and openrave apt packages")
        chroot_command(target_dir, ["sh", "-c",
                                    "echo deb http://packages.ros.org/ros/ubuntu lucid main > /etc/apt/sources.list.d/ros-latest.list"])
        chroot_command(target_dir, ["sh", "-c",
                                    "wget http://packages.ros.org/ros.key -O - | apt-key add -"])
        chroot_command(target_dir, ["add-apt-repository", "ppa:openrave/release"])
        chroot_command(target_dir, ["apt-get", "update"])
        #chroot_command(target_dir, ["apt-get", "install", "--force-yes", "-y", "ros-diamondback-ros-base", "openrave"])

def setup_user(target_dir, user, passwd):
    env = ChrootEnvironment(target_dir)
    with env:
        logger = logging.getLogger("pxe")
        logger.info("adding pxe user")
        chroot_command(target_dir, ["sh", "-c",
                                    "id %s || useradd %s -g sudo -s /bin/bash" % (user,
                                                                                  user)])
        chroot_command(target_dir, ["sh", "-c",
                                    "echo %s:%s | chpasswd" % (user, passwd)])
        chroot_command(target_dir, ["rm", "-f", "/etc/hostname"])

def update_initram(target_dir):
    env = ChrootEnvironment(target_dir)
    with env:
        logger = logging.getLogger("pxe")
        logger.info("updating initram.img")
        chroot_command(target_dir, ["sh", "-c",
                                    "echo '%s' > /etc/initramfs-tools/initramfs.conf" % (INITRAMFS_CONF)])
        chroot_command(target_dir, ["sh", "-c",
                                    "echo '%s' > /etc/fstab" % (FSTAB)])
        chroot_command(target_dir, ["sh", "-c",
                                    "echo '%s' > /etc/initramfs-tools/modules" % (INITRAM_MODULES)])
        chroot_command(target_dir, ["sh", "-c",
                                    "mkinitramfs -o /boot/initrd.img-`uname -r` `ls /lib/modules`"])

def setup_pxe_dphys(target_dir):
    env = ChrootEnvironment(target_dir)
    with env:
        logger = logging.getLogger("pxe")
        logger.info("setup pxe_dphys")
        chroot_command(target_dir,
                       ["sh", "-c",
                        """cat > /usr/local/sbin/pxe-dphys-swapfile <<EOF
%s
EOF""" % (PXE_DPHYS_SWAPFILE)])
        chroot_command(target_dir,
                       ["sh", "-c",
                        """cat > /etc/init.d/pxe-dphys-swapfile <<EOF
%s
EOF""" % (PXE_DPHYS_SWAPFILE_INIT_D)])
        chroot_command(target_dir,
                       ["sh", "-c",
                        """cat > /etc/dphys-swapfile <<EOF
%s
EOF""" % (PXE_DPHYS_CONFIG)])
        chroot_command(target_dir,
                       ["chmod", "+x", "/usr/local/sbin/pxe-dphys-swapfile"])
        chroot_command(target_dir,
                       ["chmod", "+x", "/etc/init.d/pxe-dphys-swapfile"])
        chroot_command(target_dir,
                       ["update-rc.d", "pxe-dphys-swapfile", "defaults"])

        
def generate_pxe_filesystem(template_dir, target_dir, apt_sources,
                            user, passwd):
    logger = logging.getLogger("pxe")
    if not os.path.exists(template_dir):
        generate_pxe_template_filesystem(template_dir)
    if not os.path.exists(target_dir):
        copy_template_filesystem(template_dir, target_dir, apt_sources)
    check_call(["sudo", "cp", apt_sources,
                os.path.join(target_dir, "etc", "apt", "sources.list")])
    install_apt_packages(target_dir)
    setup_user(target_dir, user, passwd)
    setup_pxe_dphys(target_dir)
    update_initram(target_dir)

def generate_top_html(db, log):
    con = open_db(db)
    machines = all_hosts(con)
    host_strs = []
    hosts = sorted(machines.keys())
    for hostname in hosts:
        ip_mac = machines[hostname]
        ip = ip_mac["ip"]
        mac = ip_mac["macaddress"]
        root = ip_mac["root"]
        alivep = ping_host(ip)
        if alivep:
            template = Template(HTML_ALIVE_HOST_TMPL)
        else:
            template = Template(HTML_DEAD_HOST_TMPL)
        host_strs.append(template.substitute({"hostname": hostname,
                                              "ip": ip,
                                              "macaddress": mac,
                                              "root": root}))
    con.close()
    # generate log
    f = open(log)
    log_lines = f.readlines()[-1000:] # max 1000 lines
    log_strs = []
    for line in log_lines:
        log_strs.append("<p>" + xml.sax.saxutils.escape(line) + "</p>")
    log_strs.reverse()
    f.close()
    html_template = Template(HTML_TMPL)
    
    return html_template.substitute({"hosts": "\n".join(host_strs),
                                     "log": "\n".join(log_strs)})

def update_dhcp_from_web():
    if global_options.overwrite_dhcp:
        logger = logging.getLogger("pxe")
        generate_dhcp(global_options.db,
                      global_options.prefix_dhcp_file,
                      global_options.subnet,
                      global_options.netmask,
                      global_options.dhcp_range_start,
                      global_options.dhcp_range_stop,
                      global_options.broadcast,
                      global_options.dns_ip,
                      global_options.domain_name,
                      global_options.gateway,
                      global_options.pxe_server,
                      global_options.pxe_filename,
                      global_options.overwrite_dhcp,
                      global_options.dhcp_conf_file)
    if global_options.generate_pxe_config_files:
        generate_pxe_config_files(global_options.db,
                                  global_options.generate_pxe_config_files,
                                  global_options.tftp_dir)
    # restart dhcp server
    check_call(["sudo", "/etc/init.d/dhcp3-server", "restart"])

def generate_error_html(e, host, port):
    return ERROR_HTML_TMPL % ("http://" + host + ":" + str(port), e)
        
class WebHandler(BaseHTTPServer.BaseHTTPRequestHandler):
    def do_GET(s):
        s.send_response(200)
        s.send_header("Content-type", "text/html")
        s.end_headers()
        try:
            if s.path.startswith("/delete"): # --delete
                delete_host = cgi.parse_qs(s.path.split("?")[1]).keys()[0]
                delete_machine(delete_host, db_name)
                update_dhcp_from_web()
                html = SUCCESS_HTML_TMPL % ("http://" +
                                            global_options.web_hostname
                                            + ":" +
                                            str(global_options.web_port))
            elif s.path.startswith("/add"): # --add
                add_host_desc = cgi.parse_qs(s.path.split("?")[1])
                add_machine(add_host_desc["hostname"][0],
                            add_host_desc["macaddress"][0],
                            add_host_desc["ip"][0],
                            add_host_desc["root"][0],
                            db_name)
                update_dhcp_from_web()
                html = SUCCESS_HTML_TMPL % ("http://" +
                                            global_options.web_hostname
                                            + ":" +
                                            str(global_options.web_port))
            elif s.path.startswith("/auto_add"): # --auto-add
                add_host_desc = cgi.parse_qs(s.path.split("?")[1])
                root_dir = add_host_desc["root"][0]
                try:
                    global_options.auto_add = root_dir
                    auto_add_vm(global_options)
                finally:
                    global_options.auto_add = None
                update_dhcp_from_web()
                html = SUCCESS_HTML_TMPL % ("http://" +
                                            global_options.web_hostname
                                            + ":" +
                                            str(global_options.web_port))
            elif s.path.startswith("/boot"):
                logger = logging.getLogger("pxe")
                html = SUCCESS_HTML_TMPL % ("http://" +
                                            global_options.web_hostname
                                            + ":" +
                                            str(global_options.web_port))
                params = cgi.parse_qs(s.path.split("?")[1])
                vmname = params["vmname"][0]
                physc = params["physical"][0]
                logger.info("invoke %s @ %s" % (vmname, physc))
                try:
                    global_options.generate_virtualbox_image = vmname
                    global_options.refer_physical_machine = physc
                    generate_virtualbox_image(global_options.db,
                                              global_options.generate_virtualbox_image,
                                              global_options.virtualbox_path,
                                              global_options.refer_physical_machine,
                                              global_options.virtualbox_cpunum,
                                              global_options.virtualbox_memsize,
                                              global_options.virtualbox_vramsize,
                                              global_options.virtualbox_macaddress,
                                              global_options.remote_user,
                                              False)
                finally:
                    global_options.generate_virtualbox_image = None
                    global_options.refer_physical_machine = None
                boot_vm(global_options.virtualbox_path, vmname, physc)
                html = SUCCESS_HTML_TMPL % ("http://" +
                                            global_options.web_hostname
                                            + ":" +
                                            str(global_options.web_port))
            else:
                html = generate_top_html(db_name, global_options.log)
            s.wfile.write(html)
        except Exception as e:
            logger = logging.getLogger("pxe")
            logger.info("error has occurred: %s" % (e))
            html = generate_error_html(e,
                                       global_options.web_hostname,
                                       global_options.web_port)
            s.wfile.write(html)
    
def run_web(options):
    db = options.db
    port = options.web_port
    overwritep = options.overwrite_dhcp
    # setup global variables
    global db_name              # dirty hack!
    global global_options
    db_name = db
    global_options = options
    BaseHTTPServer.HTTPServer((options.web_hostname, port),
                              WebHandler).serve_forever()

def generate_pxe_config_files(db, generate_pxe_config_files, tftp_dir):
    if generate_pxe_config_files:
        logger = logging.getLogger("pxe")
        logger.info("generate the pxe configuration files")
        con = open_db(db)
        machines = all_hosts(con)
        for hostname in machines.keys():
            template = Template(PXE_CONFIG_TMPL)
            mac = machines[hostname]["macaddress"]
            file_name = os.path.join(tftp_dir, "pxelinux.cfg",
                                     "01-" + mac.replace(":", "-"))
            file_str = template.substitute({"hostname": hostname,
                                            "root": machines[hostname]["root"],
                                            "tftp_dir": tftp_dir})

            f = open(file_name, "w")
            f.write(file_str)
            f.close()
            check_call(["sudo", "chown", "pxe.tftp", file_name])
            check_call(["sudo", "chmod", "g+rw", file_name])
            check_call(["sudo", "chmod", "u+rw", file_name])
            
def generate_uuid():
    pipe = os.popen("uuidgen")
    uuid = pipe.read().strip()
    pipe.close()
    return uuid

def generate_virtualbox_image(db,
                              generate_virtualbox_image,
                              virtualbox_path,
                              refer_physical_machine,
                              virtualbox_cpunum,
                              virtualbox_memsize,
                              virtualbox_vramsize,
                              virtualbox_macaddress,
                              remote_user,
                              register_vm = True):
                              
    vmname = generate_virtualbox_image
    logger = logging.getLogger("pxe")
    if not os.path.exists(virtualbox_path):
        os.makedirs(virtualbox_path)
    if refer_physical_machine:
        host = refer_physical_machine
        logger.info("get cpuinfo of %s" % (host))
        infos = parallel_util.cpuinfos([host],
                                       arch_filter = False,
                                       verbose = True,
                                       ros_filter = False,
                                       username = remote_user)
        logger.info("cpuinfo of %s => %s" % (host, infos[0]))
        cpunum = infos[0][host][0]
        memsize = int(infos[0][host][1] * 0.8 / 1000)
    else:
        cpunum = virtualbox_cpunum
        memsize = virtualbox_memsize
    vramsize = virtualbox_vramsize
    if virtualbox_macaddress:
        macaddress = virtualbox_macaddress
    else:
        # lookup macaddress from db
        con = open_db(db)
        hostname = find_by_hostname(con, vmname)
        macaddress = hostname["macaddress"]
        con.close()
    cd_uuid = "9f6f1044-98a6-406c-be64-eec39baef4cb"
    machine_uuid = generate_uuid()
    vm_path = os.path.join(virtualbox_path, vmname + ".vbox")
    f = open(vm_path, "w")
    template = Template(VIRTUALBOX_XML_TEMPLATE)
    content = template.substitute({"machine_uuid": machine_uuid,
                                   "cd_uuid": cd_uuid,
                                   "hostname": vmname,
                                   "cpunum": cpunum,
                                   "memsize": memsize,
                                   "vramsize": vramsize,
                                   "macaddress": macaddress.replace(":", "")})
    f.write(content)
    f.close()
    #print("please register %s on your virtualbox" % (vm_path))
    if register_vm:
        check_call(["VBoxManage", "registervm", vmname + ".vbox"])
    
def print_virtualbox_macaddress():
    print(generate_virtualbox_macaddress())
    
def generate_virtualbox_macaddress():
    # TODO: check duplication of macaddress
    return "08:00:27:%02x:%02x:%02x" % (int(random.random()*0xff), int(random.random()*0xff), int(random.random()*0xff))

def nslookup(ip):
    output = gethostbyaddr(ip)
    return output[0]

def print_lookup_free_host(options):
    string = lookup_free_host(options)
    print(string)

def lookup_free_host(db, dhcp_range_start, dhcp_range_stop):
    con = open_db(db)
    start = dhcp_range_start
    stop = dhcp_range_stop
    # it might be a bug
    ip_1 = start.split(".")[0]
    ip_2 = start.split(".")[1]
    ip_3 = start.split(".")[2]
    start_ip_suffix = int(start.split(".")[3])
    stop_ip_suffix = int(stop.split(".")[3])
    for ip in range(start_ip_suffix, stop_ip_suffix):
        full_ip = ".".join([ip_1, ip_2, ip_3, str(ip)])
        not_available = find_by_ip(con, full_ip)
        if not not_available:
            # TODO: check ping!
            return "%s %s" % (nslookup(full_ip), full_ip)
    print("none")

def auto_add_vm(options):
    logger = logging.getLogger("pxe")
    root = options.auto_add
    free_host_ip = lookup_free_host(options.db,
                                    options.dhcp_range_start,
                                    options.dhcp_range_stop)
    free_host = free_host_ip.split()[0]
    free_ip = free_host_ip.split()[1]
    mac_address = generate_virtualbox_macaddress()
    add_machine(free_host, mac_address, free_ip, root, options.db)
    options.overwrite_dhcp = True # force to be True
    generate_dhcp(options.db,
                  options.prefix_dhcp_file,
                  options.subnet,
                  options.netmask,
                  options.dhcp_range_start,
                  options.dhcp_range_stop,
                  options.broadcast,
                  options.dns_ip,
                  options.domain_name,
                  options.gateway,
                  options.pxe_server,
                  options.pxe_filename,
                  options.overwrite_dhcp,
                  options.dhcp_conf_file)
    options.generate_pxe_config_files = True # force to be True
    generate_pxe_config_files(options.db,
                              options.generate_pxe_config_files,
                              options.tftp_dir)
    print(free_host)

def boot_vm(vmdir, vmname, physical_machine):
    logger = logging.getLogger("pxe")
    cmd0 = ["ssh", "-t", "pxe@%s" % (physical_machine),
            "sh -c 'VBoxManage unregistervm %s || exit 0'" % (vmname)]
    cmd1 = ["ssh", "pxe@%s" % physical_machine, "mkdir -p .VirtualBox"]
    cmd2 = ["scp", os.path.join(vmdir, vmname + ".vbox"), 
            "pxe@%s:.VirtualBox" % (physical_machine)]
    cmd3 = ["ssh", "-t", "pxe@%s" % (physical_machine),
            "VBoxManage registervm %s" % (vmname + ".vbox")]
    cmd4 = ["ssh", "pxe@%s" % physical_machine,
            "screen -d -m VBoxHeadless -s %s" % vmname]
    logger.info("exec => %s" % (cmd0))
    check_call(cmd0)
    logger.info("exec => %s" % (cmd1))
    check_call(cmd1)
    logger.info("exec => %s" % (cmd2))
    check_call(cmd2)
    logger.info("exec => %s" % (cmd3))
    check_call(cmd3)
    logger.info("exec => %s" % (cmd4))
    check_call(cmd4)

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

import os
from optparse import OptionParser

import roslib; roslib.load_manifest("parallel_util")
import parallel_util
from parallel_util.pxe_manager.core import *

def parse_options():
    parser = OptionParser()
    parser.add_option("-v", "--verbose", dest = "verbose",
                      action = "store_true",
                      help = "run pxe_manager.py in verbose mode")
    parser.add_option("--log", dest = "log",
                      default = "pxe.log",
                      help = "specify the path to log file")
    parser.add_option("--boot-vm", dest = "boot_vm",
                      nargs = 2,
                      metavar = "VMNAME PHYSICAL_MACHINE",
                      help = "boot VMNAME at PHYSICAL_MACHINE.")
    parser.add_option("--db", dest = "db",
                      default = os.path.join(os.path.dirname(__file__),
                                             "pxe.db"),
                      help = """db file to configure pxe boot environment.
(defaults to pxe.db)""")
    parser.add_option("--auto-add", dest = "auto_add", nargs = 1,
                      metavar = "ROOT_DIR",
                      help = """automatically generate dhcp.conf,
pxelinux config files, filesystem and add IP and hostname into DB.
MAC address will be also generated automatically.
this option is usefull to generate a new vm.""")
    parser.add_option("--add", dest = "add", nargs = 4,
                      metavar = "HOSTNAME IP MACADDRESS ROOT_DIR",
                      help = """add new machine to db.
ROOT_DIR is a relative path from the directory specified by
--tftp-dir option.""")
    parser.add_option("--tftp-dir", dest = "tftp_dir",
                      default = "/data/tftpboot",
                      help = """root directory of tftpboot. defaults
to /data/tftpboot""")
    parser.add_option("--lookup-free-host",
                      action = "store_true",
                      help = """lookup a hostname which is able to be used
for vm""")
    parser.add_option("--generate-pxe-config-files",
                      dest = "generate_pxe_config_files",
                      action = "store_true",
                      help = """ automatically generate the configuration files
under pxelinux.cfg/""")
    parser.add_option("--web", dest = "web",
                      action = "store_true", help = """run webserver""")
    parser.add_option("--web-port", dest = "web_port",
                      type = int,
                      default = 4040,
                      help = """port of webserver (defaults to 4040)""")
    parser.add_option("--web-hostname", dest = "web_hostname",
                      default = "localhost",
                      help = """hostname of webserver
(defaults to localhost)""")
    parser.add_option("--generate-virtualbox-image",
                      metavar = "vmname",
                      help = """generate an empty virtualbox image for
pxe booting""")
    parser.add_option("--remote-user", default = None,
                      help = """username to be used
when a remote access is required""")
    parser.add_option("--refer-physical-machine",
                      metavar = "hostname",
                      help = """decide the parameters of virtualbox image from
a physical machine""")
    parser.add_option("--virtualbox-cpunum",
                      default = 8,
                      type = int,
                      help = """the number of CPUs of vm. (defaults to 8)""")
    parser.add_option("--virtualbox-memsize",
                      default = 4096,
                      type = int,
                      help = """the size of Memory of vm (in MB).
defaults to 4096""")
    parser.add_option("--virtualbox-vramsize",
                      default = 64,
                      type = int,
                      help = """the size of Video Memory of vm (in MB).
defaults to 64""")
    parser.add_option("--virtualbox-macaddress",
                      help = """the macaddress of vm.""")
    parser.add_option("--virtualbox-path",
                      default = os.path.join(os.environ["HOME"],
                                             ".VirtualBox"),
                      help = """path where vm configuration file be saved""")
    parser.add_option("--delete", dest = "delete", nargs = 1,
                      help = """delete a host from db.""")
    parser.add_option("--generate-dhcp", dest = "generate_dhcp",
                      action = "store_true",
                      help = "generate dhcp file")
    parser.add_option("--dhcp-conf-file", dest = "dhcp_conf_file",
                      default = "/etc/dhcp3/dhcpd.conf",
                      help = "dhcp configuration file to be overwritten")
    parser.add_option("--overwrite-dhcp", dest = "overwrite_dhcp",
                      action = "store_true",
                      help = """overwrite dhcp configuration file or not.
(defaults to false)""")
    parser.add_option("--restart-dhcp", dest = "restart_dhcp",
                      action = "store_true",
                      help = """restart dhcp after generating dhcpd.conf""")
    parser.add_option("--generate-virtualbox-macaddress",
                      action = "store_true",
                      help = """generate a random macaddress for virtualbox""")
    parser.add_option("--prefix-dhcp-file", dest = "prefix_dhcp_file",
                      default = os.path.join(os.path.dirname(__file__),
                                             "dhcpd.conf.pre"),
                      help = """pxe_manager only generates host syntax.
if you want to create a full dhcpd.conf, you can specify a file to be a
prefix of dhcpd.conf""")
    parser.add_option("--pxe-filename", dest = "pxe_filename",
                      default = "pxelinux.0",
                      help = "file name of pxelinux. (defaults to pxelinux.0)")
    parser.add_option("--pxe-server", dest = "pxe_server",
                      default = "192.168.101.182",
                      help = """the ip address of pxe server.
(defaults to 192.168.101.182)""")
    parser.add_option("--subnet", dest = "subnet",
                      default = "192.168.101.0",
                      help = "subnet of dhcp. (defaults to 192.168.101.0)")
    parser.add_option("--netmask", dest = "netmask",
                      default = "255.255.255.0",
                      help = "netmask of dhcp. (defaults to 255.255.255.0)")
    parser.add_option("--dhcp-range-start", dest = "dhcp_range_start",
                      default = "192.168.101.1",
                      help = """the starting ip address of dhcp
(defaults to 192.168.101.1) .""")
    parser.add_option("--dhcp-range-stop", dest = "dhcp_range_stop",
                      default = "192.168.101.127",
                      help = """the ending ip address of dhcp
(defaults to 192.168.101.127). """)
    parser.add_option("--broadcast", dest = "broadcast",
                      default = "192.168.101.255",
                      help = """broadcast of the network.
(defaults to 192.168.101.255) """)
    parser.add_option("--dns-ip", dest = "dns_ip",
                      default = "192.168.96.209",
                      help = "DNS of the network. (defaults to 192.168.96.209)")
    parser.add_option("--domain-name", dest = "domain_name",
                      default = "jsk.t.u-tokyo.ac.jp",
                      help = """domain name of the network.
(defaults to jsk.t.u-tokyo.ac.jp)""")
    parser.add_option("--gateway", dest = "gateway",
                      default = "192.168.101.254",
                      help = """gateway of the network.
(defaults to 192.168.101.254)""")
    parser.add_option("--list", dest = "list",
                      action = "store_true",
                      help = "print the list of the machines registered")
    parser.add_option("--wol", dest = "wol",
                      metavar = "HOSTNAME",
                      action = "append",
                      help = """send magick packet of WakeOnLan to the
specified host""")
    parser.add_option("--wol-port", dest="wol_port",
                      type = int,
                      default = 9,
                      help = "port of WakeOnLan")
    parser.add_option("--generate-pxe-filesystem",
                      dest = "generate_pxe_filesystem",
                      nargs = 1,
                      help = "generate a filesystem for pxe boot environment.")
    parser.add_option("--pxe-filesystem-template",
                      dest = "pxe_filesystem_template",
                      default = "/data/tftpboot/root_template",
                      nargs = 1,
                      help = """specify the directory where template of pxe
filesystem (will)  locate(s) (defaults to /data/tftpboot/root_template)""")
    parser.add_option("--pxe-filesystem-apt-sources",
                      dest = "pxe_filesystem_apt_sources",
                      default = "/etc/apt/sources.list",
                      help = """the path of apt source file of pxe filesystem
(defaults to /etc/apt/sources.list)""")
    parser.add_option("--pxe-user",
                      dest = "pxe_user",
                      default = "pxe",
                      help = """default user in the pxe filesystem
(defaults to pxe)""")
    parser.add_option("--pxe-passwd",
                      dest = "pxe_passwd",
                      default = "pxe",
                      help = """default password in the pxe filesystem
(defaults to pxe)""")
    (options, args) = parser.parse_args()
    return options

def main():
    options = parse_options()
    # first of all, create logger
    logger = create_logger(options.log)
    if options.web:
        run_web(options)
    else:
        if options.add:
            add_machine(options.add[0], options.add[2],
                        options.add[1], options.add[3], options.db)
        elif options.delete:
            delete_machine(options.delete, options.db)
        elif options.generate_dhcp:
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
                          options.dhcp_conf_file,
                          options.restart_dhcp)
        elif options.list:
            print_machine_list(options.db)
        elif options.wol:
            for m in [options.wol]:
                wake_on_lan(m[0], options.wol_port,
                            options.broadcast, options.db)
        elif options.generate_pxe_filesystem:
            generate_pxe_filesystem(options.pxe_filesystem_template,
                                    options.generate_pxe_filesystem,
                                    options.pxe_filesystem_apt_sources,
                                    options.pxe_user, options.pxe_passwd)
        elif options.generate_pxe_config_files:
            generate_pxe_config_files(options.db,
                                      options.generate_pxe_config_files,
                                      options.tftp_dir)
        elif options.generate_virtualbox_image:
            generate_virtualbox_image(options.db,
                                      options.generate_virtualbox_image,
                                      options.virtualbox_path,
                                      options.refer_physical_machine,
                                      options.virtualbox_cpunum,
                                      options.virtualbox_memsize,
                                      options.virtualbox_vramsize,
                                      options.virtualbox_macaddress,
                                      options.remote_user)
        elif options.generate_virtualbox_macaddress:
            print_virtualbox_macaddress()
        elif options.lookup_free_host:
            print_lookup_free_host(options.db,
                                   options.dhcp_range_start,
                                   options.dhcp_range_stop)
        elif options.auto_add:
            auto_add_vm(options)
        elif options.boot_vm:
            boot_vm(options.virtualbox_path,
                    options.boot_vm[0], options.boot_vm[1])
            
if __name__ == "__main__":
    main()

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
import random
from socket import gethostbyaddr
import BaseHTTPServer
import cgi
import sqlite3
import socket
import binascii
import os
from subprocess import check_call
from optparse import OptionParser
from string import Template

VIRTUALBOX_XML_TEMPLATE = """<?xml version="1.0"?>
<VirtualBox xmlns="http://www.innotek.de/VirtualBox-settings" version="1.11-linux">
  <Machine uuid="{${machine_uuid}}" name="${hostname}" OSType="Ubuntu_64" snapshotFolder="Snapshots" lastStateChange="2011-07-13T01:36:29Z">
    <MediaRegistry>
      <HardDisks/>
      <DVDImages>
        <Image uuid="{${cd_uuid}}" location="/usr/share/virtualbox/VBoxGuestAdditions.iso"/>
      </DVDImages>
      <FloppyImages/>
    </MediaRegistry>
    <ExtraData>
      <ExtraDataItem name="GUI/InfoDlgState" value="400,450,normal"/>
      <ExtraDataItem name="GUI/LastCloseAction" value="powerOff"/>
      <ExtraDataItem name="GUI/LastGuestSizeHint" value="2560,1600"/>
      <ExtraDataItem name="GUI/LastNormalWindowPosition" value="1,33,2558,1538"/>
      <ExtraDataItem name="GUI/LastScaleWindowPosition" value="960,551,640,480"/>
      <ExtraDataItem name="GUI/MiniToolBarAlignment" value="top"/>
      <ExtraDataItem name="GUI/SaveMountedAtRuntime" value="yes"/>
      <ExtraDataItem name="GUI/ShowMiniToolBar" value="yes"/>
    </ExtraData>
    <Hardware version="2">
      <CPU count="${cpunum}" hotplug="false">
        <HardwareVirtEx enabled="true" exclusive="true"/>
        <HardwareVirtExNestedPaging enabled="true"/>
        <HardwareVirtExVPID enabled="true"/>
        <PAE enabled="false"/>
        <HardwareVirtExLargePages enabled="false"/>
        <HardwareVirtForce enabled="false"/>
      </CPU>
      <Memory RAMSize="${memsize}" PageFusion="false"/>
      <HID Pointing="USBTablet" Keyboard="PS2Keyboard"/>
      <HPET enabled="false"/>
      <Chipset type="PIIX3"/>
      <Boot>
        <Order position="1" device="Floppy"/>
        <Order position="2" device="DVD"/>
        <Order position="3" device="Network"/>
        <Order position="4" device="None"/>
      </Boot>
      <Display VRAMSize="${vramsize}" monitorCount="1" accelerate3D="true" accelerate2DVideo="false"/>
      <RemoteDisplay enabled="false" authType="Null" authTimeout="5000" allowMultiConnection="true">
        <VRDEProperties>
          <Property name="TCP/Ports" value="8888"/>
        </VRDEProperties>
      </RemoteDisplay>
      <BIOS>
        <ACPI enabled="true"/>
        <IOAPIC enabled="true"/>
        <Logo fadeIn="true" fadeOut="true" displayTime="0"/>
        <BootMenu mode="MessageAndMenu"/>
        <TimeOffset value="0"/>
        <PXEDebug enabled="false"/>
      </BIOS>
      <USBController enabled="true" enabledEhci="false"/>
      <Network>
        <Adapter slot="0" enabled="true" MACAddress="${macaddress}" cable="true" speed="0" type="Am79C973">
          <DisabledModes>
            <NAT>
              <DNS pass-domain="true" use-proxy="false" use-host-resolver="false"/>
              <Alias logging="false" proxy-only="false" use-same-ports="false"/>
            </NAT>
          </DisabledModes>
          <BridgedInterface name="eth0"/>
        </Adapter>
      </Network>
      <UART>
        <Port slot="0" enabled="false" IOBase="0x3f8" IRQ="4" hostMode="Disconnected"/>
        <Port slot="1" enabled="false" IOBase="0x2f8" IRQ="3" hostMode="Disconnected"/>
      </UART>
      <LPT>
        <Port slot="0" enabled="false" IOBase="0x378" IRQ="4"/>
        <Port slot="1" enabled="false" IOBase="0x378" IRQ="4"/>
      </LPT>
      <AudioAdapter controller="AC97" driver="Pulse" enabled="true"/>
      <RTC localOrUTC="UTC"/>
      <SharedFolders/>
      <Clipboard mode="Bidirectional"/>
      <IO>
        <IoCache enabled="true" size="5"/>
        <BandwidthGroups/>
      </IO>
      <Guest memoryBalloonSize="0"/>
      <GuestProperties/>
    </Hardware>
    <StorageControllers>
      <StorageController name="IDE Controller" type="PIIX4" PortCount="2" useHostIOCache="true" Bootable="true">
        <AttachedDevice passthrough="false" type="DVD" port="1" device="0">
          <Image uuid="{${cd_uuid}}"/>
        </AttachedDevice>
      </StorageController>
      <StorageController name="SATA Controller" type="AHCI" PortCount="1" useHostIOCache="false" Bootable="true" IDE0MasterEmulationPort="0" IDE0SlaveEmulationPort="1" IDE1MasterEmulationPort="2" IDE1SlaveEmulationPort="3"/>
    </StorageControllers>
  </Machine>
</VirtualBox>
"""

PXE_DPHYS_CONFIG = """
CONF_SWAPSIZE=20480
CONF_SWAPFILE=/var/swap-`ifconfig eth0 | grep HWaddr | sed 's/.*HWaddr //g' | sed 's/ //g'`
"""

PXE_DPHYS_SWAPFILE_INIT_D = """
#!/bin/sh
# /etc/init.d/pxe-dphys-swapfile - automatically set up an swapfile
# author Neil Franklin, last modification 2006.09.15
# author Ryohei Ueda, modified for pxe booting
# This script is copyright ETH Zuerich Physics Departement,
#   use under either modified/non-advertising BSD or GPL license

# this init.d script is intended to be run from rcS.d
#   must run after  mount  of  /var  which may only happen in  S35mountall.sh
#     for this reason we can not build swapfile until after  S35mountall.sh
#       so we also need to use  init.d start|stop  to swapon|off our file
#   and sensibly before the lots of stuff which may happen in  S40networking
#   so we run it as rcS.d/S37dphys-config

### BEGIN INIT INFO
# Provides:          dphys-swapfile
# Required-Start:    $syslog
# Required-Stop:     $syslog
# Should-Start:      $local_fs
# Should-Stop:       $local_fs
# Default-Start:     2 3 4 5
# Default-Stop:
# Short-Description: Autogenerate and use a swap file
# Description:       This init.d script exists so one does not need to have a fixed size
#                    swap partition. Instead install without swap partition and then run
#                    this, with file size (re-)computed automatically to fit the current
#                    RAM size.
### END INIT INFO

# get ready to work
PATH=/usr/local/sbin:/sbin:/bin:/usr/sbin:/usr/bin
export PATH

# what we are
NAME=dphys-swapfile

case "$1" in

  start)
    /bin/echo "Starting ${NAME} swapfile setup ..."

    # (re-)size/-generate (and also first time install)
    # this will produce output, so no -n in above echo
    /usr/local/sbin/pxe-dphys-swapfile setup

    # as S35mountall.sh has already run, do this from here
    #   as there can be no swapon in /etc/fstab
    /usr/local/sbin/pxe-dphys-swapfile swapon

    /bin/echo "done."
    ;;


  stop|default-stop)
    /bin/echo -n "Stopping ${NAME} swapfile setup ..."

    # as no swapon or swapoff in /etc/fstab, do this from here
    /usr/local/sbin/pxe-dphys-swapfile swapoff

    /bin/echo ", done."
    ;;


  restart|reload|force-reload)
    /bin/echo "No daemon to (force-)re[start|load] in ${NAME}"
    ;;


 *)
    /bin/echo "Usage: $0 {start|stop}"

    exit 1
    ;;

esac

exit 0
"""

PXE_DPHYS_SWAPFILE ="""
#! /bin/sh
# /usr/local/sbin/pxe-dphys-swapfile - automatically set up an swapfile
# author Neil Franklin, last modification 2006.10.20
# author Ryohei Ueda, modified for pxe booting
# This script is copyright ETH Zuerich Physics Departement,
#   use under either BSD or GPL license

### ------ configuration for this site

# where we want the swapfile to be, this is the default
CONF_SWAPFILE=/var/swap

# size we want to force it to be, default (empty) gives 2*RAM
CONF_SWAPSIZE=


### ------ actual implementation from here on
# no user settings any more below this point

set -e

# get ready to work
PATH=/sbin:/bin:/usr/sbin:/usr/bin
export PATH


# this is what we want, 2 times RAM size
SWAPFACTOR=2


# what we are
NAME=dphys-swapfile
PNAME=dphys-swapfile

# check user config file, let user overwride settings
#   swap file place/filename and size
if [ -f /etc/"${PNAME}" ] ; then
  . /etc/"${PNAME}"
fi


case "$1" in

  setup)
    # (re-)size/-generate, fast if no memory size change

    if [ "${CONF_SWAPSIZE}" = "" ] ; then
      # compute automatic optimal size
      echo -n "computing size, "
      # this seems to be the nearest to physical RAM size, lacks about 60k
      KCORESIZE="`ls -l /proc/kcore | awk '{ print $5 }'`"
      # make MBytes which rounded down will be exactly 1 too few, so add 1
      MEMSIZE="`expr "${KCORESIZE}" / 1048576 + 1`"
      # default, without config file overwriding, swap=2*RAM
      CONF_SWAPSIZE="`expr "${MEMSIZE}" '*' "${SWAPFACTOR}"`"
    fi

    # announce end resulting config
    echo -n "want ${CONF_SWAPFILE}=${CONF_SWAPSIZE}MByte"


    # we will be later starting, and in between possible deleting/rebuilding
    #   so deactivate any allready running swapfile, to avoid errors
    "$0" swapoff



    # compare existing swapfile (if one exists) to see if it needs replacing
    if [ -f "${CONF_SWAPFILE}" ] ; then

      echo -n ", checking existing"

      # we need bytes for comparing with existing swap file
      SWAPBYTES="`expr "${CONF_SWAPSIZE}" '*' 1048576`"

      FILEBYTES="`ls -l "${CONF_SWAPFILE}" | awk '{ print $5 }'`"

      # wrong size, get rid of existing swapfile, after remake
      if [ "${FILEBYTES}" != "${SWAPBYTES}" ] ; then

        # updates to this section need duplicating in postrm script
        #   can not simply make subroutine here and call that from postrm
        #     as this script is deleted before  postrm purge  is called

        echo -n ": deleting wrong size file (${FILEBYTES})"

        # deactivate and delete existing file, before remaking for new size
        "$0" uninstall

      else

        echo -n ": keeping it"

      fi
    fi

    # if no swapfile (or possibly old one got deleted) make one
    if [ ! -f "${CONF_SWAPFILE}" ] ; then

      echo -n ", generating swapfile ..."

      # first deleting existing mount lines, if any there (same code as above)
      grep -v "^${CONF_SWAPFILE}" /etc/fstab > /etc/.fstab
      mv /etc/.fstab /etc/fstab

      dd if=/dev/zero of="${CONF_SWAPFILE}" bs=1048576 \
        count="${CONF_SWAPSIZE}" 2> /dev/null
      mkswap "${CONF_SWAPFILE}" > /dev/null

      # ensure that only root can read possibly critical stuff going in here
      chmod 600 "${CONF_SWAPFILE}"

      # do not mount swapfile via fstab, because S35mountall.sh is already done
      #   so just add warning comment line that swapfile is not in fstab
      #     and so gets mounted by this script
      # get rid of possibly already existing comment about
      #   swapfile mounted by this script
      grep -v "^# a swapfile" /etc/fstab > /etc/.fstab
      grep -v "${NAME}" /etc/.fstab > /etc/fstab
      # add new comment about this
      echo "# a swapfile is not a swap partition, so no using swapon|off" \
        "from here on, use  ${NAME} swap[on|off]  for that" >> /etc/fstab

      # and inform the user what we did
      echo -n " of ${CONF_SWAPSIZE}MBytes"

    fi

    echo

    ;;


  install)
    # synonym for setup, in case someone types this
    "$0" setup

    ;;


  swapon)
    # as there can be no swapon in /etc/fstab, do it from here
    #   this is due to no possible insertion of code (at least in Debian)
    #     between mounting of /var (where swap file most likely resides)
    #     and executing swapon, where the file already needs to be existing

    if [ -f "${CONF_SWAPFILE}" ] ; then
      losetup /dev/loop0 ${CONF_SWAPFILE}
      swapon /dev/loop0 2>&1 > /dev/null
    else
      echo "$0: ERROR: swap file ${CONF_SWAPFILE} missing!" \
          "you need to first run  $0 setup  to generate one"
    fi

    ;;


  swapoff)
    # as there can also be no swapoff in /etc/fstab, do it from here

    # first test if swap is even active, else error from swapoff
    if [ "`swapon -s | grep /dev/loop0 | \
        cut -f1 -d\  `" != "" ] ; then
      swapoff /dev/loop0 2>&1 > /dev/null
    fi

    ;;


  uninstall)
    # note: there is no install), as setup) can run from any blank system
    #   it auto-installs as side effect of recomputing and checking size

    # deactivate before deleting
    "$0" swapoff

    if [ -f "${CONF_SWAPFILE}" ] ; then
      # reclaim the file space
      rm "${CONF_SWAPFILE}"
    fi

    # and get rid of comment about swapfile mounting
    grep -v "^# a swapfile" /etc/fstab > /etc/.fstab
    grep -v "${NAME}" /etc/.fstab > /etc/fstab

    ;;


 *)
    echo "Usage: $0 {setup|swapon|swapoff|uninstall}"

    exit 1
    ;;

esac

exit 0
"""

FIND_HOST_SQL = """
select * from hosts where hostname = '${hostname}';
"""

FIND_IP_SQL = """
select * from hosts where ip = '${ip}';
"""

DB_CREATE_TABLE_SQL = """
create table hosts (
hostname text,
ip text,
macaddress text UNIQUE,
root text
);
"""

ADD_HOST_SQL = """
insert into hosts values ('${hostname}', '${ip}', '${macaddress}', '${root}');
"""

DEL_HOST_SQL = """
delete from hosts where hostname = '${hostname}';
"""

ALL_HOSTS_SQL = """
select * from hosts;
"""

APT_PACKAGES = """
python-software-properties
ssh zsh bash-completion
emacs vim wget
build-essential subversion git-core cvs
cmake ethtool python python-paramiko python-meminfo-total
initramfs-tools linux-image nfs-kernel-server
"""
#ubuntu-desktop-ja gdm

DHCP_SUBNET_TMPL = """
subnet ${subnet} netmask ${netmask} {
range dynamic-bootp ${dhcp_range_start} ${dhcp_range_stop};
option broadcast-address ${broadcast};
option domain-name-servers ${dns_ip};
option domain-name "${domain_name}";
option routers ${gateway};
filename "${pxe_filename}";
next-server ${pxe_server};
${hosts}
}
"""

DHCP_HOST_TMPL = """
  host ${hostname}{
   hardware ethernet ${mac};
   fixed-address ${ip};
   option host-name "${hostname}";
  }
"""

HTML_TMPL = """
<html>
<!DOCTYPE html PUBLIC "-//W3C//DTD XHTML 1.0 Transitional//EN"
 "http://www.w3.org/TR/xhtml1/DTD/xhtml1-transitional.dtd">
<html xmlns="http://www.w3.org/1999/xhtml" xml:lang="ja" lang="ja">
  <head>
    <meta http-equiv="Content-Type" content="text/html;charset=UTF-8" />
    <meta http-equiv="Content-Script-Type" content="text/javascript" />
    <title>PXE Manager</title>
    <style type="text/css">
dl.host_list {
 border:1px solid #999;
 width:490px;
}

dt.hostname {
 float:left;
 width:100px;
 padding:5px 0 5px 10px;
 clear:both;
 font-weight:bold;
}

dd.ip_mac {
 width:260px;
 margin-left:100px;
 padding:5px 5px 5px 10px;
 border-left:1px solid #999;
}

dl.ip_mac {
    width:260px;
    margin-left:0px;
    padding:5px 5px 5px 10px;
}
    </style>
  </head>
  <body>
    <div id="main">
      <div id="title">
        <h1>
          PXE Boot manager @ MBA
        </h1>
      </div> <!-- #title -->
      <div id="add">
       <form method="get" action="add">
         hostname
         <input type="text" name="hostname"/>
         IP
         <input type="text" name="ip"/>
         MACAddress
         <input type="text" name="macaddress"/>
         root directory
         <input type="text" name="root"/>
         <input type="submit" />
       </form>
      </div>
      <div id="host_list">
        <dl class="host_list">
          ${hosts}
        </dl>
      </div>
    </div>
  </body>
</html>
"""

HTML_HOST_TMPL = """
          <dt class="hostname">${hostname}</dt>
          <dd class="ip_mac">
            <div>
              ${ip}/${macaddress}@${root}
            </div>
            <div class="delete_host">
              <form method="get" action="delete">
                <input type="submit" value="delete" name="${hostname}"/>
              </form>
            </div>
          </dd>
"""

PXE_CONFIG_TMPL = """
menu INCLUDE pxelinux.cfg/graphics.cfg
DEFAULT vesamenu.c32
NOESCAPE 1
ALLOWOPTIONS 0
boot label in ${tftp_dir}
LABEL Lucid
      MENU LABEL Lucid-${hostname}
      MENU DEFAULT
      KERNEL ${root}/vmlinuz
      APPEND quiet splash initrd=${root}/initrd.img netboot=nfs raid=noautodetect root=/dev/nfs nfsroot=192.168.101.182:${tftp_dir}/${root} ip=dhcp rw --
"""

INITRAMFS_CONF = """
MODULES=netboot
BUSYBOX=y
COMPCACHE_SIZE=""
BOOT=nfs
DEVICE=eth0
NFSROOT=auto
"""

INITRAM_MODULES = """
sky2
e1000
tg3
bnx2
pcnet32
"""

FSTAB = """
proc /proc proc defaults 0 0
/dev/nfs / nfs defaults 1 1
none /tmp tmpfs defaults 0 0
none /var/run tmpfs defaults 0 0
none /var/lock tmpfs defaults 0 0
none /var/tmp tmpfs defaults 0 0
"""

def parse_options():
    parser = OptionParser()
    parser.add_option("-v", "--verbose", dest = "verbose",
                      action = "store_true",
                      help = "run pxe_manager.py in verbose mode")
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
                                             "VirtualBox VMs"),
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

def find_machine_tag_by_hostname(dom, hostname):
    machines = dom.getElementsByTagName("machine")
    for m in machines:
        if m.getAttribute("name") == hostname:
            return m
    return False

def get_root_pxe(dom):
    return dom.childNodes[0]

def delete_machine(hostname, db):
    con = open_db(db)
    delete_host(con, hostname)
    con.close()
    
def add_machine(hostname, mac, ip, root, db):
    con = open_db(db)
    add_host(con, hostname, ip, mac, root)
    con.close()
    
def generate_dhcp(options, db):
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
    if options.prefix_dhcp_file:
        f = open(options.prefix_dhcp_file)
        prefix_str = "".join(f.readlines())
    else:
        prefix_str = ""
    # generate dhcp subnet section
    dhcp_subnet_tmpl = Template(DHCP_SUBNET_TMPL)
    replace_dict = {"subnet": options.subnet,
                    "netmask": options.netmask,
                    "dhcp_range_start": options.dhcp_range_start,
                    "dhcp_range_stop": options.dhcp_range_stop,
                    "broadcast": options.broadcast,
                    "dns_ip": options.dns_ip,
                    "domain_name": options.domain_name,
                    "gateway": options.gateway,
                    "pxe_server": options.pxe_server,
                    "pxe_filename": options.pxe_filename,
                    "hosts": "\n".join(generated_string)}
    dhcp_subnet_str = dhcp_subnet_tmpl.substitute(replace_dict)
    if options.overwrite_dhcp:
        path = options.dhcp_conf_file
        f = open(path, "w")
        f.write(prefix_str + dhcp_subnet_str)
        f.close()
    else:
        print prefix_str + dhcp_subnet_str

def print_machine_list(db):
    con = open_db(db)
    machines = all_hosts(con)
    for hostname, ip_mac in machines.items():
        ip = ip_mac["ip"]
        mac = ip_mac["macaddress"]
        root = ip_mac["root"]
        print """%s:
  ip: %s
  mac: %s
  root: %s
""" % (hostname, ip, mac, root)

def wake_on_lan(hostname, port, broadcast, db):
    con = open_db(db)
    mac = find_by_hostname(db, hostname)["macaddress"]
    send_wol_magick_packet([mac], broadcast, port)

def chroot_command(chroot_dir, *args):
    command = ["chroot", chroot_dir]
    command.extend(*args)
    print command
    return check_call(command)
    
def generate_pxe_template_filesystem(template_dir):
    print ">>> generating template filesystem"
    try:
        check_call(["debootstrap", "lucid", template_dir])
    except:
        # remove template_dir
        print ">>> removing template dir"
        check_call(["rm", "-rf", template_dir])
        raise


def copy_template_filesystem(template_dir, target_dir, apt_sources):
    print ">>> copying filesystem"
    check_call(["cp", "-ax", template_dir, target_dir])
    print ">>> copying etc/apt/sources.list"
    check_call(["cp", apt_sources,
                os.path.join(target_dir, "etc", "apt", "sources.list")])
    return

class ChrootEnvironment():
    def __init__(self, target_dir):
        self.target_dir = target_dir
    def __enter__(self):
        target_dir = self.target_dir
        check_call(["mount", "-o", "bind", "/dev/",
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
        check_call(["umount", "-lf", os.path.join(target_dir, "dev")])
    

def install_apt_packages(target_dir):
    env = ChrootEnvironment(target_dir)
    with env:
        print ">>> installing base apt packages"
        chroot_command(target_dir, ["apt-get", "update"])
        chroot_command(target_dir, ["apt-get", "install", "wget"])
        chroot_command(target_dir, ["sh", "-c",
                                    "wget -q https://www.ubuntulinux.jp/ubuntu-ja-archive-keyring.gpg -O- | sudo apt-key add -"])
        chroot_command(target_dir, ["sh", "-c",
                                    "wget -q https://www.ubuntulinux.jp/ubuntu-jp-ppa-keyring.gpg -O- | sudo apt-key add -"])
        chroot_command(target_dir, ["sh", "-c",
                                    "sudo wget https://www.ubuntulinux.jp/sources.list.d/lucid.list -O /etc/apt/sources.list.d/ubuntu-ja.list"])
        chroot_command(target_dir, ["apt-get", "update"])
        chroot_command(target_dir,
                       ["apt-get", "install", "--force-yes", "-y"] + APT_PACKAGES.split())
        print "  >>> installing ros and openrave apt packages"
        chroot_command(target_dir, ["sh", "-c",
                                    "echo deb http://packages.ros.org/ros/ubuntu lucid main > /etc/apt/sources.list.d/ros-latest.list"])
        chroot_command(target_dir, ["sh", "-c",
                                    "wget http://packages.ros.org/ros.key -O - | apt-key add -"])
        chroot_command(target_dir, ["add-apt-repository", "ppa:openrave/release"])
        chroot_command(target_dir, ["apt-get", "update"])
        chroot_command(target_dir, ["apt-get", "install", "--force-yes", "-y", "ros-diamondback-ros-base", "openrave"])

def setup_user(target_dir, user, passwd):
    env = ChrootEnvironment(target_dir)
    with env:
        chroot_command(target_dir, ["sh", "-c",
                                    "id %s || useradd %s -g sudo -s /bin/bash" % (user,
                                                                                  user)])
        chroot_command(target_dir, ["sh", "-c",
                                    "echo %s:%s | chpasswd" % (user, passwd)])
        chroot_command(target_dir, ["rm", "-f", "/etc/hostname"])

def update_initram(target_dir):
    env = ChrootEnvironment(target_dir)
    with env:
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
    f = open(os.path.join(target_dir, "usr/local/sbin/pxe-dphys-swapfile"), "w")
    f.write(PXE_DPHYS_SWAPFILE)
    f.close()
    f = open(os.path.join(target_dir, "etc/init.d/pxe-dphys-swapfile"), "w")
    f.write(PXE_DPHYS_SWAPFILE_INIT_D)
    f.close()
    f = open(os.path.join(target_dir, "etc/dphys-swapfile"), "w")
    f.write(PXE_DPHYS_CONFIG)
    f.close()
    with env:
        chroot_command(target_dir,
                       ["chmod", "+x", "/usr/local/sbin/pxe-dphys-swapfile"])
        chroot_command(target_dir,
                       ["chmod", "+x", "/etc/init.d/pxe-dphys-swapfile"])
        chroot_command(target_dir,
                       ["update-rc.d", "pxe-dphys-swapfile", "defaults"])

        
def generate_pxe_filesystem(template_dir, target_dir, apt_sources,
                            user, passwd):
    if not os.path.exists(template_dir):
        generate_pxe_template_filesystem(template_dir)
    if not os.path.exists(target_dir):
        copy_template_filesystem(template_dir, target_dir, apt_sources)
    check_call(["cp", apt_sources,
                os.path.join(target_dir, "etc", "apt", "sources.list")])
    install_apt_packages(target_dir)
    setup_user(target_dir, user, passwd)
    setup_pxe_dphys(target_dir)
    update_initram(target_dir)

def generate_top_html(db):
    con = open_db(db)
    machines = all_hosts(con)
    host_strs = []
    for (hostname, ip_mac) in machines.items():
        ip = ip_mac["ip"]
        mac = ip_mac["macaddress"]
        root = ip_mac["root"]
        template = Template(HTML_HOST_TMPL)
        host_strs.append(template.substitute({"hostname": hostname,
                                              "ip": ip,
                                              "macaddress": mac,
                                              "root": root}))
    con.close()
    html_template = Template(HTML_TMPL)
    return html_template.substitute({"hosts": "\n".join(host_strs)})

def update_dhcp_from_web():
    if global_options.overwrite_dhcp:
        generate_dhcp(global_options, global_options.db)
    if global_options.generate_pxe_config_files:
        generate_pxe_config_files(global_options)

class WebHandler(BaseHTTPServer.BaseHTTPRequestHandler):
    def do_GET(s):
        s.send_response(200)
        s.send_header("Content-type", "text/html")
        s.end_headers()
        if s.path.startswith("/delete"): # delete
            delete_host = cgi.parse_qs(s.path.split("?")[1]).keys()[0]
            delete_machine(delete_host, db_name)
            update_dhcp_from_web()
        elif s.path.startswith("/add"): # add
            add_host_desc = cgi.parse_qs(s.path.split("?")[1])
            print add_host_desc
            add_machine(add_host_desc["hostname"][0],
                        add_host_desc["macaddress"][0],
                        add_host_desc["ip"][0],
                        add_host_desc["root"][0],
                        db_name)
            update_dhcp_from_web()
        html = generate_top_html(db_name)
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

def generate_pxe_config_files(options):
    if options.generate_pxe_config_files:
        print ">>> generate the pxe configuration files"
        db = options.db
        con = open_db(db)
        machines = all_hosts(con)
        for hostname in machines.keys():
            template = Template(PXE_CONFIG_TMPL)
            mac = machines[hostname]["macaddress"]
            file_name = os.path.join(options.tftp_dir, "pxelinux.cfg",
                                     "01-" + mac.replace(":", "-"))
            file_str = template.substitute({"hostname": hostname,
                                            "root": machines[hostname]["root"],
                                            "tftp_dir": options.tftp_dir})

            f = open(file_name, "w")
            f.write(file_str)
            f.close()
            
def generate_uuid():
    pipe = os.popen("uuidgen")
    uuid = pipe.read().strip()
    pipe.close()
    return uuid

def generate_virtualbox_image(options):
    vmname = options.generate_virtualbox_image
    if not os.path.exists(options.virtualbox_path):
        os.makedirs(options.virtualbox_path)
    if not os.path.exists(os.path.join(options.virtualbox_path,
                                       vmname)):
        os.makedirs(os.path.join(options.virtualbox_path,
                                 vmname))
    cpunum = options.virtualbox_cpunum
    memsize = options.virtualbox_memsize
    vramsize = options.virtualbox_vramsize
    macaddress = options.virtualbox_macaddress
    cd_uuid = "9f6f1044-98a6-406c-be64-eec39baef4cb"
    machine_uuid = generate_uuid()
    f = open(os.path.join(options.virtualbox_path,
                          vmname, vmname + ".vbox"),
             "w")
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

def print_virtualbox_macaddress():
    print generate_virtualbox_macaddress()
    
def generate_virtualbox_macaddress():
    # TODO: check duplication of macaddress
    return "08:00:27:%02x:%02x:%02x" % (int(random.random()*0xff), int(random.random()*0xff), int(random.random()*0xff))

def nslookup(ip):
    output = gethostbyaddr(ip)
    return output[0]

def print_lookup_free_host(options):
    string = lookup_free_host(options)
    print string

def lookup_free_host(options):
    db = options.db
    con = open_db(db)
    start = options.dhcp_range_start
    stop = options.dhcp_range_stop
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
    print "none"

def auto_add_vm(options):
    root = options.auto_add
    free_host_ip = lookup_free_host(options)
    free_host = free_host_ip.split()[0]
    free_ip = free_host_ip.split()[1]
    mac_address = generate_virtualbox_macaddress()
    if options.verbose:
        print ">>> add host: %s/%s/%s/%s" % (free_host,
                                            free_ip,
                                            mac_address,
                                            root)
    add_machine(free_host, mac_address, free_ip, root, options.db)
    if options.verbose:
        print ">>> update dhcp.conf"
    options.overwrite_dhcp = True # force to be True
    generate_dhcp(options, options.db)
    if options.verbose:
        print ">>> update pxelinux.cfg/"
    options.generate_pxe_config_files = True # force to be True
    generate_pxe_config_files(options)
    
    
def main():
    options = parse_options()
    if options.web:
        run_web(options)
    else:
        if options.add:
            add_machine(options.add[0], options.add[2],
                        options.add[1], options.add[3], options.db)
        elif options.delete:
            delete_machine(options.delete, options.db)
        elif options.generate_dhcp:
            generate_dhcp(options,
                          options.db)
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
            generate_pxe_config_files(options)
        elif options.generate_virtualbox_image:
            generate_virtualbox_image(options)
        elif options.generate_virtualbox_macaddress:
            print_virtualbox_macaddress()
        elif options.lookup_free_host:
            print_lookup_free_host(options)
        elif options.auto_add:
            auto_add_vm(options)
            
if __name__ == "__main__":
    main()

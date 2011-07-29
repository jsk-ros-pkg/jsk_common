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
      <RemoteDisplay enabled="true" authType="Null" authTimeout="5000" allowMultiConnection="true">
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
        <Adapter slot="0" enabled="true" MACAddress="${macaddress}" cable="true" speed="0" type="82545EM">
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
CONF_SWAPFILE=/var/swap-\`ifconfig eth0 | grep HWaddr | sed 's/.*HWaddr //g' | sed 's/ //g'\`
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

case "\$1" in

  start)
    /bin/echo "Starting \${NAME} swapfile setup ..."

    # (re-)size/-generate (and also first time install)
    # this will produce output, so no -n in above echo
    /usr/local/sbin/pxe-dphys-swapfile setup

    # as S35mountall.sh has already run, do this from here
    #   as there can be no swapon in /etc/fstab
    /usr/local/sbin/pxe-dphys-swapfile swapon

    /bin/echo "done."
    ;;


  stop|default-stop)
    /bin/echo -n "Stopping \${NAME} swapfile setup ..."

    # as no swapon or swapoff in /etc/fstab, do this from here
    /usr/local/sbin/pxe-dphys-swapfile swapoff

    /bin/echo ", done."
    ;;


  restart|reload|force-reload)
    /bin/echo "No daemon to (force-)re[start|load] in \${NAME}"
    ;;


 *)
    /bin/echo "Usage: \$0 {start|stop}"

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
if [ -f /etc/"\${PNAME}" ] ; then
  . /etc/"\${PNAME}"
fi


case "\$1" in

  setup)
    # (re-)size/-generate, fast if no memory size change

    if [ "\${CONF_SWAPSIZE}" = "" ] ; then
      # compute automatic optimal size
      echo -n "computing size, "
      # this seems to be the nearest to physical RAM size, lacks about 60k
      KCORESIZE="\`ls -l /proc/kcore | awk '{ print \$5 }'\`"
      # make MBytes which rounded down will be exactly 1 too few, so add 1
      MEMSIZE="\`expr "\${KCORESIZE}" / 1048576 + 1\`"
      # default, without config file overwriding, swap=2*RAM
      CONF_SWAPSIZE="\`expr "\${MEMSIZE}" '*' "\${SWAPFACTOR}"\`"
    fi

    # announce end resulting config
    echo -n "want \${CONF_SWAPFILE}=\${CONF_SWAPSIZE}MByte"


    # we will be later starting, and in between possible deleting/rebuilding
    #   so deactivate any allready running swapfile, to avoid errors
    "\$0" swapoff



    # compare existing swapfile (if one exists) to see if it needs replacing
    if [ -f "\${CONF_SWAPFILE}" ] ; then

      echo -n ", checking existing"

      # we need bytes for comparing with existing swap file
      SWAPBYTES="\`expr "\${CONF_SWAPSIZE}" '*' 1048576\`"

      FILEBYTES="\`ls -l "\${CONF_SWAPFILE}" | awk '{ print \$5 }'\`"

      # wrong size, get rid of existing swapfile, after remake
      if [ "\${FILEBYTES}" != "\${SWAPBYTES}" ] ; then

        # updates to this section need duplicating in postrm script
        #   can not simply make subroutine here and call that from postrm
        #     as this script is deleted before  postrm purge  is called

        echo -n ": deleting wrong size file (\${FILEBYTES})"

        # deactivate and delete existing file, before remaking for new size
        "\$0" uninstall

      else

        echo -n ": keeping it"

      fi
    fi

    # if no swapfile (or possibly old one got deleted) make one
    if [ ! -f "\${CONF_SWAPFILE}" ] ; then

      echo -n ", generating swapfile ..."

      # first deleting existing mount lines, if any there (same code as above)
      grep -v "^\${CONF_SWAPFILE}" /etc/fstab > /etc/.fstab
      mv /etc/.fstab /etc/fstab

      dd if=/dev/zero of="\${CONF_SWAPFILE}" bs=1048576 \
        count="\${CONF_SWAPSIZE}" 2> /dev/null
      mkswap "\${CONF_SWAPFILE}" > /dev/null

      # ensure that only root can read possibly critical stuff going in here
      chmod 600 "\${CONF_SWAPFILE}"

      # do not mount swapfile via fstab, because S35mountall.sh is already done
      #   so just add warning comment line that swapfile is not in fstab
      #     and so gets mounted by this script
      # get rid of possibly already existing comment about
      #   swapfile mounted by this script
      grep -v "^# a swapfile" /etc/fstab > /etc/.fstab
      grep -v "\${NAME}" /etc/.fstab > /etc/fstab
      # add new comment about this
      echo "# a swapfile is not a swap partition, so no using swapon|off" \
        "from here on, use  \${NAME} swap[on|off]  for that" >> /etc/fstab

      # and inform the user what we did
      echo -n " of \${CONF_SWAPSIZE}MBytes"

    fi

    echo

    ;;


  install)
    # synonym for setup, in case someone types this
    "\$0" setup

    ;;


  swapon)
    # as there can be no swapon in /etc/fstab, do it from here
    #   this is due to no possible insertion of code (at least in Debian)
    #     between mounting of /var (where swap file most likely resides)
    #     and executing swapon, where the file already needs to be existing

    if [ -f "\${CONF_SWAPFILE}" ] ; then
      losetup /dev/loop0 \${CONF_SWAPFILE}
      swapon /dev/loop0 2>&1 > /dev/null
    else
      echo "\$0: ERROR: swap file \${CONF_SWAPFILE} missing!" \
          "you need to first run  \$0 setup  to generate one"
    fi

    ;;


  swapoff)
    # as there can also be no swapoff in /etc/fstab, do it from here

    # first test if swap is even active, else error from swapoff
    if [ "\`swapon -s | grep /dev/loop0 | \
        cut -f1 -d\  \`" != "" ] ; then
      swapoff /dev/loop0 2>&1 > /dev/null
    fi

    ;;


  uninstall)
    # note: there is no install), as setup) can run from any blank system
    #   it auto-installs as side effect of recomputing and checking size

    # deactivate before deleting
    "\$0" swapoff

    if [ -f "\${CONF_SWAPFILE}" ] ; then
      # reclaim the file space
      rm "\${CONF_SWAPFILE}"
    fi

    # and get rid of comment about swapfile mounting
    grep -v "^# a swapfile" /etc/fstab > /etc/.fstab
    grep -v "\${NAME}" /etc/.fstab > /etc/fstab

    ;;


 *)
    echo "Usage: \$0 {setup|swapon|swapoff|uninstall}"

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
select * from hosts ORDER BY hostname;
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

ERROR_HTML_TMPL = """
<html>
<!DOCTYPE html PUBLIC "-//W3C//DTD XHTML 1.0 Transitional//EN"
 "http://www.w3.org/TR/xhtml1/DTD/xhtml1-transitional.dtd">
<html xmlns="http://www.w3.org/1999/xhtml" xml:lang="ja" lang="ja">
  <head>
    <META HTTP-EQUIV="Refresh" CONTENT="5; URL=%s" />
    <meta http-equiv="Content-Type" content="text/html;charset=UTF-8" />
    <meta http-equiv="Content-Script-Type" content="text/javascript" />
    <title>PXE Manager</title>
  </head>
<body>
<h1> ERROR! </h1>
<p> %s </p>
</body>
</html>
"""

SUCCESS_HTML_TMPL = """
<html>
<!DOCTYPE html PUBLIC "-//W3C//DTD XHTML 1.0 Transitional//EN"
 "http://www.w3.org/TR/xhtml1/DTD/xhtml1-transitional.dtd">
<html xmlns="http://www.w3.org/1999/xhtml" xml:lang="ja" lang="ja">
  <head>
    <META HTTP-EQUIV="Refresh" CONTENT="5; URL=%s" />
    <meta http-equiv="Content-Type" content="text/html;charset=UTF-8" />
    <meta http-equiv="Content-Script-Type" content="text/javascript" />
    <title>PXE Manager</title>
  </head>
<body>
<h1> SUCCESS SUBMIT! </h1>
</body>
</html>
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
  </head>
  <body>
    <div id="main">
      <div id="title">
        <h1>
          PXE Boot manager
        </h1>
      </div> <!-- #title -->
      <div id="add">
       <p>
       <form method="get" action="add">
         hostname
         <input type="text" name="hostname"/>
         IP
         <input type="text" name="ip"/>
         MACAddress
         <input type="text" name="macaddress"/>
         root directory
         <input type="text" name="root"/>
         <input type="submit" value="add host"/>
       </form>
       </p>
      </div> <!-- #add -->
<div id="auto_add">
<p>
<form method="get" action="auto_add">
root directory
<input type="text" name="root"/>
<input type="submit" value="add auto host"/>
</form>
</p>
</div> <!-- #auto_add -->
      <div id="host_table">
        <table border="1">
          <caption> hosts </caption>
            <tr>
              <th>hostname</th>
              <th>IP</th>
              <th>MAC</th>
              <th>ROOT directory</th>
              <th>delete</th>
              <th>boot</th>
            </tr>
          ${hosts}
        </table>
      </div> <!-- #host_table -->
<div id="log">
<h1>log</h1>
${log}
</div> <!-- #log -->
    </div> <!-- #main -->
  </body>
</html>
"""

HTML_ALIVE_HOST_TMPL = """
<tr>
<td style="background-color: #99cc00"> ${hostname} </td>
<td> ${ip} </td>
<td> ${macaddress} </td>
<td> ${root} </td>
<td>
  <form method="get" action="delete">
   <input type="submit" value="delete" name="${hostname}"/>
  </form>
</td>
<td>
booted
</td>
</tr>
"""

HTML_DEAD_HOST_TMPL = """
<tr>
<td style="background-color: #00cc99"> ${hostname} </td>
<td> ${ip} </td>
<td> ${macaddress} </td>
<td> ${root} </td>
<td>
  <form method="get" action="delete">
   <input type="submit" value="delete" name="${hostname}"/>
  </form>
</td>
<td>
  <form method="get" action="boot">
physical machine
<input type="text" name="physical"/>
<input type="hidden" value="${hostname}" name="vmname"/>
<input type="submit" value="boot"/>
  </form>
</td>
</tr>
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

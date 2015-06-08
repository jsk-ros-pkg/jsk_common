^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package jsk_network_tools
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* [jsk_network_tools] add bandwidth/rate checker for lowspeed
* [jsk_network_tools] add pub_rate param for test
* [jsk_network_tools] use service request instead of reloading param to set send_rate
* [jsk_network_tools] use float to avoid zero-division error instead of int
* [jsk_network_tools] add SetSendRate.srv
* [jsk_network_tools/silverhammer_lowspeed_streamer.py] add service for setting send_rate
* [jsk_network_tools] bugfix: fix rmem_max value; set bash option to check command executed successfly
* [jsk_network_tools] Use loginfo instead of logwarn in silverhammre
  highspeed receiver
* [jsk_network_tools] Update for synchronized topic
* [jsk_network_tools] Add jaxon_red to joint-states-compressor.l
* Merge pull request `#948 <https://github.com/jsk-ros-pkg/jsk_common/issues/948>`_ from garaemon/synchronize-timestamp
  [jsk_network_tools] Synchronize timestamp
* [jsk_network_tools] Synchronize timestamp
* Merge pull request `#947 <https://github.com/jsk-ros-pkg/jsk_common/issues/947>`_ from furushchev/ensure-kill-multiprocess
  [jsk_network_tools/silverhammer_highspeed_receiver.py] ensure terminate multiprocess
* [jsk_network_tools/silverhammer_highspeed_receiver.py] ensure terminate multiprocess
* [jsk_network_tools] Update timestamp in highspeed receiver if specified
* [jsk_network_tools] publish time information from silverhammer streamers
* [jsk_network_tools] check/fix udp buffer size when launches silverhammer_highspeed_receiver.py
* [jsk_network_tools] add shell script to expand udp receive buffer
* Contributors: Yuki Furuta, Ryohei Ueda

1.0.71 (2015-05-17)
-------------------
* [jsk_network_tools] use multiprocess on silverhammer receiver
* [jsk_network_tools/silverhammer_lowspeed_receiver] Force to disable timeout
* [jsk_network_tools] add wireshark plugin for silverhammer udp protocol
* Contributors: Yuki Furuta, Ryohei Ueda

1.0.70 (2015-05-08)
-------------------
* [jsk_network_tools] Fix bytes/bits conversions
* Contributors: Ryohei Ueda

1.0.69 (2015-05-05)
-------------------

1.0.68 (2015-05-05)
-------------------
* [jsk_network_tools] Use 1500*8 bits as default packet_size for MTU:=1500
  environment in silverhammer_highspeed communication
* Contributors: Ryohei Ueda

1.0.67 (2015-05-03)
-------------------
* [angle-vector-compress.l] 360-mode input of 0 will return 0
* [angle-vector-compress.l] add debug code (but commented out for now)
* [jsk_network_tools] Use ~robot parameter and it's initialized to ROBOT
  environment variable
* [jsk_network_tools/test/launch_joint_state_compressor.xml] set ROBOT environment for test (and this should be removed), see https://github.com/jsk-ros-pkg/jsk_common/commit/39089ecfc793ac655d45552545ddc13c1fe87b09#commitcomment-10899961
* load environment variable for setting robot in joint-state-compressor.l
* [jsk_network_tools] add test for angle-vector/JointStates compress
* [jsk_network_tools] Including pr2_description/upload_pr2.launch in order
  to set /robot_description
* [jsk_network_tools] Support jaxon in compressing/decompressing angle-vector
* Contributors: Yuki Furuta, Kei Okada, MasakiMurooka, Ryohei Ueda

1.0.66 (2015-04-03)
-------------------

1.0.65 (2015-04-02)
-------------------
* [jsk_network_tools] Support effort in joint state compressor/decompressor
* [jsk_network_tools] Latch output topic of highspeed receiver
* [jsk_network_tools] More readable warning about packet miss
* [jsk_network_tools] Add new parameter ~packet_sleep_sum not to sleep per one packet but several packets
* Contributors: Ryohei Ueda

1.0.64 (2015-03-29)
-------------------
* [jsk_network_tools] Fix typos
* [jsk_network_tools] Add dynamic_reconfigure interface to specify
  bandwidth of highspeed streamer
* [jsk_network_tools] Defaults to 280 Mbps
* [jsk_network_tools] Decide interval between sending packets based on bandwidth
* [jsk_network_tools] Do not load unused robot models when
  compress/decompress joint_states
* [jsk_network_tools] Publish the last received time as std_msgs/Time from silverhammer receivers
* [jsk_network_tools] Force to be within 0-255 when compressing joint angles
* [jsk_network_tools] Add diagnostics information to lowspeed streamer and receiver
* [jsk_network_tools] Add diagnostics information to highspeed streamer/receiver
* [jsk_network_tools] Add event_driven mode to lowspeed streamer
* [jsk_network_tools] Add event-driven mode to lowspeed streamer
* Contributors: Ryohei Ueda

1.0.63 (2015-02-19)
-------------------

1.0.62 (2015-02-17)
-------------------
* [jsk_network_tools] Add ~packet_interval to highspeed streamer to avoid
  consuming too much bandwidth
* [jsk_network_tools] latch output of joint-state-decompressor.l
* [jsk_network_tools] Support messages which has longer joints than robot model
* [jsk_network_tools] Publish the last time to send/receive messages
* Contributors: Ryohei Ueda

1.0.61 (2015-02-11)
-------------------
* [jsk_network_tools] Enable unit test
* [jsk_network_tools] Add unittest about ROS<-->UDP message conversion
* [jsk_network_tools] Fix for uint32 data
* [jsk_network_tools] Fix how to resolve uint8 array
* [jsk_network_tools] Update sample of joint states compressor
* [jsk_network_tools] Fix compressing joint-angles of infinite joint
* Contributors: Ryohei Ueda

1.0.60 (2015-02-03)
-------------------

1.0.59 (2015-02-03)
-------------------
* [jsk_network_tools] Add euslisp script to compress/decompres joint
  states. Originally implemented in jsk_pr2_startup by Y.Furuta
* [jsk_topic_tools] Add pesimistic mode for highspeed receiver
* add param to set rate
* [jsk_network_tools] Support run silverhammer_highspeed_receiver.py
  without topic_prefix
* [jsk_network_tools] Add script to check size in lowspeed network
* [jsk_network_tools] Add openni2 sample for highspeed streaming using
  silverhammer toolkit and recover message if possible of missing packets
* [jsk_network_tools] use png images for documentation
* [jsk_network_tools] Simpler implementation of lowspeed communication and
  update document. Bang Bang!
* [jsk_network_tools] Add documentation about limited network communication
* [jsk_network_tools] Script for DRC-highspeed-link communication
* [jsk_network_tools] Fix typo: OSC -> OCS
* [jsk_network_tools] for Low-bandwidth environment, add silverhammer
  toolset.
  You can communicate between two ROS-neworks over low-bandwidth network
  like DRC final.
* Contributors: Ryohei Ueda, Yusuke Furuta

1.0.58 (2015-01-07)
-------------------

1.0.57 (2014-12-23)
-------------------

1.0.56 (2014-12-17)
-------------------
* plot multiple lines
* add network plot
* Contributors: Yusuke Furuta

1.0.55 (2014-12-09)
-------------------
* fix msg error in heartbeat
* add description
* add parameter to set hz
* Contributors: Yusuke Furuta

1.0.54 (2014-11-15)
-------------------

1.0.53 (2014-11-01)
-------------------

1.0.52 (2014-10-23)
-------------------

1.0.51 (2014-10-20)
-------------------

1.0.50 (2014-10-20)
-------------------

1.0.49 (2014-10-13)
-------------------

1.0.48 (2014-10-12)
-------------------

1.0.47 (2014-10-08)
-------------------
* Contributors: Yusuke Furuta

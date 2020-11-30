# network_status.py

Publish network status

```bash
$ rostopic list
/ecublens/docker0/receive
/ecublens/docker0/receive_kbps
/ecublens/docker0/receive_mbps
/ecublens/docker0/transmit
/ecublens/docker0/transmit_kbps
/ecublens/docker0/transmit_mbps
/ecublens/eno1/receive
/ecublens/eno1/receive_kbps
/ecublens/eno1/receive_mbps
/ecublens/eno1/transmit
/ecublens/eno1/transmit_kbps
/ecublens/eno1/transmit_mbps
/ecublens/eno2/receive
/ecublens/eno2/receive_kbps
/ecublens/eno2/receive_mbps
/ecublens/eno2/transmit
/ecublens/eno2/transmit_kbps
/ecublens/eno2/transmit_mbps
/ecublens/lo/receive
/ecublens/lo/receive_kbps
/ecublens/lo/receive_mbps
/ecublens/lo/transmit
/ecublens/lo/transmit_kbps
/ecublens/lo/transmit_mbps
/ecublens/nonlocal/receive
/ecublens/nonlocal/receive_kbps
/ecublens/nonlocal/receive_mbps
/ecublens/nonlocal/transmit
/ecublens/nonlocal/transmit_kbps
/ecublens/nonlocal/transmit_mbps
```

## Publishing Topics

* `/<host name>/<interface name>/receive`

  Amount of receiving data by the interface in bps

* `/<host name>/<interface name>/receive_kbps`

  Amount of receiving data by the interface in Kbps

* `/<host name>/<interface name>/receive_mbps`

  Amount of receiving data by the interface in Mbps

* `/<host name>/<interface name>/transmit`

  Amount of transmitting data by the interface in bps

* `/<host name>/<interface name>/transmit_kbps`

  Amount of transmitting data by the interface in Kbps

* `/<host name>/<interface name>/transmit_mbps`

  Amount of transmitting data by the interface in Mbps

* `/<host name>/nonlocal/receive`

  Amount of receiving data by the non-local interfaces in bps

* `/<host name>/nonlocal/receive_kbps`

  Amount of receiving data by the non-local interfaces in Kbps

* `/<host name>/nonlocal/receive_mbps`

  Amount of receiving data by the non-local interfaces in Mbps

* `/<host name>/nonlocal/transmit`

  Amount of transmitting data by the non-local interfaces in bps

* `/<host name>/nonlocal/transmit_kbps`

  Amount of transmitting data by the non-local interfaces in Kbps

* `/<host name>/nonlocal/transmit_mbps`

  Amount of transmitting data by the non-local interfaces in Mbps

## Parameters

* `~hz` (`float`, default: `10`)

  Publish frequency

* `~skip_interfaces` (`List of string`, default: `None`)

  List of skipping interface names

## Usage

```bash
rosrun jsk_network_tools network_status.py
```

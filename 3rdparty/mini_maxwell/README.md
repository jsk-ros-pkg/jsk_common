# setup minimuxwell for DRC final setting

## Maxwell Pro setting
Run following command on your maxwell pro and you will have desktop icon to launch official script.

```
wget -q https://raw.githubusercontent.com/jsk-ros-pkg/jsk_common/master/3rdparty/mini_maxwell/scripts/download_maxwell_pro_scripts.sh -O - | bash
```

## Set up Filter Inventory
Please click 'Flter Inventory' and create twe filters:
* `drc_high_speed`

![drc_high_speed](images/minimaxwell_drc_highspeed.png)

* `drc_low_speed`

![drc_low_speed](images/minimaxwell_drc_lowspeed.png)

## Run test environment
```
rosrun mini_maxwell drc_2015_environment.py __ip:=IP_OF_MINI_MAXWELL
```

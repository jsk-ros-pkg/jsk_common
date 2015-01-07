# setup minimuxwell for DRC final setting
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

try:
    # download_data uses the ROS1-only rosbag python API; not required
    # for ros_compat / the ROS2 node scripts to import this package.
    from jsk_data.download_data import download_data
except ImportError:
    pass

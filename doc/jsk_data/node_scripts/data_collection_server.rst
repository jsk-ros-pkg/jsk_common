data_collection_server.py
=========================


What is this?
-------------

Save topics with synchronization by ``std_srvs/Trigger`` message
as ``{SAVE_DIR}/{TIMESTAMP}/{FILENAME}``.


Parameters
----------

* ``~save_dir`` (String, Default: ``~/.ros``)

  Save directory.

* ``topics`` (Required)

  This param should look like below:

.. code-block:: yaml

  topics:
    - name: /camera/rgb/image_raw
      msg_class: sensor_msgs/Image
      fname: image.png
      savetype: ColorImage
    - name: /camera/depth/image_raw
      msg_class: sensor_msgs/Image
      fname: depth.pkl
      savetype: DepthImage
..

  Currently, the supported save type is:

  - ``ColorImage``
  - ``DepthImage``


* ``params`` (Optional)

  This param should look like below:

.. code-block:: yaml

  params:
    - key: /in_hand_data_collection_main/object
      fname: label.txt
      savetype: Text
..

  Currently, the supported save type is:

  - ``Text``
  - ``YAML``

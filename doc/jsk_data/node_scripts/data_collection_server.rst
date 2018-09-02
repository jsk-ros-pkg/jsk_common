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
  - ``LabelImage``
  - ``YAML``


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

* ``method`` (String, Default: ``request``)

  - ``request``: Save data when service is called.
  - ``timer``: Save data when ``start`` service is called. Finish collecting when ``end`` service is called.
  - ``all``: Always save data.

* ``message_filters``: (Bool, Default: ``False``)

  Subscribe topics with message_filters.

* ``approximate_sync``: (Bool, Default: ``False``)

  Subscribe topic with ApproximateTimeSynchronizer in message_filters.

* ``queue_size``: (Int, Default: ``10``)

  Queue size of message_filters.

* ``slop``: (Float, Default ``0.1``)

  Slop of ApproximateTimeSynchronizer in message_filters.

* ``timestamp_save_dir``: (Bool, Default: ``True``)

  Save data in timestamped dir.

  If you set this as ``False``, data will be saved as ``{SAVE_DIR}/{FILENAME}``.



Services
--------

* ``~save_request`` (``std_srvs/Trigger``)

  Save data when ``method`` is ``request``.

* ``~start_request`` (``std_srvs/Trigger``)

  Start saving data when ``method`` is ``timer``.

* ``~end_request`` (``std_srvs/Trigger``)

  Finish saving data when ``method`` is ``timer``.

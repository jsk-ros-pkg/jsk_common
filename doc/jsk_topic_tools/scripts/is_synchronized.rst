===============
is_synchronized
===============

.. image:: images/is_synchronized.png


What is this?
=============

Tool to check if specified topics are 'synchronized' or not.
'synchronized' means timestamps completely match.


Usage
=====

.. code-block:: bash

  rosrun jsk_topic_tools is_synchronized topic1 topic2 [topic3...]

**Example**

.. code-block:: bash

  rosrun jsk_topic_tools is_synchronized /mask_image/mask /raw_image_bgr/image_color

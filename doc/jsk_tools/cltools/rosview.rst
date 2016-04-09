rosview
=======


Usage
-----

.. code-block:: bash

  $ rosview jsk_tools CMakeLists.txt
  cmake_minimum_required(VERSION 2.8.3)
  project(jsk_tools)
  ...


This is short version of:

.. code-block:: bash

  export PAGER=less
  roscat jsk_tools CMakeLists | $PAGER

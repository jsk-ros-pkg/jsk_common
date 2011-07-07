Setting up VirtualBox with PXE Boot
-----------------------------------

1. Install VirtualBox:

.. code-block:: bash

  wget -q  http://download.virtualbox.org/virtualbox/debian/oracle_vbox.asc -O- | sudo apt-key add -
  echo "deb http://download.virtualbox.org/virtualbox/debian `lsb_release -cs` contrib non-free" | sudo tee /etc/apt/sources.list.d/virtualbox.list
  sudo apt-get update
  sudo apt-get install virtualbox-4.0

2. Configure your OS:

.. code-block:: bash

  sudo /etc/init.d/vboxdrv setup

3. Add your user to the vboxusers group in /etc/group

4. Create a bridge ethernet setup:

.. code-block:: bash

  sudo apt-get install bridge-utils

5. Add this to /etc/network/interfaces::

  auto br0
  iface br0 inet dhcp
  bridge_ports eth0

6. Restart networking:

.. code-block:: bash

  sudo /etc/init.d/networking restart


7. Start virtualbox, create a new OS with no local hard drive. Set the Network Adapter to::

  Attached to: Bridged Adapter
  Name: br0
  Adapter Type: PCnet-PCI II
  Click on Cable connected 

Make sure to enable 3D acceleration and allocate the right amount of cpu and memory resources.

NOTE: It would be great to have a script that configures the new VM.

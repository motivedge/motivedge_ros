MotivEdge ROS Package
=====================

ROS1 package to serve download map in ROS system. The package will use
`SDK <https://github.com/motivedge/python_sdk>`_ to download map data
from MotivEdge server. Then use ROS :code:`map_server` to serve the map.

Pre-requirements
================

* Python >= 3.6
* An account at `Portal Motivedge <https://portal.motivedge.io/>`_
* ROS distributions: Melodic or Noetic.

Installation
============

.. code:: bash

   mkdir -p catkin_ws/src
   cd catkin_ws/src
   git clone https://github.com/motivedge/motivedge_ros.git
   cd motivedge_ros
   pip3 install -r requirements.txt
   cd ../..
   catkin build motivedge_ros # OR catkin_make
   source devel/setup.sh

How to use
==========

1. Get API token from `profile & token page <https://portal.motivedge.io/profile>`_

   .. code:: bash

       export ME_TOKEN=<api_token>

2. Find the target map's :code:`MAP_ID`. We could find this :code:`ID` at map details page of our portal site. It's next to the map name.

3. Launch map service client which includes downloading map and :code:`map_server` map service.

   .. code:: bash

       roslaunch motivedge_ros map_client.launch map_id:=<MAP_ID>

How it works
============

Currently, there are two nodes running in launch file. :code:`map_download.py` and
:code:`map_server`.

* :code:`map_download.py` node will perform the map downloading and save map files
  locally.
* :code:`map_server` is ROS `package <http://wiki.ros.org/map_server>`_. We pass the
  map yaml file location to it. :code:`map_server` will perform the robust map service.


Published Topics
================

* :code:`/metapoints_filepath`: :code:`std_msgs/String`, published from :code:`map_download.py`.

  It's the path of metadata yaml file which includs mark points/paths/blocks information.
  Other nodes could get the path and use :code:`motivedge.Client.read_mark_points`
  method to read the data inside. Then, clients could save data into database
  or directly publish to robot.

  For costmap generated from 3D map, there is no :code:`metadata.yaml` file.
  This topic will publish **empty string**.

* :code:`/map_metadata`: :code:`nav_msgs/MapMetaData`, published from :code:`map_server`. It includes the map
  yaml file content. Details please check :code:`map_server` page, `here <http://wiki.ros.org/map_server>`_

* :code:`/map`: :code:`nav_msgs/OccupancyGrid`, published from :code:`map_server`. It includes the map
  yaml file content. Details please check :code:`map_server` page, `here <http://wiki.ros.org/map_server>`_


Documentation
=============

Our portal site document is `here <https://docs.motivedge.io/ROS.html>`_ .

Contributing
============

We love sharing and welcome sharing and contributing. Please submit pull requests or raise issues in our repo.

License
=======

We are under Apache License 2.0 License.

@2022 MotivEdge

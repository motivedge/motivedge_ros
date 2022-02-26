#!/usr/bin/env python3
import os
import shutil
import rospy
from std_msgs.msg import String

from motivedge import Client
from errors import InvalidMapID


class ClientNode:
    """
    Script to download map using rospy params. Publish mark points metadata file path
    such that other scripts could read the data and save into database for instance.

    Parameters
    ----------
    map_id: int, the target map id on MotivEdge portal site
    saving_folder: str, where the downloaded maps will be save
    lidar_height: float, only used for generating costmap from 3D map.
        Indicate the height of lidar to the ground on robot.

    meta_fpath_hz: float/int, the node will publish mark points metadata yaml file path
        such that other nodes could get the path and use `motivedge.Client.read_mark_points`
        method to read the data inside. Then, clients could save data into database
        or directly publish to robot. Publish rate.
    meta_fpath_topic: str, the topic name of publishing yaml file path

    Exceptions
    ----------
    InvalidMapID
        Node will simply check the ID number, which has to be a `int` and larger than 0.
    TokenMissError
        Raised from `motivedge.Client`. No Token provide
    HTTPError
        When fetching the data from MotivEdge server, there is an HTTPError.

    Example
    -------

    Set :code:`ME_TOKEN` ENV variable.

    $ export ME_TOKEN=<api_token>

    Run node:

    $ rosrun motivedge_ros map_download.py _map_id:=<MAPID>

    OR run node with token inline:

    $ ME_TOKEN=<api_token> rosrun motivedge_ros map_download.py _map_id:=<MAPID>
    """

    def __init__(self):
        rospy.init_node("me_map_download", log_level=rospy.INFO)

        invalid_map_id = False
        try:
            self.map_id = int(rospy.get_param("~map_id", -1))
            invalid_map_id = self.map_id <= 0
        except ValueError:
            invalid_map_id = True
        if invalid_map_id:
            err_msg = f"Please input current map_id. `{self.map_id}` is invalid."
            rospy.logerr(err_msg)
            raise InvalidMapID(err_msg)

        self.saving_folder = rospy.get_param("saving_folder", "/tmp/motivedge_map")
        self.lidar_height = float(rospy.get_param("lidar_height", 0))
        self.client = Client()

        map_folder = self.download_map()

        rate = float(rospy.get_param("meta_fpath_hz", 1))
        meta_fpath_topic = rospy.get_param("meta_fpath_topic", "/metapoints_filepath")
        meta_fpath_pub = rospy.Publisher(meta_fpath_topic, String, queue_size=1)
        _loop = rospy.Rate(rate)

        # Currently, we only have two case:
        # 1. original costmap which could be edited with mark points/paths/blocks
        # 2. costmap is generated from 3D map
        # For the 2nd case, we don't support mark points metadata.
        # And no `metadata.yaml` in downloaded zip file.
        # We directly publish empty string for 2nd case.
        meta_fpath = os.path.join(map_folder, "metadata.yaml")
        if not os.path.exists(meta_fpath):
            meta_fpath = ""
            rospy.logwarn(f"3D map generated Costmap downloaded. No map meta points yaml file!")
        else:
            rospy.loginfo(f"Points metadata file saved at: {map_folder}")

        while not rospy.is_shutdown():
            meta_fpath_pub.publish(meta_fpath)
            _loop.sleep()

    def download_map(self):
        if os.path.exists(self.saving_folder):
            rospy.logwarn(f"Removing existing saving folder `{self.saving_folder}`")
            shutil.rmtree(self.saving_folder)

        map_folder = self.client.download_map(
            self.map_id,
            lidar_height=self.lidar_height,
            path=self.saving_folder
        )
        rospy.loginfo(f"New map is saved in {map_folder}. Enjoy!")
        return map_folder


if __name__ == "__main__":
    ClientNode()

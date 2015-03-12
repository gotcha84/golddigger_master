# golddigger_master

The master node that runs on the base station computer. Responsible for reading the video feed, running OpenCV coin detection, and publishing whether to turn on the LED+buzzer on the Dragonboard or not.

We set the base station to a static IP of 192.168.1.222.
You should edit + run init_network.sh first to initialize the ROS_MASTER_URI to the base station and ROS_IP to the computer running this node.

interactive_marker_tf2
======================

>This package connects interactive markers with tf2.

This package provides means to make interactive markers from any interactive marker server available as transforms.

Usage
-----

Manually add the package to your catkin workspace and run/install it from there.
All of its dependencies should be covered by a ROS desktop install.

The included launch files provide some basic usage examples.

Nodes
-----

### int_marker_to_tf.py

Subscribes to interactive marker topics and publishes their poses as transforms.

#### Subscribed topics

* `int_server/update_full`
([visualization_msgs/InteractiveMarkerInit](https://docs.ros.org/api/visualization_msgs/html/msg/InteractiveMarkerInit.html))
  * Initialization messages from an interactive marker server
* `int_server/update`
([visualization_msgs/InteractiveMarkerUpdate](https://docs.ros.org/api/visualization_msgs/html/msg/InteractiveMarkerUpdate.html))
  * Update messages from an interactive marker server

#### Published topics

* `/tf`
([geometry_msgs/TransformStamped](https://docs.ros.org/api/geometry_msgs/html/msg/TransformStamped.html))
  * Transforms corresponding to interactive marker poses

#### Parameters

* `~rate`
(Integer, default: 10 Hz)
  * Rate at which transforms are published
  * If 0, all transforms will only be published on receiving non-keep-alive updates
* `~prefix`
(String, default: "")
  * Prefix used for published frame_ids
* `~suffix`
(String, default: "")
  * Suffix used for published frame_ids
* `~broadcast_erased`
(Boolean, default: False)
  * Whether to continue broadcasting transforms of interactive markers that have been erased
* `~seq_num`
(Integer, default: 0)
  * Lowest expected sequence number from the interactive marker server
* `~server_id`
(String, default: None)
  * Name of the interactive marker server from which messages are accepted
  * If None, all messages are accepted on the subscribed topics

Launch Files
------------

* **basic_controls.launch**
  * Example that launches `basic_controls` from the `interactive_marker_tutorials` package along with `int_marker_to_tf.py`. 
* **pong.launch**
  * Example that launches `pong` from the `interactive_marker_tutorials` package along with `int_marker_to_tf.py`. 


You will have to launch RViz separately and set *Fixed Frame* accordingly.

Contributing
------------

Please report bugs and request features using the [Issue Tracker](https://github.com/krukai/interactive_marker_tf2/issues).
Pull requests are also welcome.

### Credits

* Kai Krückel (krukai@veemo.ink), Author

License
-------

This project is licensed under the 3-Clause BSD License.
See `LICENSE` for more information.

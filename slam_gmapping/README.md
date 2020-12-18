# slam_gmapping
http://www.ros.org/wiki/slam_gmapping

# Reset the map

On branch hydro-devel (works with kinetic as well), the map is reset just like in hector_mapping as soon as a message to "/syscommand" topic contains data: "reset"

Example

    rostopic pub -1 /syscommand std_msgs/String "data: 'reset'"

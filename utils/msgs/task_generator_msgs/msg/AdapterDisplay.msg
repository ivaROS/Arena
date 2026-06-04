# One rviz display entry contributed by an adapter.
string name              # display label
string topic             # full topic path (already substituted with robot ns)
string topic_type        # ROS msg type, empty if N/A
string rviz_class        # rviz plugin class (e.g. rviz_default_plugins/Path)
string config_json       # extra rviz config as a JSON object, empty if none
bool topic_must_exist    # skip display if topic has no publisher at manifest time

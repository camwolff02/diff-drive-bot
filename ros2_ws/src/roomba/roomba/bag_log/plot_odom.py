import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import matplotlib.pyplot as plt

def print_odometry_from_bag(bag_file: str, topic: str = "/gobilda_base_controller/odom"):
    # Open the ROS 2 bag file
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=bag_file, storage_id="sqlite3")
    converter_options = rosbag2_py.ConverterOptions()
    reader.open(storage_options, converter_options)
    
    # Get topic and message types
    topic_types = reader.get_all_topics_and_types()
    type_map = {t.name: t.type for t in topic_types}
    
    if topic not in type_map:
        print(f"Topic {topic} not found in the bag file.")
        return
    
    msg_type = get_message(type_map[topic])
    
    # Lists to store position data
    x_positions = []
    y_positions = []
    
    # Iterate through messages and print odometry data
    while reader.has_next():
        (topic_name, data, timestamp) = reader.read_next()
        if topic_name == topic:
            msg = deserialize_message(data, msg_type)
            x_positions.append(msg.pose.pose.position.x)
            y_positions.append(msg.pose.pose.position.y)
            print(f"Time: {timestamp}, Position: ({msg.pose.pose.position.x}, {msg.pose.pose.position.y}, {msg.pose.pose.position.z})")
    
    # Plot x and y positions
    plt.figure()
    plt.plot(x_positions, y_positions, marker='o', linestyle='-')
    plt.xlabel("X Position")
    plt.ylabel("Y Position")
    plt.title("Odometry Path")
    plt.grid()
    plt.show()
    plt.savefig('figure.png')

if __name__ == "__main__":
    print_odometry_from_bag("bag.db3")

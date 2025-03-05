import rosbag2_py

BAG = "rosbag2_2025_03_04-17_38_3/rosbag2_2025_03_04-17_38_38_0.db3"

def main(args=None):
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(
        uri=BAG,
        storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions('', '')
    reader.open(storage_options, converter_options)

    while reader.has_next():
        pos = reader.read_next().pose.pose.position
        print(f'[{pos.x}, {pos.y}]')

if __name__ == '__main__':
    main()
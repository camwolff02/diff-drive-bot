import sqlite3
import sys

def read_rosbag(db_file):
    # Connect to the SQLite database
    conn = sqlite3.connect(db_file)
    cursor = conn.cursor()

    # Retrieve all available topics
    cursor.execute("SELECT name, type FROM topics")
    topics = cursor.fetchall()
    
    print("Topics in the bag file:")
    for topic, msg_type in topics:
        print(f"- {topic} ({msg_type})")

    # Read messages from the database
    print("\nMessages in the bag file:")
    for topic, _ in topics:
        cursor.execute(f"SELECT timestamp, data FROM messages WHERE topic_id=(SELECT id FROM topics WHERE name='{topic}')")
        messages = cursor.fetchall()

        for timestamp, data in messages:
            print(f"[{timestamp}] {topic}: {data}")

    # Close the connection
    conn.close()

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python read_rosbag.py <path_to_rosbag.db3>")
        sys.exit(1)
    
    db_file = sys.argv[1]
    read_rosbag(db_file)

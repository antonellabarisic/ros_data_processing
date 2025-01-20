#!/usr/bin/env python3
import rosbag
import csv
import sys

def rosbag_to_csv(bag_file, output_dir):
    # Open the bag file
    bag = rosbag.Bag(bag_file)
    
    # Get all the topics and their types
    topics = bag.get_type_and_topic_info()[1].keys()

    for topic in topics:
        # Create a CSV file for each topic
        csv_file = f"{output_dir}/{topic.replace('/', '_')}.csv"
        with open(csv_file, 'w') as f:
            writer = csv.writer(f)
            
            # Write the header
            first_message = True
            for topic, msg, t in bag.read_messages(topics=topic):
                if first_message:
                    header = msg.__slots__
                    writer.writerow(['time'] + header)
                    first_message = False
                
                # Write the data
                row = [t.to_sec()] + [getattr(msg, field) for field in msg.__slots__]
                writer.writerow(row)
    
    # Close the bag file
    bag.close()
    print(f"Data from {bag_file} has been exported to {output_dir}")

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python rosbag_to_csv.py <bag_file> <output_dir>")
        sys.exit(1)
    
    bag_file = sys.argv[1]
    output_dir = sys.argv[2]
    rosbag_to_csv(bag_file, output_dir)

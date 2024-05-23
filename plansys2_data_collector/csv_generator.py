import rclpy
from rclpy.node import Node
from rosbags.rosbag2 import Reader
from rosbags.highlevel import AnyReader
# from plansys2_msgs.msg import ActionExecutionDataCollection

from pathlib import Path
import sys

import pandas as pd
# from rosidl_runtime_py.convert import get_message_slot_types, message_to_csv


class BagToCsvNode(Node):
    def __init__(self):
        super().__init__('bag_to_csv_node')

        self.declare_parameter('bag_file_path', '')
        self.declare_parameter('output_csv_path', '')

        bag_file_path = self.get_parameter('bag_file_path').get_parameter_value().string_value
        csv_file_path = self.get_parameter('output_csv_path').get_parameter_value().string_value

        if not bag_file_path or not csv_file_path:
            self.get_logger().error('Both bag_file_path and csv_file_path parameters are required.')
            sys.exit(1)

        self.convert_bag_to_csv(bag_file_path, csv_file_path)

    def convert_bag_to_csv(self, bag_file_path, csv_file_path):
        data = []
        
        with AnyReader([Path(bag_file_path)]) as reader:
            connections = [x for x in reader.connections if x.topic == '/action_execution_data_collection']
            for connection, timestamp, rawdata in reader.messages(connections=connections):
                msg = reader.deserialize(rawdata, connection.msgtype)
                # to_check = get_message_slot_types(ActionExecutionDataCollection())
                data.append({
                    'type': msg.action_execution.type,
                    'node_id': msg.action_execution.node_id,
                    'action': msg.action_execution.action,
                    'arguments': '_'.join(msg.action_execution.arguments),
                    'success': msg.action_execution.success,
                    'completion': msg.action_execution.completion,
                    'status': msg.action_execution.status,
                    'nominal_cost': msg.nominal_action_cost.nominal_cost,
                    'nominal_cost_std_dev': msg.nominal_action_cost.std_dev_cost,
                    'measured_cost': msg.measured_action_cost.nominal_cost,
                    'mesured_cost_std_dev': msg.measured_action_cost.std_dev_cost,
                    'estimated_cost': msg.estimated_action_cost.nominal_cost,
                    'estimated_cost_std': msg.estimated_action_cost.std_dev_cost,
                    'residual_cost': msg.residual_action_cost.nominal_cost,
                    'residual_cost_std': msg.residual_action_cost.std_dev_cost
                })
        print(data)
        df = pd.DataFrame(data)
        df.to_csv(csv_file_path, index=False)
        self.get_logger().info(f'CSV file created at {csv_file_path}')

def main(args=None):
    rclpy.init(args=args)
    node = BagToCsvNode()

    # rclpy.spin(node)
    # node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

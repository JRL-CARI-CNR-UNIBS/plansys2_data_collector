import rclpy
from rclpy.node import Node
from rosbags.rosbag2 import Reader
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore, get_types_from_msg
from plansys2_msgs.msg import ActionExecutionDataCollection, PlanExecutionDataCollection
from pathlib import Path
import sys
from builtin_interfaces.msg import Time
import pandas as pd
# from rosidl_runtime_py.convert import get_message_slot_types, message_to_csv
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError


class BagToCsvNode(Node):
    def __init__(self):
        super().__init__('bag_to_csv_node')

        self.declare_parameter('bag_file_path', '')
        self.declare_parameter('output_csv_path', '')
        self.declare_parameter('packages_to_register', [''])

        bag_file_path = self.get_parameter('bag_file_path').get_parameter_value().string_value
        csv_file_path = self.get_parameter('output_csv_path').get_parameter_value().string_value
        packages_to_register = self.get_parameter('packages_to_register').get_parameter_value().string_array_value
        print(packages_to_register)
        if not bag_file_path or not csv_file_path:
            self.get_logger().error('Both bag_file_path and csv_file_path parameters are required.')
            sys.exit(1)
        
        self.typestore = get_typestore(Stores.ROS2_HUMBLE)
        try:
            self.register_message_types(packages_to_register)
        except Exception as e:
            self.get_logger().error(f'Error registering message types: {e}')
            sys.exit(1)
        
        self.convert_bag_to_csv(bag_file_path, csv_file_path)

    def register_message_types(self, packages_to_register):
        for package in packages_to_register:
            try:
                package_share_directory = get_package_share_directory(package)
            except (PackageNotFoundError, ValueError) as exception:
                raise exception
            self.declare_parameter(package, 'msg')
            msgs_folder = self.get_parameter(package).get_parameter_value().string_value
            msgs_folder_path = Path(f"{package_share_directory}/{msgs_folder}") 

            for msg_file_path in msgs_folder_path.glob('*.msg'):
                msg_name = msg_file_path.stem
                msg_text = msg_file_path.read_text()
                self.typestore.register(get_types_from_msg(msg_text, f'{package}/{msgs_folder}/{msg_name}'))
                self.get_logger().info(f'Registered message type: {msg_name}')

    def convert_bag_to_csv(self, bag_file_path, csv_file_path):
        actions_data = []
        plans_data = []
        with AnyReader([Path(bag_file_path)]) as reader:

            connections = [x for x in reader.connections if x.topic == '/action_execution_data_collection']
            for connection, timestamp, rawdata in reader.messages(connections=connections):
                msg = self.typestore.deserialize_cdr(rawdata, connection.msgtype)

                actions_data.append({
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
                    'residual_cost_std': msg.residual_action_cost.std_dev_cost,
                    't_start': msg.t_start.sec + msg.t_start.nanosec * 1e-9,
                    't_end': msg.t_end.sec + msg.t_end.nanosec * 1e-9,
                })
        print(actions_data)
        df = pd.DataFrame(actions_data)
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

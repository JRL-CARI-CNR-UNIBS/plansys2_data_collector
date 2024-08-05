import rclpy
from rclpy.node import Node
import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt
import sys
import numpy as np

class CsvToGraphNode(Node):
    def __init__(self):
        super().__init__('csv_to_graph_node')

        self.declare_parameter('input_csv_path', '')
        self.declare_parameter('output_dir', '')
        self.declare_parameter('plot_uncertainties', True)

        input_csv_path = self.get_parameter('input_csv_path').get_parameter_value().string_value
        output_dir = self.get_parameter('output_dir').get_parameter_value().string_value
        plot_uncertainties = self.get_parameter('plot_uncertainties').get_parameter_value().bool_value

        if not input_csv_path or not output_dir:
            self.get_logger().error('Both input_csv_path and output_dir parameters are required.')
            sys.exit(1)

        self.plot_data_from_csv(input_csv_path, output_dir, plot_uncertainties)

    def plot_data_from_csv(self, input_csv_path, output_dir, plot_uncertainties):
        df = pd.read_csv(input_csv_path)
        # Add a column that is given by the difference between t_end and t_start columns
        df['duration'] = df['t_end'] - df['t_start']

        sns.set_theme(style="whitegrid")
        print(df.head())
        node_ids = df['node_id'].unique()
        for node_id in node_ids:
            node_df = df[df['node_id'] == node_id]
            arguments = node_df['arguments'].unique()

            for arg in arguments:
                arg_df = node_df[node_df['arguments'] == arg]
                print(arg_df.head())

                plt.figure(figsize=(10, 6))
                x_axis = np.arange(0, len(arg_df['nominal_cost']))
                sns.lineplot(data=arg_df, x=x_axis, y='nominal_cost', label='Nominal Cost', marker="o")
                sns.lineplot(data=arg_df, x=x_axis, y='measured_cost', label='Measured Cost', marker="o")
                sns.lineplot(data=arg_df, x=x_axis, y='estimated_cost', label='Estimated Cost', marker="o")
                sns.lineplot(data=arg_df, x=x_axis, y='residual_cost', label='Residual Cost', marker="o")
                sns.lineplot(data=arg_df, x=x_axis, y='residual_cost', label='Residual Cost', marker="o")
                sns.lineplot(data=arg_df, x=x_axis, y='duration', label='Measured ', marker="o")

                if plot_uncertainties:
                    plt.fill_between(x_axis,
                                     arg_df['nominal_cost'] - arg_df['nominal_cost_std_dev'],
                                     arg_df['nominal_cost'] + arg_df['nominal_cost_std_dev'], alpha=0.3)
                    # plt.fill_between(x_axis,
                    #                  arg_df['measured_cost'] - arg_df['mesured_cost_std_dev'],
                    #                  arg_df['measured_cost'] + arg_df['mesured_cost_std_dev'], alpha=0.3)
                    plt.fill_between(x_axis,
                                     arg_df['estimated_cost'] - arg_df['estimated_cost_std'],
                                     arg_df['estimated_cost'] + arg_df['estimated_cost_std'], alpha=0.3)
                    plt.fill_between(x_axis,
                                     arg_df['residual_cost'] - arg_df['residual_cost_std'],
                                     arg_df['residual_cost'] + arg_df['residual_cost_std'], alpha=0.3)

                plt.title(f'Costs for Node ID: {node_id}, Arguments: {arg}')
                plt.xlabel('Index')
                plt.ylabel('Cost')
                plt.legend()
                plt_path = f"{output_dir}/node_{node_id}_arguments_{arg}.png"
                plt.show()
                # plt.savefig(plt_path)
                # plt.close()
                # self.get_logger().info(f'Plot saved at {plt_path}')

def main(args=None):
    rclpy.init(args=args)
    node = CsvToGraphNode()

    # rclpy.spin(node)
    # node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

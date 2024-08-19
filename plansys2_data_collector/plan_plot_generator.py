import rclpy
from rclpy.node import Node
import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt
import sys
import plotly.express as px

class PlanPlotGeneratorNode(Node):
    def __init__(self):
        super().__init__('plan_plot_generator_node')

        self.declare_parameter('input_csv_path', '')
        # self.declare_parameter('output_dir', '')
        # self.declare_parameter('plot_uncertainties', True)

        input_csv_path = self.get_parameter('input_csv_path').get_parameter_value().string_value
        # output_dir = self.get_parameter('output_dir').get_parameter_value().string_value
        # plot_uncertainties = self.get_parameter('plot_uncertainties').get_parameter_value().bool_value

        if not input_csv_path: #or not output_dir:
            self.get_logger().error('Both input_csv_path and output_dir parameters are required.')
            sys.exit(1)

        self.plot_data_from_csv(input_csv_path)

    def plot_data_from_csv(self, input_csv_path, output_dir = None):
        df = pd.read_csv(input_csv_path)
        # Plot with Seaborn
        sns.set_theme()
        sns.lineplot(data=df, x=df.index, y='nominal_duration', label='Nominal Duration')
        sns.lineplot(data=df, x=df.index, y='plan_duration', label='Plan Duration')
        plt.title('Comparison betweeen Nominal Duration and Plan Duration')
        plt.xlabel('Plan index')
        plt.ylabel('Duration (s)')
        plt.show()

        # Plot with Plotly
        fig = px.line(df, x=df.index, y=['nominal_duration', 'plan_duration'], title='Comparison betweeen Nominal Duration and Plan Duration')
        fig.show()

def main(args=None):
    rclpy.init(args=args)
    node = PlanPlotGeneratorNode()

    # rclpy.spin(node)
    # node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

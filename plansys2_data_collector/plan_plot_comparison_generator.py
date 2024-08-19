import rclpy
from rclpy.node import Node
import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt
import sys
import plotly.express as px
import plotly.graph_objects as go

class PlanComparisonPlotGeneratorNode(Node):
    def __init__(self):
        super().__init__('plan_plot_comparison_generator_node')

        self.declare_parameter('main_path', '')
        self.declare_parameter('plans_to_compare', [''])

        main_path = self.get_parameter('main_path').get_parameter_value().string_value
        plans_to_compare = self.get_parameter('plans_to_compare').get_parameter_value().string_array_value

        if not main_path or not plans_to_compare: 
            self.get_logger().error('main_path and plans_to_compare parameters are required.')
            sys.exit(1)

        self.plot_data_from_csv(main_path, plans_to_compare)

    def plot_data_from_csv(self, main_path, plans_to_compare):
        combined_df = pd.DataFrame()
        for plan_execution in plans_to_compare:
            self.get_logger().info(f'Processing {main_path}{plan_execution}/exported_plan_execution_data_collection.csv')
            try:
                pd_plan_execution = pd.read_csv(f'{main_path}{plan_execution}/exported_plan_execution_data_collection.csv')
            except Exception as e:
                self.get_logger().error(f'Error reading {main_path}{plan_execution}/exported_plan_execution_data_collection.csv')
                continue
            pd_plan_execution['planner'] = plan_execution 
            combined_df = pd.concat([combined_df, pd_plan_execution], ignore_index=False)
        if combined_df.empty:
            self.get_logger().error('No data to plot')
            return

        # Plot with Seaborn
        sns.set_theme()
        sns.lineplot(data=combined_df, x=combined_df.index, y='nominal_duration',hue='planner')
        sns.lineplot(data=combined_df, x=combined_df.index, y='plan_duration', linestyle='--', hue='planner', legend=None)
        plt.title('Comparison betweeen Nominal Duration and Plan Duration')
        plt.xlabel('Plan index')
        plt.ylabel('Duration (s)')
        plt.legend(title='Planner')

        plt.show()

        fig = go.Figure()
        palette = px.colors.qualitative.Plotly  # or any other Plotly color palette
        color_map = {planner: palette[i % len(palette)] for i, planner in enumerate(combined_df['planner'].unique())}

        for plan_execution in plans_to_compare:
            df_plan = combined_df[combined_df['planner'] == plan_execution]
            # fig.add_trace(go.Scatter(x=df_plan.index, y=df_plan['nominal_duration'], mode='lines', name=f'Estimated Duration ({plan_execution})'))
            # fig.add_trace(go.Scatter(x=df_plan.index, y=df_plan['plan_duration'], mode='lines', line=dict(dash='dash'), name=f'Measured Duration ({plan_execution})'))
            color = color_map[plan_execution]
            fig.add_trace(go.Scatter(x=df_plan.index, y=df_plan['nominal_duration'], mode='lines', name=f'Estimated Duration ({plan_execution})', line=dict(color=color)))
            fig.add_trace(go.Scatter(x=df_plan.index, y=df_plan['plan_duration'], mode='lines', line=dict(dash='dash', color=color), name=f'Measured Duration ({plan_execution})'))

        fig.update_layout(title='Comparison between Nominal Duration and Plan Duration',
                        xaxis_title='Plan index',
                        yaxis_title='Duration (s)',
                        legend_title='Planner')
        fig.show()
def main(args=None):
    rclpy.init(args=args)
    node = PlanComparisonPlotGeneratorNode()

    # rclpy.spin(node)
    # node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

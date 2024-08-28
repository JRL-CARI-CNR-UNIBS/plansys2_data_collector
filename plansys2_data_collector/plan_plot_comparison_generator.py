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
        # sns.set_theme()
        # sns.lineplot(data=combined_df, x=combined_df.index, y='nominal_duration', hue='planner')
        # sns.lineplot(data=combined_df, x=combined_df.index, y='plan_duration', linestyle='--', hue='planner')
        # plt.title('Comparison betweeen Nominal Duration and Plan Duration')
        # plt.xlabel('Plan index')
        # plt.ylabel('Duration (s)')
        # plt.legend(title='Planner')
        # plt.show()

        sns.set_theme()

        handles = []
        labels = []

        palette = sns.color_palette("tab10", n_colors=combined_df['planner'].nunique())
        planner_colors = dict(zip(combined_df['planner'].unique(), palette))

        for plan_execution in plans_to_compare:
            df_filtered = combined_df[combined_df['planner'] == plan_execution]
            
            line1, = plt.plot(df_filtered.index, df_filtered['nominal_duration'], label=f'{plan_execution} Estimated', color=planner_colors[plan_execution])

            handles.append(line1)
            labels.append(f'Measured ({plan_execution})')
            
            line2, = plt.plot(df_filtered.index, df_filtered['plan_duration'], linestyle='--', label=f'Measured ({plan_execution})', color=planner_colors[plan_execution])

            handles.append(line2)
            labels.append(f'Estimated ({plan_execution})')

        plt.title('Comparison between Nominal Duration and Plan Duration')
        plt.xlabel('Plan index')
        plt.ylabel('Duration (s)')

        plt.legend(handles=handles, labels=labels, title='Planner')

        plt.show()

        num_plans = len(plans_to_compare)
        num_rows = (num_plans + 1) // 2  # Calcola il numero di righe necessario

        fig, axes = plt.subplots(nrows=num_rows, ncols=2, figsize=(15, 5*num_rows))
        axes = axes.flatten()  # Flattens the 2D array of axes to easily iterate over them

        palette = sns.color_palette("tab10", n_colors=num_plans)
        planner_colors = dict(zip(plans_to_compare, palette))

        for i, plan_execution in enumerate(plans_to_compare):
            df_filtered = combined_df[combined_df['planner'] == plan_execution]
            
            axes[i].plot(df_filtered.index, df_filtered['nominal_duration'], label='Estimated', color=planner_colors[plan_execution])
            axes[i].plot(df_filtered.index, df_filtered['plan_duration'], linestyle='--', label='Measured', color=planner_colors[plan_execution])
            
            axes[i].set_title(f'Comparison for {plan_execution}')
            axes[i].set_xlabel('Plan index')
            axes[i].set_ylabel('Duration (s)')
            axes[i].legend(title='Duration Type')

        if num_plans % 2 != 0:
            # Clear the last axis
            handles = []
            labels = []

            # Create a single plot with combined legend
            for plan_execution in plans_to_compare:
                df_filtered = combined_df[combined_df['planner'] == plan_execution]

                # Plot on the last axis and collect handles and labels
                line1, = axes[-1].plot(df_filtered.index, df_filtered['nominal_duration'], label=f'{plan_execution} Estimated', color=planner_colors[plan_execution])
                line2, = axes[-1].plot(df_filtered.index, df_filtered['plan_duration'], linestyle='--', label=f'(Measured) ({plan_execution})', color=planner_colors[plan_execution])

                handles.append(line1)
                handles.append(line2)
                labels.append(f'{plan_execution} (Measured)')
                labels.append(f'Estimated ({plan_execution})')

            axes[-1].set_title('Comparison between Nominal Duration and Plan Duration')
            axes[-1].set_xlabel('Plan index')
            axes[-1].set_ylabel('Duration (s)')

            # Use the collected handles and labels for the legend
            axes[-1].legend(handles=handles, labels=labels, title='Planner', loc='lower center', ncol=3)



        plt.tight_layout()
        plt.show()

            
        fig = go.Figure()
        palette = px.colors.qualitative.Plotly  # or any other Plotly color palette
        color_map = {planner: palette[i % len(palette)] for i, planner in enumerate(combined_df['planner'].unique())}

        for plan_execution in plans_to_compare:
            df_plan = combined_df[combined_df['planner'] == plan_execution]
            color = color_map[plan_execution]
            fig.add_trace(go.Scatter(x=df_plan.index, y=df_plan['nominal_duration'], mode='lines', name=f'{plan_execution} (Estimated)', line=dict(color=color)))
            fig.add_trace(go.Scatter(x=df_plan.index, y=df_plan['plan_duration'], mode='lines', line=dict(dash='dash', color=color), name=f'{plan_execution} (Measured)'))

        fig.update_layout(title='Comparison between Nominal Duration and Plan Duration',
                        xaxis_title='Plan index',
                        yaxis_title='Duration (s)',
                        legend_title='Planner')
        fig.show()


def main(args=None):
    rclpy.init(args=args)
    node = PlanComparisonPlotGeneratorNode()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

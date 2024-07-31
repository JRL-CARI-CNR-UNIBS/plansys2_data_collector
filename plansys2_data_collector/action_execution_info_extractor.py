from bag_to_csv.info_extractor import InfoExtractor

class ActionExecutionInfoExtractor(InfoExtractor):
    def __init__(self):
        super().__init__("plansys2_msgs/msg/ActionExecutionDataCollection")

    def extract_info_from_msg(self, msg, msg_type):
        self._check_msg_type(msg_type)
        
        return {
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
            'measured_cost_std_dev': msg.measured_action_cost.std_dev_cost,
            'estimated_cost': msg.estimated_action_cost.nominal_cost,
            'estimated_cost_std': msg.estimated_action_cost.std_dev_cost,
            'residual_cost': msg.residual_action_cost.nominal_cost,
            'residual_cost_std': msg.residual_action_cost.std_dev_cost,
            't_start': msg.t_start.sec + msg.t_start.nanosec * 1e-9,
            't_end': msg.t_end.sec + msg.t_end.nanosec * 1e-9,
        }

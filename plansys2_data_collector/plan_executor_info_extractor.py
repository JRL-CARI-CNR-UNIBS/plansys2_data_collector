from info_extractor_base import InfoExtractor

class PlanExecutionInfoExtractor(InfoExtractor):
    def __init__(self):
        super().__init__("PlanExecutionInfoExtractor")

    def extract_info_from_msg(self, msg, msg_type):
        self._check_msg_type(msg_type)
        return {
            'plan': msg.plan,
            'action_execution_info': msg.action_execution_status,
            't_start': msg.t_start,
            't_end': msg.t_end,
        }
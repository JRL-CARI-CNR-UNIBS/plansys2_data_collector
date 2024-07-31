from bag_to_csv.info_extractor import InfoExtractor

class PlanExecutionInfoExtractor(InfoExtractor):
    def __init__(self):
        super().__init__("plansys2_msgs/msg/PlanExecutionDataCollection")

    def extract_info_from_msg(self, msg, msg_type):
        self._check_msg_type(msg_type)
        
        plan = msg.plan
        plan_items = []

        for item in plan.items:
            plan_items.append({
                'time': item.time,
                'action': item.action,
                'duration': item. duration
            })
        item_t_end = [item['time']+item['duration'] for item in plan_items]
        
        plan_nominal_duration = max(item_t_end)
        plan_start = msg.t_start.sec + msg.t_start.nanosec * 1e-9  
        plan_end = msg.t_end.sec + msg.t_end.nanosec * 1e-9
        plan_duration = plan_end - plan_start

        return {
            'nominal_duration': plan_nominal_duration,
            'plan_duration': plan_duration,
        }
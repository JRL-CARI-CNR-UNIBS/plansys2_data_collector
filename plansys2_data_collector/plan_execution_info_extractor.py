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
    
# from bag_to_csv.info_extractor import InfoExtractor

# class PlanExecutionInfoExtractor(InfoExtractor):
#     def __init__(self):
#         super().__init__("plansys2_msgs/msg/PlanExecutionDataCollection")

#     def extract_info_from_msg(self, msg, msg_type):
#         self._check_msg_type(msg_type)
#         actions = []
#         for action in msg.action_execution_status:
#             print(action.status)
#             print(action.start_stamp.sec + action.start_stamp.nanosec * 1e-9)
#             print(action.status_stamp.sec + action.status_stamp.nanosec * 1e-9)
#             print(action.duration)
#             actions.append({
#                 'action': action.action,
#                 'action_full_name': action.action_full_name,
#                 # 'status': action.status,
#                 'start': action.start_stamp.sec + action.start_stamp.nanosec * 1e-9,
#                 'end': action.status_stamp.sec + action.status_stamp.nanosec * 1e-9,
#                 'duration': action.status_stamp.sec + action.status_stamp.nanosec * 1e-9 - action.start_stamp.sec - action.start_stamp.nanosec * 1e-9
#             })
        
        
#         plan = msg.plan
#         plan_items = []

#         for item in plan.items:
#             plan_items.append({
#                 'time': item.time,
#                 'action': item.action,
#                 'duration': item. duration
#             })
#         item_t_end = [item['time']+item['duration'] for item in plan_items]
        
#         plan_nominal_duration = max(item_t_end)
#         plan_start = msg.t_start.sec + msg.t_start.nanosec * 1e-9  
#         plan_end = msg.t_end.sec + msg.t_end.nanosec * 1e-9
#         plan_duration = plan_end - plan_start
#         return {
#             'nominal_duration': plan_nominal_duration,
#             'plan_duration': plan_duration,
#         }
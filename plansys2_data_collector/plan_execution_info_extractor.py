from bag_to_csv.info_extractor import InfoExtractor

class PlanExecutionInfoExtractor(InfoExtractor):
    def __init__(self):
        super().__init__("plansys2_msgs/msg/PlanExecutionDataCollection")
        self.plans = []

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
        # start-end of plan item 
        plan_start = msg.t_start.sec + msg.t_start.nanosec * 1e-9  
        plan_end = msg.t_end.sec + msg.t_end.nanosec * 1e-9
        plan_duration = plan_end - plan_start

        return {
            'nominal_duration': plan_nominal_duration,
            'plan_duration': plan_duration,
        }
    
    def analyze_data(self, msg, msg_type):
        self._check_msg_type(msg_type)
        
        plan = msg.plan
        # plan_items = []

        for item in plan.items:
            print(f'[{item.time}]: {item.action}, ({item.duration})')
            # plan_items.append({
            #     'time': item.time,
            #     'action': item.action,
            #     'duration': item. duration
            # })
            # print(sum([plan_item == item for plan_item in old_plan.items for old_plan in self.plans]))
        
        shared_actions = []
        for old_plan in self.plans:
            counts = 0
            for plan_item in old_plan.items:
                for item in plan.items:
                    if plan_item.action == item.action:
                        counts += 1
            shared_actions.append(counts)
        self.plans.append(plan)

        print(f"N action: {len(plan.items)}")
        print(f"N robot 1 action: {len([item for item in plan.items if 'robot1' in item.action])}")
        print(f"N robot 2 action: {len([item for item in plan.items if 'robot2' in item.action])}")
        print(f"Robot1 activity time: {sum([item.duration for item in plan.items if 'robot1' in item.action])}")
        print(f"Robot2 activity time: {sum([item.duration for item in plan.items if 'robot2' in item.action])}")
        print(f"Shared actions: {shared_actions}")
        print(f"Start: {msg.t_start.sec + msg.t_start.nanosec * 1e-9}")
        print(f"End: {msg.t_end.sec + msg.t_end.nanosec * 1e-9}")
        print("-----------------")

            
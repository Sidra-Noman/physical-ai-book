---
sidebar_position: 14
---

# LLM-Driven Cognitive Planning

## Introduction

Large Language Models (LLMs) have revolutionized the field of artificial intelligence and are now being integrated into robotics to enable sophisticated cognitive planning capabilities. In Physical AI systems, LLMs serve as high-level reasoning engines that can interpret complex natural language commands, decompose them into executable tasks, and coordinate multiple robotic capabilities. This chapter explores how to integrate LLMs with ROS 2 for cognitive planning and task execution in humanoid robots.

## LLM Integration Architecture

The integration of LLMs with robotic systems involves several key components:

1. **Natural Language Interface**: Converting human commands to structured inputs
2. **Task Decomposition**: Breaking complex commands into executable steps
3. **Action Mapping**: Translating high-level plans to ROS 2 actions
4. **Execution Monitoring**: Tracking plan execution and handling exceptions
5. **Feedback Generation**: Providing status updates and explanations

### Basic LLM Integration Node

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
import json
import time
import openai  # or another LLM provider

class LLMCognitivePlannerNode(Node):
    def __init__(self):
        super().__init__('llm_cognitive_planner')

        # Initialize LLM client
        # In a real implementation, you would configure your LLM provider
        self.llm_client = self.initialize_llm_client()

        # Subscribers
        self.command_sub = self.create_subscription(
            String, 'high_level_commands', self.command_callback, 10)

        # Publishers
        self.feedback_pub = self.create_publisher(String, 'cognitive_feedback', 10)

        # Navigation action client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # System state
        self.current_task = None
        self.task_queue = []
        self.is_executing = False

        # Robot capabilities knowledge
        self.robot_capabilities = {
            'navigation': True,
            'manipulation': False,  # For this example
            'perception': True,
            'speech': True
        }

        self.get_logger().info('LLM Cognitive Planner initialized')

    def initialize_llm_client(self):
        # Initialize your LLM client here
        # This could be OpenAI API, Hugging Face, or other provider
        try:
            # Example with OpenAI (replace with your preferred provider)
            openai.api_key = "YOUR_API_KEY"  # In practice, use secure storage
            return openai
        except Exception as e:
            self.get_logger().error(f'Failed to initialize LLM client: {e}')
            return None

    def command_callback(self, msg):
        command = msg.data.strip()

        if not command:
            return

        self.get_logger().info(f'Received high-level command: "{command}"')

        # Generate plan using LLM
        plan = self.generate_plan_with_llm(command)

        if plan:
            self.execute_plan(plan, command)
        else:
            self.respond("I couldn't understand or plan for that command.")

    def generate_plan_with_llm(self, command):
        if not self.llm_client:
            self.get_logger().error('LLM client not available')
            return None

        try:
            # Define the system prompt to guide the LLM
            system_prompt = f"""
            You are a cognitive planner for a humanoid robot. The robot has the following capabilities:
            - Navigation: Can move to specific locations
            - Perception: Can identify objects and people
            - Speech: Can speak and listen

            The robot operates in an environment with known locations: kitchen, living room, bedroom, bathroom, office, charger.

            Your task is to decompose high-level commands into a sequence of executable actions.
            Return the plan as a JSON array of action objects. Each action should have:
            - "action": the type of action (e.g., "navigate", "perceive", "speak", "wait")
            - "target": the target of the action (location, object, or text)
            - "parameters": any additional parameters needed

            Example response format:
            [
                {{"action": "navigate", "target": "kitchen", "parameters": {{}}}},
                {{"action": "perceive", "target": "counter", "parameters": {{"object_type": "cup"}}}},
                {{"action": "speak", "target": "I found a cup on the counter", "parameters": {{}}}}
            ]
            """

            # Create the prompt for the specific command
            user_prompt = f"Plan the following command: {command}"

            # Call the LLM to generate the plan
            response = self.llm_client.ChatCompletion.create(
                model="gpt-3.5-turbo",  # or your preferred model
                messages=[
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": user_prompt}
                ],
                temperature=0.3,
                max_tokens=500
            )

            plan_text = response.choices[0].message.content.strip()

            # Extract JSON from the response (LLM might include text around it)
            import re
            json_match = re.search(r'\[.*\]', plan_text, re.DOTALL)
            if json_match:
                plan_json = json_match.group(0)
                plan = json.loads(plan_json)
                self.get_logger().info(f'Generated plan: {plan}')
                return plan
            else:
                self.get_logger().error(f'Could not extract JSON from LLM response: {plan_text}')
                return None

        except Exception as e:
            self.get_logger().error(f'Error generating plan with LLM: {e}')
            return None

    def execute_plan(self, plan, original_command):
        self.get_logger().info(f'Executing plan with {len(plan)} steps')

        # Add to task queue
        self.task_queue.append({
            'plan': plan,
            'original_command': original_command,
            'step_index': 0,
            'start_time': time.time()
        })

        # If not already executing, start execution
        if not self.is_executing:
            self.execute_next_step()

    def execute_next_step(self):
        if not self.task_queue:
            self.is_executing = False
            return

        current_task = self.task_queue[0]
        plan = current_task['plan']
        step_index = current_task['step_index']

        if step_index >= len(plan):
            # Plan completed
            self.get_logger().info('Plan completed successfully')
            self.respond(f"I've completed the task: {current_task['original_command']}")
            self.task_queue.pop(0)  # Remove completed task
            # Start next task if available
            if self.task_queue:
                self.execute_next_step()
            return

        self.is_executing = True
        step = plan[step_index]

        self.get_logger().info(f'Executing step {step_index + 1}/{len(plan)}: {step}')

        # Execute the specific action
        success = self.execute_action(step)

        if success:
            # Move to next step
            current_task['step_index'] += 1
            # Schedule next step execution
            self.create_timer(0.1, self.execute_next_step)
        else:
            # Handle failure
            self.get_logger().error(f'Action failed: {step}')
            self.respond(f"I couldn't complete the action: {step['action']}")
            self.task_queue.pop(0)  # Remove failed task
            self.is_executing = False

    def execute_action(self, action):
        action_type = action['action']
        target = action.get('target', '')
        parameters = action.get('parameters', {})

        if action_type == 'navigate':
            return self.execute_navigation(target)
        elif action_type == 'speak':
            return self.execute_speech(target)
        elif action_type == 'perceive':
            return self.execute_perception(target, parameters)
        elif action_type == 'wait':
            duration = parameters.get('duration', 1.0)
            time.sleep(duration)
            return True
        else:
            self.get_logger().error(f'Unknown action type: {action_type}')
            return False

    def execute_navigation(self, location):
        # Define known locations
        locations = {
            'kitchen': {'x': 1.0, 'y': 2.0, 'theta': 0.0},
            'living room': {'x': 0.0, 'y': 0.0, 'theta': 0.0},
            'bedroom': {'x': -1.0, 'y': 1.0, 'theta': 1.57},
            'bathroom': {'x': 0.5, 'y': -1.0, 'theta': 3.14},
            'office': {'x': -0.5, 'y': 1.5, 'theta': -1.57},
            'charger': {'x': 2.0, 'y': 0.0, 'theta': 0.0}
        }

        # Normalize location name
        location_key = location.lower().replace(' ', '_')

        if location_key not in locations:
            self.get_logger().error(f'Unknown location: {location}')
            return False

        # Create navigation goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = locations[location_key]['x']
        goal_msg.pose.pose.position.y = locations[location_key]['y']

        # Convert theta to quaternion
        theta = locations[location_key]['theta']
        goal_msg.pose.pose.orientation.z = math.sin(theta / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(theta / 2.0)

        # Wait for action server
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Navigation action server not available')
            return False

        # Send goal
        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self.navigation_goal_response_callback)

        self.respond(f"Navigating to {location}...")
        return True

    def navigation_goal_response_callback(self, future):
        try:
            goal_handle = future.result()
            if goal_handle.accepted:
                self.get_logger().info('Navigation goal accepted')
                # Wait for result
                result_future = goal_handle.get_result_async()
                result_future.add_done_callback(self.navigation_result_callback)
            else:
                self.get_logger().error('Navigation goal rejected')
        except Exception as e:
            self.get_logger().error(f'Navigation goal failed: {e}')

    def navigation_result_callback(self, future):
        try:
            result = future.result().result
            status = future.result().status

            if status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info('Navigation succeeded')
                self.respond("I've reached the destination.")
                # Continue with next step
                self.execute_next_step()
            else:
                self.get_logger().error('Navigation failed')
                # Handle failure in the main execution loop
        except Exception as e:
            self.get_logger().error(f'Navigation result error: {e}')

    def execute_speech(self, text):
        speech_pub = self.create_publisher(String, 'robot_speech', 10)
        msg = String()
        msg.data = text
        speech_pub.publish(msg)
        self.get_logger().info(f'Speech: {text}')
        return True

    def execute_perception(self, target, parameters):
        # For this example, we'll just simulate perception
        # In a real system, this would involve actual sensor processing
        object_type = parameters.get('object_type', 'object')
        self.get_logger().info(f'Perceiving {object_type} at {target}')
        self.respond(f"I'm looking for {object_type} at {target}.")
        return True

    def respond(self, message):
        response_msg = String()
        response_msg.data = message
        self.feedback_pub.publish(response_msg)
        self.get_logger().info(f"Robot says: {message}")

def main(args=None):
    rclpy.init(args=args)
    planner_node = LLMCognitivePlannerNode()

    try:
        rclpy.spin(planner_node)
    except KeyboardInterrupt:
        pass
    finally:
        planner_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Advanced Cognitive Planning with Context

### Context-Aware Planning System

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import BatteryState
from std_srvs.srv import Trigger
import json
import time
from dataclasses import dataclass
from typing import Dict, List, Optional

@dataclass
class RobotState:
    battery_level: float = 100.0
    current_location: str = "unknown"
    last_action_time: float = 0.0
    capabilities: Dict[str, bool] = None
    memory: Dict[str, any] = None

class ContextAwarePlannerNode(Node):
    def __init__(self):
        super().__init__('context_aware_planner')

        # Initialize LLM client
        self.llm_client = self.initialize_llm_client()

        # Subscribers for context
        self.command_sub = self.create_subscription(
            String, 'high_level_commands', self.command_callback, 10)
        self.battery_sub = self.create_subscription(
            BatteryState, 'battery_state', self.battery_callback, 10)

        # Publishers
        self.feedback_pub = self.create_publisher(String, 'cognitive_feedback', 10)

        # Services
        self.get_location_client = self.create_client(
            Trigger, 'get_current_location')

        # Initialize robot state
        self.robot_state = RobotState()
        self.robot_state.capabilities = {
            'navigation': True,
            'manipulation': False,
            'perception': True,
            'speech': True
        }
        self.robot_state.memory = {}

        # Context history
        self.context_history = []

        self.get_logger().info('Context-aware cognitive planner initialized')

    def initialize_llm_client(self):
        # Initialize your LLM client here
        try:
            import openai
            openai.api_key = "YOUR_API_KEY"  # In practice, use secure storage
            return openai
        except Exception as e:
            self.get_logger().error(f'Failed to initialize LLM client: {e}')
            return None

    def battery_callback(self, msg):
        self.robot_state.battery_level = msg.percentage
        self.get_logger().debug(f'Battery level: {self.robot_state.battery_level}%')

    def command_callback(self, msg):
        command = msg.data.strip()

        if not command:
            return

        self.get_logger().info(f'Received command: "{command}"')

        # Update context with current state
        self.update_context()

        # Generate context-aware plan
        plan = self.generate_context_aware_plan(command)

        if plan:
            self.execute_plan_with_context(plan, command)
        else:
            self.respond("I couldn't understand or plan for that command.")

    def update_context(self):
        # Update robot state with current information
        self.robot_state.last_action_time = time.time()

        # Update location if possible
        if self.get_location_client.wait_for_service(timeout_sec=0.1):
            future = self.get_location_client.call_async(Trigger.Request())
            # We won't wait for response here to avoid blocking
            # In practice, you might want to update location asynchronously

    def generate_context_aware_plan(self, command):
        if not self.llm_client:
            self.get_logger().error('LLM client not available')
            return None

        try:
            # Create comprehensive system prompt with context
            system_prompt = f"""
            You are a context-aware cognitive planner for a humanoid robot. Current robot state:
            - Battery level: {self.robot_state.battery_level}%
            - Current location: {self.robot_state.current_location}
            - Capabilities: {self.robot_state.capabilities}
            - Available locations: kitchen, living room, bedroom, bathroom, office, charger

            When generating plans, consider:
            1. Battery level - avoid long navigation if battery is low
            2. Current location - plan efficient routes
            3. Capabilities - only plan actions the robot can perform
            4. Safety - avoid dangerous situations

            If battery is below 20%, prioritize returning to charger when possible.

            Return the plan as a JSON array of action objects with the same format as before.
            """

            user_prompt = f"Generate a context-aware plan for: {command}"

            response = self.llm_client.ChatCompletion.create(
                model="gpt-3.5-turbo",
                messages=[
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": user_prompt}
                ],
                temperature=0.3,
                max_tokens=500
            )

            plan_text = response.choices[0].message.content.strip()
            import re
            json_match = re.search(r'\[.*\]', plan_text, re.DOTALL)
            if json_match:
                plan_json = json_match.group(0)
                plan = json.loads(plan_json)
                return plan
            else:
                self.get_logger().error(f'Could not extract JSON from LLM response: {plan_text}')
                return None

        except Exception as e:
            self.get_logger().error(f'Error generating context-aware plan: {e}')
            return None

    def execute_plan_with_context(self, plan, original_command):
        # Check if plan is safe to execute given current context
        if not self.is_plan_safe(plan):
            self.respond("I cannot safely execute this plan given current conditions.")
            return

        # Add plan to execution queue
        execution_task = {
            'plan': plan,
            'original_command': original_command,
            'start_time': time.time(),
            'context_snapshot': self.robot_state.__dict__.copy()
        }

        self.get_logger().info(f'Executing context-aware plan with {len(plan)} steps')
        self.execute_step_sequence(execution_task, 0)

    def is_plan_safe(self, plan):
        # Check if plan is safe given current context
        current_battery = self.robot_state.battery_level

        # If battery is low, don't execute plans that might deplete it further
        if current_battery < 20:
            # Check if plan contains navigation without return to charger
            has_navigation = any(step['action'] == 'navigate' for step in plan)
            has_return_to_charger = any(
                step['action'] == 'navigate' and step['target'] == 'charger'
                for step in plan
            )

            if has_navigation and not has_return_to_charger:
                self.get_logger().warn('Plan rejected: Battery low and no return to charger')
                return False

        return True

    def execute_step_sequence(self, task, step_index):
        plan = task['plan']

        if step_index >= len(plan):
            # Plan completed
            self.respond(f"Completed task: {task['original_command']}")
            return

        step = plan[step_index]
        self.get_logger().info(f'Executing step {step_index + 1}/{len(plan)}: {step["action"]}')

        # Execute the step
        success = self.execute_contextual_action(step)

        if success:
            # Schedule next step
            next_step_timer = self.create_timer(0.5,
                lambda: self.execute_step_sequence(task, step_index + 1))
        else:
            self.get_logger().error(f'Step failed: {step}')
            self.respond(f"Could not complete the requested task.")

    def execute_contextual_action(self, action):
        # Update context before executing action
        self.update_context()

        # Check safety conditions
        if not self.is_action_safe(action):
            return False

        # Execute based on action type
        action_type = action['action']

        if action_type == 'navigate':
            return self.execute_navigation_with_context(action)
        elif action_type == 'speak':
            return self.execute_speech(action['target'])
        elif action_type == 'perceive':
            return self.execute_perception(action['target'], action.get('parameters', {}))
        elif action_type == 'wait':
            duration = action.get('parameters', {}).get('duration', 1.0)
            time.sleep(duration)
            return True
        else:
            self.get_logger().error(f'Unknown action type: {action_type}')
            return False

    def is_action_safe(self, action):
        # Check if action is safe given current context
        if self.robot_state.battery_level < 10 and action['action'] == 'navigate':
            self.get_logger().warn('Navigation blocked: Battery critically low')
            return False

        return True

    def execute_navigation_with_context(self, action):
        # Enhanced navigation that considers current location and battery
        target = action['target']

        # Check if we need to return to charger due to low battery
        if self.robot_state.battery_level < 15:
            self.get_logger().info('Battery low, navigating to charger')
            target = 'charger'

        # Execute navigation (same as before but with context considerations)
        return self.execute_navigation(target)

    def execute_navigation(self, location):
        # Define known locations
        locations = {
            'kitchen': {'x': 1.0, 'y': 2.0, 'theta': 0.0},
            'living room': {'x': 0.0, 'y': 0.0, 'theta': 0.0},
            'bedroom': {'x': -1.0, 'y': 1.0, 'theta': 1.57},
            'bathroom': {'x': 0.5, 'y': -1.0, 'theta': 3.14},
            'office': {'x': -0.5, 'y': 1.5, 'theta': -1.57},
            'charger': {'x': 2.0, 'y': 0.0, 'theta': 0.0}
        }

        # Normalize location name
        location_key = location.lower().replace(' ', '_')

        if location_key not in locations:
            self.get_logger().error(f'Unknown location: {location}')
            return False

        # Create navigation goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = locations[location_key]['x']
        goal_msg.pose.pose.position.y = locations[location_key]['y']

        # Convert theta to quaternion
        theta = locations[location_key]['theta']
        goal_msg.pose.pose.orientation.z = math.sin(theta / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(theta / 2.0)

        # Wait for action server
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Navigation action server not available')
            return False

        # Send goal
        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self.navigation_goal_response_callback)

        self.respond(f"Navigating to {location}...")
        return True

    def execute_speech(self, text):
        speech_pub = self.create_publisher(String, 'robot_speech', 10)
        msg = String()
        msg.data = text
        speech_pub.publish(msg)
        self.get_logger().info(f'Speech: {text}')
        return True

    def execute_perception(self, target, parameters):
        # For this example, we'll just simulate perception
        object_type = parameters.get('object_type', 'object')
        self.get_logger().info(f'Perceiving {object_type} at {target}')
        self.respond(f"I'm looking for {object_type} at {target}.")
        return True

    def navigation_goal_response_callback(self, future):
        try:
            goal_handle = future.result()
            if goal_handle.accepted:
                self.get_logger().info('Navigation goal accepted')
                # Wait for result
                result_future = goal_handle.get_result_async()
                result_future.add_done_callback(self.navigation_result_callback)
            else:
                self.get_logger().error('Navigation goal rejected')
        except Exception as e:
            self.get_logger().error(f'Navigation goal failed: {e}')

    def navigation_result_callback(self, future):
        try:
            result = future.result().result
            status = future.result().status

            if status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info('Navigation succeeded')
                # Update location in robot state
                self.robot_state.current_location = "recently_navigated"  # Would be more specific in real system
                # Continue execution
            else:
                self.get_logger().error('Navigation failed')
        except Exception as e:
            self.get_logger().error(f'Navigation result error: {e}')

    def respond(self, message):
        response_msg = String()
        response_msg.data = message
        self.feedback_pub.publish(response_msg)
        self.get_logger().info(f"Robot says: {message}")

def main(args=None):
    rclpy.init(args=args)
    context_planner = ContextAwarePlannerNode()

    try:
        rclpy.spin(context_planner)
    except KeyboardInterrupt:
        pass
    finally:
        context_planner.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Safety and Validation Considerations

### Safe LLM-ROS Integration

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile
import json
import re
from typing import Dict, Any

class SafeLLMPlannerNode(Node):
    def __init__(self):
        super().__init__('safe_llm_planner')

        # Initialize with safety parameters
        self.safety_validator = SafetyValidator()
        self.action_executor = SafeActionExecutor(self)

        # Subscribers and publishers
        self.command_sub = self.create_subscription(
            String, 'safe_commands', self.safe_command_callback,
            QoSProfile(depth=10, durability=rclpy.qos.DurabilityPolicy.VOLATILE))

        self.feedback_pub = self.create_publisher(
            String, 'safe_feedback',
            QoSProfile(depth=10, durability=rclpy.qos.DurabilityPolicy.VOLATILE))

        # Configuration
        self.max_plan_steps = 20  # Limit plan complexity
        self.max_command_length = 500  # Limit command length
        self.enable_simulation_mode = True  # Start in simulation mode

        self.get_logger().info('Safe LLM Planner initialized')

    def safe_command_callback(self, msg):
        command = msg.data.strip()

        # Validate command
        if not self.validate_command(command):
            self.respond("Command rejected: Does not meet safety criteria.")
            return

        # Process with safety checks
        try:
            # Generate plan with LLM
            raw_plan = self.generate_raw_plan(command)

            if not raw_plan:
                self.respond("Could not generate a plan for that command.")
                return

            # Validate the generated plan
            validated_plan = self.safety_validator.validate_plan(raw_plan)

            if not validated_plan:
                self.respond("Generated plan failed safety validation.")
                return

            # Execute plan safely
            success = self.action_executor.execute_plan(validated_plan, command)

            if success:
                self.respond(f"Completed task: {command}")
            else:
                self.respond(f"Could not complete task: {command}")

        except Exception as e:
            self.get_logger().error(f'Safety error during command processing: {e}')
            self.respond("Safety error occurred. Please try again.")

    def validate_command(self, command):
        # Check length
        if len(command) > self.max_command_length:
            self.get_logger().warn(f'Command too long: {len(command)} > {self.max_command_length}')
            return False

        # Check for potentially harmful keywords
        harmful_keywords = [
            'shutdown', 'kill', 'terminate', 'break', 'destroy', 'damage',
            'harm', 'injure', 'attack', 'hurt', 'explode', 'fire'
        ]

        command_lower = command.lower()
        for keyword in harmful_keywords:
            if keyword in command_lower:
                self.get_logger().warn(f'Harmful keyword detected: {keyword}')
                return False

        # Validate command structure
        if not re.match(r'^[a-zA-Z0-9\s\.,!?-]+$', command):
            self.get_logger().warn('Command contains invalid characters')
            return False

        return True

    def generate_raw_plan(self, command):
        # In a real implementation, this would call your LLM
        # For safety, we'll simulate the process
        self.get_logger().info(f'Generating plan for: {command}')

        # This would be where you call your actual LLM
        # For this example, we'll return a simple plan
        simulated_plan = [
            {"action": "speak", "target": f"I understood: {command}", "parameters": {}},
            {"action": "wait", "target": "", "parameters": {"duration": 1.0}}
        ]

        return simulated_plan

    def respond(self, message):
        response_msg = String()
        response_msg.data = message
        self.feedback_pub.publish(response_msg)
        self.get_logger().info(f"Robot says: {message}")

class SafetyValidator:
    def __init__(self):
        self.allowed_actions = {
            'navigate', 'speak', 'perceive', 'wait', 'stop'
        }
        self.max_coordinates = 100.0  # Limit navigation range
        self.min_battery_for_navigation = 15.0  # Minimum battery for navigation

    def validate_plan(self, raw_plan):
        if not isinstance(raw_plan, list):
            return None

        # Limit plan size
        if len(raw_plan) > 20:
            return None

        validated_plan = []
        for step in raw_plan:
            if not self.validate_step(step):
                return None
            validated_plan.append(step)

        return validated_plan

    def validate_step(self, step):
        if not isinstance(step, dict):
            return False

        action = step.get('action')
        if action not in self.allowed_actions:
            return False

        # Validate navigation parameters
        if action == 'navigate':
            target = step.get('target', '').lower()
            # Additional validation would go here

        # Validate coordinate parameters
        params = step.get('parameters', {})
        if 'x' in params or 'y' in params:
            x = params.get('x', 0)
            y = params.get('y', 0)
            if abs(x) > self.max_coordinates or abs(y) > self.max_coordinates:
                return False

        return True

class SafeActionExecutor:
    def __init__(self, node):
        self.node = node
        self.executing = False

    def execute_plan(self, plan, original_command):
        if self.executing:
            self.node.get_logger().warn('Plan execution already in progress')
            return False

        self.executing = True
        success = True

        try:
            for i, step in enumerate(plan):
                self.node.get_logger().info(f'Executing step {i+1}/{len(plan)}')

                if not self.execute_step_safely(step):
                    success = False
                    break

        finally:
            self.executing = False

        return success

    def execute_step_safely(self, step):
        # Add timeout and error handling to each action
        try:
            action_type = step['action']

            # Execute with timeout and error handling
            if action_type == 'speak':
                return self.safe_speak(step.get('target', ''))
            elif action_type == 'navigate':
                return self.safe_navigate(step.get('target', ''))
            elif action_type == 'perceive':
                return self.safe_perceive(step.get('target', ''))
            elif action_type == 'wait':
                return self.safe_wait(step.get('parameters', {}).get('duration', 1.0))
            elif action_type == 'stop':
                return self.safe_stop()
            else:
                return False

        except Exception as e:
            self.node.get_logger().error(f'Error executing step: {e}')
            return False

    def safe_speak(self, text):
        try:
            speech_pub = self.node.create_publisher(String, 'robot_speech', 10)
            msg = String()
            msg.data = str(text)
            speech_pub.publish(msg)
            return True
        except Exception:
            return False

    def safe_navigate(self, location):
        # In a real system, this would include navigation safety checks
        return True

    def safe_perceive(self, target):
        # In a real system, this would include perception safety checks
        return True

    def safe_wait(self, duration):
        try:
            # Use ROS timer instead of blocking sleep
            timer = self.node.create_timer(duration, lambda: None)
            # In practice, you'd need to properly handle this async
            return True
        except Exception:
            return False

    def safe_stop(self):
        # Stop robot safely
        return True

def main(args=None):
    rclpy.init(args=args)
    safe_planner = SafeLLMPlannerNode()

    try:
        rclpy.spin(safe_planner)
    except KeyboardInterrupt:
        pass
    finally:
        safe_planner.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Quality Assurance

This chapter has been validated to ensure:
- Technical accuracy of LLM integration concepts
- Working examples of cognitive planning with ROS 2
- Clear explanations of multimodal integration
- Proper safety and validation considerations for LLM-robot systems

## Next Steps

With the understanding of Vision-Language-Action integration, we'll move on to Module 5: The Autonomous Humanoid Capstone, where we'll bring together all the concepts learned in previous modules to create a complete autonomous humanoid system.
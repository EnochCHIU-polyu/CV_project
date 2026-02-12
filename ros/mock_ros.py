import time

class MockLogger:
    def info(self, msg):
        print(f"[INFO] {msg}")
    
    def warn(self, msg):
        print(f"[WARN] {msg}")
        
    def error(self, msg):
        print(f"[ERROR] {msg}")

class MockParameter:
    def __init__(self, value):
        self.value = value
    
    def get_parameter_value(self):
        return self

    @property
    def string_value(self):
        return str(self.value)

class MockPublisher:
    def __init__(self, topic_name):
        self.topic_name = topic_name
        
    def publish(self, msg):
        # In a real system, this sends data. In mock, we just print or store.
        # print(f"[MOCK ROS] Published message to {self.topic_name}")
        pass

class Node:
    def __init__(self, node_name):
        self.node_name = node_name
        self.logger = MockLogger()
        self.params = {}
        print(f"[MOCK ROS] Node '{node_name}' initialized.")

    def get_logger(self):
        return self.logger

    def create_subscription(self, msg_type, topic, callback, qos_profile):
        print(f"[MOCK ROS] Subscribed to '{topic}'")
        return None  # We trigger callbacks manually in the runner script

    def create_publisher(self, msg_type, topic, qos_profile):
        print(f"[MOCK ROS] Publisher created for '{topic}'")
        return MockPublisher(topic)

    def declare_parameter(self, name, default_value):
        self.params[name] = default_value

    def get_parameter(self, name):
        return MockParameter(self.params.get(name))
    
    def destroy_node(self):
        print(f"[MOCK ROS] Node '{self.node_name}' destroyed.")

def init(args=None):
    print("[MOCK ROS] rclpy initialized.")

def spin(node):
    print("[MOCK ROS] Spinning... (Ctrl+C to stop)")
    while True:
        time.sleep(1)

def shutdown():
    print("[MOCK ROS] Shutdown.")

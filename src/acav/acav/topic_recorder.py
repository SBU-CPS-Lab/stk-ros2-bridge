"""Run-owned, passive ROS topic capture with an independent ROS context."""
import json
import threading
from pathlib import Path


class TopicRecorder:
    def __init__(self, config, log_dir):
        import rclpy
        import yaml
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        from rosidl_runtime_py.convert import message_to_yaml
        from stk_msgs.msg import BridgeStatus, AttitudeState, AngularRate, AttitudeCommand, TwinStatus
        from stk_bridge.interfaces.ros_publishers import topic_namespace

        self.error = None
        self.counts = {}
        self._files = []
        self._stop = threading.Event()
        self._thread = None
        self._context = Context()
        self._node = None
        self._executor = None
        self._dir = Path(log_dir)
        try:
            rclpy.init(context=self._context)
            self._node = rclpy.create_node('acav_topic_recorder', context=self._context)
            self._executor = SingleThreadedExecutor(context=self._context)
            self._executor.add_node(self._node)
            with open(config.bridge_config, encoding='utf-8-sig') as f:
                bridge = yaml.safe_load(f)
            satellites = bridge.get('sensing', {}).get('satellites', ['*/Satellite/SSS-1'])
            prefix = '/' + config.prefix.strip('/')
            topics = [('status.txt', prefix+'/status', BridgeStatus),
                      ('cmd.txt', prefix+'/cmd/attitude', AttitudeCommand),
                      ('twin_status.txt', '/acav/twin_status', TwinStatus)]
            for satellite in satellites:
                ns = topic_namespace(satellite)
                suffix = '' if len(satellites) == 1 else '_'+ns
                topics.extend([('attitude'+suffix+'.txt', prefix+'/'+ns+'/attitude', AttitudeState),
                               ('rate'+suffix+'.txt', prefix+'/'+ns+'/angular_rate', AngularRate)])
            # Passive subscriptions match both reliable and best-effort publishers.
            qos = QoSProfile(depth=1000, reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.VOLATILE)
            for filename, topic, message_type in topics:
                handle = open(self._dir/filename, 'w', encoding='utf-8', buffering=1)
                self._files.append(handle)
                self.counts[topic] = 0
                def receive(message, stream=handle, name=topic):
                    stream.write(message_to_yaml(message)+'\n---\n')
                    self.counts[name] += 1
                self._node.create_subscription(message_type, topic, receive, qos)
            self._thread = threading.Thread(target=self._spin, name='acav-topic-recorder', daemon=True)
            self._thread.start()
        except BaseException:
            self.close()
            raise

    def _spin(self):
        try:
            while not self._stop.is_set():
                self._executor.spin_once(timeout_sec=0.1)
        except Exception as exc:
            self.error = str(exc)

    def close(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join()
        if self._executor is not None:
            self._executor.shutdown()
        if self._node is not None:
            self._node.destroy_node()
        if self._context.ok():
            self._context.shutdown()
        for handle in self._files:
            handle.close()
        (self._dir/'topics_summary.json').write_text(
            json.dumps({'received_messages': self.counts, 'error': self.error}, indent=2), encoding='utf-8')

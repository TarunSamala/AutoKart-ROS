import cProfile, pstats, io, time, rclpy
from rclpy.executors import SingleThreadedExecutor
from autokart_perception_ros.node import PerceptionNode
rclpy.init(args=['--ros-args','-p','mode:=live','-p','period_s:=0.02','-p','model_path:=/home/mark1/Desktop/autonomous_kart/models/yolo26s/best.onnx','-p','model_manifest:=/home/mark1/Desktop/autonomous_kart/models/yolo26s/best.json','-p','camera_library:=/home/mark1/Desktop/autonomous_kart/autonomy_stack_phase1/phase_01_perception/native/build-x86_64-linux-gnu/libhp60c_bridge.so','-p','camera_configuration:=/home/mark1/Desktop/autonomous_kart/autonomy_stack_phase1/vendor/linux_ros/linux/configurationfiles/hp60c_v2_00_20230704_configEncrypt.json'])
node=PerceptionNode(); ex=SingleThreadedExecutor(); ex.add_node(node)
end=time.time()+8
while time.time()<end: ex.spin_once(timeout_sec=0.1)
p=cProfile.Profile(); p.enable(); n0=node.sequence; end=time.time()+20
while time.time()<end: ex.spin_once(timeout_sec=0.1)
p.disable(); print('ticks/s', (node.sequence-n0)/20)
s=io.StringIO(); pstats.Stats(p,stream=s).sort_stats('cumulative').print_stats(30); print(s.getvalue()[:6000])
node.source.close(); node.destroy_node(); rclpy.shutdown()

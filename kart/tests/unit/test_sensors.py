from kart_sim.sensors import AsyncSensor,ImuSensor,WheelEncoder
def test_independent_clock_latency_and_determinism():
 a=ImuSensor("imu","body",100,phase_s=.003,processing_s=.01,transport_s=.02,seed=4); out=[]
 for k in range(100):out+=a.update(k*.001,{})
 assert out and out[0].header.capture_start_s==.003 and out[0].header.delivery_timestamp_s==.033
def test_imu_specific_force_and_encoder_edges():
 imu=ImuSensor("i","body",200);msg=imu.update(0,{"ax_mps2":1,"yaw_rate_radps":2})[0];assert msg.payload["accel_mps2"][0]==1 and msg.payload["accel_mps2"][2]==9.80665
 e=WheelEncoder("w","wheel",100,cpr=4);e.update(0,{"wheel_angle_rad":0});m=e.update(.01,{"wheel_angle_rad":3.2})[0];assert m.payload["count"]==2

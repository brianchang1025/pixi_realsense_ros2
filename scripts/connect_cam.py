import os
import sys
import tempfile
import time
import rclpy
import subprocess
from threading import Thread
from rclpy.executors import MultiThreadedExecutor
from utils.camera import Camera
from utils.utils import COLORS, launch_in_terminal, kill_all, read_single_key, prompt_default, make_payload


# Configuration
TMPDIR = os.environ.get("TMPDIR") or tempfile.mkdtemp(prefix="panda_connect.")
TOPICS_TO_SUB = {
    "color/image_raw": "sensor_msgs/msg/Image",
    "color/image_raw/compressed": "sensor_msgs/msg/CompressedImage",
    "color/camera_info": "sensor_msgs/msg/CameraInfo",

}
TOPICS_TO_PUB = {

}

def main():
    rclpy.init()

    
    ca_ns1 = prompt_default("Namespace Wrist Camera", "wrist_camera")
    serial_num1 = prompt_default("Serial Number Wrist Camera", "_827112070502")
    ca_ns2 = prompt_default("Namespace Third Person Camera", "third_person_camera")
    serial_num2 = prompt_default("Serial Number Third Person Camera", "_827112070588")

    # Start spinning in a background thread so the rest of the script can run   
    
    wrist = Camera(ca_ns1, serial_num1)
    third_person = Camera(ca_ns2, serial_num2)
    
    # Parallel Launch
    wrist.launch(launch_in_terminal)
    time.sleep(2)  # Stagger launches slightly for cleaner output
    third_person.launch(launch_in_terminal)
    time.sleep(1.5)

    wrist.is_connected = False
    third_person.is_connected = False
    while not wrist.is_connected or not third_person.is_connected:
        if not wrist.is_connected:
            print(f"Waiting for {ca_ns1} [serial: {serial_num1}] to connect...")
            wrist.is_connected = wrist._wait_for_network()
        if not third_person.is_connected:
            print(f"Waiting for {ca_ns2} [serial: {serial_num2}] to connect...")
            third_person.is_connected = third_person._wait_for_network()
        time.sleep(2)
    # 4. Health Dashboard (Now inside the class!)
    wrist.run_health_dashboard(TOPICS_TO_SUB, TOPICS_TO_PUB)
    third_person.run_health_dashboard(TOPICS_TO_SUB, TOPICS_TO_PUB)
    
    cmd = "pixi run -e jazzy rqt --perspective-file ./scripts/cameraviewer.perspective"
    launch_in_terminal("Camera Dashboard", cmd, os.path.join(TMPDIR, "rqt.log"))
    # 5. Persistent Control
    print(f"\n🚀 {COLORS['G']}Cameras Ready.{COLORS['RE']}")
    print(f"Press {COLORS['B']}'q'{COLORS['RE']} to terminate all processes and exit.")
    
    while True:
        if read_single_key().lower() == 'q':
            wrist.cleanup()
            #third_person.cleanup()
            rclpy.shutdown()
            time.sleep(1)  # Give some time for cleanup logs to print
            kill_all()
            break
    
    # 2. Shutdown RO
if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        kill_all()
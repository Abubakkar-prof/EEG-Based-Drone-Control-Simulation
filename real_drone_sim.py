import pybullet as p
import pybullet_data
import time
import math
import traceback
import random
import socket
import threading

# UDP Configuration
UDP_IP = "0.0.0.0"
UDP_PORT = 5005

class DroneController:
    def __init__(self):
        self.cmd = None
        self.running = True
        self.socket = None
        
        # Start UDP Thread
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.socket.bind((UDP_IP, UDP_PORT))
            self.socket.settimeout(0.5)
            print(f"[INFO] UDP Server Listening on port {UDP_PORT}")
            
            self.thread = threading.Thread(target=self._read_udp)
            self.thread.daemon = True
            self.thread.start()
        except Exception as e:
            print(f"[ERROR] Could not start UDP server: {e}")

    def _read_udp(self):
        while self.running:
            try:
                data, addr = self.socket.recvfrom(1024)
                cmd = data.decode('utf-8').strip()
                if cmd:
                    self.cmd = cmd
                    print(f"[RX] UDP Command: {cmd} from {addr}")
            except socket.timeout:
                pass
            except Exception as e:
                print(f"UDP read error: {e}")
                
    def get_cmd(self):
        c = self.cmd
        self.cmd = None # Consume command
        return c

    def close(self):
        self.running = False
        if self.socket:
            self.socket.close()

def create_environment():
    """Sets up a rich, functional environment for drone simulation."""
    
    # Floor
    planeId = p.loadURDF("plane.urdf")
    p.changeVisualShape(planeId, -1, rgbaColor=[0.2, 0.3, 0.2, 1])
    
    # Track Layout - "Figure 8 Slalom"
    gate_positions = [
        [5, 0, 1.5, 1.57],
        [10, 5, 2.0, 2.0],
        [5, 10, 2.5, 3.14],
        [-5, 5, 2.0, 3.8],
        [-5, -5, 1.5, 4.5],
        [5, -10, 1.5, 5.2],
        [15, 0, 1.5, 1.57]
    ]
    
    # Load Gates
    gate_ids = []
    for i, (x, y, z, yaw) in enumerate(gate_positions):
        orn = p.getQuaternionFromEuler([0, 0, yaw])
        gateId = p.loadURDF("gate.urdf", [x, y, z], orn, useFixedBase=True)
        color = [1, 0.5, 0, 1] if i % 2 == 0 else [1, 0.2, 0.2, 1] 
        p.changeVisualShape(gateId, -1, rgbaColor=color)
        gate_ids.append(gateId)
        p.addUserDebugText(str(i+1), [x, y, z+1.2], [1, 1, 1], textSize=1.5)

    # Trees
    forest_area_x = [-25, 25]
    forest_area_y = [-25, 25]
    
    for _ in range(30):
        tx = random.uniform(*forest_area_x)
        ty = random.uniform(*forest_area_y)
        
        if -12 < tx < 18 and -12 < ty < 12:
            continue
            
        scale = random.uniform(0.7, 1.3)
        try:
            p.loadURDF("tree.urdf", [tx, ty, 0], globalScaling=scale, useFixedBase=True)
        except:
            pass

    # Buildings
    building_pos = [
        [-15, 15, 0],
        [-20, 10, 0],
        [-15, 20, 0]
    ]
    
    for pos in building_pos:
        try:
            p.loadURDF("building.urdf", pos, useFixedBase=True)
        except:
            pass

    # Start/End Pads
    p.loadURDF("pad.urdf", [0, 0, 0.05], useFixedBase=True)
    p.loadURDF("pad.urdf", [20, 0, 0.05], useFixedBase=True)
    
    return gate_ids

def main():
    print("=" * 60)
    print("  REAL DRONE SIMULATION (UDP + KEYBOARD)")
    print("  Controls:")
    print("    ESP32 Keypad (UDP) OR Keyboard")
    print("    Arrow Keys/2,8,4,6: Move")
    print("    W/S/A/B: Altitude")
    print("    A/D: Rotate")
    print("=" * 60)
    
    controller = DroneController()

    try:
        # Setup PyBullet
        physicsClient = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.8)
        p.setRealTimeSimulation(0)
        
        # Visual quality settings
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
        p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1)
        p.configureDebugVisualizer(p.COV_ENABLE_WIREFRAME, 0)

        # Load Environment
        create_environment()
        
        startPos = [0, 0, 0.2]
        current_pos = [0.0, 0.0, 0.2] 
        current_yaw = 0.0
        
        # Load Drone
        print("Loading drone model...")
        droneId = p.loadURDF("drone.urdf", startPos, p.getQuaternionFromEuler([0,0,0]))
        
        # Apply colors
        p.changeVisualShape(droneId, -1, rgbaColor=[1, 1, 1, 1], specularColor=[0.8, 0.8, 0.8])
        for i in range(4):
            p.changeVisualShape(droneId, i, rgbaColor=[0.85, 0.7, 0.2, 1], specularColor=[0.8, 0.8, 0])
        
        prop_joints = [0, 1, 2, 3]

        # Speed parameters
        SPEED_XY = 14.0 / 60.0 # ~0.23 units per tick
        SPEED_Z_UP = 5.0 / 60.0
        SPEED_Z_DOWN = 3.0 / 60.0
        SPEED_ROT = 0.08
        
        # Constraints
        MAX_ALTITUDE = 30.0
        TILT_ANGLE = 0.3

        # Camera settings
        camera_distance = 3.0
        camera_pitch = -20
        camera_height_offset = 0.5

        # Latch for UDP commands
        latched_move_x = 0
        latched_move_y = 0
        latched_move_z = 0
        
        while True:
            # 1. Handle Keyboard (Momentary)
            keys = p.getKeyboardEvents()
            
            kb_move_x = 0 
            kb_move_y = 0 
            kb_move_z = 0 
            kb_rot_z = 0
            
            if p.B3G_UP_ARROW in keys and keys[p.B3G_UP_ARROW] & p.KEY_IS_DOWN: kb_move_y = 1
            if p.B3G_DOWN_ARROW in keys and keys[p.B3G_DOWN_ARROW] & p.KEY_IS_DOWN: kb_move_y = -1
            if p.B3G_LEFT_ARROW in keys and keys[p.B3G_LEFT_ARROW] & p.KEY_IS_DOWN: kb_move_x = -1
            if p.B3G_RIGHT_ARROW in keys and keys[p.B3G_RIGHT_ARROW] & p.KEY_IS_DOWN: kb_move_x = 1
            if ord('w') in keys and keys[ord('w')] & p.KEY_IS_DOWN: kb_move_z = SPEED_Z_UP
            if ord('s') in keys and keys[ord('s')] & p.KEY_IS_DOWN: kb_move_z = -SPEED_Z_DOWN
            if ord('a') in keys and keys[ord('a')] & p.KEY_IS_DOWN: kb_rot_z = 1
            if ord('d') in keys and keys[ord('d')] & p.KEY_IS_DOWN: kb_rot_z = -1
            
            # 2. Handle UDP (Latched)
            udp_cmd = controller.get_cmd()
            if udp_cmd:
                if udp_cmd == "START":
                    is_started = True
                    # Spin props slightly to show idle?
                    current_pos[2] = 0.21 # Lift slightly off ground
                    print("[START] Drone Armed & Ready!")

                elif udp_cmd == "RESET":
                    current_pos = [0.0, 0.0, 0.2]
                    current_yaw = 0.0
                    latched_move_x = 0
                    latched_move_y = 0
                    latched_move_z = 0
                    print("[RESET] Position Reset.")

                # Flight Controls (ALWAYS ACTIVE - NO LOCK)
                if udp_cmd == "FWD": latched_move_y = 1
                elif udp_cmd == "BWD": latched_move_y = -1
                
                elif udp_cmd == "LEFT": latched_move_x = -1
                elif udp_cmd == "RIGHT": latched_move_x = 1
                
                elif udp_cmd == "UP": latched_move_z = SPEED_Z_UP
                elif udp_cmd == "DOWN": latched_move_z = -SPEED_Z_DOWN
                
                elif udp_cmd == "HOVER": 
                    latched_move_x = 0
                    latched_move_y = 0
                    latched_move_z = 0
                    print("[HOVER] Holding Position")

                elif udp_cmd == "LAND": # Simple land
                    latched_move_z = -SPEED_Z_DOWN
                    latched_move_x = 0
                    latched_move_y = 0
                    print("[LAND] Descending")
            
            # Combine Inputs (Priority to Keyboard if pressed, otherwise use latch)
            final_move_x = kb_move_x if kb_move_x != 0 else latched_move_x
            final_move_y = kb_move_y if kb_move_y != 0 else latched_move_y
            final_move_z = kb_move_z if kb_move_z != 0 else latched_move_z
            final_rot_z = kb_rot_z

            # Update physics
            current_yaw += final_rot_z * SPEED_ROT
            
            # Calculate world-space velocity
            wx = final_move_y * math.cos(current_yaw) - final_move_x * math.sin(current_yaw)
            wy = final_move_y * math.sin(current_yaw) + final_move_x * math.cos(current_yaw)
            
            current_pos[0] += wx * SPEED_XY
            current_pos[1] += wy * SPEED_XY
            current_pos[2] += final_move_z
            
            # Altitude constraints
            if current_pos[2] < 0.1: current_pos[2] = 0.1
            if current_pos[2] > MAX_ALTITUDE: current_pos[2] = MAX_ALTITUDE

            # Visual tilt
            target_pitch = final_move_y * TILT_ANGLE 
            target_roll = final_move_x * TILT_ANGLE
            
            new_orn = p.getQuaternionFromEuler([target_roll, target_pitch, current_yaw])
            p.resetBasePositionAndOrientation(droneId, current_pos, new_orn)

            # Propeller animation
            p.setJointMotorControlArray(droneId, prop_joints, p.VELOCITY_CONTROL, 
                                        targetVelocities=[1200, -1200, 1200, -1200], forces=[20]*4)
            
            # Camera follow
            cam_target = [current_pos[0], current_pos[1], current_pos[2] + camera_height_offset]
            p.resetDebugVisualizerCamera(
                cameraDistance=camera_distance, 
                cameraYaw=math.degrees(current_yaw) - 90,
                cameraPitch=camera_pitch, 
                cameraTargetPosition=cam_target
            )
            
            p.stepSimulation()
            time.sleep(1./60.)

    except KeyboardInterrupt:
        print("\n\nShutting down...")
    except Exception:
        traceback.print_exc()
    finally:
        controller.close()
        input("Press Enter to exit...")

if __name__ == "__main__":
    main()


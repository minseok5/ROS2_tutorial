import time
import cflib
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig

URI = "radio://0/100/2M"

def log_data(timestamp, data, logconf):
    print(f"Timestamp: {timestamp}, Acc: ({data['acc.x']}, {data['acc.y']}, {data['acc.z']}), "
          f"Gyro: ({data['gyro.x']}, {data['gyro.y']}, {data['gyro.z']}), "
          f"Flow: ({data['motion.deltaX']}, {data['motion.deltaY']}), Height: {data['range.zrange']}")

def connected_callback(link_uri):
    print(f"Connected to {link_uri}")
    start_logging(cf)

def connection_failed_callback(link_uri, msg):
    print(f"Connection to {link_uri} failed: {msg}")

def disconnected_callback(link_uri):
    print(f"Disconnected from {link_uri}")

def start_logging(cf):
    """Start logging only after Crazyflie is ready"""
    log_config = LogConfig(name="IMU_Flow", period_in_ms=100)

    try:
        log_config.add_variable("acc.x", "float")
        log_config.add_variable("acc.y", "float")
        log_config.add_variable("acc.z", "float")
        log_config.add_variable("gyro.x", "float")
        log_config.add_variable("gyro.y", "float")
        log_config.add_variable("gyro.z", "float")
        log_config.add_variable("motion.deltaX", "float")  # Optical flow X
        log_config.add_variable("motion.deltaY", "float")  # Optical flow Y
        log_config.add_variable("range.zrange", "float")   # Height from ToF sensor

        cf.log.add_config(log_config)
        log_config.data_received_cb.add_callback(log_data)
        log_config.start()

        print("Logging started...")

        time.sleep(5)  # Log for 5 seconds

        log_config.stop()
        print("Logging stopped")

    except AttributeError:
        print("Error: TOC not available. Ensure the Crazyflie is connected.")

    cf.close_link()

def main():
    cflib.crtp.init_drivers()
    
    global cf
    cf = Crazyflie()

    # Attach callbacks
    cf.connected.add_callback(connected_callback)
    cf.connection_failed.add_callback(connection_failed_callback)
    cf.disconnected.add_callback(disconnected_callback)

    # Open link
    cf.open_link(URI)

    # Keep the script running to listen for callbacks
    time.sleep(10)

if __name__ == "__main__":
    main()

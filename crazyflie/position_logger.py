from threading import Thread
import csv 
import csv
import os 

from datetime import datetime
from queue import Queue


from cflib.crazyflie.swarm import CachedCfFactory, Swarm
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.crazyflie.log import LogConfig
import cflib.crtp



# Table that associates the crazyflie with the URI and keeps track of what body it tracks
WRIST_SENSOR = "radio://0/80/2M/9696969696"
FINGER_SENSOR = "radio://0/80/2M/6969696969"
BACK_SENSOR = "radio://0/80/2M/E7E7E7E7E7"
URI_TABLE = { BACK_SENSOR: "BACK"}
POSITION_THRESHOLD = 0.001

URI_TABLE = { FINGER_SENSOR: "FINGER", WRIST_SENSOR: "WRIST", BACK_SENSOR: "BACK"}
# URI_TABLE = { FINGER_SENSOR: "FINGER", WRIST_SENSOR: "WRIST"}



SENTINEL = "STOP"

uris = URI_TABLE.keys()


dt_now = datetime.now()
LOG_DIRECTORY = "experimental_data/"
OUTPUT_LOG = f"{LOG_DIRECTORY}{dt_now.strftime('%Y_%m_%d_%H_%M_%S.csv')}"

# Write a function that logs the positions of the crazyflie to a file
def log_positions(queue):
    def write_log(scf):
        lg_stab = LogConfig(name='position', period_in_ms=50)
        lg_stab.add_variable('stateEstimate.x', 'float')
        lg_stab.add_variable('stateEstimate.y', 'float')
        lg_stab.add_variable('stateEstimate.z', 'float')

        uri = scf.cf.link_uri
        sensor = URI_TABLE[uri]
        print(f"Logging {sensor} position")
        with SyncLogger(scf, lg_stab) as logger:
            for log_entry in logger:
                data = log_entry[1]

                x = data['stateEstimate.x']
                y = data['stateEstimate.y']
                z = data['stateEstimate.z']

                # print([sensor, datetime.now().timestamp(),x,y,z])
                queue.put([sensor, datetime.now().timestamp(),x,y,z])
            
    return write_log


def write_to_csv(fd, *args):
    csv.writer(fd).writerow(*args)

def is_empty(file): 
    return os.stat(file).st_size == 0


def log_to_file(queue, fd):
    while True:
        data = queue.get()
        if data == SENTINEL: break
        write_to_csv(fd, data)
        queue.task_done()

def wait_for_position_estimator(scf):
    print(scf.cf.link_uri,': Waiting for estimator to find position...')

    log_config = LogConfig(name='Kalman Variance', period_in_ms=500)
    log_config.add_variable('kalman.varPX', 'float')
    log_config.add_variable('kalman.varPY', 'float')
    log_config.add_variable('kalman.varPZ', 'float')

    var_y_history = [1000] * 10
    var_x_history = [1000] * 10
    var_z_history = [1000] * 10

    threshold = POSITION_THRESHOLD

    with SyncLogger(scf, log_config) as logger:
        for log_entry in logger:
            data = log_entry[1]

            var_x_history.append(data['kalman.varPX'])
            var_x_history.pop(0)
            var_y_history.append(data['kalman.varPY'])
            var_y_history.pop(0)
            var_z_history.append(data['kalman.varPZ'])
            var_z_history.pop(0)

            min_x = min(var_x_history)
            max_x = max(var_x_history)
            min_y = min(var_y_history)
            max_y = max(var_y_history)
            min_z = min(var_z_history)
            max_z = max(var_z_history)

            delta_x = max_x - min_x
            delta_y = max_y - min_y
            delta_z = max_z - min_z

            if delta_x < threshold and delta_y < threshold and delta_z < threshold:
                break
    uri = scf.cf.link_uri
    print( URI_TABLE[uri], ': Position found.')


if __name__ == '__main__':
    cflib.crtp.init_drivers()
    factory = CachedCfFactory(rw_cache='./cache')

    print("Waiting for crazyflies to connect")
    
    with Swarm(uris, factory=factory) as swarm:
        print("Successfully connected to all crazyflies")
        with open(OUTPUT_LOG, 'w') as fd:
            print("Writing to log file", f"{os.getcwd()}/{OUTPUT_LOG}")

            # Write the headers to the CSV file
            if is_empty(OUTPUT_LOG): csv.writer(fd).writerow([ 'sensor', 'time', 'x', 'y', 'z'])

            # Build the log queue so we don't have write to the file from multiple threads
            location_log_queue = Queue()
            # Build the logging function that will write to the queue
            location_logger = log_positions(location_log_queue)
            # Build the thread that will write to the file
            log_to_file_thread = Thread(target=log_to_file, args=(location_log_queue, fd))
            # Start the thread
            log_to_file_thread.start()

            try:
                print("Calibrating position sensors...")
                swarm.parallel(wait_for_position_estimator)
                print("Calibration complete. Starting logging...")
                swarm.parallel_safe(location_logger)

            except KeyboardInterrupt:
                print("Keyboard interrupt received. Stopping logging...")
                print("Waiting for logging to finish...")
                location_log_queue.put(SENTINEL)
                log_to_file_thread.join()
            except Exception as e:
                for i in range(10):
                    print("\a\a\a\a\a")

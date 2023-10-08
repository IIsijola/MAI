from cflib.crazyflie.swarm import CachedCfFactory, Swarm
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.crazyflie.log import LogConfig
import cflib.crtp


from datetime import datetime
from collections import deque
from threading import Thread

import time as t
import multiprocess

import csv 
import sys
import os 



WRIST_SENSOR = "radio://0/80/2M/9696969696"
FINGER_SENSOR = "radio://0/80/2M/6969696969"
BACK_SENSOR = "radio://0/80/2M/E7E7E7E7E7"
POSITION_THRESHOLD = 0.001

MAX_PLOT_POINTS = 1000

# Table that associates the crazyflie with the URI and keeps track of what body it tracks
# URI_TABLE = {WRIST_SENSOR: "WRIST", FINGER_SENSOR: "FINGER", BACK_SENSOR: "BACK"}
URI_TABLE = {FINGER_SENSOR: "FINGER"}
uris = URI_TABLE.keys()


dt_now = datetime.now()
START_TIME = dt_now.timestamp()
LOG_DIRECTORY = "experimental_data/"
OUTPUT_LOG = f"{LOG_DIRECTORY}{dt_now.strftime('%Y_%m_%d_%H_%M_%S.csv')}"
SENTINEL = "STOP"

def wait_for_position_estimator(scf):
    print(scf.cf.link_uri,': Waiting for estimator to find position...')

    log_config = LogConfig(name='Kalman Variance', period_in_ms=50)
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

                print(data)

                x = data['stateEstimate.x']
                y = data['stateEstimate.y']
                z = data['stateEstimate.z']

                if not queue.full():
                    queue.put([sensor, datetime.now().timestamp(),x,y,z])
            
    return write_log


def create_plot_process(uri, queue):
    from matplotlib.animation import FuncAnimation
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(2, 2, figsize=(5, 5))

    plt.show(block=False)

    plot_queue = deque(maxlen=500)

    def animate(i):
        nonlocal ax
        data = queue.get()
        if data == SENTINEL:
            print("Plotting process exiting")
            sys.exit()

        plot_queue.append(data)

        if len(plot_queue) == 500:
            for sensor,time, x,y,z in plot_queue: 
                time -= START_TIME
                ax[0][0].clear()
                ax[0][0].set_title(f"{sensor} X Position wrt Time")
                ax[0][0].plot(time, x, color="r")
                ax[0][0].set_xlabel('time')
                ax[0][0].set_ylabel('X')

                ax[0][1].clear()
                ax[0][1].set_title(f"{sensor} Y Position wrt Time")
                ax[0][1].plot(time, y, color="g")
                ax[0][1].set_xlabel('time')
                ax[0][1].set_ylabel('Y')

                ax[1][0].clear()
                ax[1][0].set_title(f"{sensor} Z Position wrt Time")
                ax[1][0].plot(time, z, color="b")
                ax[1][0].set_xlabel('time')
                ax[1][0].set_ylabel('Z')

                ax[1][1].clear()
                ax[1][1].set_title(f"{sensor} X-Z wrt Time")
                ax[1][1].plot(x, z, color="purple")
                ax[1][1].set_xlabel('X')
                ax[1][1].set_ylabel('Z')
        else:
            sensor,time, x,y,z = data
            time -= START_TIME
            # ax.scatter(x,y,z, c="black")
            ax[0][0].plot(time, x, color="r")
            ax[0][1].plot(time, y, color="g")
            ax[1][0].plot(time, z, color="b")
            ax[1][1].plot(x, z, color="purple")




            

    try:
        ani = FuncAnimation(fig, animate, interval=50)
        plt.show()
    except KeyboardInterrupt:
        pass

    # try:
    #     while True:
    #         data = queue.get()
    #         if data == SENTINEL:
    #             print("Plotting process exiting")
    #             break
    #         sensor_name, time, x, y, z  = data
    #         plot_queue.append((time, x, y, z))
    #         ax.scatter3D(x, y, z, c="black", cmap='Greens')
    #         ax.relim()
    #         ax.autoscale_view(tight=True, scalex=True, scaley=False)
    #         fig.canvas.draw()
    #         plt.pause(0.05)
    # except KeyboardInterrupt:
    #     pass

def create_sensor_plot_queues():
    return {uri: multiprocess.Queue() for uri in URI_TABLE.values()}


def write_to_csv(fd, *args):
    csv.writer(fd).writerow(*args)

def is_empty(file): 
    return os.stat(file).st_size == 0


def process_queue(fd, queue, sensor_queues):
    while True:
        # Wait for data in the queue
        data = queue.get()
        if data == SENTINEL:
            print("Logging process exiting")
            break
        # Write the data to the file
        write_to_csv(fd, data)
        # Push the data to the plot queue
        sensor_queues[ data[0] ].put(data)



def log_sensors(fd, queue):
    while True:
        # Wait for data in the queue
        data = queue.get()
        if data == SENTINEL:
            print("Logging process exiting")
            break
        # Write the data to the file
        # data = [sensor, time, x, y, z]
        write_to_csv(fd, data)




if __name__ == '__main__':
    cflib.crtp.init_drivers()
    factory = CachedCfFactory(rw_cache='./cache')
    multiprocess.set_start_method('spawn')

    with Swarm(uris, factory=factory) as swarm:
        print("Successfully connected to all crazyflies")
        with open(OUTPUT_LOG, 'w') as fd:
            print("Writing to log file", f"{os.getcwd()}/{OUTPUT_LOG}")

            # Write the headers to the CSV file
            if is_empty(OUTPUT_LOG): csv.writer(fd).writerow(['sensor', 'time', 'x', 'y', 'z'])

            # Build the queues for each sensor
            sensor_queues = create_sensor_plot_queues()

            # Build the log queue so we don't have write to the file from multiple threads
            location_log_queue = multiprocess.Queue(maxsize=50)
            # Build the logging function that will write to the queue
            location_logger = log_positions(location_log_queue)
            # Build the thread that will write to the file
            process_queue_thread = Thread(target=process_queue, args=(fd, location_log_queue, sensor_queues), daemon=True)
            # process_queue_thread = Thread(target=log_sensors, args=(fd, location_log_queue))
            # Start the thread
            process_queue_thread.start()

            # Build the processes that will plot the data
            plot_processes = []

            for uri, queue in sensor_queues.items():
                plot_function = (uri, queue)
                plot_processes.append(multiprocess.Process(target=create_plot_process, args=(uri, queue)))
                plot_processes[-1].start()

            try:
                print("Calibrating position sensors...")
                # swarm.parallel(wait_for_position_estimator)
                print("Calibration complete. Starting logging...")
                swarm.parallel_safe(location_logger)

            except KeyboardInterrupt:
                print("Keyboard interrupt received. Stopping logging...")
                print("Waiting for logging to finish...")
                location_log_queue.put(SENTINEL)
                process_queue_thread.join()
                # Wait for the queue to finish
                map(lambda q: q.put(SENTINEL), sensor_queues.values())
                map(lambda p: p.join(), plot_processes)

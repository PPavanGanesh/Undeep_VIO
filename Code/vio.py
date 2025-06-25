
from queue import Queue
from threading import Thread

from Code.config import ConfigEuRoC
from Code.image import ImageProcessor
from Code.msckf import MSCKF
from Code.utils import to_quaternion



class VIO(object):
    def __init__(self, config, img_queue, imu_queue, viewer=None):
        self.config = config
        self.viewer = viewer

        self.img_queue = img_queue
        self.imu_queue = imu_queue
        self.feature_queue = Queue()

        self.image_processor = ImageProcessor(config)
        self.msckf = MSCKF(config)

        self.img_thread = Thread(target=self.process_img)
        self.imu_thread = Thread(target=self.process_imu)
        self.vio_thread = Thread(target=self.process_feature)
        self.img_thread.start()
        self.imu_thread.start()
        self.vio_thread.start()

    def process_img(self):
        while True:
            img_msg = self.img_queue.get()
            if img_msg is None:
                self.feature_queue.put(None)
                return
            # print('img_msg', img_msg.timestamp)

            if self.viewer is not None:
                self.viewer.update_image(img_msg.cam0_image)

            feature_msg = self.image_processor.stareo_callback(img_msg)

            if feature_msg is not None:
                self.feature_queue.put(feature_msg)

    def process_imu(self):
        while True:
            imu_msg = self.imu_queue.get()
            if imu_msg is None:
                return
            # print('imu_msg', imu_msg.timestamp)

            self.image_processor.imu_callback(imu_msg)
            self.msckf.imu_callback(imu_msg)

    # def process_feature(self):
    #     while True:
    #         feature_msg = self.feature_queue.get()
    #         if feature_msg is None:
    #             return
    #         print('feature_msg', feature_msg.timestamp)
    #         result = self.msckf.feature_callback(feature_msg)

    #         if result is not None and self.viewer is not None:
    #             self.viewer.update_pose(result.cam0_pose)
    
    def process_feature(self):
        trajectory_data = []
        while True:
            feature_msg = self.feature_queue.get()
            if feature_msg is None:
                return trajectory_data
            
            print('feature_msg', feature_msg.timestamp)
            result = self.msckf.feature_callback(feature_msg)
            
            if result is not None:
                trajectory_data.append(result)
                if self.viewer is not None:
                    self.viewer.update_pose(result.cam0_pose)
        

                
def save_trajectory(trajectory_data, filename):
    """
    Save the estimated trajectory in TUM format.
    Format: timestamp tx ty tz qx qy qz qw
    """
    with open(filename, 'w') as f:
        f.write("# timestamp tx ty tz qx qy qz qw\n")
        for data in trajectory_data:
            timestamp = data.timestamp
            pose = data.pose
            position = pose.t
            # Convert rotation matrix to quaternion
            rotation = pose.R
            q = to_quaternion(rotation)
            f.write(f"{timestamp} {position[0]} {position[1]} {position[2]} {q[0]} {q[1]} {q[2]} {q[3]}\n")
        
                
# def save_ground_truth(dataset, filename):
#     """
#     Save the ground truth trajectory in the format required by rpg_trajectory_evaluation.
#     Format: timestamp tx ty tz qx qy qz qw
#     """
#     with open(filename, 'w') as f:
#         f.write("# timestamp tx ty tz qx qy qz qw\n")
#         for gt in dataset.groundtruth:
#             # EuRoC ground truth is already in the right format
#             f.write(f"{gt.timestamp} {gt.p[0]} {gt.p[1]} {gt.p[2]} {gt.q[1]} {gt.q[2]} {gt.q[3]} {gt.q[0]}\n")

# def save_ground_truth(dataset, filename):
#     """
#     Save the ground truth trajectory in the format required by rpg_trajectory_evaluation.
#     Format: timestamp tx ty tz qx qy qz qw
#     """
#     with open(filename, 'w') as f:
#         f.write("# timestamp tx ty tz qx qy qz qw\n")
#         for gt in dataset.groundtruth:
#             # EuRoC ground truth format: timestamp is the first argument when iterating
#             f.write(f"{gt[0]} {gt.p[0]} {gt.p[1]} {gt.p[2]} {gt.q[1]} {gt.q[2]} {gt.q[3]} {gt.q[0]}\n")

def save_ground_truth(dataset, filename):
    """
    Save the ground truth trajectory in the format required by rpg_trajectory_evaluation.
    Format: timestamp tx ty tz qx qy qz qw
    """
    with open(filename, 'w') as f:
        f.write("# timestamp tx ty tz qx qy qz qw\n")
        
        # Open and parse the ground truth file directly
        with open(dataset.groundtruth.path, 'r') as gt_file:
            next(gt_file)  # Skip header
            for line in gt_file:
                line_data = line.strip().split(',')
                timestamp = float(line_data[0]) * dataset.groundtruth.scaler
                
                if timestamp < dataset.groundtruth.starttime:
                    continue
                    
                p = [float(line_data[i]) for i in range(1, 4)]
                q = [float(line_data[i]) for i in range(4, 8)]
                
                f.write(f"{timestamp} {p[0]} {p[1]} {p[2]} {q[1]} {q[2]} {q[3]} {q[0]}\n")




if __name__ == '__main__':
    import time
    import argparse
    from queue import Queue
    from Code.dataset import EuRoCDataset, DataPublisher
    from Code.viewer import Viewer
    from Code.config import ConfigEuRoC

    parser = argparse.ArgumentParser()
    parser.add_argument('--path', type=str, default=r'C:\Users\pavan\Documents\CV_Project4\.venv\MH_01_easy',
                      help='Path of EuRoC MAV dataset.')
    parser.add_argument('--view', action='store_true', help='Show trajectory.')
    args = parser.parse_args()

    if args.view:
        viewer = Viewer()
    else:
        viewer = None

    dataset = EuRoCDataset(args.path)
    dataset.set_starttime(offset=40.)  # start from static state

    # Save ground truth
    save_ground_truth(dataset, "stamped_groundtruth.txt")

    img_queue = Queue()
    imu_queue = Queue()
    config = ConfigEuRoC()
    msckf_vio = VIO(config, img_queue, imu_queue, viewer=viewer)

    duration = float('inf')
    ratio = 0.4  # make it smaller if image processing and MSCKF computation is slow

    imu_publisher = DataPublisher(
        dataset.imu, imu_queue, duration, ratio)
    img_publisher = DataPublisher(
        dataset.stereo, img_queue, duration, ratio)

    # List to store trajectory data
    trajectory_data = []

    now = time.time()
    imu_publisher.start(now)
    img_publisher.start(now)

    # Add this code to wait for user input to stop processing
    try:
        print("Press Ctrl+C to stop processing and save trajectory...")
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Stopping processing...")
        
        # Wait for processing to complete
        imu_publisher.stop()
        img_publisher.stop()
        
        # Wait for the VIO threads to finish
        msckf_vio.img_thread.join()
        msckf_vio.imu_thread.join()
        msckf_vio.vio_thread.join()
        
        # Save the trajectory
        save_trajectory(trajectory_data, "estimated_trajectory.txt")
        print("Trajectory saved to estimated_trajectory.txt")

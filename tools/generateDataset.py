import os
import math
import json
import hashlib
import argparse
import numpy as np
import datetime as dt
import getpass as gt
import tkinter as tk
from tkinter import messagebox
from tkinter import filedialog
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from typing import List, Set, Tuple, Optional, Dict, Any, Generator

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py
import sensor_msgs_py.point_cloud2 as pc2
from tf_transformations import euler_from_quaternion

SLAM_MAP_TOPIC = "/graphslam/cones/global_viz"
POSE_TOPIC = "/graphslam/pose"
LAP_COUNT_TOPIC = "/graphslam/lap_count"
CAR_STATE_TOPIC = "/ekf/car_state"
POINTCLOUD_TOPIC = "/ouster/points"

class DatasetGenerator:
    def __init__(self,
                 selected_file: str,
                 lidar_rotation: float = 1.5707,
                 label_every: int = 10,
                 pc_in_global_frame: bool = False,
                 ego_motion_compensate: bool = False
                 ):
        self.selected_file: str = selected_file
        self.lidar_rotation: float = lidar_rotation
        self.label_every: int = label_every
        self.pc_in_global_frame: bool = pc_in_global_frame
        self.ego_motion_compensate: bool = ego_motion_compensate

    def _compensate_point_cloud(self, point_cloud: np.ndarray, vx: float, vy: float, yaw_rate: float) -> np.ndarray:
        """Perform ego-motion compensation on the point cloud."""
        lidarFrequency = 20  # Hz
        nPositions = 100  # Number of positions to compute over
        timeStep = 1.0 / (lidarFrequency * nPositions)  # The time step between positions
        timeScaling = nPositions / 100.0 * lidarFrequency / 10.0  # How to scale the point time to get the correct position

        yaw = np.zeros(nPositions + 1)
        x = np.zeros(nPositions + 1)
        y = np.zeros(nPositions + 1)

        for i in range(nPositions - 1, -1, -1):
            yaw[i] = yaw[i + 1] - yaw_rate * timeStep
            x[i] = x[i + 1] - timeStep * (vx * np.cos(yaw[i]) - vy * np.sin(yaw[i]))
            y[i] = y[i + 1] - timeStep * (vy * np.cos(yaw[i]) + vx * np.sin(yaw[i]))

        pcl_cloud_transformed = np.copy(point_cloud)

        for i in range(len(point_cloud)):
            iter_xyz = point_cloud[i, :3]
            iter_intensity = point_cloud[i, 3]
            ts = point_cloud[i, 4]

            if np.all(iter_xyz[:2] == 0) or np.any(np.isnan(iter_xyz)):
                pcl_cloud_transformed[i, :] = [0, 0, 0, 0, 0]
                continue

            if -5.0 < iter_xyz[0] < 0.8 and -1.0 < iter_xyz[1] < 1.0 and -2.0 < iter_xyz[2] < 2.0:
                pcl_cloud_transformed[i, :] = [0, 0, 0, 0, 0]
                continue

            t = max(min(int(ts * timeScaling * 0.000001), nPositions), 0)

            xx = iter_xyz[0] + x[t]
            yy = iter_xyz[1] + y[t]

            newxx = xx * np.cos(yaw[t]) - yy * np.sin(yaw[t])
            newyy = yy * np.cos(yaw[t]) + xx * np.sin(yaw[t])

            pcl_cloud_transformed[i, :] = [newxx, newyy, iter_xyz[2], iter_intensity, ts]

        return pcl_cloud_transformed

    def _get_local_pcl(self, 
                       pcl: np.ndarray,
                       pose: Any
                       ) -> np.ndarray:
        # Extract pose components
        x, y = pose.position.x, pose.position.y

        orientation_list = [
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)

        # Tranform pointcloud from global to local frame
        if self.pc_in_global_frame:
            for i in range(pcl.shape[0]):
                x_pc = pcl[i, 0]
                pcl[i, 0] = (x_pc - x) * math.cos(yaw) + (pcl[i, 1] - y) * math.sin(yaw)
                pcl[i, 1] = -(x_pc - x) * math.sin(yaw) + (pcl[i, 1] - y) * math.cos(yaw)

        return pcl

    def _get_local_cones(self, cones, pose,  pc_array: np.ndarray) -> Tuple[List[Tuple[float, float]], float]:
        # Extract pose components
        x, y = pose.position.x, pose.position.y

        orientation_list = [
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)

        local_cones = []
        for cone in cones:
            # Extract global coordinates
            global_x, global_y = cone

            # Transform the point to the local frame
            x_rel = (global_x - x) * math.cos(yaw) + (global_y - y) * math.sin(yaw)
            y_rel = -(global_x - x) * math.sin(yaw) + (global_y - y) * math.cos(yaw)

            local_cones.append((x_rel, y_rel))

        return local_cones, yaw

    def _read_data(self,
                   global_cones: List[Tuple[float, float]]
                   ) -> Generator[Tuple[np.ndarray, List[Tuple[float, float]], Dict[str, Any]], None, None]:
        reader = rosbag2_py.SequentialReader()
        reader.open(
            rosbag2_py.StorageOptions(uri=self.selected_file, storage_id="mcap"),
            rosbag2_py.ConverterOptions(
                input_serialization_format="cdr", output_serialization_format="cdr"
            ),
        )

        topic_types = reader.get_all_topics_and_types()

        def typename(topic_name):
            for topic_type in topic_types:
                if topic_type.name == topic_name:
                    return topic_type.type
            raise ValueError(f"topic {topic_name} not in bag")

        print("Reading bag...")
        lap = 0
        pose = None
        vx = 0.0
        vy = 0.0
        yawrate = 0.0
        while reader.has_next():
            topic, data, timestamp = reader.read_next()

            # Extract Speed
            if topic == CAR_STATE_TOPIC:
                msg_type = get_message(typename(topic))
                msg = deserialize_message(data, msg_type)
                vx = msg.linear_velocity.x
                vy = msg.linear_velocity.y
                yawrate = msg.angular_velocity.z

            # Extract Lap count
            if topic == LAP_COUNT_TOPIC:
                msg_type = get_message(typename(topic))
                msg = deserialize_message(data, msg_type)
                lap = msg.data

            # Extract Pose
            if topic == POSE_TOPIC:
                msg_type = get_message(typename(topic))
                pose = deserialize_message(data, msg_type)

            if lap < 1: # Skip the first lap
                continue

            # Extract Point Cloud
            if topic == POINTCLOUD_TOPIC:
                if pose is None:
                    continue

                msg_type = get_message(typename(topic))
                msg = deserialize_message(data, msg_type)

                if self.ego_motion_compensate:
                    pc_data = pc2.read_points(msg, field_names=("x", "y", "z", "intensity", "t"), skip_nans=True)
                    pc_array = np.array([list(p) for p in pc_data[['x', 'y', 'z', 'intensity', 't']]], dtype=np.float32).reshape(-1, 5)
                else:
                    pc_data = pc2.read_points(msg, field_names=("x", "y", "z", "intensity"), skip_nans=True)
                    pc_array = np.array([list(p) for p in pc_data[['x', 'y', 'z', 'intensity']]], dtype=np.float32).reshape(-1, 4)

                for i in range(pc_array.shape[0]):
                    x = pc_array[i, 0] * math.cos(self.lidar_rotation) - pc_array[i, 1] * math.sin(self.lidar_rotation) + 1.0
                    pc_array[i, 1] = pc_array[i, 0] * math.sin(self.lidar_rotation) + pc_array[i, 1] * math.cos(self.lidar_rotation)
                    pc_array[i, 0] = x
                    pc_array[i, 2] = pc_array[i, 2] + 0.45
                pc_array = pc_array.astype(np.float32)

                # Transform pointcloud from global to local frame
                if self.pc_in_global_frame:
                    pc_array = self._get_local_pcl(pc_array, pose)

                # Point-wise ego motion compensation (x, y and yaw rate)
                if self.ego_motion_compensate:
                    pc_array = self._compensate_point_cloud(pc_array, vx, vy, yawrate)
                pc_array = pc_array[:, :4]

                cones, yaw = self._get_local_cones(global_cones, pose, pc_array)
                odom = {
                    "timestamp": msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                    "x": pose.position.x,
                    "y": pose.position.y,
                    "z": pose.position.z,
                    "yaw": yaw,
                    "vx": vx,
                    "vy": vy,
                    "yawrate": yawrate,
                }
                pose = None

                yield pc_array, cones, odom

        del reader


    def gen_data(self, global_cones: List[Tuple[float, float]]):
        if not global_cones:
            messagebox.showinfo("Info", "No points to save.")
            return

        # Open folder dialog
        folder_path = filedialog.askdirectory()
        if not folder_path:
            return

        # Make folders
        os.makedirs(f"{folder_path}/points", exist_ok=True)
        os.makedirs(f"{folder_path}/labels", exist_ok=True)
        os.makedirs(f"{folder_path}/unlabeled_pc", exist_ok=True)

        data_info = {
            "info": {
                "tool": "generateDataset.py",
                "version": "0.1",
                "description": "Generated from MCAP file",
                "generated_on": dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                "last_updated": dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                "source": os.path.basename(self.selected_file),
                "user": gt.getuser(),
            },
            "data": [],
        }

        unlabeled_clouds = []
        for i, (pcd, cones, odom) in enumerate(self._read_data(global_cones)):
            # Ignore first one since it has not previous lidar scans
            if i == 0:
                continue

            print(f"Processing pointcloud {i}...")

            if i % self.label_every == 0:
                filenum = int(i / self.label_every)

                # Save point cloud to file
                with open(f"{folder_path}/points/{filenum:07d}.bin", "wb") as f:
                    f.write(pcd.tobytes())

                # Get the cones in the local frame
                if not cones:
                    x_values, y_values = [], []
                else:
                    x_values, y_values = zip(*cones)
                # Save to file
                with open(f"{folder_path}/labels/{filenum:07d}.txt", "w") as f:
                    for x, y in zip(x_values, y_values):
                        f.write(f"{x} {y} 0.0 0.23 0.23 0.33 0.0 Cone\n")

                # Compute checksums
                with open(f"{folder_path}/points/{filenum:07d}.bin", "rb") as f:
                    bytes = f.read()  # read file as bytes
                    pc_hash = hashlib.md5(bytes).hexdigest()
                with open(f"{folder_path}/labels/{filenum:07d}.txt", "rb") as f:
                    bytes = f.read()  # read file as bytes
                    lab_hash = hashlib.md5(bytes).hexdigest()

                # Add to metadata
                data_info["data"].append(
                    {
                        "id": filenum,
                        "odom": odom,
                        "pointcloud": {
                            "file": f"points/{filenum:07d}.bin",
                            "checksum": pc_hash,
                        },
                        "labels": {
                            "file": f"labels/{filenum:07d}.txt",
                            "checksum": lab_hash,
                        },
                        "unlabeled_clouds": unlabeled_clouds,
                    }
                )
                unlabeled_clouds = []
            else:
                filenum = int(i / self.label_every + 1)
                subfilename = int(i % self.label_every)

                # Save point cloud to file
                with open(f"{folder_path}/unlabeled_pc/{filenum:07d}_{subfilename:02d}.bin", "wb") as f:
                    f.write(pcd.tobytes())
                # Compute checksums
                with open(f"{folder_path}/unlabeled_pc/{filenum:07d}_{subfilename:02d}.bin", "rb") as f:
                    bytes = f.read()  # read file as bytes
                    pc_hash = hashlib.md5(bytes).hexdigest()

                unlabeled_clouds.append(
                    {
                        "file": f"unlabeled_pc/{filenum:07d}_{subfilename:02d}.bin",
                        "checksum": pc_hash,
                        "odom": odom,
                    }
                )

        with open(f"{folder_path}/metadata.json", "w") as f:
            json.dump(data_info, f, indent=4)

        messagebox.showinfo("Info", "Data saved!'")



class MapEditor:
    def __init__(self,
                 lidar_rotation: float = 1.5707,
                 label_every: int = 10,
                 pc_in_global_frame: bool = False,
                 ego_motion_compensate: bool = False
                 ):
        self.cones: List[Tuple[float, float]] = []
        self.selected_cones: Set[Tuple[float, float]] = set()
        self.selected_file: str = ""
        self.lidar_rotation: float = lidar_rotation
        self.label_every: int = label_every
        self.pc_in_global_frame: bool = pc_in_global_frame
        self.ego_motion_compensate: bool = ego_motion_compensate

        self.root = tk.Tk()
        self.root.title("Map editor")
        self.fig = Figure(figsize=(12, 8), dpi=100)
        self.ax = self.fig.add_subplot(111)
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.root)

        self.setup_ui()

    def setup_ui(self):
        # Setup UI components like buttons, labels, etc.
        top_frame = tk.Frame(self.root)
        top_frame.pack()

        open_button = tk.Button(top_frame, text="Open File", command=self.open_file)
        open_button.pack(side="left", padx=10)

        self.selected_file_label = tk.Label(top_frame, text=self.selected_file)
        self.selected_file_label.pack(side="left")

        self.canvas.get_tk_widget().pack()
        self.canvas.mpl_connect("button_press_event", self.on_click)

        # plot_button = tk.Button(self.root, text="Plot cones", command=self.plot_cones)
        remove_button = tk.Button(self.root, text="Remove Selected cones", command=self.remove_selected_cones)
        save_button = tk.Button(self.root, text="Export Dataset", command=self.gen_data)

        # plot_button.pack(side="left", padx=10)
        remove_button.pack(side="left", padx=10)
        save_button.pack(side="left", padx=10)

    def open_file(self):
        file_path = filedialog.askopenfilename(filetypes=[("MCAP bags", "*.mcap")])
        if not file_path:
            return

        self.selected_file = file_path

        # Read map from file
        topic, msg = self.read_map(self.selected_file)

        print("Extracting cones...")
        # Extract cones from the map
        self.cones = []
        for marker in msg.markers:
            self.cones.append((marker.pose.position.x, marker.pose.position.y))

        self.plot_cones()

    def read_map(self, selected_file: str) -> Tuple[Optional[str], Optional[Any]]:
        reader = rosbag2_py.SequentialReader()
        reader.open(
            rosbag2_py.StorageOptions(uri=selected_file, storage_id="mcap"),
            rosbag2_py.ConverterOptions(
                input_serialization_format="cdr", output_serialization_format="cdr"
            ),
        )

        topic_types = reader.get_all_topics_and_types()

        def typename(topic_name):
            for topic_type in topic_types:
                if topic_type.name == topic_name:
                    return topic_type.type
            raise ValueError(f"topic {topic_name} not in bag")

        top = None
        dat = None
        print("Reading bag...")

        popup = tk.Toplevel()
        tk.Label(popup, text="Reading bag...").grid(row=0,column=0)
        # Update number of messages read
        prog = 0
        progress = tk.StringVar()
        progress.set("Read {} messages".format(prog))
        tk.Label(popup, textvariable=progress).grid(row=1,column=0)
        popup.update_idletasks()

        while reader.has_next():
            topic, data, timestamp = reader.read_next()
            if topic == SLAM_MAP_TOPIC:
                top = topic
                dat = data
            
            prog += 1
            if prog % 100 == 0:
                progress.set("Read {} messages".format(prog))
                popup.update_idletasks()
        
        popup.destroy()

        print("Deserializing message...")

        if top is not None and dat is not None:
            msg_type = get_message(typename(top))
            msg = deserialize_message(dat, msg_type)
        del reader

        return top, msg

    def plot_cones(self):
        # Clear the existing plot
        self.ax.clear()

        # Get the cones to plot
        x_values, y_values = zip(*self.cones)

        # Plot the cones
        self.ax.scatter(x_values, y_values)

        # Highlight the selected point if any
        selected_x = [point[0] for point in self.selected_cones]
        selected_y = [point[1] for point in self.selected_cones]
        self.ax.scatter(selected_x, selected_y, color="red", label="Selected cones")

        # Set axis labels and title
        self.ax.set_xlabel("X")
        self.ax.set_ylabel("Y")
        self.ax.set_title("Point Plot")

        # Draw the updated plot
        self.canvas.draw()

    def on_click(self, event):
        if not self.cones:
            return

        # Get the coordinates of the clicked point
        x, y = event.xdata, event.ydata

        if x is not None and y is not None:
            clicked_point = None

            # Find the clicked point, if any
            min_dist = 100000.0
            for point in self.cones:
                dist = ((x - point[0]) ** 2 + (y - point[1]) ** 2) ** 0.5
                if dist < min_dist:
                    min_dist = dist
                    clicked_point = point

            if min_dist > 1.5:
                return

            if clicked_point:
                if clicked_point in self.selected_cones:
                    self.selected_cones.remove(clicked_point)
                else:
                    self.selected_cones.add(clicked_point)
                self.plot_cones()

    def remove_selected_cones(self):
        if self.selected_cones:
            self.cones = [cone for cone in self.cones if cone not in self.selected_cones]
            self.selected_cones = set()
            self.plot_cones()
        else:
            messagebox.showinfo("Info", "Select cones to remove.")

    def gen_data(self):
        datasetGenerator = DatasetGenerator(self.selected_file, self.lidar_rotation,
                                            self.label_every, self.pc_in_global_frame, 
                                            self.ego_motion_compensate)
        datasetGenerator.gen_data(self.cones)

    def run(self):
        self.root.mainloop()


def parse_args():
    parser = argparse.ArgumentParser(description="Map editor")
    parser.add_argument("--lidar-rotation", type=float, default=-1.5707, help="Lidar rotation")
    parser.add_argument("--label-every", type=int, default=10, help="Label every nth point cloud")
    parser.add_argument("--pc-in-global-frame", action="store_true", help="Indicates that the pointcloud is in global frame. This will transform it to local frame.")
    parser.add_argument("--ego-motion-compensate", action="store_true", help="Ego-motion compensate the point cloud")

    return parser.parse_args()


def main():
    args = parse_args()

    editor = MapEditor(args.lidar_rotation, args.label_every, args.pc_in_global_frame, args.ego_motion_compensate)
    editor.run()

if __name__ == "__main__":
    main()

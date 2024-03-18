import os
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

import tf2_ros
import rosbag2_py
import sensor_msgs_py.point_cloud2 as pc2
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from tf_transformations import euler_from_quaternion

SLAM_MAP_TOPIC = "/graphslam/cones/global_viz"
LAP_COUNT_TOPIC = "/graphslam/lap_count"
CAR_STATE_TOPIC = "/ekf/car_state"
POINTCLOUD_TOPIC = "/limovelo/full_pcl"

LOCAL_FRAME = "car" # Commonly named base_link
CONE_FRAME = "map" # This is the frame name used for the cones topic

# Enum for cone color
UNKOWN_C = 0
YELLOR_C = 1
BLUE_C = 2
BIG_C = 3
ORANGE_C = 4

class SceneGenerator:
    def __init__(self,
                 selected_file: str,
                 label_every: int = 10,
                 ego_motion_compensate: bool = False
                 ):
        self.selected_file: str = selected_file
        self.label_every: int = label_every
        self.ego_motion_compensate: bool = ego_motion_compensate

        self.tf_buffer = tf2_ros.Buffer()

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

    def _get_local_cones(self, cones: List[Tuple[float, float, int]], transform,  pc_array: np.ndarray) -> Tuple[List[Tuple[float, float, int]], float]:
        # Extract pose components
        orientation_list = [
            transform.rotation.x,
            transform.rotation.y,
            transform.rotation.z,
            transform.rotation.w,
        ]
        (_, _, yaw) = euler_from_quaternion(orientation_list)

        local_cones = []
        colors = []
        for cone in cones:
            local_cones.append([cone[0], cone[1], 0])
            colors.append(cone[2])
        
        np_cones = np.array(local_cones)

        # Tranform pointcloud from global to local frame
        np_cones = self._transform_array(np_cones, transform)

        local_cones = []
        for i in range(np_cones.shape[0]):
            local_cones.append([np_cones[i, 0], np_cones[i, 1], colors[i]])

        return local_cones, yaw

    def _get_mat_from_quat(self, quaternion: np.ndarray) -> np.ndarray:
        """
        note:: This was copied from https://github.com/ros2/geometry2/blob/iron/tf2_sensor_msgs/tf2_sensor_msgs/tf2_sensor_msgs.py
               since it is not available for ROS2 versions < Iron

        Convert a quaternion to a rotation matrix.

        This method is currently needed because transforms3d is not released as a `.dep` and
        would require user interaction to set up.

        For reference see: https://github.com/matthew-brett/transforms3d/blob/
        f185e866ecccb66c545559bc9f2e19cb5025e0ab/transforms3d/quaternions.py#L101

        :param quaternion: A numpy array containing the w, x, y, and z components of the quaternion
        :returns: An array containing an X, Y, and Z translation component
        """
        Nq = np.sum(np.square(quaternion))
        if Nq < np.finfo(np.float64).eps:
            return np.eye(3)

        XYZ = quaternion[1:] * 2.0 / Nq
        wXYZ = XYZ * quaternion[0]
        xXYZ = XYZ * quaternion[1]
        yYZ = XYZ[1:] * quaternion[2]
        zZ = XYZ[2] * quaternion[3]

        return np.array(
            [[1.0-(yYZ[0]+zZ), xXYZ[1]-wXYZ[2], xXYZ[2]+wXYZ[1]],
            [xXYZ[1]+wXYZ[2], 1.0-(xXYZ[0]+zZ), yYZ[1]-wXYZ[0]],
            [xXYZ[2]-wXYZ[1], yYZ[1]+wXYZ[0], 1.0-(xXYZ[0]+yYZ[0])]])

    def _transform_array(self,
            point_cloud: np.ndarray,
            transform) -> np.ndarray:
        """
        note:: This was copied from https://github.com/ros2/geometry2/blob/iron/tf2_sensor_msgs/tf2_sensor_msgs/tf2_sensor_msgs.py
               since it is not available for ROS2 versions < Iron

        Transform a bulk of points from an numpy array using a provided `Transform`.

        :param point_cloud: nx3 Array of points where n is the number of points
        :param transform: TF2 transform used for the transformation
        :returns: Array with the same shape as the input array, but with the transformation applied
        """
        # Build affine transformation
        transform_translation = np.array([
            transform.translation.x,
            transform.translation.y,
            transform.translation.z
        ])
        transform_rotation_matrix = self._get_mat_from_quat(
            np.array([
                transform.rotation.w,
                transform.rotation.x,
                transform.rotation.y,
                transform.rotation.z
            ]))

        # "Batched" matmul meaning a matmul for each point
        # First we offset all points by the translation part
        # followed by a rotation using the rotation matrix
        return np.einsum(
            'ij, pj -> pi',
            transform_rotation_matrix,
            point_cloud) + transform_translation

    def _read_data(self,
                   global_cones: List[Tuple[float, float, int]]
                   ) -> Generator[Tuple[np.ndarray, List[Tuple[float, float, int]], Dict[str, Any]], None, None]:
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
        vx = 0.0
        vy = 0.0
        yawrate = 0.0
        while reader.has_next():
            topic, data, timestamp = reader.read_next()
            
            # TF2 messages
            if topic == "/tf":
                msg_type = get_message(typename(topic))
                msg = deserialize_message(data, msg_type)
                for transform in msg.transforms:
                    self.tf_buffer.set_transform(transform, "default_authority")
            if topic == "/tf_static":
                msg_type = get_message(typename(topic))
                msg = deserialize_message(data, msg_type)
                for transform in msg.transforms:
                    self.tf_buffer.set_transform_static(transform, "default_authority")

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

            if lap < 1: # Skip the first lap
                continue

            # Extract Point Cloud
            if topic == POINTCLOUD_TOPIC:
                
                msg_type = get_message(typename(topic))
                msg = deserialize_message(data, msg_type)

                try:
                    transform = self.tf_buffer.lookup_transform(LOCAL_FRAME, msg.header.frame_id, msg.header.stamp)
                    # msgTf = do_transform_cloud(msg, transform)
                    # msgTf = self.tf_buffer.transform(msg, LOCAL_FRAME)
                except tf2_ros.TransformException as ex:
                    print("Got an exception while looking up transform: " + str(ex))
                    continue


                if self.ego_motion_compensate:
                    pc_data = pc2.read_points(msg, field_names=("x", "y", "z", "intensity", "t"), skip_nans=True)
                    pc_array = np.array([list(p) for p in pc_data[['x', 'y', 'z', 'intensity', 't']]], dtype=np.float32).reshape(-1, 5)
                else:
                    pc_data = pc2.read_points(msg, field_names=("x", "y", "z", "intensity"), skip_nans=True)
                    pc_array = np.array([list(p) for p in pc_data[['x', 'y', 'z', 'intensity']]], dtype=np.float32).reshape(-1, 4)

                pc_array = pc_array.astype(np.float32)

                msgTf = self._transform_array(pc_array[:, :3], transform.transform)
                pc_array[:, :3] = msgTf

                # Point-wise ego motion compensation (x, y and yaw rate)
                if self.ego_motion_compensate:
                    pc_array = self._compensate_point_cloud(pc_array, vx, vy, yawrate)
                pc_array = pc_array[:, :4]

                try:
                    transform = self.tf_buffer.lookup_transform(LOCAL_FRAME, CONE_FRAME, msg.header.stamp)
                except tf2_ros.TransformException as ex:
                    print("Got an exception while looking up transform: " + str(ex))
                    continue

                cones, yaw = self._get_local_cones(global_cones, transform.transform, pc_array)
                odom = {
                    "timestamp": msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                    "x": transform.transform.translation.x,
                    "y": transform.transform.translation.y,
                    "z": transform.transform.translation.z,
                    "yaw": yaw,
                    "vx": vx,
                    "vy": vy,
                    "yawrate": yawrate,
                }

                yield pc_array, cones, odom

        del reader


    def gen_data(self, global_cones: List[Tuple[float, float, int]]):
        if not global_cones:
            messagebox.showinfo("Info", "No points to save.")
            return
        
        # make sure the user understands the coloring need
        proceed = messagebox.askyesno("Proceed?", "Any cones with no color (purple) will be ignored. Do you want to proceed?")
        if not proceed:
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
                "tool": "generate_scene.py",
                "version": "0.2",
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
                    x_values, y_values, colors = [], [], []
                else:
                    x_values, y_values, colors = zip(*cones)
                # Save to file
                with open(f"{folder_path}/labels/{filenum:07d}.txt", "w") as f:
                    for x, y, c in zip(x_values, y_values, colors):
                        if c == YELLOR_C:
                            cone_type = 'Cone_Yellow'
                        elif c == BLUE_C:
                            cone_type = 'Cone_Blue'
                        elif c == BIG_C:
                            cone_type = 'Cone_Big'
                        elif c == ORANGE_C:
                            cone_type = 'Cone_Orange'
                        else:
                            continue
                        f.write(f"{x} {y} 0.0 0.23 0.23 0.33 0.0 {cone_type}\n")

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
                 label_every: int = 10,
                 ego_motion_compensate: bool = False
                 ):
        self.cones: List[Tuple[float, float, int]] = []
        self.selected_cones: Set[Tuple[float, float, int]] = set()
        self.selected_file: str = ""
        self.label_every: int = label_every
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

        # create buttons for setting colors
        yellow_btn = tk.Button(self.root, text="Set Yellow", command=lambda: self.set_color(YELLOR_C))
        blue_btn = tk.Button(self.root, text="Set Blue", command=lambda: self.set_color(BLUE_C))
        big_btn = tk.Button(self.root, text="Set Big", command=lambda: self.set_color(BIG_C))
        orange_btn = tk.Button(self.root, text="Set Orange", command=lambda: self.set_color(ORANGE_C))

        # plot_button.pack(side="left", padx=10)
        remove_button.pack(side="left", padx=10)
        save_button.pack(side="left", padx=10)
        yellow_btn.pack(side="left", padx=10)
        blue_btn.pack(side="left", padx=10)
        big_btn.pack(side="left", padx=10)
        orange_btn.pack(side="left", padx=10)

    def set_color(self, color: int):
        if not self.selected_cones:
            messagebox.showinfo("Info", "Select cones to set color.")
            return

        for cone in self.selected_cones:
            self.cones.remove(cone)
            self.cones.append((cone[0], cone[1], color))
        self.selected_cones = set()
        self.plot_cones()

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
            self.cones.append((marker.pose.position.x, marker.pose.position.y, UNKOWN_C))

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
        x_values, y_values, _ = zip(*self.cones)

        # Plot the cones
        colors = []
        for cone in self.cones:
            if cone[2] == YELLOR_C:
                colors.append("yellow")
            elif cone[2] == BLUE_C:
                colors.append("blue")
            elif cone[2] == BIG_C:
                colors.append("black")
            elif cone[2] == ORANGE_C:
                colors.append("orange")
            else:
                colors.append("purple")
        self.ax.scatter(x_values, y_values, color=colors, label="Cones")

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
        sceneGenerator = SceneGenerator(self.selected_file, self.label_every, self.ego_motion_compensate)
        sceneGenerator.gen_data(self.cones)

    def run(self):
        self.root.mainloop()


def parse_args():
    parser = argparse.ArgumentParser(description="Map editor")
    parser.add_argument("--label-every", type=int, default=20, help="Label every nth point cloud")
    parser.add_argument("--ego-motion-compensate", action="store_true", help="Ego-motion compensate the point cloud")

    return parser.parse_args()


def main():
    args = parse_args()

    editor = MapEditor(args.label_every, args.ego_motion_compensate)
    editor.run()

if __name__ == "__main__":
    main()


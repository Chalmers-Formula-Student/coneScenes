import os
import json
import tkinter as tk
from tkinter import ttk
from tkinter import filedialog
import zipfile
import numpy as np


class DatasetBuilder:
    def __init__(self, dir: str, JSON_FILE: str):
        self.dir = dir
        self.JSON_FILE = JSON_FILE

        self.scenes = []
        self.selections = []

    def build_dataset(self):

        def set_dataset(dataset_type):
            selected_items = table.selection()
            for item in selected_items:
                table.set(item, column='Split', value=dataset_type)
                self.selections[int(item)] = dataset_type

        metadata_path = os.path.abspath(self.JSON_FILE)

        with open(metadata_path, 'r') as json_file:
            data = json.load(json_file)["data"]

            # Create the main window
            root = tk.Tk()
            root.title("Scene Exporter")

            # Configure resizing behavior
            root.columnconfigure(0, weight=1)
            root.rowconfigure(0, weight=1)

            # Create a frame for the table
            table_frame = ttk.Frame(root, padding=10)
            table_frame.pack(expand=True, fill="both")

            # Create a treeview for the table
            table = ttk.Treeview(table_frame, columns=("Split", "Scene", "Team", "LiDAR", "Description"), show="headings")
            table.heading("Split", text="Split")
            table.heading("Scene", text="Scene")
            table.heading("Team", text="Team")
            table.heading("LiDAR", text="LiDAR")
            table.heading("Description", text="Description")
            table.column("Split", width=100)

            # Set stretch=True for auto-dimensioning columns
            for col in table['columns']:
                table.column(col, stretch=True)
            
            table.pack(expand=True, fill="both")

            # Create buttons for setting dataset
            button_frame = ttk.Frame(root)
            button_frame.pack(fill="x", pady=10)

            set_train_button = ttk.Button(button_frame, text="Set Train", command=lambda: set_dataset("train"))
            set_train_button.pack(side="left", padx=5)

            set_val_button = ttk.Button(button_frame, text="Set Val", command=lambda: set_dataset("val"))
            set_val_button.pack(side="left", padx=5)

            set_test_button = ttk.Button(button_frame, text="Set Test", command=lambda: set_dataset("test"))
            set_test_button.pack(side="left", padx=5)

            export_button = ttk.Button(button_frame, text="Export", command=self._export)
            export_button.pack(side="right", padx=5)

            # Populate the table with data
            for i, item in enumerate(data):
                table.insert("", tk.END, values=("unused", item["name"], item["team"], item["lidar"], item["description"]), iid=i)
                self.scenes.append(item)
                self.selections.append("unused")


        root.mainloop()


    def _export(self):
        print("Exporting scenes to dataset...")

        # Ask directory to export to
        export_dir = filedialog.askdirectory()
        print(f"Exporting to {export_dir}")

        # Create folder structure
        os.makedirs(os.path.join(export_dir, "points"))
        os.makedirs(os.path.join(export_dir, "labels"))
        os.makedirs(os.path.join(export_dir, "ImageSets"))
        os.makedirs(os.path.join(export_dir, "tmp"))

        train_ids = []
        val_ids = []
        test_ids = []

        count = 0
        # Get the scenes to export
        for i, scene in enumerate(self.scenes):
            if self.selections[i] != "unused":
                print(f"Exporting {scene['name']} to {self.selections[i]}")

                print(f"Extracting {scene['file_path']}...")
                with zipfile.ZipFile(os.path.join(self.dir, scene['file_path']), 'r') as zip_ref:
                    zip_ref.extractall(os.path.join(export_dir, "tmp"))

                new_ids = self._copy_files(os.path.join(export_dir, "tmp"), os.path.join(export_dir, "points"), os.path.join(export_dir, "labels"), count)
                os.system(f"rm -rf {os.path.join(export_dir, 'tmp')}")

                count = new_ids[-1] + 1

                if self.selections[i] == "train":
                    train_ids.extend(new_ids)
                elif self.selections[i] == "val":
                    val_ids.extend(new_ids)
                elif self.selections[i] == "test":
                    test_ids.extend(new_ids)

        train_ids = list(map(str, train_ids))
        val_ids = list(map(str, val_ids))
        test_ids = list(map(str, test_ids))

        with open(os.path.join(export_dir, 'ImageSets', 'train.txt'), 'w') as file:
            file.write('\n'.join(train_ids))
        with open(os.path.join(export_dir, 'ImageSets', 'val.txt'), 'w') as file:
            file.write('\n'.join(val_ids))
        with open(os.path.join(export_dir, 'ImageSets', 'test.txt'), 'w') as file:
            file.write('\n'.join(test_ids))

        os.system(f"rm -rf {os.path.join(export_dir, 'tmp')}")

        print("Export complete.")

    
    def _copy_files(self, src_dir, points_dir, labels_dir, start_id):
        # Open metadata file
        with open(os.path.join(src_dir, "metadata.json"), 'r') as json_file:
            data = json.load(json_file)

            ids = []
            for i, item in enumerate(data["data"]):
                self._filter_and_copy(item, src_dir, points_dir, labels_dir, start_id + i)
                ids.append(start_id + i)
        
        return ids


    def _filter_and_copy(self, item, src_dir, points_dir, labels_dir, id):
        """Opens the pointcloud and labels and filters any label that does not have points in the pointcloud."""
        # Read the pointcloud
        pointcloud = np.fromfile(os.path.join(src_dir, item["pointcloud"]["file"]), dtype=np.float32).reshape(-1, 4)

        # Read the labels
        with open(os.path.join(src_dir, item["labels"]["file"]), 'r') as file:
            lines = file.readlines()

        labels = []
        for line in lines:
            labels.append(line.strip().split())

        # Filter the labels
        filtered_labels = []
        for label in labels:
            # find minimum z of pointcloud within the bounding box
            x, y, z, w, l, h = map(float, label[:6])

            # Filter the points
            filtered_points = pointcloud[(pointcloud[:, 0] > x - w/2) & 
                                         (pointcloud[:, 0] < x + w/2) & 
                                         (pointcloud[:, 1] > y - l/2) & 
                                         (pointcloud[:, 1] < y + l/2)
                                         ]

            if len(filtered_points) == 0:
                continue

            # find minimum z of pointcloud within the bounding box
            min_z = np.min(filtered_points[:, 2])

            # set the label to the minimum z (drops boudning box to the ground)
            label[2] = min_z.item() - 0.02 # 2cm below the lowest point
            filtered_labels.append(label)

        # Copy the pointcloud
        os.system(f"cp {os.path.join(src_dir, item['pointcloud']['file'])} {os.path.join(points_dir, f'{id:07}.bin')}")

        # Write the labels
        with open(os.path.join(labels_dir, f'{id:07}.txt'), 'w') as file:
            for label in filtered_labels:
                file.write(" ".join(map(str, label)) + '\n')

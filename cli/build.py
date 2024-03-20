import os
import json
import tkinter as tk
from tkinter import ttk

def build_dataset(dir: str, JSON_FILE: str):

    def set_dataset(dataset_type):
        selected_items = table.selection()
        for item in selected_items:
            table.set(item, column='Split', value=dataset_type)

    metadata_path = os.path.abspath(JSON_FILE)

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

        export_button = ttk.Button(button_frame, text="Export", command=_export())
        export_button.pack(side="right", padx=5)

        # Populate the table with data
        for i, item in enumerate(data):
            table.insert("", tk.END, values=("unused", item["name"], item["team"], item["lidar"], item["description"]), iid=i)

    root.mainloop()


def _export():
    print("Exporting scenes to dataset...")

    # Ask directory to export to
    export_dir = tk.filedialog.askdirectory()
    print(f"Exporting to {export_dir}")

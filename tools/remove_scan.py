import os
import json
import tkinter as tk
from tkinter import filedialog


def remove_scan(dir: str, file_id: int):
        """ Save the labels to same file. """
        
        # edit json metadata checksum
        with open(f"{dir}/metadata.json", 'r+') as f:
            data = json.load(f)
            # find idx from current filename
            print(f"Id {file_id} to be removed")

            # find idx from json
            for idx, entry in enumerate(data['data']):
                if entry['id'] == file_id:
                    print(f"Found idx {idx} to be removed")
                    break
            else:
                print(f"Id {file_id} not found")
                return
            
            # remove files
            os.remove(f"{dir}/{data['data'][idx]['pointcloud']['file']}")
            os.remove(f"{dir}/{data['data'][idx]['labels']['file']}")

            # remove entry
            del data['data'][idx]

            f.seek(0)        # <--- should reset file position to the beginning.
            json.dump(data, f, indent=4)
            f.truncate()     # remove remaining part

if __name__ == '__main__':

    # Folder selection dialog
    root = tk.Tk()
    root.withdraw()
    folder_selected = filedialog.askdirectory()

    print(folder_selected)

    while True:
        # Ask for an ID to remove
        id = input("Enter ID to remove: ")

        # convert to int
        try:
            id = int(id)
        except ValueError:
            print("Invalid ID")
            continue
        
        # Remove the scan
        remove_scan(folder_selected, id)

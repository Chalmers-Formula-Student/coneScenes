import os
import json
import click
import pkg_resources
import zipfile
import datetime as dt

from cli.checksum import get_checksum

def add_scene(file: str, JSON_FILE: str):
    """Add a new data file to the dataset.
    The file should be a zip file containing the dataset."""

    # Calculate the checksum
    checksum = get_checksum(file)

    metadata_path = pkg_resources.resource_filename(__name__, JSON_FILE)
    metadata_path = os.path.abspath(metadata_path)
    print(metadata_path)

    with open(metadata_path, 'r') as json_file:
        data_info = json.load(json_file)
    
    # Step 1: Check if the file is already in the dataset
    for entry in data_info['data']:
        if entry['file_path'] == os.path.basename(file):
            click.echo(f"File {file} is already in the dataset.")
            return
    
    # Step 2: Ask the user to input information about the scene
    temp_name = os.path.basename(file).split('.')[-2]
    name = click.prompt("Enter the name you want to give to the scene", default=os.path.basename(temp_name), type=str)
    team = click.prompt("Enter the team name", default="", type=str)
    description = click.prompt("Enter a description for the scene", default="", type=str)
    label_every = click.prompt("Label every nth frame (e.g. 20)", default=0, type=int)

    click.echo("Information about the LiDAR used:")
    lidar_make = click.prompt("LiDAR manufacturer?", default="", type=str)
    lidar_model = click.prompt("Enter the model of the LiDAR (just model, e.g. Ouster OS1-64 bellow horizon just say OS1 since 64 is the beam count indicator)", default="", type=str)
    lidar_config = click.prompt("Enter the beam configuration of the LiDAR or leave empty if not applicable (e.g. Ouster has Gradient, uniform, bellow horizon configs for beam distribuition)", default="", type=str)
    lidar_vres = click.prompt("Vertical resolution of the LiDAR", default=0, type=int)
    lidar_hres = click.prompt("Horizontal resolution of the LiDAR", default=0, type=int)
    lidar_fov = click.prompt("Field of view of the LiDAR (e.g. 360)", default=0, type=int)
    lidar_freq = click.prompt("Frequency of the LiDAR in Hz", default=0, type=int)

    lidar_loc = click.prompt("LiDAR in the car (e.g. nose, mainhoop, frontwing...)", default="", type=str)

    # Step 3: Count the number of frames in the dataset
    labeled_frames, unlabeled_frames = _count_frames(file)

    # TODO: add lidar scan number counter
    data_info['data'].append({
        "name": name,
        "team": team,
        "description": description,
        "label_every": label_every,
        "lidar": {
            "manufacturer": lidar_make,
            "model": lidar_model,
            "config": lidar_config,
            "vres": lidar_vres,
            "hres": lidar_hres,
            "fov": lidar_fov,
            "frequency": lidar_freq,
            "location": lidar_loc,
        },
        "num_labelled_frames": labeled_frames,
        "num_unlabelled_frames": unlabeled_frames,
        "file_path": os.path.basename(file),
        "checksum": checksum
    })

    major, minor = map(int, data_info['version'].split('.'))
    minor += 1  # Increment the minor version

    # Step 4: Update the JSON data
    data_info['version'] = f'{major}.{minor}'
    data_info['last_updated'] = dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    print(data_info)
    
    with open(metadata_path, 'w') as json_file:
        json.dump(data_info, json_file, indent=4)


def _count_frames(file: str):
    """Count the number of LiDAR scans in the given file."""
    
    with zipfile.ZipFile(file, 'r') as zip_ref:
        files = zip_ref.namelist()
        label_files = [f for f in files if f.endswith('.txt')]
        lidar_files = [f for f in files if f.endswith('.bin')]

    labeled_frames = len(label_files)
    unlabled_frames = len(lidar_files) - labeled_frames

    return labeled_frames, unlabled_frames
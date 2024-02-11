# Create a new Python script, e.g., cfsds.py
import click
import json
import os
import pkg_resources
import datetime as dt
from termcolor import colored

from cli.checksum import get_checksum

# The JSON file containing file paths and checksums
JSON_FILE = '../data.json'

@click.group()
def conescenes():
    """coneScenes CLI - A CLI tool to help manage the dataset."""
    pass

@conescenes.command()
@click.argument('dir', type=click.Path(exists=True))
def doctor(dir):
    """Check for missing or corrupted data files."""

    metadata_path = pkg_resources.resource_filename(__name__, JSON_FILE)
    metadata_path = os.path.abspath(metadata_path)

    with open(metadata_path, 'r') as json_file:
        data_info = json.load(json_file)

    data_info = data_info['data']

    missing_files = []
    corrupted_files = []
    
    for entry in data_info:
        file_path = entry['file_path']
        checksum = entry['checksum']
        
        file_abs_path = os.path.join(dir, os.path.basename(file_path))
        
        if not os.path.exists(file_abs_path):
            missing_files.append(f"{file_path}")

            colored_failed = colored("FAILED", "black", "on_red")
            print(f"{colored_failed} File {file_path} does not exist in the directory given")
        else:
            # Calculate checksum for the file in the folder
            file_checksum = get_checksum(file_abs_path)
            if file_checksum != checksum:
                corrupted_files.append(f"{file_path}")

                colored_failed = colored("FAILED", "black", "on_red")
                print(f"{colored_failed} File {file_path} exists but is corrupted")
            else:
                # Color only the word "FAILED"
                colored_failed = colored("PASSED", "black", "on_green")
                print(f"{colored_failed} File {file_path} exists and checksum matches")
    
    if missing_files:
        click.echo("Missing files:")
        for file in missing_files:
            click.echo(f"  {file}")
    
    if corrupted_files:
        click.echo("Corrupted files:")
        for file in corrupted_files:
            click.echo(f"  {file}")
    
    if not missing_files and not corrupted_files:
        click.echo("All files are present and checksums match.")


@conescenes.command()
@click.argument('file', type=click.Path(exists=True))
def add(file):
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
    name = click.prompt("Enter the name you want to give to the scene", default=os.path.basename(file))
    team = click.prompt("Enter the team name", default="")
    description = click.prompt("Enter a description for the scene", default="")
    lidar = click.confirm("What LiDAR was used?")
    lidar_loc = click.prompt("Enter the location of the LiDAR in the car (e.g. nose)", default="")

    # TODO: add lidar scan number counter
    data_info['data'].append({
        "name": name,
        "team": team,
        "description": description,
        "lidar": lidar,
        "lidar_location": lidar_loc,
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


conescenes.add_command(doctor)
conescenes.add_command(add)

if __name__ == '__main__':
    conescenes()

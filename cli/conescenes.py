# Create a new Python script, e.g., cfsds.py
import click
import json
import os
import pkg_resources
import datetime as dt
from termcolor import colored

from cli.build import build_dataset
from cli.add import add_scene

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

    # Check that file exists
    if not os.path.exists(file):
        click.echo(f"File {file} does not exist.")
        return
    
    # Check that file is a zip file
    if not file.endswith('.zip'):
        click.echo(f"File {file} is not a zip file.")
        return
    
    add_scene(file, JSON_FILE)
    

@conescenes.command()
def build():
    """Select scenes and build a new dataset."""
    metadata_path = pkg_resources.resource_filename(__name__, JSON_FILE)

    build_dataset("", metadata_path)


conescenes.add_command(doctor)
conescenes.add_command(add)

if __name__ == '__main__':
    conescenes()

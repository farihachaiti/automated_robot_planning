#!/usr/bin/env python3
import subprocess

def run_dos2unix(file_path):
    try:
        # Run dos2unix command with the file path as argument
        subprocess.run(["dos2unix", file_path], check=True)
        print("dos2unix conversion successful.")
    except subprocess.CalledProcessError as e:
        # If an error occurs, print the error message
        print(f"Error: {e}")

# Example usage
if __name__ == "__main__":
    file_path = "create_map_"  # Replace this with your file path
    run_dos2unix(file_path)

import os
import shutil
from pathlib import Path

# Define source folders and destination folder
source_dirs = ['/home/student/Documents/costum_dataset/warehouse_tare/images/cam0', '/home/student/Documents/costum_dataset/warehouse_tare/images/cam1', '/home/student/Documents/costum_dataset/warehouse_tare/images/cam2', '/home/student/Documents/costum_dataset/warehouse_tare/images/cam3']
destination_dir = '/home/student/Documents/costum_dataset/warehouse_tare/merged_images'

# Create destination directory if it doesn't exist
os.makedirs(destination_dir, exist_ok=True)

# Initialize image counter
img_counter = 1

# Supported image extensions
valid_extensions = {'.jpg', '.jpeg', '.png', '.bmp', '.tiff'}

# Iterate through each folder
for folder in source_dirs:
    for filename in sorted(os.listdir(folder)):
        file_path = os.path.join(folder, filename)
        ext = Path(filename).suffix.lower()
        if os.path.isfile(file_path) and ext in valid_extensions:
            # Define new filename with leading zeros
            new_filename = f"{img_counter:03d}.jpg"
            new_file_path = os.path.join(destination_dir, new_filename)
            shutil.copyfile(file_path, new_file_path)
            img_counter += 1

print(f"Copied and renamed {img_counter - 1} images to '{destination_dir}'")

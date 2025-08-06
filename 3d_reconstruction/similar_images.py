import os
import cv2
import numpy as np
from skimage.metrics import structural_similarity as ssim
from multiprocessing import Pool, cpu_count
from itertools import combinations
from functools import partial

def load_and_resize_gray(image_path, size=(256, 256)):
    img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        return None
    return cv2.resize(img, size)

def compare_pair(pair, images, threshold):
    path1, path2 = pair
    img1 = images[path1]
    img2 = images[path2]
    if img1 is None or img2 is None:
        return None
    score, _ = ssim(img1, img2, full=True)
    if score >= threshold:
        return path2  # Mark second image for deletion
    return None

def remove_similar_images_parallel(folder_path, threshold=0.95):
    print(f"Scanning folder: {folder_path}")
    image_files = [f for f in os.listdir(folder_path) if f.lower().endswith(('.jpg', '.jpeg', '.png'))]
    image_paths = [os.path.join(folder_path, f) for f in image_files]

    print(f"Loading and resizing {len(image_paths)} images...")
    images = {path: load_and_resize_gray(path) for path in image_paths}

    pairs = list(combinations(images.keys(), 2))
    print(f"Comparing {len(pairs)} image pairs using {cpu_count()} cores...")

    with Pool(cpu_count()) as pool:
        compare_func = partial(compare_pair, images=images, threshold=threshold)
        results = pool.map(compare_func, pairs)

    to_delete = set(filter(None, results))
    print(f"\n{len(to_delete)} duplicates found. Deleting...")

    for path in to_delete:
        try:
            os.remove(path)
            print(f"Deleted: {path}")
        except Exception as e:
            print(f"Error deleting {path}: {e}")

    print(f"\nDone. {len(to_delete)} files deleted.")

# Example usage
if __name__ == "__main__":
    folder = "/home/student/Documents/costum_dataset/warehouse_tare/merged_images"  # ← CHANGE THIS
    remove_similar_images_parallel(folder, threshold=0.95)

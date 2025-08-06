import cv2
import os

input_dir = '/home/student/Documents/costum_dataset/garage/frames'
output_dir = '/home/student/Documents/costum_dataset/garage/images'
threshold = 130.0  # Tune this value for your video

os.makedirs(output_dir, exist_ok=True)
frame_counter = 1  # continuous numbering starts from 1
for f in sorted(os.listdir(input_dir)):
    path = os.path.join(input_dir, f)
    img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        continue
    variance = cv2.Laplacian(img, cv2.CV_64F).var()
    print(f"{f}: blur={variance:.2f}")
    if variance > threshold:
        new_filename = f"frame_{frame_counter:05d}.png"
        cv2.imwrite(os.path.join(output_dir, new_filename), cv2.imread(path))
        frame_counter += 1

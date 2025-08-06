## 1. Take a video of the scene

## 2. Preparation

### 1.Get frames
```bash
mkdir frames
ffmpeg -i your_video.mp4 -vf fps=4 frames/frame_%05d.png
```

### 2. Compress the frames
```bash
cd frames
mogrify -resize 33% *.png
```
Note: For training triangle splatting models on RTX 4080 (16GB), the upper limit of images is: 150 images, each ~1MB.

### 3. Ignore blurry images
```bash
python check_blurry.py
```

### 4. Colmap reconstruction

### 5. Colmap distorter
```bash
colmap image_undistorter \
    --image_path path/to/images \
    --input_path path/to/sparse/model \
    --output_path path/to/undistorted \
    --output_type COLMAP \
    --max_image_size 2000
```
Make sure the folder looks like this: /sparse/0/...

## 3. 3D reconstruction

### 1. Colmap photogrammetry
Build a dense model in Colmap gui.

### 2. SuGaR
```bash
python train_full_pipeline.py -s <path to COLMAP dataset> -r dn_consistency --high_poly True --export_obj True
```

### 3. 3DGRUT
#### Training and exporting usdz:
```bash
docker start -ai determined_montalcini
python train.py --config-name apps/colmap_3dgut.yaml path=datasets/garage/ out_dir=runs experiment_name=garage dataset.downsample_factor=1 export_usdz.enabled=true
```

#### Visualizing training result:
```bash
python train.py --config-name apps/colmap_3dgut.yaml path=datasets/warehouse_manual/ with_gui=True test_last=False export_ingp.enabled=False resume=runs/warehouse_manual/warehouse_manual-1407_144510/ckpt_last.pt

python train.py --config-name apps/colmap_3dgut.yaml path=datasets/warehouse/ with_gui=True test_last=False export_ingp.enabled=False resume=runs/warehouse/warehouse-1407_071323/ckpt_last.pt
```
### 4. Triangle splatting
#### Training
Before training, use following to avoid VRAM fragmentation issues:
```bash
export PYTORCH_CUDA_ALLOC_CONF=expandable_segments:True
```

Trainig:
```bash
python train.py -s /home/student/Documents/triangle-splatting/datasets/tables2 -m /home/student/Documents/triangle-splatting/runs/tables2 --eval --save_iterations 10000 13000 15000 18000 20000
```
#### Mesh
```bash
python mesh.py   --model_path /home/student/Documents/triangle-splatting/runs/tables2   --source_path /home/student/Documents/triangle-splatting/datasets/tables2
```

#### Game engine
```bash
python create_off.py --checkpoint_path /home/student/Documents/triangle-splatting/runs/tables2/point_cloud/iteration_18000/point_cloud_state_dict.pt --output_name mesh_colored.off
```

# Difix3D 
```bash
CUDA_VISIBLE_DEVICES=0 python examples/gsplat/simple_trainer_difix3d.py default \
    --data_dir data/warehouse --data_factor 1 \
    --result_dir outputs/difix3d/gsplat/warehouse --no-normalize-world-space --test_every 1 --ckpt checkpoints_gs/warehouse.pt
```

TODO: ZED + AEDE wall scanning
1. ZED point cloud filtering/YOLO wall detection/ZED+segformer?, get wall point cloud
2. waypoint generation and following
3. Coverage check: infomation gain (area/length of walls) converge
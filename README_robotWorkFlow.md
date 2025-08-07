0. Pull the repos using vcs tool
```bash
export PATH=$HOME/.local/bin:$PATH
source ~/.bashrc  # or: source ~/.zshrc
vcs import < ~/Documents/Production/docker/volumes/ros2_ws/src/aiina/src/aiina.repos
```

1. Run the container
```bash
cd /home/yuanyan/Documents/Production/docker
make shell
source install/setup.bash
```

2. Start all services
```bash
start
```
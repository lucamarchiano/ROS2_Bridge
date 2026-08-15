import docker
import time

CONTAINER_NAME = "flightstack"

def _get_docker_client(sim_type):
    """Helper function to establish the correct Docker connection."""
    try:
        if sim_type == "SIL":
            print(f"[PyChrono] Detected {sim_type} mode. Connecting to local PC Docker daemon...")
            return docker.from_env()
        else:
            print(f"[PyChrono] Detected {sim_type} mode. Connecting to ODROID Docker daemon via SSH...")
            return docker.DockerClient(base_url='ssh://odroid@server.local')
    except Exception as e:
        print(f"[Error] Failed to connect to Docker daemon: {e}")
        exit(1)

def start_flightstack(sim_type):
    """Injects the ROS 2 launch command into the Flightstack tmux session."""
    client = _get_docker_client(sim_type)
    print(f"[PyChrono] Ghost-typing START command into '{CONTAINER_NAME}' terminal...")
    
    try:
        container = client.containers.get(CONTAINER_NAME)
        ros_cmd = "ros2 launch flightstack_server sil_lokstep_launch.yaml"
        tmux_cmd = f"tmux send-keys -t fstack_session '{ros_cmd}' Enter"
        container.exec_run(tmux_cmd, detach=True)
        
        print("[PyChrono] Flightstack is spinning up...")
        time.sleep(3) 
    except docker.errors.NotFound:
        print(f"[Error] Container '{CONTAINER_NAME}' not found.")

def stop_flightstack(sim_type):
    """Injects a clean Ctrl+C to stop ROS 2, leaving the terminal open."""
    client = _get_docker_client(sim_type)
    print(f"[PyChrono] Ghost-typing Ctrl+C into '{CONTAINER_NAME}' terminal...")
    
    try:
        container = client.containers.get(CONTAINER_NAME)
        tmux_cmd = "tmux send-keys -t fstack_session C-c"
        container.exec_run(tmux_cmd, detach=True)
        
        print("[PyChrono] Shutting down ROS 2 nodes cleanly...")
        time.sleep(2) 
    except docker.errors.NotFound:
        print(f"[Error] Container '{CONTAINER_NAME}' not found.")
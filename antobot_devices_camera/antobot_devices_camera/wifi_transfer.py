import os
import yaml
import time
import tempfile
import subprocess
from pathlib import Path
from .utils import safe_yaml_write, SSH_PREFIX


def ssh_cmd(*args):
    return SSH_PREFIX + list(args)


class WiFiTransfer:
    """
    Handles SSH/SFTP connections over Wi-Fi, uploading files to a remote server.
    """

    TEST_FILE_SIZE = 1024 * 1024 

    def __init__(self, host, port, username, password=None, key_filename=None):
        self.host = host
        self.port = port
        self.username = username
        self.password = password
        self.key_filename = key_filename
    

    def check_wifi_connectivity(self):
        """ Only needed for scouting system with robot when transfer videos to orin agx via local wifi network """
        wifi_name = os.getenv("WIFI_NAME")
        wifi_password = os.getenv("WIFI_PASSWORD") 

        if not wifi_name or not wifi_password:
            print("[WiFiTransfer] WIFI_NAME or WIFI_PASSWORD is not set.")
            return False

        try:
            # Step 0: Check current WiFi connection
            print("Checking current WiFi connection...")
            check_connection_result = subprocess.run(
                ssh_cmd("sudo nmcli", "-t", "-f", "active,ssid", "device", "wifi"),
                check=True,
                capture_output=True,
                text=True
            )
            wifi_list = check_connection_result.stdout.strip().split('\n')

            for line in wifi_list:
                active, ssid = line.split(":")
                if active == "yes" and ssid == wifi_name:
                    print(f"[WiFiTransfer] Already connected to {wifi_name}.")
                    return True

            # Step 1: Force WiFi rescan
            print("Rescanning for WiFi networks...")
            subprocess.run(
                ssh_cmd("sudo nmcli", "device", "wifi", "rescan"),
                check=True,
                capture_output=True,
                text=True
            )
            time.sleep(5)  # Small wait after rescan to allow

            # Step 2: Try connecting
            result = subprocess.run(
                ssh_cmd("sudo nmcli", "device", "wifi", "connect", wifi_name, "password", wifi_password),
                check=True,
                capture_output=True,
                text=True
            )
            print(f"Command output:{result.stdout}")
            if result.returncode == 0:
                return True
            else:
                return False
        except subprocess.CalledProcessError as e:
            print(f"Command failed:{e.stderr}")
            return False
    
        

    def check_wifi_upload_availability(self):
        """
        method pings the server to determine if we are connected to the internet
        sets attribute
        """

        """Check if the SSH port is open using Python's socket module."""
        wifi_name = os.getenv("WIFI_NAME")

        if (wifi_name is not None and self.check_wifi_connectivity()) or wifi_name is None:
            
            response = os.system(f"nc -zv {self.host} {self.port}")
            if response == 0:
                return True
            else:
                return False 
    
    def get_upload_bandwidth(self, remote_path="/home/antobotserver"):
        """
        Measures Wi-Fi upload bandwidth by rsync-ing a small test file to the server.
        Returns upload speed in MB/s.
        """

        # Create a temporary file inside the container FS
        temp_file = tempfile.NamedTemporaryFile(delete=False)
        temp_file.truncate(self.TEST_FILE_SIZE)
        temp_file.close()

        remote_target = f"{self.username}@{self.host}:{remote_path}/wifi_upload_test.bin"

        # Prepare rsync command
        cmd = [
            "rsync",
            "--no-compress",      # raw speed test, no compression
            "--inplace",          # don't create a temp file on remote side
            "-ah",                # archive + human-readable
            temp_file.name,
            remote_target
        ]

        try:
            # Time the transfer
            start = time.time()
            result = subprocess.run(cmd, capture_output=True, text=True)
            end = time.time()

            if result.returncode != 0:
                print(f"[WiFiTransfer] rsync failed: {result.stderr.strip()}")
                return 0.0

            elapsed = end - start
            speed = (self.TEST_FILE_SIZE / (1024 * 1024)) / elapsed if elapsed > 0 else 0.0
            print(f"[WiFiTransfer] Upload speed: {speed:.2f} MB/s")

            # Clean up remote and local test file
            subprocess.run(
                ["ssh", f"{self.username}@{self.host}", "rm", "-f", f"{remote_path}/wifi_upload_test.bin"],
                capture_output=True
            )

            return speed

        except Exception as e:
            print(f"[WiFiTransfer] Error during Wi-Fi bandwidth test: {e}")
            return 0.0

        finally:
            # Always clean up the local temp file
            if os.path.exists(temp_file.name):
                os.remove(temp_file.name)


    def update_upload_state(self, session_path, robot_id = None, remote_path="/home/antobotserver"):
        """
        Mark *upload_complete* in both the local and remote manifest.yaml files for this session.
        """
        session_name = Path(session_path).name
        local_manifest = Path(session_path) / "manifest.yaml"
        if robot_id is None:
            remote_manifest = Path(remote_path) / session_name / "manifest.yaml"
        else:
            remote_manifest = Path(remote_path) / session_name / robot_id / "manifest.yaml"
            print(f"[WiFiTransfer] remote_manifest:{remote_manifest}.")

        # --- 1. Update local manifest.yaml ---
        if local_manifest.exists():
            data = yaml.safe_load(local_manifest.read_text())
            data["upload_complete"] = True
            safe_yaml_write(local_manifest, data)   # atomic write + fsync
            print(f"Local manifest updated for {session_name}")
        else:
            print(f"Local manifest not found: {local_manifest}")

        # --- 2. Update remote manifest.yaml via rsync ---
        try:
            cmd = [
                "rsync",
                "-ah",
                "--no-compress",
                str(local_manifest),
                f"{self.username}@{self.host}:{remote_manifest}"
            ]
            os.sync()
            subprocess.run(cmd, capture_output=True, text=True, check=True)
            print(f"Remote manifest updated for {session_name}")
        except subprocess.CalledProcessError as e:
            print(f"Failed to update remote manifest for {session_name}: {e}")
            raise




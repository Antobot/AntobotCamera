import os
import yaml
import subprocess
import shutil
import tempfile
import time
from pathlib import Path
from .utils import safe_yaml_write, SSH_PREFIX


# -----------------------------------------------------------------------------
# USBTransfer
# -----------------------------------------------------------------------------
class USBTransfer:
    """Low‑level USB utilities: *does not* touch manifests anymore."""

    TEST_FILE_SIZE = 1024 * 1024 

    def __init__(self, usb_dir: str):
        self.usb_dir = usb_dir

    # ---------------------------------------------------------------------
    # Availability (via ssh)
    # ---------------------------------------------------------------------
    def is_usb_connected(self) -> bool:
        """True only if the mount‑point exists *and* the host reports it mounted."""
        cmd = SSH_PREFIX + [f"mountpoint -q {self.usb_dir}"]
        return subprocess.call(" ".join(cmd), shell=True) == 0

    # ---------------------------------------------------------------------
    # Safe unmount (via ssh)
    # ---------------------------------------------------------------------
    def unmount(self, retries: int = 5) -> bool:
        """Try `umount` on the host until success or retries exhausted."""
        for _ in range(retries):
            if not self.is_usb_connected():
                return True
            subprocess.run(
                " ".join(SSH_PREFIX + [f"sudo umount {self.usb_dir}"]),
                shell=True,
                check=False,
            )
            time.sleep(1)
            if not self.is_usb_connected():
                return True
        print("USB unmount failed – device still busy on host")
        return False

    # ---------------------------------------------------------------------
    # Quick bandwidth probe (runs under TransferManager.usb_lock!)
    # ---------------------------------------------------------------------
    def get_copy_bandwidth(self) -> float:
        if not self.is_usb_connected():
            return 0.0

        # Create a temporary file inside the container FS
        temp_file = tempfile.NamedTemporaryFile(delete=False)
        temp_file.truncate(self.TEST_FILE_SIZE)
        temp_file.close()

        usb_test_path = os.path.join(self.usb_dir, "usb_speed_test.bin")
        start = time.time()
        try:
            shutil.copy2(temp_file.name, usb_test_path)
            os.sync()  # flush buffers for realistic timing
        except OSError:
            return 0.0
        finally:
            elapsed = max(time.time() - start, 1e-6)
            os.remove(temp_file.name)
            if os.path.exists(usb_test_path):
                os.remove(usb_test_path)

        return (self.TEST_FILE_SIZE / (1024 * 1024)) / elapsed  # MB/s

    
    def update_upload_state(self, session_path: str):
        """
        Mark *usb_transfer_complete* in both the USB‑side and local
        manifest.yaml files for this session.
        """
        session_name   = Path(session_path).name
        usb_manifest   = Path(self.usb_dir) / session_name / "manifest.yaml"
        local_manifest = Path(session_path)  / "manifest.yaml"

        for mpath in (usb_manifest, local_manifest):
            if not mpath.exists():
                print(f"manifest not found: {mpath}")
                continue

            data = yaml.safe_load(mpath.read_text())
            data["usb_transfer_complete"] = True
            safe_yaml_write(mpath, data)   # atomic write + fsync

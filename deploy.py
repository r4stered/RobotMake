import subprocess
import os

print("Copying library files over...")

lib_path = "./build/rio_release_shared/deploy"

for filename in os.listdir(lib_path):
    subprocess.run(
        [
            "scp",
            os.path.join(lib_path, filename),
            "lvuser@172.22.11.2:/usr/local/frc/third-party/lib",
        ]
    )

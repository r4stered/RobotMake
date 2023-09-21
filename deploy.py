import os
import paramiko
import hashlib
import json


class MySFTPClient(paramiko.SFTPClient):
    def put_dir(self, source, target):
        """Uploads the contents of the source directory to the target path. The
        target directory needs to exists. All subdirectories in source are
        created under target.
        """
        for item in os.listdir(source):
            if os.path.isfile(os.path.join(source, item)):
                print(item)
                self.put(os.path.join(source, item), "%s/%s" % (target, item), progress)
            else:
                self.mkdir("%s/%s" % (target, item), ignore_existing=True)
                self.put_dir(os.path.join(source, item), "%s/%s" % (target, item))

    def mkdir(self, path, mode=511, ignore_existing=False):
        """Augments mkdir by adding an option to not fail if the folder exists"""
        try:
            super(MySFTPClient, self).mkdir(path, mode)
        except IOError:
            if ignore_existing:
                pass
            else:
                raise


def progress(so_far, total, bar_length=20):
    fraction = so_far / total

    arrow = int(fraction * bar_length - 1) * "-" + ">"
    padding = int(bar_length - len(arrow)) * " "

    ending = "\n" if so_far == total else "\r"

    print(f"Progress: [{arrow}{padding}] {int(fraction*100)}%", end=ending)


def sha256sum(filename):
    with open(filename, "rb", buffering=0) as f:
        return hashlib.file_digest(f, "sha256").hexdigest()


server_usb = "172.22.11.2"

rio_username = "lvuser"
rio_password = ""

lib_dir = "./build/rio_release_shared/deploy"
lib_remote_dir = "/usr/local/frc/third-party/lib/"

deploy_dir = "./deploy"
deploy_remote_dir = "/home/lvuser/deploy/"

exec_dir = "./build/rio_release_shared/test_exec"
exec_remote_dir = "/home/lvuser/"

transport = paramiko.Transport((server_usb, 22))
print("Connecting to rio...")
transport.connect(username=rio_username, password=rio_password)
sftp = MySFTPClient.from_transport(transport)

print("Copying library files...")
deploy_cache_dict = {}
with open("./build/rio_release_shared/deploycache.json", "r+") as deploy_cache_file:
    old_cache = json.loads(deploy_cache_file.read() or "{}")
    for filename in os.listdir(lib_dir):
        file_path_and_name = os.path.join(lib_dir, filename)
        deploy_cache_dict[file_path_and_name] = sha256sum(file_path_and_name)
        # if the file doesnt exist in the old cache, or it does exist and is different than before
        if (old_cache.get(file_path_and_name, None) == None) or (
            old_cache.get(file_path_and_name, None) != None
            and old_cache[file_path_and_name] != deploy_cache_dict[file_path_and_name]
        ):
            print(file_path_and_name)
            sftp.put(
                file_path_and_name,
                os.path.join(lib_remote_dir, filename),
                progress,
                confirm=True,
            )
    print("Done copying library files.")

print("Copying deploy folder contents...")
sftp.put_dir(deploy_dir, deploy_remote_dir)
print("Done copying deploy files.")

with open("./build/rio_release_shared/deploycache.json", "w") as deploy_cache_file:
    deploy_cache_file.write(json.dumps(deploy_cache_dict))

print("Removing old robot program...")
channel = transport.open_session()
channel.exec_command("rm /home/lvuser/frcUserProgram")
channel.close()
print("Done removing old robot program.")

print("Copying robot program...")
sftp.put(exec_dir, os.path.join(exec_remote_dir, "frcUserProgram"), progress, True)
print("Done copying robot program.")

sftp.close()

print("Starting robot program!")
# Sets correct permissions and ownership for the robot program
channel = transport.open_session()
channel.exec_command(
    "chmod +x /home/lvuser/frcUserProgram; chown lvuser /home/lvuser/frcUserProgram; setcap cap_sys_nice+eip /home/lvuser/frcUserProgram;"
)
channel.close()

# Sets up the correct parameters for starting the program
channel = transport.open_session()
channel.exec_command('echo "/home/lvuser/frcUserProgram" > /home/lvuser/robotCommand')
channel.close()

# Kills currently running robot program and starts new one
channel = transport.open_session()
channel.exec_command(
    "sync; source /etc/profile.d/natinst-path.sh; /usr/local/frc/bin/frcKillRobot.sh -t -r 2> /dev/null"
)
channel.close()
transport.close()
print("Bye!")

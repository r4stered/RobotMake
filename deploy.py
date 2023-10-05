import os
import paramiko
import hashlib
import json
import sys
import argparse


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


def main(argv):
    parser = argparse.ArgumentParser(
        description="Deploys built C++ robot programs to a roborio."
    )
    parser.add_argument(
        "-f",
        "--force",
        type=bool,
        help="If true, force redeploy lib files.",
    )
    parser.add_argument(
        "-d",
        "--debug",
        type=bool,
        help="Will deploy debug binary and lauch gbd server on rio.",
    )

    args = vars(parser.parse_args(argv))

    transport = paramiko.Transport((server_usb, 22))
    print("Connecting to rio...")
    transport.connect(username=rio_username, password=rio_password)
    sftp = MySFTPClient.from_transport(transport)

    is_debug = args["debug"]

    build_folder = ""
    if is_debug:
        build_folder = "rio_debug_shared"
    else:
        build_folder = "rio_release_shared"

    lib_dir = f"./build/{build_folder}/deploy"
    lib_remote_dir = "/usr/local/frc/third-party/lib/"

    deploy_dir = "./deploy"
    deploy_remote_dir = "/home/lvuser/deploy/"

    exec_dir = f"./build/{build_folder}/robotProgram"
    exec_remote_dir = "/home/lvuser/"

    deploy_cache_path = f"./build/{build_folder}/deploycache.json"

    print("Copying library files...")
    deploy_cache_dict = {}
    with open(deploy_cache_path, "w+") as deploy_cache_file:
        old_cache = json.loads(deploy_cache_file.read() or "{}")
        for filename in os.listdir(lib_dir):
            file_path_and_name = os.path.join(lib_dir, filename)
            deploy_cache_dict[file_path_and_name] = sha256sum(file_path_and_name)
            # if the file doesnt exist in the old cache, or it does exist and is different than before
            if (
                args["force"] == True
                or (old_cache.get(file_path_and_name, None) == None)
                or (
                    old_cache.get(file_path_and_name, None) != None
                    and old_cache[file_path_and_name]
                    != deploy_cache_dict[file_path_and_name]
                )
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

    with open(deploy_cache_path, "w") as deploy_cache_file:
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

    if is_debug:
        start_command = 'echo "gdbserver host:8349 /home/lvuser/frcUserProgram" > /home/lvuser/robotCommand'
    else:
        start_command = 'echo "/home/lvuser/frcUserProgram" > /home/lvuser/robotCommand'

    channel = transport.open_session()
    channel.exec_command(start_command)
    channel.close()

    # Kills currently running robot program and starts new one
    channel = transport.open_session()
    channel.exec_command(
        "sync; source /etc/profile.d/natinst-path.sh; /usr/local/frc/bin/frcKillRobot.sh -t -r 2> /dev/null"
    )
    channel.close()
    transport.close()
    print("Bye!")


if __name__ == "__main__":
    main(sys.argv[1:])

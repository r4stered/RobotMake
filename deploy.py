import os
import paramiko


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
for filename in os.listdir(lib_dir):
    print(filename)
    sftp.put(
        os.path.join(lib_dir, filename),
        os.path.join(lib_remote_dir, filename),
        progress,
        confirm=True,
    )
print("Done copying library files.")

print("Copying deploy folder contents...")
sftp.put_dir(deploy_dir, deploy_remote_dir)
print("Done copying deploy files.")

print("Copying robot program...")
sftp.put(exec_dir, os.path.join(exec_remote_dir, "frcUserProgram"), progress, True)
print("Done copying robot program.")

sftp.close()


print("Starting robot program!")
channel = transport.open_session()
channel.exec_command(
    "sync; . /etc/profile.d/natinst-path.sh; /usr/local/frc/bin/frcKillRobot.sh -t -r 2> /dev/null"
)

transport.close()
print("Bye!")

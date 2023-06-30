import os
import shutil

if __name__ == "__main__":
    with open(os.path.join("pyrealsense2", "config", "install_manifest.txt"), "r") as f:
            install_manifest = [line.strip() for line in f.readlines() if line]

    username = os.listdir('/home')[0]

    # format in the .whl
    # whl/
    #   bin/
    #   lib/
    #   include/
    #   Documents/

    for path in install_manifest:
        split_path = path[1:].split("/")
        if split_path[0] == "home":
            split_path[1] = username
            path = os.path.join(split_path)
        libpath = split_path[2:]
        parent_path = os.path.dirname(path)
        if not os.path.exists(parent_path):
            os.makedirs(parent_path)
        shutil.copy(os.path.join(*libpath), path)
        if split_path[2] == "OFF":
            shutil.copy(os.path.join(*libpath), "./cortano/")

    curr_dir = os.getcwd()
    with open(os.path.join(curr_dir, "scripts", "nvcortexnano.service"), "r") as fp:
        service_lines = [line.replace("/usr/local/cortano", curr_dir + "cortano") \
                         for line in fp.readlines() if line]
    with open(os.path.join(curr_dir, "scripts", "nvcortexnano.service"), "w") as fp:
        for line in service_lines:
             fp.write(line + "\n")
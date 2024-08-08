import os
import shutil

if __name__ == "__main__":
    curr_dir = os.getcwd()
    with open(os.path.join(curr_dir, "scripts", "nvcortexnano.service"), "r") as fp:
        service_lines = [line.replace("/usr/local/cortano", os.path.join(curr_dir, "cortano")) \
                         for line in fp.readlines() if line]
    with open(os.path.join(curr_dir, "scripts", "nvcortexnano.service"), "w") as fp:
        for line in service_lines:
             fp.write(line + "\n")
    with open(os.path.join(curr_dir, "scripts", "nvv5nano.service"), "r") as fp:
        service_lines = [line.replace("/usr/local/cortano", os.path.join(curr_dir, "cortano")) \
                         for line in fp.readlines() if line]
    with open(os.path.join(curr_dir, "scripts", "nvv5nano.service"), "w") as fp:
        for line in service_lines:
             fp.write(line + "\n")
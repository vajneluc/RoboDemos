#!/usr/bin/env python3

# Simple pipe filter for ROS topic echo messages. Takes comma separated list of extraction rules as an argument. 
# Without it, it returns the original data.
#
# Example usage:
#
#    ros2 topic echo /tf | ./pipefilter.py transforms[0].transform.rotation.w,transforms[0].header
#

import sys
import yaml


keys = []
if len(sys.argv) > 1:
    keys = sys.argv[1].split(",")

buffer = []


def get_field(data, key):
    if key is not None and key != "":
        if key[0] == ".":
            key = key[1:]
        if key[0].isalpha(): 
            prefix = ""
            rest = key
            while len(rest ) > 0:
                c = rest[0]
                if not c.isalpha():
                    break
                else:
                    prefix += c
                    rest = rest[1:]
            return get_field(data[prefix], rest)
        elif key[0] == "[":
            prefix, rest = key[1:].split("]")
            index = int(prefix)
            return get_field(data[index], rest)
        else:
            print("# Error: unrecognized prefix")
    return data

def write_buffer(buffer, keys):
    txt = "\n".join(buffer)
    try:
        data = yaml.safe_load(txt)
        datas = [get_field(data, key) for key in keys]        
        for data, key in zip(datas, keys):
            print(f"## {key}")
            print(yaml.dump(data))
        print("---")
    except:
        print("# Error parsing YAML!")


for line in sys.stdin:
    line = line.rstrip("\r\n")
    if line == "---":
        write_buffer(buffer, keys)
        buffer = []
    else:
        buffer.append(line)



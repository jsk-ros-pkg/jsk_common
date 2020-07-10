#!/usr/bin/env python

import rospy
import time
import subprocess
import signal
import os
import sys
import argparse
import re
import shutil
try:
    import colorama
except:
    print("Please install colorama by pip install colorama")
    sys.exit(1)
from colorama import Fore, Style
from jsk_topic_tools.master_util import isMasterAlive


def runROSBag(topics, size, save_dir):
    """
    run rosbag and return Popen object
    """
    cmd = 'roslaunch jsk_data rosbag_always_run_rosbag.launch'
    formatted_topics = [t for t in topics.split(' ') if t]
    args = cmd.split(' ') + ["TOPICS:=" + topics + ""] + ["SIZE:=" + size] + ["OUTPUT:=" + save_dir + '/rosbag']
    print(args)
    return subprocess.Popen(args)

def parseBagFile(bag):
    # bag file name is ...
    # 'rosbag_YYYY-MM-DD-HH-mm-SS_i.bag'
    regexp = 'rosbag_(\d\d\d\d)-(\d\d)-(\d\d)-(\d\d)-(\d\d)-(\d\d)_([\d]+).bag'
    result = re.match(regexp, bag)
    if not result:
        return None
    else:
        return [result.group(f) for f in range(1, 8)]

def mkdirForBag(root_dir, bag):
    # mkdir like ${root_dir}/${YYYY}/${MM}/${DD}/${HH}
    parse = parseBagFile(bag)
    YYYY = parse[0]
    MM = parse[1]
    DD = parse[2]
    HH = parse[3]
    directory = os.path.join(root_dir, YYYY, MM, DD, HH)
    if not os.path.exists(directory):
        os.makedirs(directory)
    return directory
    
def moveBagFiles(save_dir, bags):
    for bag in bags:
        move_dir = mkdirForBag(save_dir, bag)
        from_file = os.path.join(save_dir, bag)
        to_file = os.path.join(move_dir, bag)
        print('moving file %s -> %s' % (from_file, to_file))
        shutil.move(from_file, to_file)
                    
def watchFileSystem(save_dir, max_size):
    files = os.listdir(save_dir)
    target_bags = [f for f in files 
                   if f.startswith('rosbag') 
                   and f.endswith('.bag')]
    moveBagFiles(save_dir, target_bags)
    # check size of directory
    checkDirectorySize(save_dir, int(max_size))

def getDirectorySize(start_path = '.'):
    "size unit is Mega bytes"
    total_size = 0
    for dirpath, dirnames, filenames in os.walk(start_path):
        for f in filenames:
            fp = os.path.join(dirpath, f)
            try:
                total_size += os.path.getsize(fp)
            except:
                pass
    return total_size / 1000.0 / 1000.0

def keyFuncToSortBag(bag):
    parse = parseBagFile(os.path.basename(bag))
    parse[len(parse) - 1] = str(int(parse[len(parse) - 1])).zfill(4)
    concatenated_string = reduce(lambda x, y: x + y, parse)
    return int(concatenated_string)
    
def listBagsSortedByDate(save_dir):
    bags = []
    for dirpath, dirnames, filenames in os.walk(save_dir):
        for f in filenames:
            if f.startswith('rosbag') and (f.endswith('.bag') or f.endswith('.active')):
                bags.append(os.path.join(dirpath, f))
    return sorted(bags, key=keyFuncToSortBag)
    
def removeOldFiles(save_dir, max_size, current_size):
    files = listBagsSortedByDate(save_dir)
    remove_size = current_size - max_size
    for f in files:
        the_size = os.path.getsize(f)
        print(Fore.GREEN + 'removing %s (%d)' % (f, the_size / 1000 / 1000) + Fore.RESET)
        os.remove(f)
        # Send desktop notification
        subprocess.check_output(['notify-send', "Removed %s (%d)" % (f, the_size / 1000 / 1000)])
        remove_size = remove_size - the_size / 1000.0 / 1000.0
        if remove_size < 0:
            return
    
def checkDirectorySize(save_dir, max_size):
    size = getDirectorySize(save_dir)
    # print('current directory size is %fM (max is %dM)' % (size, int(max_size)))
    if size > max_size:
        removeOldFiles(save_dir, max_size, size)
    
g_rosbag_process = False

def restartROSBag(topics, size, save_dir):
    global g_rosbag_process
    print('Running rosbag...')
    g_rosbag_process = runROSBag(topics, size, save_dir)

def killChildProcesses(ppid):
    output = subprocess.check_output(['ps', '--ppid=' + str(ppid), '--no-headers'])
    for process_line in output.split('\n'):
        strip_process_line = process_line.strip()
        if strip_process_line:
            pid = strip_process_line.split(' ')[0]
            name = strip_process_line.split(' ')[-1]
            print('killing %s' % (name))
            os.kill(int(pid), signal.SIGINT)

def killROSBag():
    global g_rosbag_process
    if g_rosbag_process:
        print('Killing rosbag ...')
        rosbag_pid = g_rosbag_process.pid
        try:
            killChildProcesses(rosbag_pid)
            g_rosbag_process.send_signal(subprocess.signal.SIGINT)
        except:
            pass
    
def main(topics, size, save_dir, max_size, rate = 1):
    if not os.path.exists(save_dir):
        os.makedirs(save_dir)
    previous_master_state = None
    try:
        while True:
            master_state = isMasterAlive()
            if not master_state and previous_master_state:
                print("kill rosbag")
                killROSBag()
            elif master_state and not previous_master_state:
                print("restart rosbag")
                restartROSBag(topics, size, save_dir)
            watchFileSystem(save_dir, max_size)
            previous_master_state = master_state
            time.sleep(1.0 / rate)
    except Exception as e:
        time.sleep(1)
        watchFileSystem(save_dir, max_size)
    finally:
        killROSBag()


        
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='rosbag record regardless of rosmaster status')
    parser.add_argument('--topics', help="topics to record", required=True)
    parser.add_argument('--size', help="size of each rosbag", required=True)
    parser.add_argument('--save-dir', help="directory to store rosbag", required=True)
    parser.add_argument('--max-size', help="maximum size of rosbags in save_dir", required=True, type=int)
    args = parser.parse_args()
    main(args.topics, args.size, args.save_dir, args.max_size)

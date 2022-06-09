from __future__ import division

import datetime
import math
import os.path as osp
import re
import shutil
import subprocess
import sys
import tempfile
import time
import wave

import audio_common_msgs.msg
import cv2
from moviepy.editor import VideoFileClip
import numpy as np
import rosbag
import rospy
from tqdm import tqdm

from jsk_rosbag_tools.cv import img_to_msg
from jsk_rosbag_tools.merge import merge_bag


def to_seconds(date):
    return time.mktime(date.timetuple())


def mediainfo(filepath):
    prober = 'ffprobe'
    command_args = [
        "-v", "quiet",
        "-show_format",
        "-show_streams",
        filepath
    ]

    command = [prober, '-of', 'old'] + command_args
    res = subprocess.Popen(command, stdout=subprocess.PIPE)
    output = res.communicate()[0].decode("utf-8")

    if res.returncode != 0:
        command = [prober] + command_args
        output = subprocess.Popen(
            command,
            stdout=subprocess.PIPE).communicate()[0].decode("utf-8")
    rgx = re.compile(r"(?:(?P<inner_dict>.*?):)?(?P<key>.*?)\=(?P<value>.*?)$")
    info = {}
    if sys.platform == 'win32':
        output = output.replace("\r", "")
    for line in output.split("\n"):
        # print(line)
        mobj = rgx.match(line)

        if mobj:
            # print(mobj.groups())
            inner_dict, key, value = mobj.groups()

            if inner_dict:
                try:
                    info[inner_dict]
                except KeyError:
                    info[inner_dict] = {}
                info[inner_dict][key] = value
            else:
                info[key] = value

    return info


def nsplit(xlst, n):
    total_n = len(xlst)
    d = int((total_n + n - 1) / n)
    i = 0
    ret = []
    while i < total_n:
        ret.append(xlst[i:i + d])
        i += d
    return ret


def get_video_duration(video_path):
    video_path = str(video_path)
    if not osp.exists(video_path):
        raise OSError("{} not exists".format(video_path))
    cap = cv2.VideoCapture(video_path)
    fps = cap.get(cv2.CAP_PROP_FPS)
    frame_count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    cap.release()
    duration = frame_count / fps
    return duration


def get_video_n_frame(video_path):
    video_path = str(video_path)
    if not osp.exists(video_path):
        raise OSError("{} not exists".format(video_path))
    clip = VideoFileClip(video_path)
    return int(clip.duration * clip.fps)


def get_video_fps(video_path):
    video_path = str(video_path)
    if not osp.exists(video_path):
        raise OSError("{} not exists".format(video_path))
    clip = VideoFileClip(video_path)
    return clip.fps


def load_frame(video_path, start=0.0, duration=-1,
               target_size=None, sampling_frequency=None):
    video_path = str(video_path)
    vid = cv2.VideoCapture(video_path)
    fps = vid.get(cv2.CAP_PROP_FPS)
    vid.set(cv2.CAP_PROP_POS_MSEC, start)
    vid_avail = True
    if sampling_frequency is not None:
        frame_interval = int(math.ceil(fps * sampling_frequency))
    else:
        frame_interval = 1
    cur_frame = 0
    while True:
        stamp = float(cur_frame) / fps
        vid_avail, frame = vid.read()
        if not vid_avail:
            break
        if duration != -1 and stamp > start + duration:
            break
        if target_size is not None:
            frame = cv2.resize(frame, target_size)
        yield frame, stamp
        cur_frame += frame_interval
        vid.set(cv2.CAP_PROP_POS_FRAMES, cur_frame)
    vid.release()


def count_frames(video_path, start=0.0, duration=-1,
                 sampling_frequency=None):
    video_duration = get_video_duration(video_path)
    video_duration -= start
    if duration > 0:
        video_duration = max(video_duration - duration, 0)
    fps = get_video_fps(video_path)
    if sampling_frequency is not None:
        return int(math.ceil(
            video_duration * fps
            / int(math.ceil(fps * sampling_frequency))))
    else:
        return int(math.ceil(video_duration * fps))


def video_to_bag(video_filepath, bag_output_filepath,
                 topic_name, compress=False, audio_topic_name='/audio',
                 no_audio=False,
                 base_unixtime=None,
                 fps=None,
                 show_progress_bar=True):
    if fps is not None:
        sampling_frequency = 1.0 / fps
    else:
        sampling_frequency = None
    if base_unixtime is None:
        base_unixtime = to_seconds(datetime.datetime.now())

    topic_name = topic_name.lstrip('/compressed')
    if compress is True:
        topic_name = osp.join(topic_name, 'compressed')

    tmpdirname = tempfile.mkdtemp("", 'tmp', None)
    video_out = osp.join(tmpdirname, 'video.tmp.bag')
    n_frame = count_frames(video_filepath,
                           sampling_frequency=sampling_frequency)
    if show_progress_bar:
        progress = tqdm(total=n_frame)
    with rosbag.Bag(video_out, 'w') as outbag:
        for img, timestamp in load_frame(
                video_filepath,
                sampling_frequency=sampling_frequency):
            if show_progress_bar:
                progress.update(1)
            msg = img_to_msg(img, compress=compress)
            sec = int(base_unixtime + timestamp)
            nsec = ((base_unixtime + timestamp) * (10 ** 9)) % (10 ** 9)
            ros_timestamp = rospy.rostime.Time(sec, nsec)
            msg.header.stamp = ros_timestamp
            outbag.write(topic_name, msg, ros_timestamp)
    if show_progress_bar:
        progress.close()

    extract_audio = True
    if no_audio is False:
        wav_filepath = osp.join(tmpdirname, 'tmp.wav')
        cmd = "ffmpeg -i '{}' '{}'".format(
            video_filepath, wav_filepath)
        proc = subprocess.Popen(cmd, shell=True,
                                stdout=subprocess.PIPE,
                                stderr=subprocess.PIPE)
        proc.wait()

        try:
            wf = wave.open(wav_filepath, mode='rb')
            sample_rate = wf.getframerate()
            wf.rewind()
            buf = wf.readframes(-1)
            if wf.getsampwidth() == 2:
                data = np.frombuffer(buf, dtype='int16')
            elif wf.getsampwidth() == 4:
                data = np.frombuffer(buf, dtype='int32')
            data = data.reshape(-1, wf.getnchannels())
            media_info = mediainfo(wav_filepath)
        except RuntimeError:
            extract_audio = False

        if extract_audio:
            rate = 100
            n = int(np.ceil(data.shape[0] / (sample_rate // rate)))
            channels = data.shape[1]

            audio_out = osp.join(tmpdirname, 'audio.tmp.bag')
            with rosbag.Bag(audio_out, 'w') as outbag:
                audio_info = audio_common_msgs.msg.AudioInfo(
                    channels=channels,
                    sample_rate=sample_rate,
                    sample_format=media_info['codec_name'].upper(),
                    bitrate=int(media_info['bit_rate']),
                    coding_format='wave')
                outbag.write(audio_topic_name + '_info',
                             audio_info, ros_timestamp)
                for i, audio_data in enumerate(nsplit(data, n)):
                    msg = audio_common_msgs.msg.AudioData()
                    msg.data = audio_data.reshape(-1).tobytes()
                    timestamp = i * 0.01
                    sec = int(base_unixtime + timestamp)
                    nsec = (
                        (base_unixtime + timestamp) * (10 ** 9)) % (10 ** 9)
                    ros_timestamp = rospy.rostime.Time(sec, nsec)
                    outbag.write(audio_topic_name, msg, ros_timestamp)
            merge_bag(video_out, audio_out, bag_output_filepath)
        else:
            shutil.move(video_out, bag_output_filepath)
    else:
        shutil.move(video_out, bag_output_filepath)

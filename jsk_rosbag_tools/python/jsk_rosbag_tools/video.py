from __future__ import division

import datetime
import math
import os.path as osp
import shutil
import subprocess
import tempfile

import audio_common_msgs.msg
import numpy as np
from pydub.utils import mediainfo
import rosbag
import rospy
import skvideo.io
import soundfile as sf
from tqdm import tqdm

from jsk_rosbag_tools.cv import img_to_msg
from jsk_rosbag_tools.cv_compat import cv2
from jsk_rosbag_tools.merge import merge_bag


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
    metadata = skvideo.io.ffprobe(video_path)
    return float(metadata['video']['@duration'])


def get_video_avg_frame_rate(video_path):
    video_path = str(video_path)
    if not osp.exists(video_path):
        raise OSError("{} not exists".format(video_path))
    metadata = skvideo.io.ffprobe(video_path)
    a, b = metadata['video']['@avg_frame_rate'].split('/')
    a = int(a)
    b = int(b)
    return a / b


def get_video_creation_time(video_path):
    metadata = skvideo.io.ffprobe(video_path)
    tag_dict = {}
    for tag in metadata['video']['tag']:
        tag_dict[tag['@key']] = tag['@value']
    if 'creation_time' not in tag_dict:
        return None
    creation_time = tag_dict['creation_time']
    created_at = datetime.datetime.strptime(
        creation_time, '%Y-%m-%dT%H:%M:%S.%fZ')
    return created_at


def get_video_n_frame(video_path):
    video_path = str(video_path)
    if not osp.exists(video_path):
        raise OSError("{} not exists".format(video_path))
    metadata = skvideo.io.ffprobe(video_path)
    if '@nb_frames' not in metadata['video']:
        fps = get_video_avg_frame_rate(video_path)
        return int(fps * get_video_duration(video_path))
    return int(metadata['video']['@nb_frames'])


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


def video_to_bag(video_filepath, bag_output_filepath,
                 topic_name, compress=False, audio_topic_name='/audio',
                 no_audio=False,
                 base_unixtime=None,
                 show_progress_bar=True):
    if base_unixtime is None:
        base_unixtime = get_video_creation_time(video_filepath)
        if base_unixtime is None:
            base_unixtime = datetime.datetime.now()
        base_unixtime = base_unixtime.timestamp()

    topic_name = topic_name.lstrip('/compressed')
    if compress is True:
        topic_name = osp.join(topic_name, 'compressed')

    with tempfile.TemporaryDirectory() as tmpdirname:
        video_out = osp.join(tmpdirname, 'video.tmp.bag')
        n_frame = get_video_n_frame(video_filepath)
        if show_progress_bar:
            progress = tqdm(total=n_frame)
        with rosbag.Bag(video_out, 'w') as outbag:
            for img, timestamp in load_frame(video_filepath):
                if show_progress_bar:
                    progress.update(1)
                msg = img_to_msg(img, compress=compress)
                sec = int(base_unixtime + timestamp)
                nsec = ((base_unixtime + timestamp) * (10 ** 9)) % (10 ** 9)
                ros_timestamp = rospy.rostime.Time(sec, nsec)
                msg.header.stamp = ros_timestamp
                outbag.write(topic_name, msg, ros_timestamp)

        extract_audio = True
        if no_audio is False:
            wav_filepath = osp.join(tmpdirname, 'tmp.wav')
            cmd = "ffmpeg -i '{}' '{}'".format(
                video_filepath, wav_filepath)
            subprocess.run(cmd, shell=True,
                           stdout=subprocess.PIPE,
                           stderr=subprocess.PIPE)

            try:
                data, sample_rate = sf.read(wav_filepath, dtype='int16')
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
                        nsec = ((base_unixtime + timestamp)
                                * (10 ** 9)) % (10 ** 9)
                        ros_timestamp = rospy.rostime.Time(sec, nsec)
                        outbag.write(audio_topic_name, msg, ros_timestamp)
                merge_bag(video_out, audio_out, bag_output_filepath)
            else:
                shutil.move(video_out, bag_output_filepath)
        else:
            shutil.move(video_out, bag_output_filepath)

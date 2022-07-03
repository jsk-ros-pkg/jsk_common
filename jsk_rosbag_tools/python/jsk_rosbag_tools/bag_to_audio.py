import os
import os.path as osp

import numpy as np
import rosbag
from scipy.io.wavfile import write as wav_write

from jsk_rosbag_tools.extract import extract_oneshot_topic
from jsk_rosbag_tools.info import get_topic_dict
from jsk_rosbag_tools.makedirs import makedirs


def bag_to_audio(bag_filepath,
                 wav_outpath,
                 topic_name='/audio',
                 audio_info_topic_name=None,
                 samplerate=44100,
                 channels=1,
                 overwrite=True):
    if os.path.exists(wav_outpath) and overwrite is False:
        raise FileExistsError('{} file already exists.'.format(wav_outpath))
    topic_dict = get_topic_dict(bag_filepath)
    if topic_name not in topic_dict:
        return

    audio_info_topic_name = audio_info_topic_name or topic_name + '_info'
    # if audio_info_topic exists, extract info from it.
    if audio_info_topic_name in topic_dict:
        audio_info = extract_oneshot_topic(bag_filepath, audio_info_topic_name)
        if audio_info is not None:
            samplerate = audio_info.sample_rate
            channels = audio_info.channels

    bag = rosbag.Bag(bag_filepath)
    audio_buffer = []
    for _, msg, _ in bag.read_messages(topics=[topic_name]):
        if msg._type == 'audio_common_msgs/AudioData':
            buf = np.frombuffer(msg.data, dtype='int16')
            buf = buf.reshape(-1, channels)
            audio_buffer.append(buf)
    audio_buffer = np.concatenate(audio_buffer, axis=0)

    makedirs(osp.dirname(wav_outpath))
    wav_write(wav_outpath, rate=samplerate, data=audio_buffer)

    valid = os.stat(wav_outpath).st_size != 0
    if valid is False:
        return False

    return True

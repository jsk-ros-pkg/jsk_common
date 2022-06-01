import datetime
import os
import os.path as osp
import sys
import tempfile

import cv2
import moviepy.editor as mp
from moviepy.video.io.ffmpeg_writer import FFMPEG_VideoWriter

from jsk_rosbag_tools.extract import extract_audio
from jsk_rosbag_tools.extract import extract_image_topic
from jsk_rosbag_tools.extract import get_image_topic_names
from jsk_rosbag_tools.makedirs import makedirs
from jsk_rosbag_tools.topic_name_utils import topic_name_to_file_name


def bag_to_video(input_bagfile,
                 output_filepath=None,
                 output_dirpath=None,
                 image_topic=None,
                 image_topics=None,
                 fps=30,
                 samplerate=16000,
                 channels=1,
                 audio_topic='/audio'):
    """Create video from rosbag file.

    Specify only either output_filepath or output_dirpath.
    If output_filepath is specified, specify image_topic.
    If output_dirpath is specified, image_topics can be specified.
    If image_topic_names is None, make all color images into video.

    """
    if not os.path.exists(input_bagfile):
        print('Input bagfile {} not exists.'.format(input_bagfile))
        sys.exit(1)

    if output_filepath is not None and output_dirpath is not None:
        raise ValueError(
            'Specify only either output_filepath or output_dirpath.')

    output_filepaths = []
    target_image_topics = []
    if output_filepath is not None:
        if image_topic is None:
            raise ValueError(
                'If output_filepath is specified, specify image_topic.')
        output_filepaths.append(output_filepath)
        target_image_topics.append(image_topic)
        wav_outpath = tempfile.NamedTemporaryFile(suffix='.wav').name
    else:
        # output_dirpath is specified case.
        if image_topics is None:
            image_topics = get_image_topic_names(
                input_bagfile, rgb_only=True)
        target_image_topics = image_topics

        for image_topic in target_image_topics:
            output_filepaths.append(
                osp.join(
                    output_dirpath,
                    topic_name_to_file_name(image_topic) + '.mp4'))
        wav_outpath = osp.join(output_dirpath, '{}.wav'.format(
            topic_name_to_file_name(audio_topic)))

    audio_exists = extract_audio(input_bagfile, wav_outpath,
                                 samplerate=samplerate,
                                 channels=channels,
                                 topic_name=audio_topic)

    dt = 1.0 / fps
    for image_topic, output_filepath in zip(target_image_topics,
                                            output_filepaths):
        makedirs(osp.dirname(output_filepath))
        if audio_exists:
            tmp_videopath = tempfile.NamedTemporaryFile(suffix='.mp4').name
        else:
            tmp_videopath = output_filepath

        images = extract_image_topic(input_bagfile, image_topic)

        # remove 0 time stamp
        stamp = 0.0
        while stamp == 0.0:
            stamp, _, img, _ = next(images)
        start_stamp = stamp

        creation_time = datetime.datetime.utcfromtimestamp(start_stamp)
        time_format = '%y-%m-%d %h:%M:%S'
        writer = FFMPEG_VideoWriter(
            tmp_videopath,
            (img.shape[1], img.shape[0]),
            fps, logfile=None,
            ffmpeg_params=[
                '-metadata',
                'creation_time={}'.format(
                    creation_time.strftime(time_format)),
            ])

        current_time = 0.0
        cur_img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        for stamp, _, bgr_img, _ in images:
            rgb_img = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2RGB)
            aligned_stamp = stamp - start_stamp
            while current_time < aligned_stamp:
                current_time += dt
                writer.write_frame(cur_img)
            cur_img = rgb_img
            writer.write_frame(cur_img)
            current_time += dt
        writer.close()

        if audio_exists:
            clip_output = mp.VideoFileClip(tmp_videopath).subclip().\
                set_audio(mp.AudioFileClip(wav_outpath))
            clip_output.write_videofile(
                output_filepath)

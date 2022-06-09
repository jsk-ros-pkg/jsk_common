import datetime
import os
import os.path as osp
import sys
import tempfile

import cv2
from moviepy.audio.io.AudioFileClip import AudioFileClip
from moviepy.video.io.ffmpeg_writer import FFMPEG_VideoWriter
from moviepy.video.io.VideoFileClip import VideoFileClip
from tqdm import tqdm

from jsk_rosbag_tools.bag_to_audio import bag_to_audio
from jsk_rosbag_tools.extract import extract_image_topic
from jsk_rosbag_tools.extract import get_image_topic_names
from jsk_rosbag_tools.image_utils import \
    resize_keeping_aspect_ratio_wrt_target_size
from jsk_rosbag_tools.info import get_topic_dict
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
                 audio_topic='/audio',
                 show_progress_bar=True):
    """Create video from rosbag file.

    Specify only either output_filepath or output_dirpath.
    If output_filepath is specified, specify image_topic.
    If output_dirpath is specified, image_topics can be specified.
    If image_topic_names is None, make all color images into video.

    """
    if not os.path.exists(input_bagfile):
        print('[bag_to_video] Input bagfile {} not exists.'
              .format(input_bagfile))
        sys.exit(1)

    if output_filepath is not None and output_dirpath is not None:
        raise ValueError(
            'Specify only either output_filepath or output_dirpath.')

    output_filepaths = []
    target_image_topics = []
    candidate_topics = get_image_topic_names(input_bagfile)

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
            # record all color topics
            image_topics = get_image_topic_names(input_bagfile,
                                                 rgb_only=True)
        target_image_topics = image_topics

        for image_topic in target_image_topics:
            output_filepaths.append(
                osp.join(
                    output_dirpath,
                    topic_name_to_file_name(image_topic) + '.mp4'))
        wav_outpath = osp.join(output_dirpath, '{}.wav'.format(
            topic_name_to_file_name(audio_topic)))

    # check topics exist.
    not_exists_topics = list(filter(
        lambda tn: tn not in candidate_topics, target_image_topics))
    if len(not_exists_topics) > 0:
        raise ValueError(
            'Topics that are not included in the rosbag are specified.'
            ' {}'.format(list(not_exists_topics)))

    print('[bag_to_video] Extracting audio from rosbag file.')
    audio_exists = bag_to_audio(input_bagfile, wav_outpath,
                                samplerate=samplerate,
                                channels=channels,
                                topic_name=audio_topic)

    dt = 1.0 / fps
    for image_topic, output_filepath in zip(target_image_topics,
                                            output_filepaths):
        print('[bag_to_video] Creating video of {} from rosbag file {}.'
              .format(image_topic, input_bagfile))
        filepath_dir = osp.dirname(output_filepath)
        if filepath_dir:
            makedirs(filepath_dir)
        if audio_exists:
            tmp_videopath = tempfile.NamedTemporaryFile(suffix='.mp4').name
        else:
            tmp_videopath = output_filepath

        images = extract_image_topic(input_bagfile, image_topic)
        topic_info_dict = get_topic_dict(input_bagfile)[image_topic]
        n_frame = topic_info_dict['messages']

        if show_progress_bar:
            progress = tqdm(total=n_frame)

        # remove 0 time stamp
        stamp = 0.0
        while stamp == 0.0:
            stamp, _, img, _ = next(images)
            if show_progress_bar:
                progress.update(1)
        start_stamp = stamp
        width, height = img.shape[1], img.shape[0]

        creation_time = datetime.datetime.utcfromtimestamp(start_stamp)
        time_format = '%y-%m-%d %h:%M:%S'
        writer = FFMPEG_VideoWriter(
            tmp_videopath,
            (width, height),
            fps, logfile=None,
            ffmpeg_params=[
                '-metadata',
                'creation_time={}'.format(
                    creation_time.strftime(time_format)),
            ])

        current_time = 0.0
        cur_img = resize_keeping_aspect_ratio_wrt_target_size(
            cv2.cvtColor(img, cv2.COLOR_BGR2RGB), width=width, height=height)
        for i, (stamp, _, bgr_img, _) in enumerate(images):
            if show_progress_bar:
                progress.update(1)
            aligned_stamp = stamp - start_stamp
            while current_time < aligned_stamp:
                current_time += dt
                writer.write_frame(cur_img)
            cur_img = resize_keeping_aspect_ratio_wrt_target_size(
                cv2.cvtColor(bgr_img, cv2.COLOR_BGR2RGB),
                width=width, height=height)
            writer.write_frame(cur_img)
            current_time += dt
        writer.close()

        if show_progress_bar:
            progress.close()

        if audio_exists:
            print('[bag_to_video] Combine video and audio')
            clip_output = VideoFileClip(tmp_videopath).subclip().\
                set_audio(AudioFileClip(wav_outpath))
            clip_output.write_videofile(
                output_filepath,
                verbose=False,
                logger='bar' if show_progress_bar else None)
        print('[bag_to_video] Created video is saved to {}'
              .format(output_filepath))

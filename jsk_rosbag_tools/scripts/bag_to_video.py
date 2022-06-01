#!/usr/bin/env python

import argparse
import datetime
import os
import sys

import cv2
import moviepy.editor as mp
from moviepy.video.io.ffmpeg_writer import FFMPEG_VideoWriter

from jsk_rosbag_tools.extract import extract_audio
from jsk_rosbag_tools.extract import extract_image_topic
from jsk_rosbag_tools.extract import get_image_topic_names


def main():
    parser = argparse.ArgumentParser(description='rosbag to video')
    parser.add_argument('--out', '-o', default='',
                        help='output directory path')
    parser.add_argument('--fps', default=30,
                        type=int)
    parser.add_argument('--samplerate', '-r', type=int, help='sampling rate',
                        default='16000')
    parser.add_argument('--channels',
                        type=int, default=1, help='number of input channels')
    parser.add_argument('--audio-topic', type=str, default='/audio')
    parser.add_argument('--image-topics', type=str,
                        nargs='+', help='Topic name to extract.')
    parser.add_argument('input_bagfile')
    args = parser.parse_args()

    input_bagfile = args.input_bagfile
    if not os.path.exists(input_bagfile):
        print('Input bagfile {} not exists.'.format(input_bagfile))
        sys.exit(1)

    if len(args.out) == 0:
        basename = os.path.basename(input_bagfile)
        dirname = os.path.dirname(input_bagfile)
        filename, _ = os.path.splitext(basename)
        out = os.path.join(dirname, filename)
    else:
        out = args.out
    os.makedirs(out, mode=0o777, exist_ok=True)

    fps = args.fps
    dt = 1.0 / fps

    wav_outpath = os.path.join(out, 'audio.wav')
    audio_exists = extract_audio(input_bagfile, wav_outpath,
                                 samplerate=args.samplerate,
                                 channels=args.channels,
                                 topic_name=args.audio_topic)

    candidates_topic_names = get_image_topic_names(
        input_bagfile, rgb_only=True)
    if args.image_topics is None:
        topic_names = candidates_topic_names
    else:
        topic_names = [tn for tn in args.image_topics
                       if tn in candidates_topic_names]

    for topic_name in topic_names:
        images = extract_image_topic(input_bagfile, topic_name)

        # remove 0 time stamp
        stamp = 0.0
        while stamp == 0.0:
            stamp, _, img, _ = next(images)
        start_stamp = stamp

        if topic_name[0] == '/':
            replaced_topic_name = topic_name[1:]
        else:
            replaced_topic_name = topic_name
        replaced_topic_name = replaced_topic_name.replace('/', '--slash--')
        output_videopath = os.path.join(
            out, "{}.mp4".format(replaced_topic_name))

        creation_time = datetime.datetime.utcfromtimestamp(start_stamp)
        time_format = '%y-%m-%d %h:%M:%S'
        writer = FFMPEG_VideoWriter(
            output_videopath,
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
            output_video_with_audio_path = os.path.join(
                out, "{}-with-audio.mp4".format(replaced_topic_name))
            clip_output = mp.VideoFileClip(output_videopath).subclip().\
                set_audio(mp.AudioFileClip(wav_outpath))
            clip_output.write_videofile(
                output_video_with_audio_path)


if __name__ == '__main__':
    main()

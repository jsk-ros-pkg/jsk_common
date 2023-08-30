#!/usr/bin/env python

import rospy
from audio_common_msgs.msg import AudioData
import wave
import time

class AudioRecorder:
    def __init__(self):
        rospy.init_node('audio_recorder', anonymous=True)
        self.channels = rospy.get_param('~channels', 1)
        self.depth = rospy.get_param('~depth', 16)
        self.sample_rate = rospy.get_param('~sample_rate', 16000)
        self.audio_data = bytearray()
        self.start_time = None
        self.save_path = rospy.get_param('~file_name', "/tmp/recorded_wave_audio.wav")
        print(self.save_path)

        self.audio_sub = rospy.Subscriber('~input/audio', AudioData, self.audio_callback)
        rospy.loginfo("Waiting for audio topic ...")

    def audio_callback(self, data):
        if self.start_time is None:
            self.start_time = time.time()
            rospy.loginfo("Recording audio.")

        self.audio_data.extend(data.data)
        self.save_audio_data()

    def save_audio_data(self):
        with wave.open(self.save_path, 'wb') as wf:
            wf.setnchannels(self.channels)
            wf.setsampwidth(self.depth // 8)
            wf.setframerate(self.sample_rate)

            wf.writeframes(self.audio_data)

def main():

    recorder = AudioRecorder()

    rospy.spin()

if __name__ == '__main__':
    main()

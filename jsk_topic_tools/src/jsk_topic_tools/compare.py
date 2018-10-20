from threading import Lock
import time
import traceback

import rospy


class ROSTopicCompare(object):
    subscriberArray = []
    topicSizesArray = []
    topicTimesArray = []
    scaleType = 0
    captureSize = 100
    lock = Lock()

    def __init__(self, scale="KB", captureSize=100):
        if scale == "B":
            self.scaleType = 1
        elif scale == "KB":
            self.scaleType = 2
        elif scale == "MB":
            self.scaleType = 3

        self.captureSize = captureSize

    def _gen_callback(self):
        self.topicSizesArray.append([])
        self.topicTimesArray.append([])
        _topic_num = len(self.subscriberArray)

        def _callback(msg):
            topic_num = _topic_num
            try:
                t = time.time()
                with self.lock:
                    self.topicTimesArray[topic_num].append(t)
                    self.topicSizesArray[topic_num].append(len(msg._buff))
                    assert(len(self.topicTimesArray[topic_num]) ==
                           len(self.topicSizesArray[topic_num]))
                    if len(self.topicTimesArray[topic_num]) > self.captureSize:
                        self.topicTimesArray[topic_num].pop(0)
                        self.topicSizesArray[topic_num].pop(0)
            except Exception:
                traceback.print_exc()
        return _callback

    def registerTopic(self, topic_name):
        sub = rospy.Subscriber(topic_name, rospy.AnyMsg, self._gen_callback())
        self.subscriberArray.append(sub)
        print("subscribed as %d: %s" %
              (len(self.subscriberArray) - 1, topic_name))

    def isAllTopicAvailable(self, size):
        with self.lock:
            return all([len(t) > size for t in self.topicTimesArray])

    def getTotalBytes(self, i):
        return sum(self.topicSizesArray[i])

    def getMaxByte(self, i):
        return max(self.topicSizesArray[i])

    def getMinByte(self, i):
        return min(self.topicSizesArray[i])

    def getMessageNum(self, i):
        return len(self.topicTimesArray[i])

    def getStartTime(self, i):
        return self.topicTimesArray[i][0]

    def getEndTime(self, i):
        return self.topicTimesArray[i][-1]

    def getBandwidth(self, i):
        return self.getTotalBytes(i) / \
            (self.getEndTime(i) - self.getStartTime(i))

    def printBandWidth(self):
        current_time = time.time()

        # first lookup the longest topic name
        longest_topic_len = 0
        for i in range(len(self.subscriberArray)):
            if len(self.subscriberArray[i].name) > longest_topic_len:
                longest_topic_len = len(self.subscriberArray[i].name)
        row_format = "{:>" + str(longest_topic_len + 1) + "}" + "{:>11}" * 5
        print("")
        print(row_format.format("Name", "avg/sec", "avg/msg",
                                "min/msg", "max/msg", "quant"))
        for i in range(len(self.subscriberArray)):
            try:
                n = self.getMessageNum(i)
                start_time = self.getStartTime(i)
                total_bytes = self.getTotalBytes(i)
                bytes_per_sec = 1.0 * total_bytes / (current_time - start_time)
                bytes_per_msg = total_bytes / n
                max_size = self.getMaxByte(i)
                min_size = self.getMinByte(i)
                res = [bytes_per_sec, bytes_per_msg, min_size, max_size]

                if self.scaleType == 1: # B
                    bps, bpm, ms, Ms = ["%.2fB" % v for v in res]
                elif self.scaleType == 2: # KB
                    bps, bpm, ms, Ms = ["%.2fKB" % (v/1000) for v in res]
                elif self.scaleType == 3: # MB
                    bps, bpm, ms, Ms = ["%.2fMB" % (v/1000000) for v in res]
                else:
                    raise "error: invalid scale type"
                    quit()
                print(row_format.format(
                    self.subscriberArray[i].name, bps, bpm, ms, Ms, n))
            except Exception:
                pass

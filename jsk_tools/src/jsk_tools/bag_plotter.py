import rosbag
import yaml
import matplotlib.pyplot as plt
import time
from progressbar import *
import sys
import argparse
import rospy
try:
    import colorama
except:
    print "Please install colorama by pip install colorama"
    sys.exit(1)
from colorama import Fore, Style
def accessMessageSlot(msg, field):
    if len(field) == 0:
        return msg
    return accessMessageSlot(getattr(msg, field[0]), field[1:])

class PlotData():
    def __init__(self, options):
        self.topics = options["topic"]
        fields = options["field"]
        self.fields_orig = fields
        self.fields = [f.split("/") for f in fields]
        self.values = []
        for i in range(len(fields)):
            self.values.append([])
        self.options = options
    def addValue(self, topic, value):
        if topic not in self.topics:
            return
        for target_topic, i in zip(self.topics, range(len(self.topics))):
            if target_topic == topic:
                self.values[i].append((value.header.stamp,
                                       accessMessageSlot(value, self.fields[i])))
    def filter(self, start_time, end_time):
        for i in range(len(self.values)):
            self.values[i] = [v for v in self.values[i]
                              if v[0] >= start_time and
                              v[0] <= end_time]
    def plot(self, min_stamp, fig, layout):
        ax = fig.add_subplot(*layout)
        for vs, i in zip(self.values, range(len(self.values))):
            xs = [v[0].to_sec() - min_stamp.to_sec() for v in vs]
            ys = [v[1] for v in vs]
            ax.plot(xs, ys, label=self.topics[i] + "/" + self.fields_orig[i])
        ax.set_title(self.options["title"])
        if self.options["legend"]:
            ax.legend()
        self.ax = ax

class BagPlotterException(Exception):
    pass

class BagPlotter():
    def __init__(self):
        pass
        # self.bag_file = bag_file
        # self.conf_file = conf_file
    def parse(self):
        parser = argparse.ArgumentParser(description='Plot from bag file')
        parser.add_argument('config',
                            help='yaml file to configure plot')
        parser.add_argument('bag', nargs="+",
                            help='bag file to plot')
        parser.add_argument('--duration', '-d', type=int,
                            help='Duration to plot')
        parser.add_argument('--start-time', '-s', type=int, default=0,
                            help='Start time to plot')
        args = parser.parse_args()
        self.bag_file = args.bag
        self.conf_file = args.config
        self.duration = args.duration
        self.start_time = args.start_time
    def processConfFile(self):
        """
        conf file format is:
        global:
          layout: "vertical" or "horizontal"
        plots:
          - title: "title"
            type: "line" or "hist"
            topics:
              - topic: "topic name"
                field: "field name"
              - topic: "topic name"
                field: "field name"
            legend: true
          - title: "title"
            type: "line" or "hist"
            topics:
              - topic: "topic name"
                field: "field name"
              - topic: "topic name"
                field: "field name"
            legend: true
        """
        with open(self.conf_file) as f:
            data = yaml.load(f)
            self.setGlobalOptions(data)
            self.setPlotOptions(data)
    def readOption(self, option, name, default_value):
        if option.has_key(name):
            return option[name]
        else:
            return default_value
    def setGlobalOptions(self, data):
        global_options = self.readOption(data, "global", dict())
        self.global_options = dict()
        self.global_options["layout"] = self.readOption(global_options, "layout", "vertical")
    def setPlotOptions(self, data):
        plot_options = self.readOption(data, "plots", [])
        if len(plot_options) == 0:
            raise BagPlotterException("No plots section in conf file")
        self.plot_options = []
        self.all_topics = set()
        self.topic_data = []
        for opt in plot_options:
            if not opt.has_key("title"):
                raise BagPlotterException("plot config requires title section")
            opt["type"] = self.readOption(opt, "type", "line")
            opt["legend"] = self.readOption(opt, "legend", True)
            
            if not opt.has_key("topic"):
                raise BagPlotterException("plots config requires topic section")
            if not opt.has_key("field"):
                raise BagPlotterException("plots config requires fields section")
            if isinstance(opt["topic"], str):
                opt["topic"] = [opt["topic"]]
            if isinstance(opt["field"], str):
                opt["field"] = [opt["field"]]
            if len(opt["topic"]) != len(opt["field"]):
                raise BagPlotterException("lengt of topic and field should be same")
            for topic in opt["topic"]:
                self.all_topics.add(topic)
                
            self.topic_data.append(PlotData(opt))
            self.plot_options.append(opt)
    def plot(self):
        plt.interactive(True)
        fig = plt.figure(facecolor="1.0")
        min_stamp = None
        max_stamp = None
        no_valid_data = True
        for abag in self.bag_file:
            with rosbag.Bag(abag) as bag:
                info = yaml.load(bag._get_yaml_info())
                message_num = info["messages"]
                widgets = [Fore.GREEN + "%s: " % (abag) + Fore.RESET, Percentage(), Bar()]
                pbar = ProgressBar(maxval=message_num, widgets=widgets).start()
                counter = 0
                for topic, msg, timestamp in bag.read_messages():
                    pbar.update(counter)
                    if topic in self.all_topics:
                        for topic_data in self.topic_data:
                            topic_data.addValue(topic, msg)
                            no_valid_data = False
                        if min_stamp:
                            if min_stamp > msg.header.stamp:
                                min_stamp = msg.header.stamp
                        else:
                            min_stamp = msg.header.stamp
                        if max_stamp:
                            if max_stamp < msg.header.stamp:
                                max_stamp = msg.header.stamp
                        else:
                            max_stamp = msg.header.stamp
                    counter = counter + 1
                pbar.finish()
        if no_valid_data:
            print Fore.RED + "Cannot find valid data in bag files, valid topics are:\n%s" % ", ".join(self.all_topics) + Fore.RESET
            return
        print ("""Plot from %s to %s (%d secs)""" %
               (str(time.ctime(min_stamp.to_sec())),
                str(time.ctime(max_stamp.to_sec())),
                (max_stamp - min_stamp).to_sec()))
        start_time = rospy.Duration(self.start_time) + min_stamp
        if self.duration:
            end_time = start_time + rospy.Duration(self.duration)
        else:
            end_time = max_stamp
        for topic_data in self.topic_data:
            topic_data.filter(start_time, end_time)
        for topic_data, i in zip(self.topic_data, range(len(self.topic_data))):
            topic_data.plot(start_time,
                            fig, (len(self.topic_data), 1, i))
        fig.subplots_adjust(hspace=0.4)
        plt.draw()
        plt.show()
        while True:
            plt.pause(1)
    def run(self):
        self.processConfFile()
        self.plot()

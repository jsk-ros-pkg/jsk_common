#!/usr/bin/env python

import rosbag
import yaml
import matplotlib
import sys
if "-o" in sys.argv:            # write to file mode
    matplotlib.use("AGG")
else:
    matplotlib.use("WXAgg")
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.widgets import Button
import time
from progressbar import *
from std_msgs.msg import Header
import argparse
import rospy
import re

try:
    import colorama
except:
    print("Please install colorama by pip install colorama")
    sys.exit(1)
from colorama import Fore, Style

class MessageWrapper():
    def __init__(self, obj, header):
        self.__field__ = dict()
        self.obj = obj
        self.header = header

class MessageFieldAccessor():
    _is_slot_array_regexp = re.compile("\[([0-9]+)\]$")
    _extract_slot_array_regexp = re.compile("(.*)\[([0-9]+)\]$")
    def __init__(self, field):
        # field = ["hoge", "fuga", "piyo[3]"]
        self.parsed_fields = []
        for f in field:
            if self._is_slot_array_regexp.search(f):
                res = self._extract_slot_array_regexp.match(f)
                self.parsed_fields.append(res.group(1))
                self.parsed_fields.append(int(res.group(2)))
            else:
                self.parsed_fields.append(f)
    def parse(self, msg):
        for f in self.parsed_fields:
            if isinstance(f, int):
                msg = msg[f]
            else:
                msg = getattr(msg, f)
        return msg

def expandArrayFields(fields, topics, colors):
    ret_fields = []
    ret_topics = []
    ret_colors = []
    for f, t, c in zip(fields, topics, colors):
        search = re.search("\[.*\]" , f)
        if search:
            tmp=search.group().strip("[]").split(",")
            def colon2fig(txt):
                ret=re.match("([0-9]+):([0-9]+)",txt)
                if ret:
                    return range(int(ret.group(1)),int(ret.group(2)))
                else:
                    return [int(txt)]
            tmp=map(colon2fig,tmp)
            fList=reduce(lambda x,y: x+y,tmp)
            for i in fList:
                ret_fields.append(re.sub("\[.*\]", "[" + str(i) + "]", f))
                ret_topics.append(t)
                ret_colors.append(c)
        else:
            ret_fields.append(f)
            ret_topics.append(t)
            ret_colors.append(c)
    return (ret_fields, ret_topics, ret_colors)

class PlotData():
    def __init__(self, options):
        self.legend_font_size = 8
        self.label_font_size = 8
        self.title_font_size = 8
        (self.fields_orig, self.topics, self.colors) = expandArrayFields(options["field"], options["topic"], options["color"])
        self.fields = [f.split("/") for f in self.fields_orig]
        self.field_accessors = [MessageFieldAccessor(f) for f in self.fields]
        self.time_offset = options["time_offset"]
        if "label" in options:
            self.label = options["label"]
        else:
            self.label = None
        if "xlabel" in options:
            self.xlabel = options["xlabel"]
        else:
            self.xlabel = None
        if "ylabel" in options:
            self.ylabel = options["ylabel"]
        else:
            self.ylabel = None
        self.values = []
        for i in range(len(self.fields)):
            self.values.append([])
        self.options = options
    def addValue(self, topic, value):
        if topic not in self.topics:
            return
        for target_topic, i in zip(self.topics, range(len(self.topics))):
            if target_topic == topic:
                self.values[i].append((value.header.stamp,
                                       self.field_accessors[i].parse(value.obj)))
    def filter(self, start_time, end_time):
        for i in range(len(self.values)):
            self.values[i] = [v for v in self.values[i]
                              if v[0] >= start_time and
                              v[0] <= end_time]
    def plot(self, min_stamp, fig, layout, show_legend, share_ax = None):
        if share_ax:
            ax = fig.add_subplot(layout, sharex=share_ax)
        else:
            ax = fig.add_subplot(layout)
        for vs, i in zip(self.values, range(len(self.values))):
            xs = [v[0].to_sec() - min_stamp.to_sec() + self.time_offset for v in vs]
            ys = [v[1] for v in vs]
            if self.label:
                ax.plot(xs, ys, label=self.label[i], color=self.colors[i])
            else:
                ax.plot(xs, ys, label=self.topics[i] + "/" + self.fields_orig[i], color=self.colors[i])
        ax.set_title(self.options["title"], fontsize=self.title_font_size)
        if self.xlabel:
            ax.set_xlabel(self.xlabel)
        if self.ylabel:
            ax.set_ylabel(self.ylabel)
        ax.tick_params(labelsize=self.label_font_size)
        if show_legend and self.options["legend"]:
            legend = ax.legend(prop={'size': self.legend_font_size}, frameon=False)
        ax.minorticks_on()
        ax.grid(True)
        self.ax = ax
        return ax

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
        parser.add_argument('--no-title', action='store_true', help='Do not show title')
        parser.add_argument('-o', help='Write to file')
        args = parser.parse_args()
        self.output_file = args.o
        self.bag_file = args.bag
        self.conf_file = args.config
        self.duration = args.duration
        self.start_time = args.start_time
        self.no_title = args.no_title
    def processConfFile(self):
        """
        conf file format is:
        global:
          layout: "vertical" or "horizontal"
          legend_font_size: font size for the legend
          label_font_size: font size for the axis label
          title_font_size: font size for each plot
        plots:
          - title: "title 1"
            type: "line" or "hist"
            topic: [topic1, topic2, ...]
            field: [field1, field2, ...]
            color: [color1, color2, ...]
            legend: true
          - title: "title 2"
            type: "line" or "hist"
            topic: [topic1, topic2, ...]
            field: [field1, field2, ...]
            color: [color1, color2, ...]
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
        self.global_options["legend_font_size"] = self.readOption(global_options, "legend_font_size", 8)
        self.global_options["label_font_size"] = self.readOption(global_options, "label_font_size", 8)
        self.global_options["title_font_size"] = self.readOption(global_options, "title_font_size", 8)
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
            opt["layout"] = self.readOption(opt, "layout", None)
            opt["time_offset"] = self.readOption(opt, "time_offset", 0)
            if self.global_options["layout"] == "manual" and opt["layout"] == None:
                raise BagPlotterException("Need to specify layout field for manual layout")
            if not opt.has_key("topic"):
                raise BagPlotterException("plots config requires topic section")
            if not opt.has_key("field"):
                raise BagPlotterException("plots config requires fields section")
            if isinstance(opt["topic"], str):
                opt["topic"] = [opt["topic"]]
            if isinstance(opt["field"], str):
                opt["field"] = [opt["field"]]
            if not opt.has_key("color"):
                opt["color"] = [None]*len(opt["topic"])
            if not isinstance(opt["color"], list):                
                opt["color"] = [opt["color"]]
            if len(opt["topic"]) != len(opt["field"]):
                raise BagPlotterException("length of topic and field should be same")
            if len(opt["topic"]) != len(opt["color"]):
                raise BagPlotterException("length of topic and color should be same")
            for topic in opt["topic"]:
                self.all_topics.add(topic)
            self.topic_data.append(PlotData(opt))
            self.topic_data[-1].legend_font_size = self.global_options["legend_font_size"]
            self.topic_data[-1].label_font_size = self.global_options["label_font_size"]
            self.topic_data[-1].title_font_size = self.global_options["title_font_size"]
            self.plot_options.append(opt)
    def layoutGridSize(self):
        if self.global_options["layout"] == "vertical":
            return (len(self.topic_data), 1)
        elif self.global_options["layout"] == "horizontal":
            return (1, len(self.topic_data))
        elif self.global_options["layout"] == "manual":
            max_x = 0
            max_y = 0
            for topic_data in self.topic_data:
                max_x = max(topic_data.options["layout"][0], max_x)
                max_y = max(topic_data.options["layout"][1], max_y)
            return (max_y + 1, max_x + 1)
    def layoutPosition(self, gs, topic_data, i):
        if self.global_options["layout"] == "vertical":
            return gs[i]
        elif self.global_options["layout"] == "horizontal":
            return gs[i]
        elif self.global_options["layout"] == "manual":
            return gs[topic_data.options["layout"][1], topic_data.options["layout"][0]]

    def plot(self):
        plt.interactive(True)
        min_stamp = None
        max_stamp = None
        no_valid_data = True
        for abag in self.bag_file:
            with rosbag.Bag(abag) as bag:
                info = yaml.load(bag._get_yaml_info())
                message_num = sum([topic["messages"] for topic in info["topics"]
                                   if topic["topic"] in self.all_topics])
                widgets = [Fore.GREEN + "%s: " % (abag) + Fore.RESET, Percentage(), Bar()]
                pbar = ProgressBar(maxval=max(1, message_num), widgets=widgets).start()
                counter = 0
                read_data = [(topic, msg, timestamp)
                             for topic, msg, timestamp
                             in bag.read_messages(topics=self.all_topics)]
                for topic, msg, timestamp in read_data:
                    pbar.update(counter)
                    # check topic has header field
                    if not hasattr(msg, "header"):
                        msg = MessageWrapper(msg, Header())
                        msg.header.stamp = timestamp
                    else:
                        msg = MessageWrapper(msg, msg.header)
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
            print(Fore.RED + "Cannot find valid data in bag files, valid topics are:\n%s" % ", ".join(self.all_topics) + Fore.RESET)
            return
        title = ("""Plot %s using %s from [%s] to [%s] (%d secs)""" %
                 (", ".join(self.bag_file),
                  self.conf_file,
                  str(time.ctime(min_stamp.to_sec())),
                  str(time.ctime(max_stamp.to_sec())),
                  (max_stamp - min_stamp).to_sec()))
        start_time = rospy.Duration(self.start_time) + min_stamp
        if self.duration:
            end_time = start_time + rospy.Duration(self.duration)
        else:
            end_time = max_stamp
        for topic_data in self.topic_data:
            topic_data.filter(start_time, end_time)
            
        fig = plt.figure(facecolor="1.0")
        self.fig = fig
        if not self.no_title:
            fig.suptitle(title)
        self.show_legend = True
        fig.canvas.mpl_connect('key_press_event', self.keyPress)
        # Compute layout
        self.start_time = start_time
        self.plotAll(fig, start_time, self.show_legend)
        if self.output_file:
            fig = matplotlib.pyplot.gcf()
            fig.set_size_inches(40, 30)
            fig.savefig(self.output_file)
        else:
            self.printUsage()
            self.runp = True
            while self.runp:
                plt.pause(1)
    def keyPress(self, event):
        if event.key == "l" or event.key == "L":
            self.toggleLegend()
        elif event.key == "q" or event.key == "Q":
            self.runp = False
    def printUsage(self):
        print("Usage::")
        print("  l or L:  toggle legend")
        print("  q or Q:  quit")
    def toggleLegend(self):
        self.show_legend = not self.show_legend
        plt.clf()
        self.plotAll(self.fig, self.start_time, self.show_legend)
    def plotAll(self, fig, start_time, show_legend):
        grid_size = self.layoutGridSize()
        gs = gridspec.GridSpec(*grid_size)
        ax = None
        for topic_data, i in zip(self.topic_data, 
                                 range(len(self.topic_data))):
            ax = topic_data.plot(start_time,
                                 fig,
                                 self.layoutPosition(gs, topic_data, i),
                                 show_legend, share_ax=ax)
        fig.subplots_adjust(hspace=0.4)
        plt.draw()
        plt.show()

    def run(self):
        self.processConfFile()
        self.plot()

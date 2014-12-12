#!/usr/bin/python

import Gnuplot
import time
import numpy
import rospy
from std_msgs.msg import Float32

gp=Gnuplot.Gnuplot()

plot_topic = rospy.get_param("~plot_topic", [('/eth0/receive', 'reveive'), ('eth0/transmit', 'transmit')])

max_y = rospy.get_param("~max_y", 1200000)
data_scale = rospy.get_param("~data_scale", 0.001)

data_rate = rospy.get_param("~data_rate", 100)
period = rospy.get_param("~period", 120)
drop_num = rospy.get_param("~drop_num", 10) #drop 9 topic out of 10
drop_index = [0 for i in range(len(plot_topic))]

xrange = (period, 0)
yrange = (0, (int)(max_y * data_scale))

xlabel = rospy.get_param("~x_label", "Time (s)")
ylabel = rospy.get_param("~y_label", "Network Data Rates (kbps)")
title = rospy.get_param("~title", "Network Status")

plot_size = data_rate * period / drop_num

y_list = [numpy.zeros(plot_size) for i in range(len(plot_topic))]
x = numpy.arange(plot_size) / float(plot_size / period)

def callback(msg, index):
    global drop_index
    if drop_index[index] != drop_num:
        drop_index[index] = drop_index[index] + 1
        return
    drop_index[index] = 0

    global y_list

    y_list[index] = numpy.delete(y_list[index], plot_size - 1)
    y_list[index] = numpy.insert(y_list[index], 0, msg.data * data_scale)
    try:
        d_list = [Gnuplot.Data(x, y, title=plot_topic[i][1], with_='lines linewidth 2') for (i, y) in enumerate(y_list)]
        if len(d_list) == 1:
            gp.plot(d_list[0])
        elif len(d_list) == 2:
            gp.plot(d_list[0], d_list[1])
        elif len(y_list) == 3:
            gp.plot(d_list[0], d_list[1])
        elif len(y_list) == 4:
            gp.plot(d_list[0], d_list[1])
        elif len(y_list) == 5:
            gp.plot(d_list[0], d_list[1])
        elif len(y_list) == 6:
            gp.plot(d_list[0], d_list[1])
    except:
        pass

if __name__ == '__main__':
    rospy.init_node("network_plot")
    for i in range(len(plot_topic)):
        topic = plot_topic[i][0]
        rospy.Subscriber(topic, Float32, callback, callback_args = i)

    gp.set(xrange=xrange, yrange=yrange, title=title, xlabel=xlabel, ylabel=ylabel)
    rospy.spin()

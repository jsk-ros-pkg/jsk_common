#!/usr/bin/python
import Gnuplot
import time
import numpy
import rospy
from std_msgs.msg import Float32

gp=Gnuplot.Gnuplot()

data_rate = 100
period = 120
drop_num = 10 #drop 9 topic out of 10
i = 0

xrange = (period, 0)
yrange = (0, 1000000)
xlabel = "Time (s)"
ylabel = "Network Data Rates (Bps)"
title = "Network Status"

plot_size = data_rate * period / drop_num

y = numpy.zeros(plot_size)
x = numpy.arange(plot_size) / float(plot_size / period)

def callback(msg):
    global i
    if i != drop_num:
        i = i + 1
        return
    i = 0

    global y
    y = numpy.delete(y, plot_size - 1)
    y = numpy.insert(y, 0, msg.data)

    d = Gnuplot.Data(x, y, with_='lines linewidth 1')
    gp.plot(d)


rospy.init_node("test")
rospy.Subscriber("/eth0/receive", Float32, callback)

gp.set(xrange=xrange, yrange=yrange, title=title, xlabel=xlabel, ylabel=ylabel)
rospy.spin()

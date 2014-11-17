#!/bin/env python

#**********************************************************************
# Copyright (c) 2013 InterWorking Labs, All Rights Reserved           *
#                                                                     *
#              RESTRICTED RIGHTS LEGEND                               *
#                                                                     *
# This software is provided with RESTRICTED RIGHTS.                   *
#                                                                     *
# Use, duplication, or disclosure by the U.S. Government is subject   *
# to restrictions set forth in subparagraph (c)(1)(ii) of the Rights  *
# in Technical Data and Computer Software clause at DFARS             *
# 252.227-7013 or subparagraphs (c)(1) and (2) of the Commercial      *
# Computer Software - Restricted Rights at 48 CFR 52.227-19, as       *
# applicable.  The "Manufacturer" for purposes of these regulations   *
# is InterWorking Labs, PO Box 66190, Scotts Valley, California 95060 *
# U.S.A.                                                              *
#**********************************************************************

#**********************************************************************
# $Id: periodic26.py 464 2013-12-03 00:43:56Z karl $
#**********************************************************************

# http://stackoverflow.com/questions/2398661/schedule-a-repeating-event-in-python-3

have_argparse = False
try:
    import argparse
    have_argparse = True
except:
    import optparse

import datetime

import platform
import sys
import syslog
import threading
import time
import traceback

import mm2client
import setfilters26 as setfilters

#**********************************************************************
# SetupJobs()
#
# Here is where we set up the jobs to be done and the schedule to
# do them.
#**********************************************************************
def SetupJobs(Jobs, mm_hostname):

    #******************************
    # Do not change these...
    LANA_to_LANB = True
    LANB_to_LANA = False
    #******************************

    BandSettings = mm2client.Bands()

    # A Band is for our 100ms/1megabit path
    # B band for the 1000ms/1megabit path
    A_Band = 1
    B_Band = 2

    #**************************************************
    # Here we set rate limitation value that we will use
    #**************************************************
    OneMegabit = 1000000	# Some might argue that one megabit is 1048576 (2**20)
    OneKilobit = 100000		# Some might argue that one kilobit is 1024000

    #**************************************************
    # Here we set the two delay values.
    # Note: These are round trip times.
    #**************************************************
    LongRoundtrip = 1000
    ShortRoundtrip = 100

    # We will divide the round trip into two equal one-way parts.
    HalfLongRoundtrip = LongRoundtrip/2
    HalfShortRoundtrip = ShortRoundtrip/2

    #**************************************************
    # Set up our classification filters.
    # We will be using these as a two-way switch to
    # send selected packets first into the A band and then
    # into the B band and then back to the A band etc.
    # We will let any other forms of traffic flow via
    # Band 5 which will get no impairments.
    # There things that don't get picked up by the
    # filters below are almost certainly low traffic
    # things like OSPF.
    #**************************************************
    Filt_ToBandA = [setfilters.FiltSetting("arp", A_Band),
                    setfilters.FiltSetting("ipv4", A_Band),
                    setfilters.FiltSetting("ipv6", A_Band)
                    ]

    Filt_ToBandB = [setfilters.FiltSetting("arp", B_Band),
                    setfilters.FiltSetting("ipv4", B_Band),
                    setfilters.FiltSetting("ipv6", B_Band)
                    ]

    # Define A_Band to have 100ms round trip, 1megabit/second rate limit
    BandSettings.SetDelayAmount(A_Band, LANA_to_LANB, HalfShortRoundtrip)
    BandSettings.SetDelayAmount(A_Band, LANB_to_LANA, HalfShortRoundtrip)
    BandSettings.SetDelayReorder(A_Band, LANA_to_LANB, False)
    BandSettings.SetDelayReorder(A_Band, LANB_to_LANA, False)
    BandSettings.SetRateLimit(A_Band, LANA_to_LANB, OneMegabit)
    BandSettings.SetRateLimit(A_Band, LANB_to_LANA, OneMegabit)

    # Define B_Band with 1000ms round trip, 1kilobit/second rate limit
    BandSettings.SetDelayAmount(B_Band, LANA_to_LANB, HalfLongRoundtrip)
    BandSettings.SetDelayAmount(B_Band, LANB_to_LANA, HalfLongRoundtrip)
    BandSettings.SetDelayReorder(B_Band, LANA_to_LANB, False)
    BandSettings.SetDelayReorder(B_Band, LANB_to_LANA, False)
    BandSettings.SetRateLimit(B_Band, LANA_to_LANB, OneKilobit)
    BandSettings.SetRateLimit(B_Band, LANB_to_LANA, OneKilobit)

    #**************************************************
    # Create an initial job at start-time zero.
    # Here is where we establish our baseline.
    # Note that this baseline is a different thing than
    # the defaults, which are simply the absence of
    # filters and impairments.
    #**************************************************
    Jobs.AddRequest("Establish Baseline - 100ms RT delay, 1,000,000 bit/sec rate limit", mm_hostname, 0,
                    BandSettings, Filt_ToBandA, Filt_ToBandA)

    #**************************************************
    # Set up the repeating schedule of jobs
    #**************************************************
    interval = 60	# Seconds per item
    items_per_group = 2
    num_groups = 720	# Number of groups ==> 24 hours

    #Remember range() stops at < stop value not <=
    for secs in range(interval, items_per_group*num_groups*interval, items_per_group*interval):
        Jobs.AddRequest("Job %u: 1000ms RT delay, 100,000 bit/sec rate limit" % \
                            secs, mm_hostname, secs, None, Filt_ToBandB, Filt_ToBandB)
        nxtsecs = secs + interval
        Jobs.AddRequest("Job %u: 100ms RT delay, 1,000,000 bit/sec rate limit" % \
                            nxtsecs, mm_hostname, nxtsecs, None, Filt_ToBandA, Filt_ToBandA)
    #Jobs.PrintMe()

#**********************************************************************
# ShowMessage()
#**********************************************************************
UseSyslog = False # Set to True to send things to the syslog
TeamName = "TEAM UNKNOWN" # Should be set to the actual team name

def ShowMessage(*args):
    datetime.datetime.now().isoformat()
    msg = datetime.datetime.now().isoformat() \
          + ' "' + TeamName + '" ' \
          + " ".join(args)
    print msg
    if UseSyslog:
        syslog.syslog(syslog.LOG_INFO, msg)

#**********************************************************************
# RepeatedTimer
#
# Features:
#  - Standard library only, no external dependencies.
#  - Start() and stop() are safe to call multiple times even if the
#    timer has already started/stopped.
#  - Function to be called can have positional and named arguments.
#  - You can change interval anytime, it will be effective after
#    next run. Same for args, kwargs and even the function.
#**********************************************************************
class RepeatedTimer(object):
    def __init__(self, interval, function, *args, **kwargs):
        self._timer     = None
        self.function   = function
        self.interval   = interval
        self.args       = args
        self.kwargs     = kwargs
        self.is_running = False
        self.start()

    def _run(self):
        self.is_running = False
        self.start()
        self.function(*self.args, **self.kwargs)

    def start(self):
        if not self.is_running:
            self._timer = threading.Timer(self.interval, self._run)
            self._timer.start()
            self.is_running = True

    def stop(self):
        self._timer.cancel()
        self.is_running = False
        

#**********************************************************************
# DoRequest
#**********************************************************************
class DoRequest(object):
    def __init__(self, reqname, mm2host, dowhen, bands, a2b_filtmap, b2a_filtmap):
        """
        @param reqname: The name of the request, a single line of text.
        @type reqname: String
        @param mm2host: The name or IP address of the target host.
        @type mm2host: String
        @param dowhen: A datetime object indicating when to do perform this request.
        @type dowhen: datetime
        @param bands: A mm2client.Bands object containing the desired impairment settings.
                      May be None.
        @type bands: mm2client.Bands
        @param a2b_filtmap: A sequence of setfilters.FiltSetting objects containing the desired LanA to LanB filters settings.
                            May be an empty sequence.
        @type a2b_filtmap: A sequence of setfilters.FiltSetting objects.
        @param b2a_filtmap: A sequence of setfilters.FiltSetting objects containing the desired LanB to LanA filters settings.
                            May be an empty sequence.
        @type b2a_filtmap: A sequence of setfilters.FiltSetting objects.
        """
        self.__Name = reqname
        self.__MM2HostName = mm2host
        self.__DoWhen = dowhen
        self.__Bands = bands
        self.__A2BFiltmap = a2b_filtmap
        self.__B2AFiltmap = b2a_filtmap
        self.__Done = False

    def __lt__(self, other):
        return self.__DoWhen < other.__DoWhen
    
    def __le__(self, other):
        return self.__DoWhen <= other.__DoWhen

    def __eq__(self, other):
        return self.__DoWhen == other.__DoWhen

    def __ne__(self, other):
        return self.__DoWhen != other.__DoWhen

    def __gt__(self, other):
        return self.__DoWhen > other.__DoWhen

    def __ge__(self, other):
        return self.__DoWhen <= other.__DoWhen

    def __repr__(self):
        return repr({
                "Name": self.__Name,
                "MM2HostName": self.__MM2HostName,
                "DoWhen": self.__DoWhen,
                "Bands": repr(self.__Bands),
                "A2BFiltMap": repr(self.__A2BFiltmap),
                "B2AFiltMap": repr(self.__B2AFiltmap),
                "Done": self.__Done
                })

    def __str__(self):
        return self.__repr__()

    @property
    def Name(self):
        return self.__Name

    @property
    def MM2HostName(self):
        return self.__MM2HostName

    @property
    def DoWhen(self):
        return self.__DoWhen

    @property
    def Bands(self):
        return self.__Bands

    @property
    def A2BFiltmap(self):
        return self.__A2BFiltmap

    @property
    def B2AFiltmap(self):
        return self.__B2AFiltmap

    @property
    def IsDone(self):
        return self.__Done

    def IsReadyToGo(self, now):
        return (not self.__Done) and (self.__DoWhen <= now)

    def Run(self, now):
        global AllFilterNames
        assert self.IsReadyToGo(now)
        self.__Done = True
        SetMM(self.__MM2HostName, self.__Bands, 
              self.__A2BFiltmap, self.__B2AFiltmap, AllFilterNames)

#**********************************************************************
# RunList
#**********************************************************************
class RunList(object):
    def __init__(self, now, skipsecs):
        self.__WorkList = []
        self.__StartTime = now
        if (skipsecs is not None) and (skipsecs > 0):
            self.__StartTime = self.__StartTime - datetime.timedelta(0, skipsecs)

    def AddRequest(self, reqname, mm2host, startsec, bands,
                   a2b_filtmap, b2a_filtmap):
        dowhen = self.__StartTime + datetime.timedelta(0, startsec)
        self.__WorkList.append(DoRequest(reqname, mm2host, dowhen, bands,
                                         a2b_filtmap, b2a_filtmap))
        self.__WorkList.sort()

    def RunNextRequest(self, nowtime):
        # timedelta = nowtime - self.__StartTime
        # We are going to do a sequential scan of the work list to
        # find the first one that has not already been done and
        # that has reached its start time.
        # This check happens about once a second.
        for ndx in range(len(self.__WorkList)):
            req = self.__WorkList[ndx]
            if req.IsReadyToGo(nowtime):
                ShowMessage("Starting task \"%s\"" % req.Name)
                try:
                    req.Run(nowtime)
                except Exception, err:
                    ShowMessage("Exception in Task \"%s\" - %s" % (req.Name, str(err)))
                del self.__WorkList[ndx]
                return True
        return False

    @property
    def LastRequestStartTime(self):
        if len(self.__WorkList) == 0:
               return datetime.datetime.now()
        return self.__WorkList[-1].DoWhen

    def PrintMe(self):
        for d in self.__WorkList:
            ShowMessage("RunRequest", str(d))

#**********************************************************************
# SetMM()
#**********************************************************************
def SetMM(mm_hostname, bands, a2bfilt_vals, b2afilt_vals, all_filter_names):

    if bands is not None:
        mm2client.ChangeBandsOnMM(bands, mm_hostname)
    if (a2bfilt_vals is not None) or (b2afilt_vals is not None):
        setfilters.SetFiltMap(mm_hostname, a2bfilt_vals, b2afilt_vals,
                              all_filter_names)

#**********************************************************************
# We will try to avoid redundant queries to fetch filter names by
# caching it.
# It is simply a list of strings.
# A value of None means that it will be fetched each time rather than
# cached.
#**********************************************************************
AllFilterNames = None

#**********************************************************************
#
#**********************************************************************
pvt = platform.python_version_tuple()
if (int(pvt[0]) == 2) and (int(pvt[1]) < 7):
    def TotalSeconds(td):
        return (td.microseconds + (td.seconds + td.days * 24 * 3600) * 10**6) / 10**6
else:
    def TotalSeconds(td):
        return td.total_seconds()

#**********************************************************************
# main()
#**********************************************************************
def main():
    global UseSyslog
    global AllFilterNames
    global TeamName

    #******************************
    # jobtick() - Where we run the job
    #******************************
    def jobtick():
        Jobs.RunNextRequest(datetime.datetime.now())

    #******************************
    # Deal with the arguments
    #******************************
    if have_argparse:
        parser = argparse.ArgumentParser(description="Start impairment pattern.")

        parser.add_argument("-s", "--skip", type=int,
                            help="Skip ahead: start at N seconds.")

        parser.add_argument("-C", "--do_not_use_initial_defaults", action="store_true",
                            dest="no_initial_defaults",
                            help="Do not establish defaults before running scheduled tasks.")

        parser.add_argument("-D", "--initial_defaults_then_exit", action="store_true",
                            dest="initial_defaults_only",
                            help="Establish defaults and then exit, supersedes -C.")

        parser.add_argument("-l", "--loop", action="store_true",
                            help="Loop/Cycle forever.")

        parser.add_argument("-S", "--syslog", action="store_true",
                            help="Send reports to the system defined syslog server (level LOG_INFO).")

        parser.add_argument("-T", "--team", required=True, metavar="<team name>",
                            help="Team name.")

        parser.add_argument("mm_hostname",  metavar="<Mini Maxwell host name or IP address>",
                            help='The host name or IP address of the Mini Maxwell/Maxwell G.')

        pargs = parser.parse_args()

        TeamName = pargs.team
        if pargs.syslog:
            UseSyslog = True

        mm_hostname = pargs.mm_hostname

        do_initial_defaults_only = pargs.initial_defaults_only
        do_no_initial_defaults = pargs.no_initial_defaults
        do_skipsecs = pargs.skip
        do_loop = pargs.loop

    else:
        parser = optparse.OptionParser("""usage: %prog [-h] [-s SKIP] [-C] [-D] [-l] [-S] -T <team name>
                   <Mini Maxwell host name or IP address>""")

        parser.add_option("-s", "--skip", type=int,
                            help="Skip ahead: start at N seconds.")

        parser.add_option("-C", "--do_not_use_initial_defaults", action="store_true",
                          dest="no_initial_defaults",
                          help="Do not establish defaults before running scheduled tasks.")

        parser.add_option("-D", "--initial_defaults_then_exit", action="store_true",
                          dest="initial_defaults_only",
                          help="Establish defaults and then exit, supersedes -C.")

        parser.add_option("-l", "--loop", action="store_true",
                          help="Loop/Cycle forever.")

        parser.add_option("-S", "--syslog", action="store_true",
                          help="Send reports to the system defined syslog server (level LOG_INFO).")

        parser.add_option("-T", "--team", metavar="<team name>",
                          help="Team name.")

        (options, args) = parser.parse_args()

        TeamName = options.team
        if TeamName is None:
            parser.error("Team name must be supplied.")

        if options.syslog:
            UseSyslog = True

        try:
            mm_hostname = args[0]
        except:
            parser.error("Mini Maxwell host name or IP address must be supplied.")

        do_initial_defaults_only = options.initial_defaults_only
        if do_initial_defaults_only is None:
            do_initial_defaults_only = False

        do_no_initial_defaults = options.no_initial_defaults
        if do_no_initial_defaults is None:
            do_no_initial_defaults = False

        do_skipsecs = options.skip

        do_loop = options.loop
        if do_loop is None:
            do_loop = False

    if AllFilterNames is None:
        AllFilterNames = setfilters.GetAllFilterNames(mm_hostname)

    #**************************************************
    # Do we just set the defaults and then exit?
    #**************************************************
    if do_initial_defaults_only:
        # Coerce default filter settings and impairments to a known (default) state
        ShowMessage("Coercing filters and impairments to default state, then exiting")
        SetMM(mm_hostname, mm2client.Bands(), [], [], AllFilterNames)
        ShowMessage("Stopping - Complete")
        sys.exit()

    #**************************************************
    # Set default values (unless suppressed)
    #**************************************************
    if do_no_initial_defaults:
        ShowMessage("Not setting initial defaults")
    else:
        # Coerce default filter settings and impairments to a known (default) state
        ShowMessage("Coercing filters and impairments to default state")
        SetMM(mm_hostname, mm2client.Bands(), [], [], AllFilterNames)

    skipsecs = do_skipsecs

    #**************************************************
    # Main scheduling loop
    # Note: This loop does not repeat setting of the
    # default values.  But it does repeat the initial
    # (time zero) job.
    # Also note that any skipping of initial time is
    # done only on the first iteration of this loop.
    # When looping the first job of the new iteration
    # will start immediately.
    #**************************************************
    while True:
        StartTime = datetime.datetime.now()

        Jobs = RunList(StartTime, skipsecs)
        skipsecs = 0 # We only do the skip on the first cycle

        SetupJobs(Jobs, mm_hostname)

        # Come up to date with initial and skipped tasks.
        ShowMessage("Running initial and skipped tasks to establish state.")
        while Jobs.RunNextRequest(StartTime):
            pass # Yes, we want to pass, all the work is done in the RunNextRequest()

        ShowMessage("Beginning scheduled events")

        # Give ourselves a lifetime.
        # Add a few seconds to let the last job complete.
        #sleepsecs = (Jobs.LastRequestStartTime - datetime.datetime.now()).total_seconds() + 5
        sleepsecs = TotalSeconds(Jobs.LastRequestStartTime - datetime.datetime.now()) + 5

        # Begin the timer system with a one second interval
        rt = RepeatedTimer(1, jobtick)
        try:
            ShowMessage("Start running for %u seconds." % sleepsecs)
            time.sleep(sleepsecs)
        finally:
            rt.stop() # better in a try/finally block to make sure the program ends!

        if not do_loop:
            break

        ShowMessage("Beginning again")

    ShowMessage("Stopping - Finished")

#**********************************************************************
# Entry
#**********************************************************************
if __name__ == "__main__":

    try:
        main()
    except Exception, err:
        ShowMessage("Unhandled exception:", str(err))
        traceback.print_exc()

    except KeyboardInterrupt:
        ShowMessage("Keyboard Interruptception")

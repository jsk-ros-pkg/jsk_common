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
# $Id: setfilters26.py 464 2013-12-03 00:43:56Z karl $
#**********************************************************************

have_argparse = False
try:
    import argparse
    have_argparse = True
except:
    pass

import httplib
import json
import os
import sys
import urllib

#**********************************************************************
# GetAllFilterNames()
#**********************************************************************
def GetAllFilterNames(mm_hostname):
    mm2_config_in_json = GetMM2ConfigInJson(mm_hostname)
    jdata = json.loads(mm2_config_in_json)
    full_config = jdata[2]
    return set([str(filt['name']) for filt in full_config["filters"]])

#**********************************************************************
# GetMM2ConfigInJson()
#**********************************************************************
def GetMM2ConfigInJson(mm2name):
    conn = httplib.HTTPConnection(mm2name)
    conn.request("GET", "/mm2/mpc/mm2.py/get_config_json")
    resp = conn.getresponse()

    if (resp.status/100) != 2:
        raise Exception,resp.reason
    data = resp.read()
    conn.close()
    return data

#**********************************************************************
# class FiltSetting
#**********************************************************************
class FiltSetting(object):

    def __init__(self, filtname, toband):
        bnum = int(toband)
        if (bnum <= 0) or (bnum > 5):
            raise ValueError, "Band number for \"%s\" must be in range 1..5" % filtname
        self.FiltName = filtname
        self.ToBand = bnum

#**********************************************************************
# SetFiltMap()
#**********************************************************************
def SetFiltMap(mm2host, a2bfilt_vals, b2afilt_vals, all_filter_names=None):

    if all_filter_names is None:
        all_filter_names = GetAllFilterNames(mm2host)

    # Survive a None for either of the lists of filter settings
    if (a2bfilt_vals is None):
        a2bfilt_vals = []
    if (b2afilt_vals is None):
        b2afilt_vals = []

    # Make sure we do not have any stray filter names
    for fvals in (a2bfilt_vals, b2afilt_vals):
        for filt in fvals:
            if filt.FiltName not in all_filter_names:
                raise ValueError, "Filter name \"%s\" is not present" % filt.FiltName

    offa2b = all_filter_names.difference(set(map(lambda x: x.FiltName, a2bfilt_vals)))
    offb2a = all_filter_names.difference(set(map(lambda x: x.FiltName, b2afilt_vals)))

    fdict = {}
    for fn in offa2b:
        fdict["DBNDS_%s" % fn] = "down-band-none"
    for fn in offb2a:
        fdict["UBNDS_%s" % fn] = "up-band-none"

    for fs in a2bfilt_vals:
        fdict["DBNDS_%s" % fs.FiltName] = "down-band-%d" % fs.ToBand
    for fs in b2afilt_vals:
        fdict["UBNDS_%s" % fs.FiltName] = "up-band-%d" % fs.ToBand

    fdict["SUBMIT_FILTERMAP_BUTTON"] = "Submit"

    conn = httplib.HTTPConnection(mm2host)
    params = urllib.urlencode(fdict)
    headers = {"Content-type": "application/x-www-form-urlencoded",
               "Accept": "text/plain"}
    conn.request("POST", "/mm2/mpc/mm2.py/process_filtmap", params, headers)
    resp = conn.getresponse()

    if (resp.status/100) != 2:
        raise Exception,resp.reason
    data = resp.read()
    conn.close()

#**********************************************************************
# Entry
#**********************************************************************
if __name__ == "__main__":
    a2bfilts = []
    b2afilts = []

    parser = argparse.ArgumentParser(description="Set classification filter map.")

    parser.add_argument("-A", action="append", nargs=2, dest="a2b",
                        help="Specify a filter name and band number for packats flowing LanA to Lan B.")

    parser.add_argument("-B", action="append", nargs=2, dest="b2a",
                        help="Specify a filter name and band number for packats flowing LanB to Lan A.")

    parser.add_argument("-d", "--defaults", action="store_true",
                        help="Turn all filters off (default state).")

    parser.add_argument("-f", "--filters", action="store_true",
                        help="List all the flters, do nothing else.")

    parser.add_argument("mm_hostname",
			help='The host name or IP address of the Mini Maxwell/Maxwell G.')

    pargs = parser.parse_args()
    
    mm_hostname = pargs.mm_hostname
    a2bargs = pargs.a2b
    b2aargs = pargs.b2a

    if pargs.filters:
        try:
            all_filter_names = GetAllFilterNames(mm_hostname)
        except Exception, err:
            print "HTTP Exception occurred:", str(err)
            sys.exit(1)

        for filt in all_filter_names:
            print filt
        sys.exit(0)

    if pargs.defaults:
        SetFiltMap(mm_hostname, a2bfilts, b2afilts)
        sys.exit(0)

    if a2bargs is not None:
        for farg in a2bargs:
            try:
                a2bfilts.append(FiltSetting(farg[0], farg[1]))
            except Exception, err:
                print "Parameter error: %s - Aborting." % str(err)
                sys.exit(1)

    if b2aargs is not None:
        for farg in b2aargs:
            try:
                b2afilts.append(FiltSetting(farg[0], farg[1]))
            except Exception, err:
                print "Parameter error: %s - Aborting." % str(err)
                sys.exit(1)

    SetFiltMap(mm_hostname, a2bfilts, b2afilts)

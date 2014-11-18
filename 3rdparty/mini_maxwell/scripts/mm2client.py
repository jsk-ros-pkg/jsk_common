#!/bin/env python
#**********************************************************************
# Copyright (c) 2011 InterWorking Labs, All Rights Reserved           *
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
# $Id: mm2client.py 436 2013-08-20 22:50:01Z karl $
#**********************************************************************

# This file contains epydoc markup.

# Make sure that the Python version is new enough
import sys
import platform
pvt = platform.python_version_tuple()
if (int(pvt[0]) == 2) and (int(pvt[1]) < 6):
    print "This program requires Python 2.6 or later."
    sys.exit(1)
del pvt

# Standard Python libraries
import exceptions
import json
import httplib
import os
import types
import urllib

class MiniMaxClientException(exceptions.Exception):
    pass

#**********************************************************************
# GetMM2ConfigInJson()
#**********************************************************************
def GetMM2ConfigInJson(mm2name):
    """
    Read the configuration information from a named Mini Maxwell.
    Return a structure

    The parts of the structure are these:
       jdata[0] - Data Version
       jdata[1] - Timestamp
       jdata[2] - Full Configuration dictionary

    The parts of the full configuration dictionary are:
       full_config["general"]
       full_config["upstream_impairments"]
       full_config["downstream_impairments"]
       full_config["upstream-filters"]
       full_config["downstream-filters"]
       full_config["filters"]
    """
    conn = httplib.HTTPConnection(mm2name)
    conn.request("GET", "/mm2/mpc/mm2.py/get_config_json")
    resp = conn.getresponse()

    if (resp.status/100) != 2:
        raise Exception,resp.reason
    jdata = resp.read()
    conn.close()
    return jdata

#**********************************************************************
# GetMM2Config()
#**********************************************************************
def GetMM2Config(mm2name):
    return json.loads(GetMM2ConfigInJson(mm2name))
    
#**********************************************************************
# class BandSettings
#**********************************************************************
class BandSettings(object):
    ##MM_MAXRATE = 1000000000	# 1 gigabit, 1x10**9
    MM_MAXRATE = 100000000	# 100 megabits, 1x10**8

    MM_DELAY_DISTRIBUTIONS = ("none", "normal", "pareto", "paretonormal",
                              "experimental")

    PCNTFMT = "2.5f"

    SetProcs = {
                "delay_amount": "SetDelayAmount",
                "delay_variation": "SetDelayVariation",
                "delay_distribution": "SetDelayDistribution",
                "delay_correlation": "SetDelayCorrelation",
                "delay_reorder": "SetDelayReorder",
                "drop_amount": "SetDropAmount",
                "drop_correlation": "SetDropCorrelation",
                "duplication_amount": "SetDupAmount",
                "duplication_correlation": "SetDupCorrelation",
                "reorder_gap": "SetReorderGap",
                "reorder_correlation": "SetReorderCorrelation",
                "reorder_amount": "SetReorderAmount",
                "bitrate_limit": "SetRateLimit",
                "corruption_amount": "SetCorruptionAmount",
                "corruption_correlation": "SetCorruptionCorrelation"
                }

    def __init__(self, bandnumber, leftflag):
        """
        @param bandnumber: Band number, an integer in range 1..5
        @type bandnumber: int or long
        @param leftflag: True if LAN-A to LAN-B, false if LAN-B to LAN-A
        @type leftflag: bool
        """
        if (not isinstance(bandnumber, (types.IntType, types.LongType))) or \
           (bandnumber < 1) or (bandnumber > 5):
            raise MiniMaxClientException, "Band number not integer in range 1..5"

        if not isinstance(leftflag, types.BooleanType):
            raise MiniMaxClientException, "Left/right flag not a boolean"

        self.__BandNumber = bandnumber
        self.__LeftFlag = leftflag
        # Load default values
        self.SetDefaults()

    #**************************************************
    # Accessor methods
    #**************************************************
    @property
    def BandNumber(self):
        return self.__BandNumber

    @property
    def LeftFlag(self):
        return self.__LeftFlag

    @property
    def DelayAmount(self):
        return self.__DelayAmount

    @property
    def DelayVariation(self):
        return self.__DelayVariation

    @property
    def DelayCorrelation(self):
        return self.__DelayCorrelation

    @property
    def DelayDistribution(self):
        return self.__DelayDistribution

    @property
    def DelayReorder(self):
        return self.__DelayReorder

    @property
    def DropAmount(self):
        return self.__DropAmount

    @property
    def DropCorrelation(self):
        return self.__DropCorrelation

    @property
    def DupAmount(self):
        return self.__DupAmount

    @property
    def DupCorrelation(self):
        return self.__DupCorrelation

    @property
    def ReorderGap(self):
        return self.__ReorderGap

    @property
    def ReorderAmount(self):
        return self.__ReorderAmount

    @property
    def ReorderCorrelation(self):
        return self.__ReorderCorrelation

    @property
    def CorruptionAmount(self):
        return self.__CorruptionAmount

    @property
    def CorruptionCorrelation(self):
        return self.__CorruptionCorrelation

    @property
    def RateLimit(self):
        return self.__RateLimit

    #**************************************************
    # Utility methods
    #**************************************************
    def Validate(self):
        if (self.__DelayVariation < 0) or (self.__DelayVariation > self.__DelayAmount):
            raise MiniMaxClientException, "Delay Variation not in range 0 .. Delay Amount"
        return True

    #**************************************************
    # Mutator methods
    #**************************************************
    def SetDefaults(self):
        # Load default values
        self.__DelayAmount = 0
        self.__DelayVariation = 0
        self.__DelayCorrelation = 0
        self.__DelayDistribution = "none"
        self.__DelayReorder = False
        self.__DropAmount = 0
        self.__DropCorrelation = 0
        self.__DupAmount = 0
        self.__DupCorrelation = 0
        self.__ReorderGap = 0
        self.__ReorderAmount = 0
        self.__ReorderCorrelation = 0
        self.__CorruptionAmount = 0
        self.__CorruptionCorrelation = 0
        self.__RateLimit = self.__class__.MM_MAXRATE

    def SetDelayAmount(self, val):
        if (val < 0) or (val > 60000):
            raise MiniMaxClientException, "Delay Amount not in range 0 .. 60000"
    	self.__DelayAmount = val

    def SetDelayVariation(self, val):
        if (val < 0) or (val > 60000):
            raise MiniMaxClientException, "Delay Amount not in range 0 .. 60000"
        self.__DelayVariation = val

    def SetDelayCorrelation(self, val):
        if (val < 0.0) or (val > 100.0):
            raise MiniMaxClientException, "Delay Correlation not in range 0 .. 100"
        self.__DelayCorrelation = val

    def SetDelayDistribution(self, val):
        lcval = val.lower()
        if val not in self.__class__.MM_DELAY_DISTRIBUTIONS:
            raise MiniMaxClientException, "Delay distribution must be one of %s" % \
                str(self.__class__.MM_DELAY_DISTRIBUTIONS)
        self.__DelayDistribution = val

    def SetDelayReorder(self, val):
        self.__DelayReorder = bool(val)

    def SetDropAmount(self, val):
        if (val < 0.0) or (val > 100.0):
            raise MiniMaxClientException, "Drop Amount not in range 0 .. 100"
        self.__DropAmount = val

    def SetDropCorrelation(self, val):
        if (val < 0.0) or (val > 100.0):
            raise MiniMaxClientException, "Drop Correlation not in range 0 .. 100"
        self.__DropCorrelation = val

    def SetDupAmount(self, val):
        if (val < 0.0) or (val > 100.0):
            raise MiniMaxClientException, "Duplication Amount not in range 0 .. 100"
        self.__DupAmount = val

    def SetDupCorrelation(self, val):
        if (val < 0.0) or (val > 100.0):
            raise MiniMaxClientException, "Duplication Correlation not in range 0 .. 100"
        self.__DupCorrelation = val

    def SetReorderGap(self, val):
        if (val < 0) or (val > 1024):
            raise MiniMaxClientException, "Reorder Gap not in range 0 .. 1024"
        self.__ReorderGap = val

    def SetReorderAmount(self, val):
        if (val < 0.0) or (val > 100.0):
            raise MiniMaxClientException, "Reorder Amount not in range 0 .. 100"
        self.__ReorderAmount = val

    def SetReorderCorrelation(self, val):
        if (val < 0.0) or (val > 100.0):
            raise MiniMaxClientException, "Reorder Correlation not in range 0 .. 100"
        self.__ReorderCorrelation = val

    def SetCorruptionAmount(self, val):
        if (val < 0.0) or (val > 100.0):
            raise MiniMaxClientException, "Corruption Amount not in range 0 .. 100"
        self.__CorruptionAmount = val

    def SetCorruptionCorrelation(self, val):
        if (val < 0.0) or (val > 100.0):
            raise MiniMaxClientException, "Corruption Correlation not in range 0 .. 100"
        self.__CorruptionCorrelation = val

    def SetRateLimit(self, val):
        if (val < 128) or (val > self.__class__.MM_MAXRATE):
            raise MiniMaxClientException, "Rate Limit not in range 128 .. %u" % \
                self.__class__.MM_MAXRATE
        self.__RateLimit = val

    def SetByCnfigName(self, name, val):
        """
        Set an item using the name used in the Mini Maxwell configuration file.
        """
        try:
            (self.__class__.__dict__[self.__class__.SetProcs[name]])(self, val)
        except:
            pass

    def AsDict(self):
        if self.__LeftFlag:
            snum = (2 * self.__BandNumber) - 1
        else:
            snum = 2 * self.__BandNumber
        snumstr = str(snum)
        tdict = { \
            	"DELAY_AMOUNT_" + snumstr: str(self.__DelayAmount),
                "DELAY_VAR_" + snumstr: str(self.__DelayVariation),
                "DELAY_CORR_" + snumstr: format(self.__DelayCorrelation, self.__class__.PCNTFMT),
                "DELAY_CURVE_" + snumstr: self.__DelayDistribution.lower(),
                "GAP_" + snumstr: self.__ReorderGap,
                "REORDER_AMOUNT_" + snumstr: format(self.__ReorderAmount, self.__class__.PCNTFMT),
                "REORDER_CORR_" + snumstr: format(self.__ReorderCorrelation, self.__class__.PCNTFMT),
                "DROP_AMOUNT_" + snumstr: format(self.__DropAmount, self.__class__.PCNTFMT),
                "DROP_CORR_" + snumstr: format(self.__DropCorrelation, self.__class__.PCNTFMT),
                "DUP_AMOUNT_" + snumstr: format(self.__DupAmount, self.__class__.PCNTFMT),
                "DUP_CORR_" + snumstr: format(self.__DupCorrelation, self.__class__.PCNTFMT),
                "CORRUPTION_AMOUNT_" + snumstr: format(self.__CorruptionAmount, self.__class__.PCNTFMT),
                "CORRUPTION_CORR_" + snumstr: format(self.__CorruptionCorrelation, self.__class__.PCNTFMT),
                "RATE_LIMIT_" + snumstr: str(self.__RateLimit)
                }
        # We only want to send checkboxes when they are checked.
        if self.__DelayReorder:
            tdict["DELAY_REORDER_" + snumstr] = str(self.__DelayReorder)
        return tdict

#**********************************************************************
# class Bands
#**********************************************************************
class Bands(object):
    def __init__(self):
        self.__LeftBands = (
            BandSettings(1, True), BandSettings(2, True), BandSettings(3, True),
            BandSettings(4, True), BandSettings(5, True))
        
        self.__RightBands = (
            BandSettings(1, False), BandSettings(2, False), BandSettings(3, False),
            BandSettings(4, False), BandSettings(5, False))

    def GetLeftBand(self, bnum):
        if (bnum < 1) or (bnum > 5):
            raise MiniMaxClientException, "Band number must be in in range 1..5"
        return self.__LeftBands[bnum-1]

    def GetRightBand(self, bnum):
        if (bnum < 1) or (bnum > 5):
            raise MiniMaxClientException, "Band number must be in in range 1..5"
        return self.__RightBands[bnum-1]

    def GetBand(self, bnum, leftflag):
        if (bnum < 1) or (bnum > 5):
            raise MiniMaxClientException, "Band number must be in in range 1..5"
        if leftflag:
            return self.__LeftBands[bnum-1]
        return self.__RightBands[bnum-1]

    def Validate(self):
        for bnd in self.__LeftBands:
            if not bnd.Validate():
                return False
        for bnd in self.__RightBands:
            if not bnd.Validate():
                return False
        return True

    def SetDefaults(self):
        for bnd in self.__LeftBands:
            bnd.SetDefaults()
        for bnd in self.__RightBands:
            bnd.SetDefaults()

    def SetBandToDefaults(self, bnum, leftflag):
        self.GetBand(bnum, leftflag).SetDefaults()

    def SetDelayAmount(self, bnum, leftflag, val):
        self.GetBand(bnum, leftflag).SetDelayAmount(val)

    def SetDelayVariation(self, bnum, leftflag, val):
        self.GetBand(bnum, leftflag).SetDelayVariation(val)

    def SetDelayCorrelation(self, bnum, leftflag, val):
        self.GetBand(bnum, leftflag).SetDelayCorrelation(val)

    def SetDelayDistribution(self, bnum, leftflag, val):
        self.GetBand(bnum, leftflag).SetDelayDistribution(val)

    def SetDelayReorder(self, bnum, leftflag, val):
        self.GetBand(bnum, leftflag).SetDelayReorder(val)

    def SetDropAmount(self, bnum, leftflag, val):
        self.GetBand(bnum, leftflag).SetDropAmount(val)

    def SetDropCorrelation(self, bnum, leftflag, val):
        self.GetBand(bnum, leftflag).SetDropCorrelation(val)

    def SetDupAmount(self, bnum, leftflag, val):
        self.GetBand(bnum, leftflag).SetDupAmount(val)

    def SetDupCorrelation(self, bnum, leftflag, val):
        self.GetBand(bnum, leftflag).SetDupCorrelation(val)

    def SetReorderGap(self, bnum, leftflag, val):
        self.GetBand(bnum, leftflag).SetReorderGap(val)

    def SetReorderAmount(self, bnum, leftflag, val):
        self.GetBand(bnum, leftflag).SetReorderAmount(val)

    def SetReorderCorrelation(self, bnum, leftflag, val):
        self.GetBand(bnum, leftflag).SetReorderCorrelation(val)

    def SetCorruptionAmount(self, bnum, leftflag, val):
        self.GetBand(bnum, leftflag).SetCorruptionAmount(val)

    def SetCorruptionCorrelation(self, bnum, leftflag, val):
        self.GetBand(bnum, leftflag).SetCorruptionCorrelation(val)

    def SetRateLimit(self, bnum, leftflag, val):
        self.GetBand(bnum, leftflag).SetRateLimit(val)

    def AsDict(self):
        rdict = {}
        for ndx in range(5):
            rdict.update(self.__LeftBands[ndx].AsDict())
            rdict.update(self.__RightBands[ndx].AsDict())
        return rdict

#**********************************************************************
# GetCurrentBands()
#**********************************************************************
def GetCurrentBands(mm2name):
    """
    Construct a Bands object using the current configuration on
    a named Mini Maxwell
    """
    bnds = Bands()

    current = GetMM2Config(mm2name)
    full_config = current[2]
    # Left column (LAN-A to LAN-B)
    downstream_impairs = full_config["downstream_impairments"]
    # Right column (LAN-B to LAN-A)
    upstream_impairs = full_config["upstream_impairments"]
    
    bnum = 1
    for itm in downstream_impairs:
        bnd = bnds.GetLeftBand(bnum)
        for imp in itm.items():
            bnd.SetByCnfigName(imp[0], imp[1])
        bnum += 1

    bnum = 1
    for itm in upstream_impairs:
        bnd = bnds.GetRightBand(bnum)
        for imp in itm.items():
            bnd.SetByCnfigName(imp[0], imp[1])
        bnum += 1

    return bnds

#**********************************************************************
# ChangeBandsOnMM()
#**********************************************************************
def ChangeBandsOnMM(bandsobj, mm2host):
    """
    @param bandsobj: A Bands object containing the desired impairment settings.
    @type bandsobj: Bands
    @param mm2host: The hostname or IP address of the Mini Maxwell.
    @type mm2host: string
    """

    bandsobj.Validate()

    conn = httplib.HTTPConnection(mm2host)
    bdict = bandsobj.AsDict()
    bdict["SUBMIT_FLOWIMPAIR_BUTTON"] = "Submit"
    params = urllib.urlencode(bdict)
    headers = {"Content-type": "application/x-www-form-urlencoded",
               "Accept": "text/plain"}
    conn.request("POST", "/mm2/mpc/mm2.py/process_flows", params, headers)
    resp = conn.getresponse()

    if (resp.status/100) != 2:
        raise Exception,resp.reason
    data = resp.read()
    conn.close()
    return

#**********************************************************************
# Entry point for command line invocation
#**********************************************************************
if __name__ == "__main__":

    if len(sys.argv) != 2:
        print "Usage: %s <minimaxwell-hostname-or-ip-address>" % sys.argv[0]
        print "Aborting"
        sys.exit(1)

    mm2name = sys.argv[1]

    #************************************************************
    # Update the existing impairments.
    # if the Mini Maxwell is a Revision 12 unit than only the
    # values set below will be altered in the Mini Maxwell
    # For revision 11 units values not set below will be set
    # to their default values.
    #
    # WARNING - Reading the existing configuration via JSON
    # is only on Mini Maxwell Revisions 12 or higher.
    #
    # Most Revision 11 Mini Maxwells may be field upgraded to
    # Revision 12.
    #
    # For new Mini Maxwells the revision is 12.
    # To learn the revision number, look on the home page of
    # the Mini Maxwell.  On the top line there is a number
    # that looks something like this: "v2.0/12", "v1.0/11",
    # "v2.0/11" or "v1.0/11".  The important number is the one
    # after the slash ('/') character.  That number is the
    # revision number.
    #************************************************************

    # CHANGE THE FOLLOWING AS APPRORIATE FOR YOUR UNIT
    MM_REV = 12

    if MM_REV >= 12:
        # Read the current settings from the Mini Maxwell
        try:
            bnds = GetCurrentBands(mm2name)
        except:
            bnds = Bands()

    else: # MM_REV < 12
    	#************************************************************
        # For Rev 11 Mini Maxwells:
        # The following code can be used to establish a new set of
        # impairment settings.
        # Values not set below will be set to the default value.
    	#************************************************************
    	bnds = Bands()


    #**************************************************
    # locally modify those settings
    # (This is local -- the Mini Maxwell is not affected)
    #
    # What happens to values not set below depends on the
    # Mini Maxwell revision as described above.
    #**************************************************

    # Use bnds.SetDefaults() to clear all settings on all bands to the default values.
    # Use bnds.SetBandToDefaults(band_number, left_right_flag) to clear a given
    #  band (left or right side) to its default values.
    # Alternatively one can create a new Bands object which will contain the
    # default settings by saying:
    #  bnds = Bands()

    # In the methods below these are the parameters:
    #       Band number (1..5)
    #       Left column (LAN-A to LAN-B): True, else False
    #       The value to be set
    #  bnds.SetDelayAmount(band_number, left_right_flag, value)
    #  bnds.SetDelayVariation(band_number, left_right_flag, value)
    #  bnds.SetDelayCorrelation(band_number, left_right_flag, value)
    #  bnds.SetDelayDistribution(band_number, left_right_flag, value)
    #  bnds.SetDelayReorder(band_number, left_right_flag, booleanvalue)
    #  bnds.SetDropAmount(band_number, left_right_flag, value)
    #  bnds.SetDropCorrelation(band_number, left_right_flag, value)
    #  bnds.SetDupAmount(band_number, left_right_flag, value)
    #  bnds.SetDupCorrelation(band_number, left_right_flag, value)
    #  bnds.SetReorderGap(band_number, left_right_flag, value)
    #  bnds.SetReorderAmount(band_number, left_right_flag, value)
    #  bnds.SetReorderCorrelation(band_number, left_right_flag, value)
    #  bnds.SetCorruptionAmount(band_number, left_right_flag, value)
    #  bnds.SetCorruptionCorrelation(band_number, left_right_flag, value)
    #  bnds.SetRateLimit(band_number, left_right_flag, value)

    #bnds.SetDelayAmount(1, True, 119)
    #bnds.SetCorruptionAmount(5, True, 89)
    #bnds.SetCorruptionAmount(5, False, 56)
    #bnds.SetDelayDistribution(5, True, "paretonormal")
    #bnds.SetDelayDistribution(5, False, "pareto")
    #bnds.SetDelayDistribution(5, True, "normal")
    #bnds.SetDelayDistribution(5, False, "none")
        
    bnds.SetDelayAmount(5, True, 100)
    bnds.SetDelayVariation(5, True, 10)
    bnds.SetDelayReorder(5, True, True)
    bnds.SetDelayReorder(5, False, False)

    #**************************************************
    # Inject the updated settings into the Mini Maxwell
    #**************************************************
    ChangeBandsOnMM(bnds, mm2name)

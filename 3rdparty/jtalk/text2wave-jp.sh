#!/bin/bash 

INPUT_FILE=$1;shift;
OUTPUT_FILE=/tmp/_jtalk_sound_$$.wav
while getopts o: OPT
do
    case ${OPT} in
	"o")
	    OUTPUT_FILE=${OPTARG};shift;shift;;
	esac
done

VOICE=`rospack find jtalk`/hts_voice_nitech_jp_atr503_m001-1.00
DICTIONARY=`rospack find jtalk`/open_jtalk_dic_utf_8-1.00

PATH=`rospack find jtalk`/bin:$PATH
LD_LIBRARY_PATH=`rospack find jtalk`/lib:$LD_LIBRARY_PATH
`rospack find jtalk`/bin/open_jtalk -td $VOICE/tree-dur.inf -tf $VOICE/tree-lf0.inf -tm $VOICE/tree-mgc.inf -md $VOICE/dur.pdf -mf $VOICE/lf0.pdf -mm $VOICE/mgc.pdf -df $VOICE/lf0.win1 -df $VOICE/lf0.win2 -df $VOICE/lf0.win3 -dm $VOICE/mgc.win1 -dm $VOICE/mgc.win2 -dm $VOICE/mgc.win3 -ef $VOICE/tree-gv-lf0.inf -em $VOICE/tree-gv-mgc.inf -cf $VOICE/gv-lf0.pdf -cm $VOICE/gv-mgc.pdf -k  $VOICE/gv-switch.inf -x  $DICTIONARY -ow $OUTPUT_FILE -ot /tmp/_jtalk_log_$$.txt $INPUT_FILE
#rm -f /tmp/_jtalk_{log,input}_$$.*


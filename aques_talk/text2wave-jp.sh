#!/bin/bash

INPUT_FILE=$1;shift;
OUTPUT_FILE=/tmp/_aques_talk_sound_$$.wav
JPTEXT_FILE=/tmp/_aques_talk_text_$$.txt
PHONT_FILE=aq_f1b.phont
while getopts p:o: OPT
do
    case ${OPT} in
	"o")
	    OUTPUT_FILE=${OPTARG};;
	"p")
	    PHONT_FILE=${OPTARG};;
	esac
done

PHONT_FILE=`rospack find aques_talk`/phont/$PHONT_FILE
PATH=`rospack find aques_talk`/bin:$PATH
nkf -j $INPUT_FILE | kakasi -JH | nkf -w | \
    sed -e 's/，/、/g' | sed -e 's/,/、/g' | \
    sed -e 's/．/。/g' | sed -e 's/\./。/g' > \
    $JPTEXT_FILE
cat $JPTEXT_FILE 1>&2
`rospack find aques_talk`/bin/SampleTalk -p $PHONT_FILE -o $OUTPUT_FILE $JPTEXT_FILE
#rm -f /tmp/_jtalk_{log,input}_$$.*


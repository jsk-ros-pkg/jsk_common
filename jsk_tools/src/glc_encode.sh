#!/bin/bash

usage_exit() {
    echo "Usage : $0 <glc_file> [--gtest_output file] [-o|--output OUTPUT] [--ctx NUM]" 1>&2
    exit 1
}
trap usage_exit ERR

GLC_FILENAME=$1
BASE_NAME=`basename $GLC_FILENAME .glc`
BASE_DIR=`dirname $1`
MEDIA_DIR=/tmp

GETOPT=`getopt -o o: -l output:,text,ctx:,gtest_output: -- "$@"` ; [ $? != 0 ] && usage_exit
eval set -- "$GETOPT"
while true
do
  case $1 in
  --gtest_output)  OUTFILE=$2      ; shift 2 ;;
  -o|--output)     BASE_DIR=`dirname $2`; BASE_NAME=`basename $2`    ; shift 2 ;;
  -h)   usage_exit ;;
  --)   shift ; break ;;
  *)   shift ; break ;;
  esac
done

echo "[glc_encode] glc file name : $GLC_FILENAME"
echo "[glc_encode] file base name: $BASE_NAME"
echo "[glc_encode] file base dir : $BASE_DIR"
echo "[glc_encode] gtest output  : $OUTFILE"

OUTFILE=${OUTFILE/xml:/}
if [ "${OUTFILE}" ]; then
    cat<<EOF > ${OUTFILE}
<testsuite errors="0" failures="0" name="unittest.suite.TestSuite" tests="1" time="0.0">
  <testcase classname="__main__.TestGlcEncode" name="test_glc_encode" time="0.0"></testcase>
  <system-out><![CDATA[]]></system-out>
  <system-err><![CDATA[]]></system-err>
</testsuite>
EOF
fi

cp -rf ${GLC_FILENAME} ${MEDIA_DIR}/${BASE_NAME}.glc
VIDEO_STREAM=`glc-play -i 1 ${MEDIA_DIR}/${BASE_NAME}.glc | grep \ video\ stream | sed -e "s/\[\ *\([0-9.]*\)s.*$/\1/"`
#
STARTTIME=1.0
STREAMSIZE=0
for video_stream in $VIDEO_STREAM; do
    echo "[glc_encode] video-stream  : $video_stream"
    if [ $(echo "$video_stream > $STARTTIME" | bc) -eq 1 ] ; then STARTTIME=$video_stream; fi
    STREAMSIZE=`expr ${STREAMSIZE} + 1`
done
echo "[glc_encode] start time    : $STARTTIME"

CTX=1
for video_stream in $VIDEO_STREAM; do
    if [ ${STREAMSIZE} -eq 1 ]; then
	TMP_FILENAME=${MEDIA_DIR}/${BASE_NAME}.mpg
	MP4_FILENAME=${BASE_DIR}/${BASE_NAME}.mp4
	OGV_FILENAME=${BASE_DIR}/${BASE_NAME}.ogv
	WEBM_FILENAME=${BASE_DIR}/${BASE_NAME}.webm
	PNG_FILENAME=${BASE_DIR}/${BASE_NAME}.png
    else
	TMP_FILENAME=${MEDIA_DIR}/${BASE_NAME}-${CTX}.mpg
	MP4_FILENAME=${BASE_DIR}/${BASE_NAME}-${CTX}.mp4
	OGV_FILENAME=${BASE_DIR}/${BASE_NAME}-${CTX}.ogv
	WEBM_FILENAME=${BASE_DIR}/${BASE_NAME}-${CTX}.webm
	PNG_FILENAME=${BASE_DIR}/${BASE_NAME}-${CTX}.png
    fi
    echo "[glc_encode] mp4 file name : $MP4_FILENAME"
    echo "[glc_encode] ogv file name : $OGV_FILENAME"
    echo "[glc_encode] png file name : $PNG_FILENAME"
    #
    echo "[glc_encode] convert  from ${BASE_NAME}.glc to ${MP4_FILENAME}"
    glc-play ${MEDIA_DIR}/${BASE_NAME}.glc  -y ${CTX} -o - | ffmpeg -itsoffset  -$STARTTIME -i - -sameq -y ${MP4_FILENAME} > /dev/null
    #
    echo "[glc_encode] convert  from ${MP4_FILENAME} to ${OGV_FILENAME}"
    ffmpeg2theora ${MP4_FILENAME} --out ${OGV_FILENAME} > /dev/null
    #
    CAPTUREPOINT=`ffmpeg -i ${MP4_FILENAME} 2>&1 | grep Duration | cut -d " " -f 4 | sed  s/,// | awk 'BEGIN{FS=":"}{print ($1*60*60+$2*60+$3)/2}'`
    rm -f ${PNG_FILENAME}
    echo "[glc_encode] convert  from ${MP4_FILENAME} to ${PNG_FILENAME} at ${CAPTUREPOINT}"
    ffmpeg -y -i ${MP4_FILENAME} -vframes 1 -ss ${CAPTUREPOINT} ${PNG_FILENAME}

    CTX=`expr ${CTX} + 1`
done



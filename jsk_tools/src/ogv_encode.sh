#!/bin/bash

gtestout_begin() {
    local outfile=$1
cat<<EOF > ${outfile}
<testsuite errors="0" failures="0" name="unittest.suite.TestSuite" tests="1" time="0.0">
  <testcase classname="__main__.TestGlcEncode" name="test_glc_encode" time="0.0"></testcase>
  <system-out><![CDATA[]]></system-out>
  <system-err><![CDATA[
EOF
}
gtestout_end() {
    local outfile=$1
cat<<EOF >> ${outfile}
]]></system-err>
</testsuite>
EOF
}

usage_exit() {
    echo "Usage : $0 <ogv_file> [--gtest_output file] [-o|--output OUTPUT]" 1>&2
    exit 1
}
trap usage_exit ERR

OGV_FILENAME=$1
BASE_NAME=`basename $OGV_FILENAME .ogv`
BASE_DIR=`dirname $1`
MEDIA_DIR=/tmp

GETOPT=`getopt -o o: -l output:,text,gtest_output: -- "$@"` ; [ $? != 0 ] && usage_exit
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


echo "[glc_encode] ogv file name : $OGV_FILENAME"
echo "[glc_encode] file base name: $BASE_NAME"
echo "[glc_encode] file base dir : $BASE_DIR"

# write gtest
OUTFILE=${OUTFILE/xml:/}
if [ "${OUTFILE}" ]; then
    gtestout_begin ${OUTFILE}
fi

# setup variables
MP4_FILENAME=${BASE_DIR}/${BASE_NAME}.mp4
WEBM_FILENAME=${BASE_DIR}/${BASE_NAME}.webm
PNG_FILENAME=${BASE_DIR}/${BASE_NAME}.png

echo "[glc_encode] mp4 file name : $MP4_FILENAME" 2>&1 | tee -a ${OUTFILE}
echo "[glc_encode] png file name : $PNG_FILENAME" 2>&1 | tee -a ${OUTFILE}

# convert from ogv to mp4
echo "[glc_encode] convert  from ${OGV_FILENAME} to ${MP4_FILENAME}" 2>&1 | tee -a ${OUTFILE}
arista-transcode ${OGV_FILENAME} -o ${MP4_FILENAME} 2>&1 | tee -a ${OUTFILE}

# convert from mp4 to png
CAPTUREPOINT=`ffmpeg -i ${MP4_FILENAME} 2>&1 | grep Duration | cut -d " " -f 4 | sed  s/,// | awk 'BEGIN{FS=":"}{print ($1*60*60+$2*60+$3)/2}'`
rm -f ${PNG_FILENAME}
echo "[glc_encode] convert  from ${MP4_FILENAME} to ${PNG_FILENAME} at ${CAPTUREPOINT}" 2>&1 | tee -a ${OUTFILE}
ffmpeg -y -i ${MP4_FILENAME} -vframes 1 -ss ${CAPTUREPOINT} ${PNG_FILENAME} 2>&1 | tee -a ${OUTFILE}

# check size
identify ${PNG_FILENAME} 2>&1 | tee -a ${OUTFILE}
BPP=`identify -format "%[fx:10000000*b/(w*h)]" ${PNG_FILENAME}`
echo "[glc_encode] (image file size)/(image pixel size) is $BPP (fail if < 1)" 2>&1 | tee -a ${OUTFILE}
if [ "`echo \"$BPP < 1\" | bc`" == 1 ] ; then exit 1; fi

if [ "${OUTFILE}" ]; then
    gtestout_end ${OUTFILE}
fi
exit 0

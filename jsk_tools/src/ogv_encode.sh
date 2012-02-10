#!/bin/bash

usage_exit() {
    echo "Usage : $0 <ogv_file> [--gtest_output file] [-o|--output OUTPUT]" 1>&2
    exit 1
}
OGV_FILENAME=$1
BASE_NAME=`basename $OGV_FILENAME .ogv`
BASE_DIR=`dirname $1`
MEDIA_DIR=/tmp
GIF_RATE=1
GIF_DELAY=10
CTXNUM=1

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

MP4_FILENAME=${BASE_NAME}.mp4
GIF_FILENAME=${BASE_NAME}.gif

echo "[ogl_encode] ogv file name : $OGV_FILENAME"
echo "[ogl_encode] mp4 file name : $MP4_FILENAME"
echo "[ogl_encode] gif file name : $GIF_FILENAME"
echo "[ogl_encode] file base name: $BASE_NAME"
echo "[ogl_encode] file base dir : $BASE_DIR"

OUTFILE=${OUTFILE/xml:/}
cat<<EOF > ${OUTFILE}
<testsuite errors="0" failures="0" name="unittest.suite.TestSuite" tests="1" time="0.0">
  <testcase classname="__main__.TestOglEncode" name="test_ogl_encode" time="0.0"></testcase>
  <system-out><![CDATA[]]></system-out>
  <system-err><![CDATA[]]></system-err>
</testsuite>
EOF

#
rm -rf ${MEDIA_DIR}/${BASE_NAME}_ogv_*.png ${MEDIA_DIR}/${BASE_NAME}_gifogv_*.gif

cp -rf ${OGV_FILENAME} ${MEDIA_DIR}/${BASE_NAME}.ogv
arista-transcode ${MEDIA_DIR}/${BASE_NAME}.ogv -o ${BASE_DIR}/${MP4_FILENAME}
ffmpeg -i ${BASE_DIR}/${MP4_FILENAME} -s vga -r $GIF_RATE ${MEDIA_DIR}/${BASE_NAME}_ogv_%03d.png

# make gif files for animation
# get list of ogv*.png files for animation
giflength=`ls -al ${MEDIA_DIR}/${BASE_NAME}_ogv_*.png | wc -l`
gifcount=0
currimg="${MEDIA_DIR}/${BASE_NAME}_ogv_001.png"
nextcount=2
nextimg=`printf /tmp/${BASE_NAME}_ogv_%03d.png $nextcount`
skipcount=0
while [ -f $nextimg ]; do
    SIMILAR=`compare -metric mse $currimg $nextimg null: 2>&1`
    COMPARE_RET=$?
    echo "compare $currimg $nextimg $COMPARE_RET/$SIMILAR (skip:$skipcount, gif:$gifcount, len: $giflength)" 1>&2
    if [ $COMPARE_RET -eq 1 ]; then
	SIMILAR=10000
    else
	SIMILAR=`echo $SIMILAR | cut -d ' ' -f1 | cut -d '.' -f1`
    fi
    if [ $SIMILAR -gt 300 ]; then ## currimg and nextimg is too dissimilar
	skipcount=`expr $skipcount + 1`
	currimg=$nextimg
    fi
    if [ $skipcount -ge 2 ]; then ## skip first 2 images
    	gifimg=`printf ${MEDIA_DIR}/${BASE_NAME}_gifogv_%03d.gif $gifcount`
    	convert $nextimg $gifimg
    	gifcount=`expr $gifcount + 1`
	currimg=$nextimg
    fi
    nextcount=`expr $nextcount + 1`
    nextimg=`printf ${MEDIA_DIR}/${BASE_NAME}_ogv_%03d.png $nextcount`
done
if [ ! -f ${MEDIA_DIR}/${BASE_NAME}_gifogv_010.gif ]; then
    nextcount=3
    nextimg=`printf ${MEDIA_DIR}/${BASE_NAME}_ogv_%03d.png $nextcount`
    while [ -f $nextimg ]; do
    	gifimg=`printf ${MEDIA_DIR}/${BASE_NAME}_gifogv_%03d.gif $nextcount`
	echo  "[ogv_encode] use $gifimg"
    	convert $nextimg $gifimg
    	nextcount=`expr $nextcount + 1`
	nextimg=`printf ${MEDIA_DIR}/${BASE_NAME}_ogv_%03d.png $nextcount`
    done
fi
echo "[ogv_encode] write gif file to ${BASE_DIR}/${GIF_FILENAME}"
gifsicle --colors 256 --loop --delay ${GIF_DELAY} ${MEDIA_DIR}/${BASE_NAME}_gifogv_*.gif -o ${BASE_DIR}/${GIF_FILENAME}

rm -rf $MEDIA_DIR/ogv_${BASE_NAME}_*.png $MEDIA_DIR/gifogv_${BASE_NAME}_*.gif


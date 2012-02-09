#!/bin/bash

usage_exit() {
    echo "Usage : $0 <glc_file> [--gtest_output file] [-o|--output OUTPUT] [--ctx NUM]" 1>&2
    exit 1
}
GLC_FILENAME=$1
BASE_NAME=`basename $GLC_FILENAME .glc`
BASE_DIR=`dirname $1`
MEDIA_DIR=/tmp
GIF_RATE=3
GIF_DELAY=10
CTXNUM=1

GETOPT=`getopt -o o: -l output:,text,ctx:,gtest_output: -- "$@"` ; [ $? != 0 ] && usage_exit
eval set -- "$GETOPT"
while true
do
  case $1 in
  --gtest_output)  OUTFILE=$2      ; shift 2 ;;
  -o|--output)     BASE_DIR=`dirname $2`; BASE_NAME=`basename $2`    ; shift 2 ;;
  --ctx)           CTXNUM=$2       ; shift 2 ;;
  -h)   usage_exit ;;
  --)   shift ; break ;;
  *)   shift ; break ;;
  esac
done

MP4_FILENAME=${BASE_NAME}.mp4
GIF_FILENAME=${BASE_NAME}.gif

echo "[glc_encode] glc file name : $GLC_FILENAME"
echo "[glc_encode] mp4 file name : $MP4_FILENAME"
echo "[glc_encode] gif file name : $GIF_FILENAME"
echo "[glc_encode] file base name: $BASE_NAME"
echo "[glc_encode] file base dir : $BASE_DIR"

OUTFILE=${OUTFILE/xml:/}
cat<<EOF > ${OUTFILE}
<testsuite errors="0" failures="0" name="unittest.suite.TestSuite" tests="1" time="0.0">
  <testcase classname="__main__.TestGlcEncode" name="test_glc_encode" time="0.0"></testcase>
  <system-out><![CDATA[]]></system-out>
  <system-err><![CDATA[]]></system-err>
</testsuite>
EOF

# if ctx is 0 -> generate each ctx
if [ $CTXNUM -eq 0 ] ; then
  cp -rf ${GLC_FILENAME} ${MEDIA_DIR}/${BASE_NAME}.glc
  CTX=`glc-play ${MEDIA_DIR}/${BASE_NAME}.glc -i 2 | grep "stream id" | wc -l`
  for ID in $(seq $CTX)
  do
    $0 ${MEDIA_DIR}/${BASE_NAME}.glc --ctx $ID -o ${BASE_DIR}/${BASE_NAME}-${ID}
  done
  exit
fi

#
rm -rf ${MEDIA_DIR}/${BASE_NAME}_glc_*.png ${MEDIA_DIR}/${BASE_NAME}_gifglc_*.gif

glc-play ${GLC_FILENAME} -o - -y $CTXNUM | ffmpeg -i - -sameq -y ${BASE_DIR}/${MP4_FILENAME}
ffmpeg -i ${BASE_DIR}/${MP4_FILENAME} -r $GIF_RATE ${MEDIA_DIR}/${BASE_NAME}_glc_%03d.png

# make gif files for animation
# get list of glc*.png files for animation
gifcount=0
currimg="${MEDIA_DIR}/${BASE_NAME}_glc_001.png"
nextcount=2
nextimg=`printf /tmp/${BASE_NAME}_glc_%03d.png $nextcount`
skipcount=0
while [ -f $nextimg ]; do
    SIMILAR=`compare -metric mse $currimg $nextimg null: 2>&1`
    COMPARE_RET=$?
    echo "compare $currimg $nextimg $COMPARE_RET/$SIMILAR (skip:$skipcount, gif:$gifcount)" 1>&2
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
    	gifimg=`printf ${MEDIA_DIR}/${BASE_NAME}_gifglc_%03d.gif $gifcount`
    	convert $nextimg $gifimg
    	gifcount=`expr $gifcount + 1`
	currimg=$nextimg
    fi
    nextcount=`expr $nextcount + 1`
    nextimg=`printf ${MEDIA_DIR}/${BASE_NAME}_glc_%03d.png $nextcount`
done
if [ ! -f ${MEDIA_DIR}/${BASE_NAME}_gifglc_010.gif ]; then
    nextcount=3
    nextimg=`printf ${MEDIA_DIR}/${BASE_NAME}_glc_%03d.png $nextcount`
    while [ -f $nextimg ]; do
    	gifimg=`printf ${MEDIA_DIR}/${BASE_NAME}_gifglc_%03d.gif $nextcount`
	echo  "[glc_encode] use $gifimg"
    	convert $nextimg $gifimg
    	nextcount=`expr $nextcount + 1`
	nextimg=`printf ${MEDIA_DIR}/${BASE_NAME}_glc_%03d.png $nextcount`
    done
fi
echo "[glc_encode] write gif file to ${BASE_DIR}/${GIF_FILENAME}"
gifsicle --colors 256 --loop --delay ${GIF_DELAY} ${MEDIA_DIR}/${BASE_NAME}_gifglc_*.gif -o ${BASE_DIR}/${GIF_FILENAME}

rm -rf $MEDIA_DIR/glc_${BASE_NAME}_*.png $MEDIA_DIR/gifglc_${BASE_NAME}_*.gif


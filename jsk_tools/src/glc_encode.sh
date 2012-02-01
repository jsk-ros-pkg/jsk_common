#!/bin/bash

usage_exit() {
    echo "Usage : $0 <glc_file> [--gtest_output file] [-o|--output OUTPUT] [--ctx NUM]" 1>&2
    exit 1
}
GLC_FILENAME=$1
MP4_FILENAME=${GLC_FILENAME%.glc}.mp4
CTXNUM=1

GETOPT=`getopt -o o: -l output:,ctx:,gtest_output: -- "$@"` ; [ $? != 0 ] && usage_exit
eval set -- "$GETOPT"
while true
do
  case $1 in
  --gtest_output)     OUTFILE=$2      ; shift 2 ;;
  -o|--output)        MP4_FILENAME=$2 ; shift 2 ;;
  --ctx)              CTXNUM=$2       ; shift 2 ;;
  -h)   usage_exit ;;
  --)   shift ; break ;;
  *)   shift ; break ;;
  esac
done

OUTFILE=${OUTFILE/xml:/}
cat<<EOF $OUTFILE
<testsuite errors="0" failures="0" name="unittest.suite.TestSuite" tests="1" time="0.0">
  <testcase classname="__main__.TestGlcEncode" name="test_glc_encode" time="0.0"></testcase>
  <system-out><![CDATA[]]></system-out>
  <system-err><![CDATA[]]></system-err>
</testsuite>
EOF

#
echo "glc-play $GLC_FILENAME -o - -y $CTXNUM | ffmpeg -i - -sameq -y $MP4_FILENAME"
glc-play $GLC_FILENAME -o - -y $CTXNUM | ffmpeg -i - -sameq -y $MP4_FILENAME


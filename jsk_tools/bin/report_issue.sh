#!/bin/bash

ALL_PACKAGES=`rospack list | cut -f1 -d' ' | sort`
OUTPUT_FILE=$(mktemp)
output_package() {
    pkg=$1
    pkg_dir=$(rospack find $pkg)
    if [[ $pkg_dir == /opt/ros* ]]; then
        rosdep resolve $pkg >/dev/null
        if [ $? = 0 ] ; then
            version=$(apt-show-versions $(rosdep resolve $pkg | tail -n 1) | sed -e 's%[a-zA-Z][-\s/]*%%g' | awk '{ printf($0) }')
            echo $pkg "| apt | ${version}" >> ${OUTPUT_FILE}
        else
            echo $pkg "| apt | unknown"  >> ${OUTPUT_FILE}
        fi
    else
        version=$(cd $pkg_dir; git rev-parse HEAD)
        if [ $? = 0 ] ; then
            echo $pkg "| source | ${version}"  >> ${OUTPUT_FILE}
        else
            echo $pkg "| source | not-git"  >> ${OUTPUT_FILE}
        fi
    fi
}

echo \`\`\` >> ${OUTPUT_FILE}

echo "$(wstool info)" >> ${OUTPUT_FILE}
echo \`\`\` >> ${OUTPUT_FILE}

echo "Package | Type | version"  >> ${OUTPUT_FILE}
echo "------- | ---- | -------"  >> ${OUTPUT_FILE}


for pkg in $ALL_PACKAGES
do
    echo processing $pkg...
    output_package $pkg
done
# output_package cmd_vel_mux
CONTENT=$(sed -e 's/\r//' -e's/\t/\\t/g' -e 's/"/\\"/g' "${OUTPUT_FILE}" | awk '{ printf($0 "\\n") }')
curl -X POST \
    --data-binary "{\"files\": {\"packages.md\": {\"content\": \"$CONTENT\"}}}" \
    https://api.github.com/gists | grep html_url | head -n 1 | awk '{ printf($2) }'
echo

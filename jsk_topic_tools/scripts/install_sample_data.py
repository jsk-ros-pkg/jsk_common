#!/usr/bin/env python

from jsk_data import download_data


def main():
    PKG = 'jsk_topic_tools'

    download_data(
        pkg_name=PKG,
        path='sample/data/sample_tf.bag',
        url='https://drive.google.com/uc?id=15i7dPbMxrxR0j6KsH5mSgIWrNNTrYoV4',
        md5='13cd56d8d85cdf80139bbe5f7a9e2ef0',
        extract=False,
    )


if __name__ == '__main__':
    main()

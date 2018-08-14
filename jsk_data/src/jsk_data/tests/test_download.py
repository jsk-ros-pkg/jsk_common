#!/usr/bin/env python
# -*- coding: utf-8 -*-

from nose.tools import assert_true

from jsk_data.download_data import download_data


def test_download_data_0(self):
    succeeded = download_data(
        pkg_name=PKG,
        path='trained_data/drill_svm.xml',
        url='https://drive.google.com/uc?id=0B5hRAGKTOm_KWW11R0FTX0xjTDg',
        md5='762d0da4bcbf50e0e92939372988901a',
        quiet=quiet,
    )
    assert_true(succeeded)

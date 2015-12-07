#!/usr/bin/env python
# -*- coding: utf-8 -*-

from nose.tools import assert_equal

from jsk_topic_tools.name_utils import unresolve_name


def test_unresolve_name_0():
    node_name = 'relay_0'
    name = 'relay_0/input'
    assert_equal(unresolve_name(node_name, name), '~input')


def test_unresolve_name_1():
    node_name = 'relay_0'
    name = 'relay_1/input'
    assert_equal(unresolve_name(node_name, name), name)

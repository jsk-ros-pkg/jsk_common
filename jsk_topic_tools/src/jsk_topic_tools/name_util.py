#!/usr/bin/env python
# -*- coding: utf-8 -*-


def unresolve_name(node_name, name):
    suffix = node_name + '/'
    if name.startswith(suffix):
        return '~{0}'.format(name[len(suffix):])
    return name

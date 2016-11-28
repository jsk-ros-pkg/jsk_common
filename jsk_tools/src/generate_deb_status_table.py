#!/usr/bin/env python

import argparse
import collections

import tabulate


def generate_deb_status_table(package, rosdistro_from, rosdistro_to):
    DISTROS = collections.OrderedDict()
    DISTROS['indigo'] = ['saucy', 'trusty']
    DISTROS['jade'] = ['trusty', 'vivid']
    DISTROS['kinetic'] = ['wily', 'xenial']

    table = []
    for bit, arch in zip([32, 64], ['i386', 'amd64']):
        if not table:  # first row
            headers = ['Package']
        row = ['{} ({}-bit)'.format(package, bit)]
        for distro, os_list in DISTROS.items():
            if not (ord(rosdistro_from) <= ord(distro[0]) <=
                    ord(rosdistro_to)):
                continue

            for os in os_list:
                if not table:  # first row
                    headers.append(
                        '{} ({})'.format(distro.capitalize(), os.capitalize()))

                url = 'http://build.ros.org/job/{prefix_ros}bin_u{prefix_os}{bit}__{package}__ubuntu_{os}_{arch}__binary'  # NOQA
                url = url.format(
                    bit=bit,
                    arch=arch,
                    prefix_os=os[0].upper(),
                    prefix_ros=distro[0].upper(),
                    package=package,
                    os=os,
                )
                template_md = '[![Build Status]({url}/badge/icon)]({url})'
                row.append(template_md.format(url=url))
        table.append(row)

    print(tabulate.tabulate(table, headers=headers, tablefmt='pipe'))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('package')
    parser.add_argument('-f', '--rosdistro-from', default='i')
    parser.add_argument('-t', '--rosdistro-to', default='k')
    args = parser.parse_args()

    package = args.package
    rosdistro_from = args.rosdistro_from
    rosdistro_to = args.rosdistro_to

    generate_deb_status_table(package, rosdistro_from, rosdistro_to)


if __name__ == '__main__':
    main()

#!/usr/bin/env python
# -*- coding: utf-8 -*-

import argparse
import yaml
import shutil


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('rviz_file', help='Rviz config filename')
    args = parser.parse_args()

    rviz_file = args.rviz_file

    rviz_cfg = yaml.load(open(rviz_file))
    displays = rviz_cfg['Visualization Manager']['Displays']

    print('Which do you wanna copy?:')
    for i, d in enumerate(displays):
        print('- {id} name: {Name}, class: {Class}'.format(id=i, **d))
    selected = int(raw_input('select: '))
    yn = raw_input('''- {id} name: {Name}, class: {Class}
You selected above. Are you sure? [yN]:'''\
        .format(id=selected, **displays[selected]))

    if yn.lower() != 'y':
        print('Aborted!')
        return

    shutil.copy(rviz_file, rviz_file + '.bak')
    copy_element = displays[selected].copy()
    copy_element['Name'] += ' Copied'
    rviz_cfg['Visualization Manager']['Displays'].append(copy_element)
    yaml.dump(rviz_cfg, open(rviz_file, 'wb'))


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('Aborted!')

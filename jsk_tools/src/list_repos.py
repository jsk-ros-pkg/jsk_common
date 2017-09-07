#!/usr/bin/env python

import argparse

from jsk_tools.github_lib import login_github


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--rosinstall', action='store_true',
                        help='Outputs with rosinstall format.')
    args = parser.parse_args()

    rosinstall = args.rosinstall

    gh = login_github()

    for org_name in ['jsk-ros-pkg', 'start-jsk']:
        org = gh.get_organization(org_name)
        for repo in org.get_repos():
            if rosinstall:
                if repo.private:
                    uri_prefix = 'git@github.com:'
                else:
                    uri_prefix = 'https://github.com/'
                print('''\
- git:
    local-name: {repo.full_name}
    uri: {uri_prefix}{repo.full_name}.git
    version: {repo.default_branch}'''.format(repo=repo,
                                             uri_prefix=uri_prefix))
            else:
                print(repo.full_name)


if __name__ == '__main__':
    main()

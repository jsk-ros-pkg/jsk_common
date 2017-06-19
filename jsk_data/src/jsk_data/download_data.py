import hashlib
import os
import os.path as osp
import re
import shlex
import subprocess
import shutil
import sys
import tarfile
import zipfile

import rosbag.rosbag_main
import rospkg


def extract_file(path, to_directory='.'):
    print("Extracting '{path}'...".format(path=path))
    if path.endswith('.zip'):
        opener, mode = zipfile.ZipFile, 'r'
    elif path.endswith('.tar.gz') or path.endswith('.tgz'):
        opener, mode = tarfile.open, 'r:gz'
    elif path.endswith('.tar.bz2') or path.endswith('.tbz'):
        opener, mode = tarfile.open, 'r:bz2'
    else:
        raise ValueError("Could not extract '%s' as no appropriate "
                         "extractor is found" % path)

    cwd = os.getcwd()
    os.chdir(to_directory)
    root_files = []
    try:
        file = opener(path, mode)
        try:
            file.extractall()
            root_files = list(set(name.split('/')[0]
                                  for name in file.getnames()))
        finally:
            file.close()
    finally:
        os.chdir(cwd)
    print('...done')
    return root_files


def decompress_rosbag(path, quiet=False):
    print("Decompressing '{path}'...".format(path=path))
    argv = [path]
    if quiet:
        argv.append('--quiet')
    rosbag.rosbag_main.decompress_cmd(argv)
    print('...done')


def download(client, url, output, quiet=False, chmod=True):
    print("Downloading file from '{url}'...".format(url=url))
    cmd = '{client} {url} -O {output}'.format(client=client, url=url,
                                              output=output)
    if quiet:
        cmd += ' --quiet'
    subprocess.call(shlex.split(cmd))
    if chmod:
        os.chmod(output, 0766)
    print('...done')


def check_md5sum(path, md5):
    # validate md5 string length if it is specified
    if md5 and len(md5) != 32:
        raise ValueError('md5 must be 32 charactors\n'
                         'actual: {} ({} charactors)'.format(md5, len(md5)))
    print("Checking md5sum of '{path}'...".format(path=path))
    is_same = hashlib.md5(open(path, 'rb').read()).hexdigest() == md5
    print('...done')
    return is_same


def is_google_drive_url(url):
    m = re.match('^https?://drive.google.com/uc\?id=.*$', url)
    return m is not None


def download_data(pkg_name, path, url, md5, download_client=None,
                  extract=False, compressed_bags=None, quiet=True, chmod=True):
    """Install test data checking md5 and rosbag decompress if needed.
       The downloaded data are located in cache_dir, and then linked to specified path.
       cache_dir is set by environment variable `JSK_DATA_CACHE_DIR` if defined, set by ROS_HOME/data otherwise."""
    if download_client is None:
        if is_google_drive_url(url):
            download_client = 'gdown'
        else:
            download_client = 'wget'
    if compressed_bags is None:
        compressed_bags = []
    if not osp.isabs(path):
        # get package path
        rp = rospkg.RosPack()
        try:
            pkg_path = rp.get_path(pkg_name)
        except rospkg.ResourceNotFound:
            print('\033[31m{name} is not found in {path}\033[0m'
                  .format(name=pkg_name, path=rp.list()))
            return
        pkg_path = rp.get_path(pkg_name)
        path = osp.join(pkg_path, path)
        if not osp.exists(osp.dirname(path)):
            try:
                os.makedirs(osp.dirname(path))
            except OSError as e:
                print('\033[31mCould not make direcotry {dir} {err}\033[0m'
                      .format(dir=osp.dirname(path), err=e))
                return
    # prepare cache dir
    if "JSK_DATA_CACHE_DIR" in os.environ:
        cache_root_dir = os.getenv("JSK_DATA_CACHE_DIR")
    else:
        cache_root_dir = osp.join(os.getenv('ROS_HOME', osp.expanduser('~/.ros')), "data")
    cache_dir = osp.join(cache_root_dir, pkg_name)
    if not osp.exists(cache_dir):
        os.makedirs(cache_dir)
        if chmod:
            os.chmod(cache_dir, 0777)
    cache_file = osp.join(cache_dir, osp.basename(path))
    # check if cache exists, and update if necessary
    if not (osp.exists(cache_file) and check_md5sum(cache_file, md5)):
        if osp.exists(cache_file):
            os.remove(cache_file)
        download(download_client, url, cache_file, quiet=quiet, chmod=chmod)
    if osp.islink(path):
        # overwrite the link
        os.remove(path)
        os.symlink(cache_file, path)
    elif not osp.exists(path):
        os.symlink(cache_file, path)  # create link
    else:
        # not link and exists so skipping
        sys.stderr.write("WARNING: '{0}' exists\n".format(path))
        return
    if extract:
        # extract files in cache dir and create symlink for them
        extracted_files = extract_file(cache_file, to_directory=cache_dir)
        for file_ in extracted_files:
            file_ = osp.join(cache_dir, file_)
            dst_path = osp.join(osp.split(path)[0], osp.basename(file_))
            if osp.islink(dst_path):
                os.remove(dst_path)
            elif osp.exists(dst_path) and not osp.isdir(dst_path):
                os.remove(dst_path)
            elif osp.exists(dst_path) and osp.isdir(dst_path):
                shutil.rmtree(dst_path)
            os.symlink(file_, dst_path)
    for compressed_bag in compressed_bags:
        if not osp.isabs(compressed_bag):
            rp = rospkg.RosPack()
            pkg_path = rp.get_path(pkg_name)
            compressed_bag = osp.join(pkg_path, compressed_bag)
        decompress_rosbag(compressed_bag, quiet=quiet)

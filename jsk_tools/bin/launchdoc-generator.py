#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Copyright (C) 2011 Tokyo University
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#     http://www.apache.org/licenses/LICENSE-2.0
# 
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# author: Rosen Diankov
from __future__ import with_statement # for python 2.5
import roslib
import os
import fnmatch
import tempfile
import re
import codecs
from optparse import OptionParser

from xml.parsers.expat import ExpatError
import xml.etree.ElementTree as ElementTree

encode = 'utf-8'
sphinx_conf = """
# -*- coding: utf-8 -*-
import roslib; roslib.load_manifest('jsk_tools')
import sys, os, jsk_tools

extensions = ['sphinx.ext.autodoc', 'sphinx.ext.intersphinx', 'sphinx.ext.todo', 'sphinx.ext.ifconfig', 'jsk_tools.shellblock_directive', 'jsk_tools.video_directive']
templates_path = ['.templates']
source_suffix = '.rst'
#source_encoding = 'utf-8'
master_doc = 'index'
project = u'%(project)s'
copyright = u'2011, %(author)s'

version = '1.0'
release = '1.0'
#language = None
#today_fmt = '%%B %%d, %%Y'
exclude_patterns = []
#default_role = None
#add_function_parentheses = True
#add_module_names = True
show_authors = True
pygments_style = 'sphinx'
#modindex_common_prefix = []

# -- Options for HTML output ---------------------------------------------------
html_theme = 'sphinxdoc'
#html_theme_options = {}
#html_theme_path = []
#html_title = None
#html_short_title = None
#html_logo = None
#html_favicon = None
html_static_path = ['.static']
html_last_updated_fmt = '%%H:%%M:%%S %%b %%d, %%Y'
#html_use_smartypants = True
#html_sidebars = {}
#html_additional_pages = {}
#html_domain_indices = True
#html_use_index = True
#html_split_index = False
#html_show_sourcelink = True
#html_show_sphinx = True
#html_show_copyright = True
#html_use_opensearch = ''
#html_file_suffix = None
htmlhelp_basename = '%(project)sdoc'

intersphinx_mapping = {'http://docs.python.org/': None}
"""

def locate(pattern, root=os.curdir):
    '''Locate all files matching supplied filename pattern in and below
    supplied root directory.'''
    for path, dirs, files in os.walk(os.path.abspath(root)):
        files.sort()
        for filename in fnmatch.filter(files, pattern):
            yield os.path.join(path, filename)

if __name__ == '__main__':
    parser = OptionParser(description='create sphinx documentation for XML files.',
                          usage='%prog [options] ros-package-name')
    parser.add_option('--extension', action='store', type='string', dest='extension', default='launch',
                      help="the extension of the files to look for (default=%default)")
    parser.add_option('--tag', action='store', type='string', dest='tag', default='sphinxdoc',
                      help="the tag in the xml file to look for (default=%default)")
    parser.add_option('--output_dir', action='store', type='string', dest='output_dir', default='launchdoc',
                      help="the output directory (default=%default)")
    parser.add_option('--output_filename', action='store', type='string', dest='output_filename', default='index.rst',
                      help="the output file name (default=%default)")
    parser.add_option('--nomakefile', action='store_true', dest='nomakefile', default=False,
                      help="if set will not output a makefile (default=%default)")
    parser.add_option('--setlocalmovie', action='store_true', dest='setlocalmovie', default=True,
                      help="if set will output index.rst with local movie path instead of jenkins path (default=%default)")
    (options, args) = parser.parse_args()
    pkgdir = roslib.packages.get_pkg_dir(args[0])
    manifest = roslib.manifest.parse_file(os.path.join(pkgdir,'manifest.xml'))
    atts = {'project':args[0],'author':manifest.author,'license':manifest.license,'brief':manifest.brief,'description':re.sub('\\n','\n  ',manifest.description)}
    atts['titlebar'] = '='*(17+len(args[0]))
    sphinxdoc = """%(project)s ROS Launch Files\n%(titlebar)s\n
**Description:** %(brief)s

  %(description)s

**License:** %(license)s

"""%atts
    for fullfilename in locate('*.'+options.extension,pkgdir):
        path, filename = os.path.split(fullfilename)
        try:
            parser = ElementTree.parse(fullfilename)
        except (SyntaxError, ExpatError) as e:
            print("%s has errors!"%fullfilename,e)
            continue

        sphinxdoc += '%s\n%s\n\n'%(filename,'-'*len(filename))
        sphinxdoc += '.. code-block:: bash\n\n  roslaunch %s %s\n\n'%(args[0],filename)
        tag = parser.find(options.tag)
        if tag is not None:
            raw_text = tag.text

            if not options.setlocalmovie:
                raw_text_split = raw_text.splitlines()
                insert_counter = 0
                for (i,rt) in zip(range(0,len(raw_text_split)),raw_text_split):
                    if rt.find("video::") >= 0:
                        video_text = rt.split(" ")
                        for (j,vt) in zip(range(0,len(video_text)),video_text):
                            if vt.find("video::") >= 0:
                                tmp_text = os.path.basename(video_text[j + 1])
                                raw_text_split.insert(i + 1 + insert_counter, "  :url: http://jenkins.jsk.imi.i.u-tokyo.ac.jp:8080/job/agentsystem-test/lastSuccessfulBuild/artifact/%s-example/_images/%s"%(args[0],tmp_text))
                                insert_counter = insert_counter + 1
                raw_text = "\n".join(raw_text_split)

            sphinxdoc += raw_text+'\n\n'
            parser.getroot().remove(tag)
            tf = tempfile.TemporaryFile()
            parser.write(tf)
            tf.seek(0)
            sxml = tf.read()
            sphinxdoc += """Contents
########

.. code-block:: xml

  %s

"""%re.sub('\\n','\n  ',sxml)

    try:
        os.mkdir(os.path.join(pkgdir,options.output_dir))
    except OSError:
        pass
    Makefile = """
all:
\tsphinx-build -b html . %s
"""%os.path.relpath(os.path.join(pkgdir,'doc/launch'),os.path.join(pkgdir,options.output_dir))
    codecs.open(os.path.join(pkgdir,options.output_dir,options.output_filename),'w',encode).write(sphinxdoc)
    codecs.open(os.path.join(pkgdir,options.output_dir,'conf.py'),'w',encode).write(sphinx_conf%atts)
    if not options.nomakefile:
        open(os.path.join(pkgdir,options.output_dir,'Makefile'),'w').write(Makefile)

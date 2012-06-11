# copy from openrave/docs/sphinxext/ directory
#   2012 6/11 : added url tag
import docutils.nodes
from docutils.parsers.rst import Directive, directives
import subprocess, shlex, sys, shutil, os
from docutils.parsers.rst.roles import set_classes
from docutils.nodes import General, Inline, Element

class video(General, Element):
    pass

class VideoDirective(Directive):
    required_arguments = 1
    optional_arguments = 0
    final_argument_whitespace = True
    option_spec = {'poster': directives.unchanged,
                   'width': directives.length_or_percentage_or_unitless,
                   'url' :  directives.unchanged}

    def run(self):
        videoname = self.arguments[0]
        poster = self.options.pop('poster', videoname+".png")
        width = self.options.pop('width', 600)
        url = self.options.pop('url', "")
        node = video();
        node['videoname']=videoname
        node['poster']=poster
        node['width']=width
        node['url']=url
        return [node]

def html_visit_video(self, node):
    self.body.append(self.starttag(node, 'p', CLASS='video'))
    # start video
    src = node['poster']
    dest= os.path.basename(src);
    self.body.append('  <video controls autoplay poster="_images/%s" width="%s">\n'%(dest,node['width']))
    self.builder.images[src]=dest
    # sources
    for codec in ['ogv', 'mp4'] :
        src = "%s.%s"%(node['videoname'],codec)
        dest= os.path.basename(src);
        if node['url'] != "":
            self.body.append('    <source src="%s.%s">\n'%(node['url'],codec))
        else:
            self.body.append('    <source src="_images/%s" >\n'%(dest))
        self.builder.images[src]=dest
    # finish video
    self.body.append('    <p> Your browser does not support the <code>video</code> element. \n')
    if node['url'] != "":
        self.body.append('    <a href="%s.%s"> %s </a>\n'%(node['url'],codec,dest))
    else:
        self.body.append('    <a href="_images/%s"> %s </a>\n'%(dest, dest))
    self.body.append('    </p>\n')
    self.body.append('  </video>\n')
    self.body.append('</p>\n')
    raise docutils.nodes.SkipNode

def setup(app):
    app.add_node(video,
                 html=(html_visit_video, None))
    app.add_directive('video', VideoDirective)

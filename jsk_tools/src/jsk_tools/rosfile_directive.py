# copy from openrave/docs/sphinxext/ directory
import docutils.nodes
from docutils.parsers.rst import Directive
import subprocess, shlex
import roslib,os

class ROSFileDirective(Directive):
    required_arguments = 0
    optional_arguments = 0
    final_argument = False
    option_spec = {}
    has_content = True
    def run(self):
        self.assert_has_content()
        args = shlex.split(self.content[0].encode('ascii'))
        resource = roslib.packages.find_resource(args[0], args[1])
        try :
            text = open(resource[0],'r').read()
        except :
            raise ValueError('failed: %s'%text)
        return [docutils.nodes.literal_block(text=text)]

def setup(app):
    app.add_directive('ros-file', ROSFileDirective)

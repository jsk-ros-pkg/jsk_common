from importlib import import_module

import rospy


def expr_eval(expr, modules=None):
    modules = modules or {}
    def eval_fn(topic, m, t):
        return eval(expr, modules, {'m': m, 'topic': topic, 't': t})
    return eval_fn


def import_modules(import_list):
    modules = {}
    for module in import_list:
        try:
            mod = import_module(module)
        except ImportError:
            rospy.logerr('Failed to import module: %s' % module)
        else:
            modules[module] = mod
    return modules

"""
Utilities for meta-class and decorator operations
"""

import inspect
import functools

def method_decorator(func_dec):
    """Decorates a decorator such that it defers application on instance_method objects"""
    @functools.wraps(func_dec)
    def decorator(f):
        argspec = inspect.getargspec(f)
        if argspec.args and argspec.args[0] == 'self':
            # store the decorator, to be applied later
            f.__method_dec = func_dec
        else:
            f = func_dec(f)
        return f
    return decorator

class HasMethodDecorators(object):
    """A trait class to apply all deferred method decorators"""
    def __init__(self):
        super(HasMethodDecorators, self).__init__()
        for k, v in inspect.getmembers(self):
            dec = getattr(v, '__method_dec', None)
            if dec is not None:
                self.__dict__[k] = dec(v)


class InitedDescriptor(object):
    def __setup__(self, obj, obj_type, attr_name):
        """
        Like __get__, but called on owning class / object construction
        Params:
            obj        the instance being setup on, or None if a class
            obj_type   the class being setup on
            attr_name  the name of the attribute the object was assigned to
        """
        pass


class HasInitedDescriptorsMeta(type):
    def __init__(cls, name, bases, dct):
        super(HasInitedDescriptorsMeta, cls).__init__(name, bases, dct)
        for attr_name, value in inspect.getmembers(cls):
            if isinstance(value, InitedDescriptor):
                value.__setup__(None, cls, attr_name)


class HasInitedDescriptors(object):
    """ A trait class to initialize all InitedDescriptors"""
    __metaclass__ = HasInitedDescriptorsMeta

    def __init__(self):
        for attr_name, value in inspect.getmembers(self):
            if isinstance(value, InitedDescriptor):
                value.__setup__(self, self.__class__, attr_name)
        super(HasInitedDescriptors, self).__init__()
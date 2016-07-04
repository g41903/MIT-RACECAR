import inspect
import rospy
import functools
import types

# from .
import metautils as mu

def Subscriber(name, data_class=None, **kwargs):
	"""
	Decorate a method or function as subscribing to an event:

		def Foo(Node):
			@Subscriber('/channel', Float32)
			def on_channel(self, msg):
				pass

		@Subscriber('/channel', Float32)
		def on_channel(msg):
			pass

		@Subscriber(lambda:
			msg_filters.TimeSynchronizer([
		    	msg_filters.Subscriber('/channel', Float32),
		    	msg_filters.Subscriber('/channel2', Float32)
		    ], queue_size=10)
		)
		def on_both_channels(msg):
			pass
	"""

	# handle message_filter
	if data_class is None:
		if isinstance(name, basestring):
			raise TypeError('Missing second argument')
		producer = name
		if not callable(producer):
			raise TypeError('Argument must be a lambda function, not a value')

		@mu.method_decorator
		def decorator(f):
			result = producer()
			try:
				result.registerCallback(f)
			except:
				raise ValueError('Expected the function to return a message_filter object')
			return f

	# handle proxied call to Subscriber
	elif isinstance(name, basestring):
		@mu.method_decorator
		def decorator(f):
			rospy.Subscriber(name, data_class, callback=f, **kwargs)
			return f

	else:
		raise TypeError("Argument 'name' must be a string")

	return decorator


class Publisher(mu.InitedDescriptor):
	def __init__(self, name, data_class, **kwargs):
		self._args = (name, data_class, kwargs)

	def __setup__(self, node, nodecls, attr_name):
		if node is not None:
			ch_name, data_class, kw = self._args
			# overwrite ourselves
			setattr(node, attr_name, rospy.Publisher(ch_name, data_class, **kw))

def Timer(period, oneshot=False):
	@mu.method_decorator
	def decorator(f):
		rospy.Timer(period, f, oneshot=oneshot)
		return f

	return decorator


class Param(mu.InitedDescriptor):
	"""
	Represents a ROS parameter. Is a class descriptor, so only works inside a
	class. Valid usage:

		class MyThing(object):
			someparam = Param('~someparam', int, default=1)

			def foo(self):
				print self.someparam

	When used with the `Node` class, the above can be abbrebiated as:

		class MyThing(Node):
			someparam = Param(int, default=1)

			def foo(self):
				print self.someparam
	"""

	NO_DEFAULT = object()

	def __init__(self, name=None, type_=None, default=NO_DEFAULT):
		"""
		Invokable as:

			Param(name="name", type_=int, ...)
			Param("name", type_=int, ...)
			Param(type_=int, ...)
			Param(int, ...)

		The name argument is optional, and will be infered by the attribute it
		is assigned to
		"""
		# ok_types = (int, float, string, bool)
		self.name = None
		if isinstance(name, basestring):
			self.name = name

		elif isinstance(name, type):
			if type_ is not None:
				raise TypeError('Argument \'type\' was passed in multiple times')
			type_ = name

		if type_ is None:
			raise TypeError('Missing argument \'type_\', or a typename was passed instead of a type')
		elif not isinstance(type_, type):
			raise TypeError('Argument \'type_\' must be a type')
		else:
			self.type_ = type_

		self.default = default

	def __get__(self, obj, cls=None):
		if obj is None:
			return self

		# the python descriptor method
		if self.default is self.NO_DEFAULT:
			res = rospy.get_param(self.name)
		else:
			res = rospy.get_param(self.name, self.default)

		if res is None:
			return None
		else:
			return self.type_(res)

	def __set__(self, obj, value):
		# the other python descriptor method
		rospy.setparam(self.name, value)

	def __setup__(self, node, nodecls, attr):
		""" Infer name from owning class, if not explicitly set """
		if node is None:
			if self.name is None:
				self.name = '~{}'.format(attr)
			if not hasattr(nodecls, '__params__'):
				nodecls.__params__ = set()

			nodecls.__params__.add(self)

	def __repr__(self):
		if self.default is self.NO_DEFAULT:
			return 'Param({!r}, type={})'.format(self.name, self.type_)
		else:
			return 'Param({!r}, type={}, default={!r})'.format(self.name, self.type_, self.default)


class Node(mu.HasInitedDescriptors, mu.HasMethodDecorators):
	pass


def invoke_from_ros_params(f):
	"""
	Example:

		def foo(a=2, b=2, c=3):
			pass

		res = invoke_from_ros_params(foo)

	"""
	argspec = inspect.getargspec(f)
	NO_ARG = object()
	args = argspec.args
	defs = (NO_ARG,)*(len(args) - len(argspec.defaults)) + argspec.defaults

	kwargs = {}
	for arg, default in zip(args, defaults):
		if default is not NO_ARG:
			kwargs[arg] = rospy.get_param('~{}'.format(arg), default)
		else:
			kwargs[arg] = rospy.get_param('~{}'.format(arg))

	return f(**kwargs)

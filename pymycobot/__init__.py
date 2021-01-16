from __future__ import absolute_import
import platform


name = 'pymycobot'

version = (platform.python_version())

if version.startswith('2.7'):
    __all__ = ['mycobot', 'common']
else:
    __all__ = ['mycobot', 'mycobot3', 'common']


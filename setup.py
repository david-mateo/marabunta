import os
#from setuptools import setup
from distutils.core import setup

def read(fname):
        return open(os.path.join(os.path.dirname(__file__), fname)).read()
setup(
        name = 'marabunta' ,
        version = '1.0' ,
        description = 'Library for design and control of artificial swarms.' ,
        long_description = read("README.md") ,
        url = 'https://github.com/david-mateo/marabunta' ,
        author = 'David Mateo' ,
        author_email = 'david.mateo.valderrama@gmail.com' ,
        license = 'GPLv3' ,
        keywords = 'control simulation robot swarming' ,
        packages = ['marabunta', 'marabunta.models'] ,
        )

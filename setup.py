from setuptools import setup, find_packages

setup(
    name='fieldpath',
    version='0.1.0',    
    description='Generate vector coverage paths for agricultural fields',
    url='https://github.com/HB0N0/FieldPath',
    author='Hannes Bosch',
    author_email='hbono@gmx.de',
    license='BSD 2-clause',
    packages=['fieldpath'],
    install_requires=['shapely>=2.0',
                      'numpy',
                      'matplotlib',
                      'dubins',
                      ],)
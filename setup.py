from setuptools import setup, find_packages

setup(
    name='fieldpath',
    version='0.2.1',    
    description='Generate vector coverage paths for agricultural fields',
    url='https://github.com/HB0N0/FieldPath',
    author='Hannes Bosch',
    author_email='hbono@gmx.de',
    license='BSD 2-clause',
    packages=find_packages(),
    install_requires=['shapely>=2.0',
                      'numpy',
                      'matplotlib',
                      ],)
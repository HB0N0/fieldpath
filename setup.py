from setuptools import setup, find_packages

setup(
    name='fieldpath',
    version='0.2.2',
    description='Generate vector coverage paths for agricultural fields',
    url='https://github.com/HB0N0/FieldPath',
    author='Hannes Bosch',
    author_email='hbono@gmx.de',
    license='BSD 3-Clause',
    packages=find_packages(),
    python_requires='>=3.9',
    install_requires=['shapely>=2.0',
                      'numpy',
                      'matplotlib',
                      ],)
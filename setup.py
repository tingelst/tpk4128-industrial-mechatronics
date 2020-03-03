#!/usr/bin/env python

from setuptools import setup, find_packages

setup(name='tpk4128',
      version='spring-2020',
      description='Python modules for the course TPK4128 Industrial Mechatronics at NTNU',
      author='Lars Tingelstad',
      author_email='lars.tingelstad@ntnu.no',
      url='https://www.ntnu.no/studier/emner/TPK4128',
      packages=find_packages(),
      install_requires=[
          'numpy',
          'opencv-python',
      ],
      )

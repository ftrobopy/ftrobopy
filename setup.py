from distutils.core import setup, Extension

setup(name='ftrobopy',
      description='Python Interface for Fischertechnik ROBOTICS TXT Controller',
      version='1.67',
      author='Torsten Stuehn',
      author_email='Torsten Stuehn',
      url='https://github.com/ftrobopy/ftrobopy',
      license='MIT',
      py_modules=['ftrobopy'],
      ext_modules=[Extension('ftrobopytools',
                    sources = ['src/ftrobopytools.c'],
                    libraries = ['SDL'])]
      )

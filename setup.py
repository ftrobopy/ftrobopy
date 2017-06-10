from distutils.core import setup, Extension

setup(name='ftrobopy',
      description='Python Interface for Fischertechnik ROBOTICS TXT Controller',
      version='1.75',
      author='Torsten Stuehn',
      author_email='Torsten Stuehn',
      url='https://github.com/ftrobopy/ftrobopy',
      download_url='https://github.com/ftrobopy/ftrobopy/archive/1.75.tar.gz',
      license='MIT',
      py_modules=['ftrobopy'],
      ext_modules=[Extension('ftrobopytools',
                    sources = ['src/ftrobopytools.c'],
                    libraries = ['SDL'])]
      )

from setuptools import setup

version = "0.0.1"

with open('README.md') as f:
    long_description = f.read()

setup(
    name='pymavswarm',
    version=version,
    zip_safe=True,
    description='Python library used to communicate with robotic swarms using MAVLink',
    long_description_type='text/markdown',
    long_description=long_description,
    url='https://github.com/unl-nimbus-lab/pymavswarm',
    author='Evan Palmer',
    author_email='evanp922@gmail.com',
    classifiers=[
        'Development Status :: 3 - Alpha',
        'Intended Audience :: Science/Research',
        'License :: OSI Approved :: GNU General Public License v3 (GPLv3)',
        'Operating System :: OS Independent',
        'Programming Language :: Python',
        'Topic :: Scientific/Engineering'
    ],
    license='GPLv3',
    packages=[
        'pymavswarm'
    ],
    setup_requires=[
        "setuptools", 
        "wheel"
    ],
    install_requires=[
        'pymavlink>=2.3.3',
        'pyserial>=3.0'
    ],
)
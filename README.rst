==========
pymavswarm
==========

.. image:: https://img.shields.io/pypi/v/pymavswarm?color=gr
    :target: https://pypi.org/project/pymavswarm/
    :alt: PyPI Version

.. image:: https://img.shields.io/github/commit-activity/m/unl-nimbus-lab/pymavswarm
    :target: https://github.com/unl-nimbus-lab/pymavswarm/commits/main
    :alt: Contribution Frequency

.. image:: https://img.shields.io/github/license/unl-nimbus-lab/pymavswarm
    :target: https://github.com/unl-nimbus-lab/pymavswarm/blob/main/LICENSE
    :alt: License: GPLv3

.. image:: https://img.shields.io/github/issues/unl-nimbus-lab/pymavswarm?color=purple
    :target: https://github.com/unl-nimbus-lab/pymavswarm/issues
    :alt: Open Issues

.. image:: https://readthedocs.org/projects/pymavswarm/badge/?version=latest
    :target: https://pymavswarm.readthedocs.io/en/latest/?badge=latest
    :alt: Documentation Status

.. read-me-summary-begin

What is pymavswarm?
-------------------

``pymavswarm`` is a Python library implemented to enable interaction with drone
swarms using the MAVLink protocol. This library supports reading MAVLink
messages sent from multiple agents in a swarm and sending MAVLink messages to
agents within the swarm. Such functionality ultimately enables development of
new swarm applications such as ground control stations.

Main features
-------------

Here are some of the main features of ``pymavswarm``:

- Easily send MAVLink commands to multiple drones, simultaneously
- Monitor the state of swarm drone agents
- Implement custom commands and swarm algorithms
- Develop applications for on-board companion computers
- Implement custom swarm ground control stations
- Log incoming MAVLink messages for future evaluation and debugging
- Construct pre-planned missions
- Multi-agent collision avoidance using reachability analysis

Installation
------------

``pymavswarm`` may be installed from `PyPI`_ by running:

.. _PyPI: https://pypi.org/project/pymavswarm/#description

.. code-block:: console

    $ python3 -m pip install pymavswarm

To build ``pymavswarm`` from source, first ensure that at least Python 3.10 is
installed. Once this dependency has been met, refer to the steps below:

1. Clone the project `repository`_
2. Navigate to the ``pymavswarm/`` base directory

.. _repository: https://github.com/unl-nimbus-lab/pymavswarm

.. code-block:: console

    $ cd path/to/pymavswarm/

3. Install the ``pymavswarm`` Python package

.. code-block:: console

    $ python3 -m pip install .

Quick start
-----------

``pymavswarm`` has been implemented to enable easy interfacing with drone
swarms. Refer to the following code snippet for a simple example to get started
with the library. Additional examples may be found in the project `examples`_.

.. _examples: https://github.com/unl-nimbus-lab/pymavswarm/tree/main/examples

.. code-block:: python

    import time

    from pymavswarm import MavSwarm

    # Create a new pymavswarm interface
    mavswarm = MavSwarm()

    # Establish a connection with a USB telemetry device
    mavswarm.connect('/dev/ttyUSB0', 115200, 255, 0)

    # Wait for the swarm to be populated
    while not mavswarm.agents:
        pass

    # Arm each agent in the swarm
    mavswarm.arm()

    # Briefly delay to allow all agents to arm
    time.sleep(5.0)

    # Disarm each agent in the swarm
    mavswarm.disarm()

    # Close the pymavswarm connection
    mavswarm.disconnect()

Getting help
------------

If you have questions regarding ``pymavswarm`` usage or contribution please ask a
question on our `Discussions`_ board!

.. _Discussions: https://github.com/unl-nimbus-lab/pymavswarm/discussions

.. read-me-summary-end

Contributing
------------

All contributions and ideas are welcome! Detailed guidelines regarding how to
contribute can be found in the `contribution guidelines`_.

.. _contribution guidelines: https://github.com/unl-nimbus-lab/pymavswarm/blob/main/.github/CONTRIBUTING.md

License
-------

``pymavswarm`` is released under the GNU General Public License v3 or later

==========
Change Log
==========

All contributions to the ``pymavswarm`` system are documented here. ``pymavswarm``
utilizes `Semantic Versioning`_ for project versioning.

.. _Semantic Versioning: https://semver.org/

[v1.2.0] - 2022-09-11
---------------------

Removed
^^^^^^^

- Ability to specify a single target agent ID using a tuple when sending commands, and
  now always require a list argument: `PR #120`_

.. _PR #120: https://github.com/unl-nimbus-lab/pymavswarm/pull/120


[v1.1.1] - 2022-09-09
---------------------

Fixed
^^^^^

- Acknowledgement failure when sending navigation commands (e.g., waypoints): `PR #124`_

.. _PR #124: https://github.com/unl-nimbus-lab/pymavswarm/pull/124


[v1.1.0] - 2022-09-04
---------------------

Added
^^^^^

- Support for sending debug vectors: `PR #118`_

.. _PR #118: https://github.com/unl-nimbus-lab/pymavswarm/pull/118


[v1.0.0] - 2022-08-13
---------------------

Added
^^^^^

- User interface to enable simple interaction with drone fleets or
  drone swarms
- Log file parsing to support developers that need to evaluate their logs
- Collision avoidance using real-time reachability analysis

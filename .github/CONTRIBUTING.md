# Contributing to pymavswarm

Thank you for considering contributing to `pymavswarm`! This document serves
to provide a set of guidelines for contributing.

Contributions include but are not restricted to:

- Contributing to the `pymavswarm` codebase
- Writing documentation
- Writing tests
- Performing field tests
- Reporting bugs
- Answering [discussion questions](https://github.com/unl-nimbus-lab/pymavswarm/discussions)

## Workflow

- Send all pull requests to the `main` branch (unless otherwise requested).
- Limit each pull request to resolving a single
  [issue](https://github.com/unl-nimbus-lab/pymavswarm/issues).
- It is your responsibility to ensure that your development branch is up-to-date
  with the `main` branch. You may either rebase on `main` or merge `main` into
  your development branch.
- Always test and document your code. We also encourage performing field tests
  for significant changes.
- Ensure that your changes pass our CI. We will not review your PR until the CI
  passes.

## Setting up a local development environment

`pymavswarm` provides two ways to configure a local development environment:

1. Using our [VSCode development container](https://code.visualstudio.com/docs/remote/containers)
2. Using [Pipenv](https://pipenv.pypa.io/en/latest/)

A VSCode development container has been provided to offer a fully sandboxed
development environment. This environment includes all development Python
packages (e.g., `pre-commit`) used by the `pymavswarm` development team and
utilizes a variety of VSCode extensions that make it easy to run tests and to
ensure that all style conventions are followed. Follow the instructions
[here](https://code.visualstudio.com/docs/remote/containers) to learn how to
install and launch development containers using VSCode. Once these steps have
been completed, launch the `pymavswarm` project using the provided development
image. Launching the development image will also install `pymavswarm` in
editable mode.

A `Pipfile` has also been provided to enable support for creating and managing
a [virtual environment](https://virtualenv.pypa.io/en/latest/) using
[Pipenv](https://pipenv.pypa.io/en/latest/). To use a virtual environment,
first ensure that `pipenv` has been [installed](https://pipenv.pypa.io/en/latest/).
After successfully installing `pipenv`, navigate to the base `pymavswarm/`
directory:

```bash
cd path/to/pymavswarm/
```

Next, install the project dependencies:

```bash
pipenv install --dev
```

We also recommend installing `pymavswarm` in editable mode:

```bash
pip3 install -e .
```

Finally, launch the virtual environment:

```bash
pipenv shell
```

At this point, running the project tests should work and pass (in both the
development container and the virtual environment):

```bash
python3 -m unittest
```

## Coding guidelines

The following section documents the `pymavswarm` coding guidelines (e.g.,
styling, convention, etc.)

### Linting

`pymavswarm` uses `pre-commit` to run code formatting checks such as `black`,
`flake8`, `pydocstyle`, and `isort`. If you have installed the project
using one of our recommended local development configurations, you may run
`pre-commit` using the following command:

```bash
pre-commit run --all-files
```

We *strongly* recommend running `pre-commit` before committing your code to
ensure that your commit follows our code style conventions. Any warnings from
these checks will cause the CI to fail.

### Type hints

`pymavswarm` uses [PEP 484](https://peps.python.org/pep-0484/) type-hints. Any
new development should use type hints. When using type-hints, it is preferred
that built-in types are used (see [PEP 585](https://peps.python.org/pep-0585/)).
The `Optional` type-hint should be avoided in favor of `| None`. For example,
rather than

```python
from typing import Optional

agent_location: Optional[Position] = None
```

You should use

```python
from __future__ import annotations

agent_location: Position | None = None
```

Commonly used types will appear in `pymavswarm.types`. These should be used
where applicable.

## Writing documentation

`pymavswarm` uses Sphinx to generate online developer documentation from
docstrings. Broadly, docstrings should adhere to
[PEP 257](https://peps.python.org/pep-0257/), unless otherwise specified.

All docstrings should use triple quotation marks. Multi-line docstrings should
start on new-lines. Parameters and their types should be prefaced with `:param`
and `:type`, respectively. All methods should have a short summary. An extended
summary should be used when an in-depth explanation of a method is required.

The following example demonstrates the Sphinx markdown conventions used by
`pymavswarm`:

```python
def compute_location(
    current_location: Position | None = None
  ) -> Position | None:
    """
    Demonstrate how to write a docstring.

    Docstrings are a great way to add developer documentation.

    :param current_location: current location of an agent, defaults to None
    :type current_location: Position | None, optional
    :return: computed agent location
    :rtype: Position | None
    """
    return current_location
```

### Testing

Unit tests should be implemented using `unittest`. Unit tests should be
implemented whenever possible. Functional tests should be named `def test_*`.
It is suggested that field tests are also performed when adding a code
contribution; however, this is not required (we understand that not everyone
has a fleet of drones laying around).

## Support

If you have questions regarding your contribution, please create a new
discussion post on the `pymavswarm` [Discussions](https://github.com/unl-nimbus-lab/pymavswarm/discussions)
board.

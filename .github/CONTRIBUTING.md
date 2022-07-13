# Contributing to pymavswarm

Thank you for considering contributing to `pymavswarm`! This document serves to provide a set of guidelines for contributing.

Contributions include but are not restricted to:

- Contributing to the `pymavswarm` codebase
- Writing documentation
- Writing tests
- Performing field tests
- Reporting bugs
- Answering [discussion questions](https://github.com/unl-nimbus-lab/pymavswarm/discussions)

## Workflow

## Setting up a local development environment

`pymavswarm` provides two ways to configure a local development environment:

1. Using a [VSCode development container](https://code.visualstudio.com/docs/remote/containers)
2. Using [Pipenv](https://pipenv.pypa.io/en/latest/)

A VSCode development container has been provided to offer a fully sandboxed
development environment. This environment includes all development Python
packages (e.g., `pre-commit`) used by the `pymavswarm` development team and
utilizes a variety of VSCode packages that make it easy to run tests and to
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

## Writing documentation

## Support

If you have questions regarding your contribution, please create a new
discussion post on the `pymavswarm` [Discussions](https://github.com/unl-nimbus-lab/pymavswarm/discussions)
board.

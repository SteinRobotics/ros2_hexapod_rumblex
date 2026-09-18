import pathlib

_PACKAGE_ROOT = pathlib.Path(__file__).parent.joinpath('..').resolve()


def package_resource_path(*parts) -> pathlib.Path:
    """Resolve a path relative to the package root directory."""
    return _PACKAGE_ROOT.joinpath(*parts)

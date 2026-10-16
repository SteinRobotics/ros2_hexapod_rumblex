import pathlib

_PACKAGE_ROOT = pathlib.Path(__file__).parent.joinpath('..').resolve()


def package_resource_path(*parts) -> pathlib.Path:
    """Resolve a path relative to the package root directory."""
    source_path = _PACKAGE_ROOT.joinpath(*parts)
    if not parts or parts[0] != 'models' or source_path.exists():
        return source_path
    from ament_index_python.packages import get_package_share_directory
    return pathlib.Path(get_package_share_directory('rumblex_communication')).joinpath(*parts)

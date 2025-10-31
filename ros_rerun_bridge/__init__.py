from importlib.metadata import PackageNotFoundError, version

try:
    __version__ = version("ros-rerun-bridge")
except PackageNotFoundError:  # during dev
    __version__ = "0.0.0.dev"

# Re-exports
from .main import run

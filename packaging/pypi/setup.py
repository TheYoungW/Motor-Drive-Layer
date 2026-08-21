import os
import shutil
from pathlib import Path

from setuptools import Distribution, setup
from setuptools.command.bdist_wheel import bdist_wheel as _bdist_wheel
from setuptools.command.build import build as _build
from setuptools.command.build_py import build_py as _build_py


def _resolve_articore_runtime_path() -> Path:
    here = Path(__file__).resolve()
    repo_root = here.parents[2]
    lib_name = "libarticore_runtime.so"
    candidates = []
    env = os.getenv("ARTICORE_RUNTIME_LIB")
    if env:
        candidates.append(Path(env).expanduser())
    candidates.extend(
        [
            repo_root
            / "builds"
            / "cmake"
            / "direct-socketcanfd"
            / "articore_runtime"
            / lib_name,
            repo_root
            / "builds"
            / "cmake"
            / "default"
            / "articore_runtime"
            / lib_name,
            repo_root
            / "builds"
            / "cmake"
            / "default"
            / "articore_runtime"
            / "Release"
            / lib_name,
            repo_root / "builds" / "cmake" / "ci" / "articore_runtime" / lib_name,
            repo_root / "build" / "articore_runtime" / lib_name,
            repo_root
            / "build"
            / "articore_runtime"
            / "Release"
            / lib_name,
            repo_root / "cpp_damiao" / "build" / "articore_runtime" / lib_name,
            repo_root
            / "cpp_damiao"
            / "build"
            / "articore_runtime"
            / "Release"
            / lib_name,
            here.parent / "src" / "motor_drive_layer_native" / "lib" / lib_name,
        ]
    )
    for path in candidates:
        if path.exists():
            return path
    tried = "\n".join(f"- {path}" for path in candidates)
    raise RuntimeError(
        "Cannot locate articore_runtime shared library for wheel build.\n"
        f"Tried:\n{tried}\n"
        "Build the unified native targets first with "
        "`cmake -S . -B builds/cmake/default && "
        "cmake --build builds/cmake/default`."
    )


class BuildInArtifactTree(_build):
    def initialize_options(self):
        super().initialize_options()
        repo_root = Path(__file__).resolve().parents[2]
        self.build_base = str(repo_root / "builds" / "packages" / "pypi-build")


class BuildPyWithRuntime(_build_py):
    def run(self):
        super().run()
        dst_dir = Path(self.build_lib) / "motor_drive_layer_native" / "lib"
        if dst_dir.exists():
            shutil.rmtree(dst_dir)
        dst_dir.mkdir(parents=True, exist_ok=True)
        articore_src = _resolve_articore_runtime_path()
        shutil.copy2(articore_src, dst_dir / articore_src.name)

class BdistWheelWithRuntime(_bdist_wheel):
    def finalize_options(self):
        super().finalize_options()
        self.root_is_pure = False

    def get_tag(self):
        _, _, platform_tag = super().get_tag()
        return "py3", "none", platform_tag


class BinaryDistribution(Distribution):
    """Install the ctypes shared library into platlib, not purelib."""

    def has_ext_modules(self) -> bool:
        return True


setup(
    cmdclass={
        "build": BuildInArtifactTree,
        "build_py": BuildPyWithRuntime,
        "bdist_wheel": BdistWheelWithRuntime,
    },
    distclass=BinaryDistribution,
)

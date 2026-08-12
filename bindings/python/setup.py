import os
import platform
import shutil
import sys
from pathlib import Path

from setuptools import Distribution, setup
from setuptools.command.bdist_wheel import bdist_wheel as _bdist_wheel
from setuptools.command.build_py import build_py as _build_py


def _platform_lib_name() -> str:
    if sys.platform.startswith("win"):
        return "motor_abi.dll"
    if sys.platform == "darwin":
        return "libmotor_abi.dylib"
    return "libmotor_abi.so"


def _articore_platform_lib_name() -> str:
    if sys.platform.startswith("win"):
        return "articore_runtime.dll"
    if sys.platform == "darwin":
        return "libarticore_runtime.dylib"
    return "libarticore_runtime.so"


def _dm_device_platform_sources() -> list[tuple[str, Path]]:
    machine = platform.machine().lower()
    if sys.platform.startswith("linux"):
        if machine in {"x86_64", "amd64"}:
            return [
                ("v1.0.0", Path("linux/libdm_device.so")),
                ("v1.1.0", Path("linux/x86_64/libdm_device.so")),
            ]
        if machine in {"aarch64", "arm64"}:
            return [
                ("v1.1.0", Path("linux/arm64/libdm_device.so")),
                ("v1.0.0", Path("aarch64/libdm_device.so")),
            ]
    if sys.platform == "darwin":
        if machine in {"arm64", "aarch64"}:
            return [("v1.1.0", Path("macos/arm64/libdm_device.dylib"))]
        if machine in {"x86_64", "amd64"}:
            return [("v1.1.0", Path("macos/x86_64/libdm_device.dylib"))]
    if sys.platform.startswith("win") and machine in {"x86_64", "amd64"}:
        return [
            ("v1.1.0", Path("windows/msvc/dm_device.dll")),
            ("v1.0.0", Path("msvc/dm_device.dll")),
        ]
    return []


def _candidate_dm_device_paths() -> list[Path]:
    here = Path(__file__).resolve()
    repo_root = here.parents[2]
    sources = _dm_device_platform_sources()
    candidates: list[Path] = []

    env = os.getenv("MOTOR_DM_DEVICE_LIB")
    if env:
        candidates.append(Path(env).expanduser())

    if not sources:
        return candidates

    for version, rel in sources:
        candidates.append(repo_root / "third_party" / "dm_device" / version / rel)
        candidates.append(repo_root / "dm-device-sdk" / "C&C++" / "lib" / version / rel)
        candidates.append(repo_root.parent / "dm-device-sdk" / "C&C++" / "lib" / version / rel)
    candidates.append(
        here.parent
        / "src"
        / "motor_drive_layer"
        / "lib"
        / "dm_device"
        / sources[0][1].name
    )
    return candidates


def _bundle_dm_device_runtime() -> bool:
    raw = os.getenv("MOTOR_DM_DEVICE_BUNDLE", "1").strip().lower()
    return raw not in {"0", "false", "off", "no"}


def _find_dm_device_path() -> Path | None:
    if not _bundle_dm_device_runtime():
        return None
    candidates = _candidate_dm_device_paths()
    for path in candidates:
        if path.exists():
            return path
    tried = "\n".join(f"- {path}" for path in candidates)
    raise RuntimeError(
        "DM_Device runtime bundling is enabled, but no vendor library was found.\n"
        f"Tried:\n{tried}\n"
        "Add the redistributable runtime under third_party/dm_device or set "
        "MOTOR_DM_DEVICE_LIB. Set MOTOR_DM_DEVICE_BUNDLE=0 only for an "
        "intentional runtime-free development build."
    )


def _find_dm_device_support_libraries() -> list[Path]:
    if not _bundle_dm_device_runtime():
        return []
    machine = platform.machine().lower()
    if not sys.platform.startswith("linux") or machine not in {"x86_64", "amd64"}:
        return []
    here = Path(__file__).resolve()
    repo_root = here.parents[2]
    candidates = [
        repo_root / "third_party" / "libstdcxx" / "linux" / "x86_64" / "libstdc++.so.6"
    ]
    for path in candidates:
        if path.exists():
            return [path]
    tried = "\n".join(f"- {path}" for path in candidates)
    raise RuntimeError(
        "The bundled Linux x86_64 DM_Device runtime requires its compatible "
        f"libstdc++.so.6, but it was not found.\nTried:\n{tried}"
    )


def _candidate_abi_paths() -> list[Path]:
    here = Path(__file__).resolve()
    repo_root = here.parents[2]
    lib_name = _platform_lib_name()
    candidates: list[Path] = []

    env = os.getenv("MOTOR_DRIVE_LAYER_LIB")
    if env:
        candidates.append(Path(env).expanduser())

    candidates.append(repo_root / "build" / "cpp_damiao" / lib_name)
    candidates.append(repo_root / "build" / "cpp_damiao" / "Release" / lib_name)
    candidates.append(repo_root / "cpp_damiao" / "build" / lib_name)
    candidates.append(repo_root / "cpp_damiao" / "build" / "Release" / lib_name)
    candidates.append(here.parent / "src" / "motor_drive_layer" / "lib" / lib_name)
    return candidates


def _resolve_abi_path() -> Path:
    for p in _candidate_abi_paths():
        if p.exists():
            return p
    tried = "\n".join(f"- {p}" for p in _candidate_abi_paths())
    raise RuntimeError(
        "Cannot locate motor_abi shared library for wheel build.\n"
        f"Tried:\n{tried}\n"
        "Build native libraries first (`cmake -S . -B build && cmake --build build`) "
        "or set MOTOR_DRIVE_LAYER_LIB."
    )


def _resolve_articore_runtime_path() -> Path:
    here = Path(__file__).resolve()
    repo_root = here.parents[2]
    lib_name = _articore_platform_lib_name()
    candidates = []
    env = os.getenv("ARTICORE_RUNTIME_LIB")
    if env:
        candidates.append(Path(env).expanduser())
    candidates.extend(
        [
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
            here.parent / "src" / "motor_drive_layer" / "lib" / lib_name,
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
        "`cmake -S . -B build && cmake --build build`."
    )


class BuildPyWithAbi(_build_py):
    def run(self):
        super().run()
        abi_src = _resolve_abi_path()
        dst_dir = Path(self.build_lib) / "motor_drive_layer" / "lib"
        if dst_dir.exists():
            shutil.rmtree(dst_dir)
        dst_dir.mkdir(parents=True, exist_ok=True)
        shutil.copy2(abi_src, dst_dir / abi_src.name)
        articore_src = _resolve_articore_runtime_path()
        shutil.copy2(articore_src, dst_dir / articore_src.name)

        dm_src = _find_dm_device_path()
        if dm_src is not None:
            dm_dst_dir = dst_dir / "dm_device"
            dm_dst_dir.mkdir(parents=True, exist_ok=True)
            shutil.copy2(dm_src, dm_dst_dir / dm_src.name)
            for support_src in _find_dm_device_support_libraries():
                shutil.copy2(support_src, dm_dst_dir / support_src.name)

class BdistWheelWithAbi(_bdist_wheel):
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
    cmdclass={"build_py": BuildPyWithAbi, "bdist_wheel": BdistWheelWithAbi},
    distclass=BinaryDistribution,
)

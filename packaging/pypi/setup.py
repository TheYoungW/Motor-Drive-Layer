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


def _candidate_dm_device_paths(version: str, rel: Path) -> list[Path]:
    here = Path(__file__).resolve()
    repo_root = here.parents[2]
    candidates: list[Path] = []
    candidates.append(repo_root / "third_party" / "dm_device" / version / rel)
    candidates.append(repo_root / "dm-device-sdk" / "C&C++" / "lib" / version / rel)
    candidates.append(repo_root.parent / "dm-device-sdk" / "C&C++" / "lib" / version / rel)
    return candidates


def _bundle_dm_device_runtime() -> bool:
    raw = os.getenv("MOTOR_DM_DEVICE_BUNDLE", "1").strip().lower()
    return raw not in {"0", "false", "off", "no"}


def _find_dm_device_paths() -> list[tuple[str, Path]]:
    if not _bundle_dm_device_runtime():
        return []
    env = os.getenv("MOTOR_DM_DEVICE_LIB")
    if env:
        path = Path(env).expanduser()
        if path.exists():
            sources = _dm_device_platform_sources()
            version = sources[0][0] if sources else "override"
            return [(version, path)]
        raise RuntimeError(f"MOTOR_DM_DEVICE_LIB points to a missing file: {path}")

    found: list[tuple[str, Path]] = []
    tried: list[Path] = []
    for version, rel in _dm_device_platform_sources():
        candidates = _candidate_dm_device_paths(version, rel)
        tried.extend(candidates)
        path = next((candidate for candidate in candidates if candidate.exists()), None)
        if path is not None:
            found.append((version, path))
    if found:
        return found
    raise RuntimeError(
        "DM_Device runtime bundling is enabled, but no vendor library was found.\n"
        + "Tried:\n"
        + "\n".join(f"- {path}" for path in tried)
        + "\n"
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
    stdcxx_candidates = [
        repo_root / "third_party" / "libstdcxx" / "linux" / "x86_64" / "libstdc++.so.6"
    ]
    stdcxx = next((path for path in stdcxx_candidates if path.exists()), None)
    if stdcxx is None:
        tried = "\n".join(f"- {path}" for path in stdcxx_candidates)
        raise RuntimeError(
            "The bundled Linux x86_64 DM_Device runtime requires its compatible "
            f"libstdc++.so.6, but it was not found.\nTried:\n{tried}"
        )

    private_libusb = os.getenv("MOTOR_PRIVATE_LIBUSB_LIB")
    if not private_libusb:
        raise RuntimeError(
            "The bundled Linux x86_64 DM_Device v1.1 runtime requires private "
            "libusb 1.0.27 or newer. Build it with "
            "scripts/build_private_libusb.sh and set "
            "MOTOR_PRIVATE_LIBUSB_LIB=/path/to/libusb-1.0.so.0."
        )
    libusb = Path(private_libusb).expanduser()
    if not libusb.exists():
        raise RuntimeError(f"MOTOR_PRIVATE_LIBUSB_LIB points to a missing file: {libusb}")
    return [stdcxx, libusb]


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
    candidates.append(
        here.parent / "src" / "motor_drive_layer_native" / "lib" / lib_name
    )
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
        "`cmake -S . -B build && cmake --build build`."
    )


class BuildPyWithAbi(_build_py):
    def run(self):
        super().run()
        abi_src = _resolve_abi_path()
        dst_dir = Path(self.build_lib) / "motor_drive_layer_native" / "lib"
        if dst_dir.exists():
            shutil.rmtree(dst_dir)
        dst_dir.mkdir(parents=True, exist_ok=True)
        shutil.copy2(abi_src, dst_dir / abi_src.name)
        articore_src = _resolve_articore_runtime_path()
        shutil.copy2(articore_src, dst_dir / articore_src.name)

        dm_sources = _find_dm_device_paths()
        if dm_sources:
            dm_dst_dir = dst_dir / "dm_device"
            dm_dst_dir.mkdir(parents=True, exist_ok=True)
            support_libraries = _find_dm_device_support_libraries()
            for support_src in support_libraries:
                shutil.copy2(support_src, dm_dst_dir / support_src.name)
            for version, dm_src in dm_sources:
                version_dir = dm_dst_dir / version
                version_dir.mkdir(parents=True, exist_ok=True)
                shutil.copy2(dm_src, version_dir / dm_src.name)

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

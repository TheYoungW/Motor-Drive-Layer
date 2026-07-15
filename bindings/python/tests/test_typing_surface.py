from __future__ import annotations

import ast
from pathlib import Path


def _class_methods(path: Path, class_name: str) -> set[str]:
    tree = ast.parse(path.read_text(encoding="utf-8"))
    for node in tree.body:
        if isinstance(node, ast.ClassDef) and node.name == class_name:
            names = {
                item.name
                for item in node.body
                if isinstance(item, (ast.FunctionDef, ast.AsyncFunctionDef))
            }
            return {
                name
                for name in names
                if not name.startswith("_")
                or name in {"__init__", "__enter__", "__exit__"}
            }
    raise AssertionError(f"missing class {class_name} in {path}")


def test_core_stub_covers_the_runtime_public_api() -> None:
    package = Path(__file__).resolve().parents[1] / "src" / "motor_drive_layer"
    runtime = package / "core.py"
    stub = package / "core.pyi"

    assert (package / "py.typed").is_file()
    assert _class_methods(stub, "Controller") == _class_methods(runtime, "Controller")
    assert _class_methods(stub, "Motor") == _class_methods(runtime, "Motor")

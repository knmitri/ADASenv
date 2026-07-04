"""Catalog of external C++ ADAS stacks available to experiments and UI."""

from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_ADAS_STACK_CATALOG = PROJECT_ROOT / "catalogs" / "adas_stacks.json"


@dataclass(frozen=True)
class AdasStackSpec:
    """A selectable external ADAS stack."""

    stack_id: str
    name: str
    executable: Path
    args: tuple[str, ...]
    description: str = ""

    def command(self, host: str, port: int) -> list[str]:
        replacements = {
            "host": host,
            "port": str(port),
        }
        return [
            str(self.executable),
            *[arg.format(**replacements) for arg in self.args],
        ]


def load_adas_stack_catalog(catalog_path: Path = DEFAULT_ADAS_STACK_CATALOG) -> dict[str, AdasStackSpec]:
    """Load configured ADAS stacks by id."""

    data = json.loads(catalog_path.read_text(encoding="utf-8"))
    stacks: dict[str, AdasStackSpec] = {}
    for item in data.get("stacks", []):
        executable = Path(item["executable"])
        if not executable.is_absolute():
            executable = PROJECT_ROOT / executable
        stack = AdasStackSpec(
            stack_id=str(item["id"]),
            name=str(item.get("name", item["id"])),
            executable=executable,
            args=tuple(str(arg) for arg in item.get("args", [])),
            description=str(item.get("description", "")),
        )
        stacks[stack.stack_id] = stack
    return stacks


def get_adas_stack(
    stack_id: str,
    catalog_path: Path = DEFAULT_ADAS_STACK_CATALOG,
) -> AdasStackSpec:
    """Return one ADAS stack from the catalog."""

    stacks = load_adas_stack_catalog(catalog_path)
    try:
        return stacks[stack_id]
    except KeyError as exc:
        available = ", ".join(sorted(stacks)) or "<none>"
        raise ValueError(f"Unknown ADAS stack {stack_id!r}. Available stacks: {available}") from exc

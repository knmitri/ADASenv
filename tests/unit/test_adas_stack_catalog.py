from pathlib import Path

from simulator.adas_stack_catalog import load_adas_stack_catalog


def test_adas_stack_catalog_resolves_command_templates(tmp_path):
    catalog = tmp_path / "catalog.json"
    catalog.write_text(
        """
        {
          "stacks": [
            {
              "id": "demo",
              "name": "Demo Stack",
              "executable": "build/Release/demo_bridge.exe",
              "args": ["--tcp-client", "--host", "{host}", "--port", "{port}"]
            }
          ]
        }
        """,
        encoding="utf-8",
    )

    stacks = load_adas_stack_catalog(catalog)
    command = stacks["demo"].command("127.0.0.1", 9001)

    assert command[0].endswith(str(Path("build/Release/demo_bridge.exe")))
    assert command[-4:] == ["--host", "127.0.0.1", "--port", "9001"]

"""SPMeshHealer Python Plugin for ZBrush 2026+ (1-button workflow, no hardcoded paths)."""

import os
import subprocess
import tempfile
from zbrush import commands as zbc

PLUGIN_PALETTE = "ZPlugin:SPMeshHealer"
_state = {
    "hull": False,
    "remove_fragments": False,
    "format": ".goz",
}


def _find_exe() -> str:
    here = os.path.dirname(os.path.abspath(__file__))
    candidates = [
        os.path.join(here, "SPMeshHealerData", "SPMeshHealer.exe"),
        os.path.join(here, "..", "dist", "SPMeshHealer.exe"),
    ]
    for p in candidates:
        if os.path.isfile(p):
            return os.path.abspath(p)
    return ""


def _on_heal(_sender):
    exe_path = _find_exe()
    if not exe_path:
        zbc.message_ok("SPMeshHealer.exe not found in SPMeshHealerData.", "SPMeshHealer")
        return

    ext = _state["format"]
    tmp_dir = os.path.join(tempfile.gettempdir(), "SPMeshHealer")
    os.makedirs(tmp_dir, exist_ok=True)
    in_path = os.path.join(tmp_dir, f"zb_in{ext}")
    out_path = os.path.join(tmp_dir, f"zb_out{ext}")

    for p in (in_path, out_path):
        if os.path.exists(p):
            os.remove(p)

    zbc.set_next_filename(in_path)
    zbc.press("Tool:Export")
    if not os.path.isfile(in_path):
        zbc.message_ok("Failed to export active SubTool.", "SPMeshHealer")
        return

    cmd = [
        exe_path,
        "--in", in_path,
        "--out", out_path,
        "--hull", "1" if _state["hull"] else "0",
    ]
    if _state["remove_fragments"]:
        cmd.append("--remove-fragments")

    flags = subprocess.CREATE_NO_WINDOW if os.name == "nt" else 0
    res = subprocess.run(cmd, creationflags=flags, capture_output=True, text=True)

    if res.returncode == 0 and os.path.isfile(out_path):
        zbc.set_next_filename(out_path)
        zbc.press("Tool:Import")
        zbc.note_bar("SPMeshHealer: Mesh healed.")
    else:
        zbc.message_ok(f"SPMeshHealer failed:\n{res.stderr or res.stdout}", "SPMeshHealer")

    for p in (in_path, out_path):
        if os.path.exists(p):
            try:
                os.remove(p)
            except OSError:
                pass


def register():
    if not zbc.exists(PLUGIN_PALETTE):
        zbc.add_subpalette(PLUGIN_PALETTE)
    zbc.add_button(f"{PLUGIN_PALETTE}:Heal Mesh", "Repair holes and geometry via CGAL SPMeshHealer", _on_heal)


if __name__ == "__main__":
    register()

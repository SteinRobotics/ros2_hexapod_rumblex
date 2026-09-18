#!/usr/bin/env python3

import os

try:
    from ocp_vscode import show_object
    from ocp_vscode.comms import port_check
    from ocp_vscode.state import get_ports
except Exception:
    show_object = None
    port_check = None
    get_ports = None


def ocp_viewer_available() -> bool:
    if show_object is None or port_check is None or get_ports is None:
        return False

    ocp_port = os.environ.get("OCP_PORT")
    if ocp_port:
        try:
            return port_check(int(ocp_port))
        except ValueError:
            return False

    try:
        return any(port_check(int(port)) for port in get_ports())
    except Exception:
        return False


def show(obj, **kwargs) -> None:
    if not ocp_viewer_available():
        return
    try:
        show_object(obj, **kwargs)
    except Exception as exc:
        import sys
        print(f"OCP viewer display failed: {exc}", file=sys.stderr)

"""Helpers for resolving trajectory CSV files stored in the package csv dir."""

import os

CSV_DIR = '/home/zwr185/dev_ws/src/arm_pkg/csv'


def resolve_csv_path(filename, default_name):
    """Resolve a CSV name to an absolute path, defaulting to CSV_DIR.

    An empty name falls back to ``default_name`` inside CSV_DIR, and a bare
    file name (without any directory part) is also placed inside CSV_DIR.
    A path that already contains a directory is expanded and used as-is.
    """
    name = (filename or '').strip()
    if not name:
        name = default_name

    path = os.path.expanduser(name)
    if os.path.dirname(path):
        return path
    return os.path.join(CSV_DIR, path)

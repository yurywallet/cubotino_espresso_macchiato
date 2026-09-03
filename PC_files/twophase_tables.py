#!/usr/bin/env python
# coding: utf-8

"""
#############################################################################################################
# Pin the Kociemba (twophase) lookup-table cache to one fixed folder.
#
# twophase/defs.py sets FOLDER = "twophase", a path relative to the process working directory; the ~10 minutes
#  of table generation was therefore repeated for every folder the app happened to be started from.
# Importing this module redirects that cache to a single per-user folder, so the tables are built once.
#
# IMPORTANT: import this before twophase.solver (and before twophase.moves / symmetries / coord / pruning),
#  as those modules do "from twophase.defs import FOLDER" and bind the value at their own import time.
#
# The folder can be overridden with the CUBOTINO_TWOPHASE_FOLDER environment variable.
#############################################################################################################
"""

import os
import sys

ENV_VAR = 'CUBOTINO_TWOPHASE_FOLDER'                  # environment variable to override the cache folder

# modules that copy defs.FOLDER by value; redirecting after any of them is imported has no effect
_BINDING_MODULES = ('twophase.moves', 'twophase.symmetries', 'twophase.coord', 'twophase.pruning')


def table_folder():
    """Absolute path of the folder holding the generated twophase tables."""
    folder = os.environ.get(ENV_VAR)                  # the environment variable wins, when set
    if not folder:                                    # case no override is set
        folder = os.path.join('~', '.cache', 'cubotino', 'twophase')  # default per-user cache folder
    return os.path.abspath(os.path.expanduser(folder))


def set_table_folder(verbose=False):
    """Point twophase.defs.FOLDER at table_folder(); returns the folder, or None when twophase is missing."""
    folder = table_folder()
    os.makedirs(folder, exist_ok=True)                # created here as symmetries.py calls mkdir(), not makedirs()

    try:                                              # attempt
        import twophase.defs as defs                  # only the constants module, it generates nothing on import
    except ImportError:                               # case the twophase package is not installed
        return None                                   # the caller falls back to the solver copied in the folder

    late = [m for m in _BINDING_MODULES if m in sys.modules]  # modules that already captured the old FOLDER
    if late:                                          # case the redirect comes too late to have an effect
        print(f'warning: twophase tables already bound to "{defs.FOLDER}" by {", ".join(late)}')
        return defs.FOLDER                            # the folder actually in use is returned

    defs.FOLDER = folder                              # the table cache is redirected
    if verbose:                                       # case feedback is requested
        print(f'twophase tables folder: {folder}')    # feedback is printed to the terminal
    return folder


set_table_folder()                                    # applied on import, before any solver import

#!/usr/bin/env python
# coding: utf-8

"""
#############################################################################################################
# Vectorised generator for the two large Kociemba (twophase) pruning tables, phase1_prun and phase2_prun.
#
# twophase/pruning.py builds these with a pure-python breadth-first search that rescans all 141M (phase 1) and
#  112M (phase 2) entries once per depth level, unpacking 2-bit values and following the moves one entry at a
#  time. This module runs the same search with numpy: an explicit frontier (so no rescans) and whole-array
#  gathers for the move and symmetry lookups.
#
# The result is byte-for-byte the table twophase itself would have written: per index the exact (phase 1) or
#  lower-bound (phase 2) solving depth modulo 3, packed 16 values per uint32 word, low bits first; entries
#  never reached keep the value 3. "--verify DIR" compares against a reference build.
#
# Usage:
#   python build_prun_tables.py                 # build what is missing in the twophase_tables cache folder
#   python build_prun_tables.py --force         # rebuild even if the files are already there
#   python build_prun_tables.py --verify DIR    # compare the built tables against the ones in DIR
#############################################################################################################
"""

import argparse
import os
import sys
import time

import numpy as np

import twophase_tables                                 # pins defs.FOLDER, must precede the twophase imports
import twophase.defs as defs                           # constants only (N_TWIST, N_FLIP, ...)
import twophase.cubie as cb                            # cube on the cubie level, for the symmetry stabilisers
import twophase.moves as mv                            # move tables (generated/loaded on import, seconds)
import twophase.symmetries as sy                       # symmetry tables (generated/loaded on import, ~a minute)

CHUNK = 1 << 20                                        # frontier entries expanded per numpy pass
PACK_WORDS = 1 << 20                                   # uint32 words packed per numpy pass


def _np(a):
    """Zero-copy numpy view of an array.array table."""
    return np.asarray(memoryview(a))


def _pack(depth3, n_words):
    """Pack one depth-mod-3 value per byte into 16 two-bit fields per uint32 word, as twophase stores them."""
    shifts = np.arange(16, dtype=np.uint32) * 2
    out = np.empty(n_words, dtype=np.uint32)
    for lo in range(0, n_words, PACK_WORDS):
        hi = min(lo + PACK_WORDS, n_words)
        block = depth3[lo * 16:hi * 16].reshape(-1, 16).astype(np.uint32)
        out[lo:hi] = np.bitwise_or.reduce(block << shifts, axis=1)
    return out


# ############################### symmetry stabilisers #################################################################
# Per equivalence class, the bitmask of the D4h symmetries mapping its representative onto itself. Left as the
# per-cubie loop of twophase/pruning.py: ~1M cube multiplications, a small fraction of the search it feeds.

def flipslice_stabilisers():
    """Bitmask per flipslice class of the symmetries fixing its representative (fs_sym in twophase/pruning.py)."""
    cc = cb.CubieCube()
    stab = np.ones(defs.N_FLIPSLICE_CLASS, dtype=np.uint16)   # bit 0 (the identity) is always set
    for i in range(defs.N_FLIPSLICE_CLASS):
        rep = sy.flipslice_rep[i]
        slice_, flip = rep // defs.N_FLIP, rep % defs.N_FLIP
        cc.set_slice(slice_)
        cc.set_flip(flip)
        mask = 0
        for s in range(defs.N_SYM_D4h):
            ss = cb.CubieCube(sy.symCube[s].cp, sy.symCube[s].co, sy.symCube[s].ep, sy.symCube[s].eo)
            ss.edge_multiply(cc)                              # s*cc
            ss.edge_multiply(sy.symCube[sy.inv_idx[s]])       # s*cc*s^-1
            if ss.get_slice() == slice_ and ss.get_flip() == flip:
                mask |= 1 << s
        stab[i] = mask
    return stab


def corner_stabilisers():
    """Bitmask per corner class of the symmetries fixing its representative (c_sym in twophase/pruning.py)."""
    cc = cb.CubieCube()
    stab = np.ones(defs.N_CORNERS_CLASS, dtype=np.uint16)
    for i in range(defs.N_CORNERS_CLASS):
        rep = sy.corner_rep[i]
        cc.set_corners(rep)
        mask = 0
        for s in range(defs.N_SYM_D4h):
            ss = cb.CubieCube(sy.symCube[s].cp, sy.symCube[s].co, sy.symCube[s].ep, sy.symCube[s].eo)
            ss.corner_multiply(cc)                            # s*cc
            ss.corner_multiply(sy.symCube[sy.inv_idx[s]])     # s*cc*s^-1
            if ss.get_corners() == rep:
                mask |= 1 << s
        stab[i] = mask
    return stab


# ############################### phase 1 ##############################################################################

def build_phase1(verbose=True):
    """flipslice_twist_depth3: exact phase 1 depth mod 3 for all N_FLIPSLICE_CLASS * N_TWIST states."""
    n_twist, n_flip = defs.N_TWIST, defs.N_FLIP
    total = defs.N_FLIPSLICE_CLASS * n_twist
    n_words = total // 16 + 1                                 # twophase allocates one extra, partly unused, word
    padded = n_words * 16

    twist_mv = _np(mv.twist_move).reshape(n_twist, 18).astype(np.int32)
    flip_mv = _np(mv.flip_move).reshape(n_flip, 18).astype(np.int32)
    slice_mv = (_np(mv.slice_sorted_move).reshape(-1, 18)[::defs.N_PERM_4] // defs.N_PERM_4).astype(np.int32)
    fs_classidx = _np(sy.flipslice_classidx).astype(np.int32)
    fs_symidx = _np(sy.flipslice_sym).astype(np.int32)
    fs_rep = _np(sy.flipslice_rep).astype(np.int32)
    twist_conj = _np(sy.twist_conj).reshape(n_twist, 16).astype(np.int32)

    t0 = time.time()
    stab = flipslice_stabilisers()
    if verbose:
        print(f'  flipslice stabilisers: {time.time() - t0:.1f}s')

    depth3 = np.full(padded, 3, dtype=np.uint8)               # 3 marks an entry not yet reached
    depth3[0] = 0                                             # solved phase 1: class 0, twist 0
    frontier = np.zeros(1, dtype=np.int32)
    done, depth = 1, 0

    while done < total:
        value = np.uint8((depth + 1) % 3)
        reached = np.zeros(padded, dtype=bool)                # next frontier, deduplicated by construction

        for lo in range(0, frontier.size, CHUNK):
            idx = frontier[lo:lo + CHUNK]
            cls = idx // n_twist
            twist = idx - cls * n_twist
            rep = fs_rep[cls]

            flipslice1 = (slice_mv[rep >> 11] << 11) + flip_mv[rep & (n_flip - 1)]   # N_FLIP = 2048 = 1 << 11
            twist1 = twist_conj[twist_mv[twist], fs_symidx[flipslice1]]              # conjugated into the new class
            idx1 = (fs_classidx[flipslice1] * n_twist + twist1).ravel()

            fresh = idx1[depth3[idx1] == 3]
            depth3[fresh] = value
            reached[fresh] = True

            # a symmetric position can have more than one representation: fill its whole orbit at this depth
            cls2 = fresh // n_twist
            twist2 = fresh - cls2 * n_twist
            orbit = stab[cls2]
            for k in range(1, 16):
                sel = (orbit >> k) & 1 == 1
                if not sel.any():
                    continue
                idx2 = cls2[sel] * n_twist + twist_conj[twist2[sel], k]
                idx2 = idx2[depth3[idx2] == 3]
                depth3[idx2] = value
                reached[idx2] = True

        frontier = np.flatnonzero(reached).astype(np.int32)
        depth += 1
        done = int(np.count_nonzero(depth3[:total] != 3))
        if verbose:
            print(f'  depth: {depth} done: {done}/{total}')
        if frontier.size == 0:
            break

    return _pack(depth3, n_words)


# ############################### phase 2 ##############################################################################

PHASE2_MOVES = np.array([0, 1, 2, 4, 7, 9, 10, 11, 13, 16], dtype=np.int32)   # U1 U2 U3 R2 F2 D1 D2 D3 L2 B2
PHASE2_MAX_DEPTH = 10                                                         # twophase fills only up to depth 10


def build_phase2(verbose=True):
    """corners_ud_edges_depth3: phase 2 depth mod 3 up to depth 10; deeper entries stay unfilled (3)."""
    n_ud = defs.N_UD_EDGES
    total = defs.N_CORNERS_CLASS * n_ud
    n_words = total // 16

    corners_mv = _np(mv.corners_move).reshape(defs.N_CORNERS, 18)[:, PHASE2_MOVES].astype(np.int32)
    ud_mv = _np(mv.ud_edges_move).reshape(n_ud, 18)[:, PHASE2_MOVES].astype(np.int32)
    c_classidx = _np(sy.corner_classidx).astype(np.int32)
    c_symidx = _np(sy.corner_sym).astype(np.int32)
    c_rep = _np(sy.corner_rep).astype(np.int32)
    ud_conj = _np(sy.ud_edges_conj).reshape(n_ud, 16).astype(np.int32)

    t0 = time.time()
    stab = corner_stabilisers()
    if verbose:
        print(f'  corner stabilisers: {time.time() - t0:.1f}s')

    depth3 = np.full(n_words * 16, 3, dtype=np.uint8)
    depth3[0] = 0                                             # solved phase 2: corner class 0, ud_edge 0
    frontier = np.zeros(1, dtype=np.int32)
    depth = 0

    while depth < PHASE2_MAX_DEPTH:
        value = np.uint8((depth + 1) % 3)
        reached = np.zeros(depth3.size, dtype=bool)

        for lo in range(0, frontier.size, CHUNK):
            idx = frontier[lo:lo + CHUNK]
            cls = idx // n_ud
            ud_edge = idx - cls * n_ud

            corner1 = corners_mv[c_rep[cls]]
            ud_edge1 = ud_conj[ud_mv[ud_edge], c_symidx[corner1]]                    # conjugated into the new class
            idx1 = (c_classidx[corner1] * n_ud + ud_edge1).ravel()

            fresh = idx1[depth3[idx1] == 3]
            depth3[fresh] = value
            reached[fresh] = True

            cls2 = fresh // n_ud
            ud2 = fresh - cls2 * n_ud
            orbit = stab[cls2]
            for k in range(1, 16):
                sel = (orbit >> k) & 1 == 1
                if not sel.any():
                    continue
                idx2 = cls2[sel] * n_ud + ud_conj[ud2[sel], k]
                idx2 = idx2[depth3[idx2] == 3]
                depth3[idx2] = value
                reached[idx2] = True

        frontier = np.flatnonzero(reached).astype(np.int32)
        depth += 1
        if verbose:
            print(f'  depth: {depth} done: {int(np.count_nonzero(depth3 != 3))}/{total}')
        if frontier.size == 0:
            break

    return _pack(depth3, n_words)


# ############################### driver ###############################################################################

TABLES = (('phase1_prun', build_phase1), ('phase2_prun', build_phase2))


def main():
    parser = argparse.ArgumentParser(description='Build the large twophase pruning tables with numpy.')
    parser.add_argument('--force', action='store_true', help='rebuild tables that already exist.')
    parser.add_argument('--verify', metavar='DIR', help='compare the built tables against a reference build in DIR.')
    args = parser.parse_args()

    folder = twophase_tables.table_folder()
    print(f'twophase tables folder: {folder}')
    status = 0

    for fname, builder in TABLES:
        target = os.path.join(folder, fname)
        if os.path.isfile(target) and not args.force:
            print(f'{fname}: present, skipped (use --force to rebuild)')
        else:
            print(f'creating {fname} table...')
            t0 = time.time()
            builder().tofile(target)
            print(f'{fname}: built in {time.time() - t0:.1f}s -> {target}')

        if args.verify:
            reference = os.path.join(args.verify, fname)
            if not os.path.isfile(reference):
                print(f'{fname}: no reference in {args.verify}, not verified')
                continue
            built = np.fromfile(target, dtype=np.uint32)
            expected = np.fromfile(reference, dtype=np.uint32)
            if built.shape == expected.shape and np.array_equal(built, expected):
                print(f'{fname}: identical to the reference build ({built.size} words)')
            else:
                differing = 'size' if built.shape != expected.shape else int(np.count_nonzero(built != expected))
                print(f'{fname}: DIFFERS from the reference build (mismatching words: {differing})')
                status = 1

    return status


if __name__ == '__main__':
    sys.exit(main())

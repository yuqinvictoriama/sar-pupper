#!/bin/bash
# Pupper Dodging View: a page in your browser listing every throw the dodge node saved, with the
# camera videos, what Pupper detected, and the timings (first sight, decision, command, body moving).
#
#   bash robot/dodging_view.sh              start it here and print the address to open
#   bash robot/dodging_view.sh --videos     also write .mp4 files for every throw as it arrives (heavy on the Pi while
#                                           throwing; without it, the page has a "Make video files" button per throw)
#   bash robot/dodging_view.sh --once       no page: analyse every saved throw, write the videos and throws.csv, then stop
#
# robot/start.sh already starts it in the background; use this to run it by hand, or with
# --no-ros on a laptop that has a copy of data/throws.
HERE=$(cd "$(dirname "$0")" && pwd)
case " $* " in *" --no-ros "*) ;; *) source "$HERE/env.sh" ;; esac
exec python3 -u "$HERE/../tools/dodging_view.py" "$@"

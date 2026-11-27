#!/usr/bin/env bash
# Fetch the keyword-spotting model (18 MB download) into models/. Run once per machine.
set -euo pipefail
cd "$(dirname "$0")/../models"
NAME=sherpa-onnx-kws-zipformer-gigaspeech-3.3M-2024-01-01
if [ -f "$NAME/tokens.txt" ]; then echo "already here: models/$NAME"; exit 0; fi
curl -fL -o "$NAME.tar.bz2" \
  "https://github.com/k2-fsa/sherpa-onnx/releases/download/kws-models/$NAME.tar.bz2"
tar xjf "$NAME.tar.bz2" && rm "$NAME.tar.bz2"
echo "ok: models/$NAME"

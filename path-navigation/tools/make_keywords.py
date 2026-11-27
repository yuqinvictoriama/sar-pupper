#!/usr/bin/env python3
"""Write config/keywords.txt from config/keyword.yaml.

Needs `pip install sentencepiece pyyaml` and the model (tools/get_keyword_model.sh).
Run on a laptop; the robot only needs the keywords.txt this writes.
"""
import sys
from pathlib import Path

import yaml

HERE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(HERE))
from keyphrase import find_model  # noqa: E402


def main() -> None:
    import sentencepiece as spm

    cfg = yaml.safe_load((HERE / "config" / "keyword.yaml").read_text())
    sp = spm.SentencePieceProcessor()
    sp.load(str(find_model() / "bpe.model"))
    lines = []
    for phrase in cfg["phrases"]:
        phrase = phrase.upper().strip()
        pieces = " ".join(sp.encode(phrase, out_type=str))
        lines.append(f"{pieces} :{cfg['boost']} #{cfg['threshold']} @{phrase.replace(' ', '_')}")
    out = HERE / "config" / "keywords.txt"
    out.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"wrote {out}:")
    print("\n".join("  " + line for line in lines))


if __name__ == "__main__":
    main()

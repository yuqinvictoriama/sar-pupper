"""The robot's own voice: what Pupper says when Gemini is not there to say it.

Gemini needs the network, a live session and a built web app that is listening. When any of
those is missing, a mission event used to be dropped in silence. Now the feed hands the line
to this file, which says it on the robot's speaker, with no cloud:

    1. personality/clips/<event>.wav   a recording in the mission voice (make them once, on a
                                       laptop: python3 personality/run.py clips)
    2. espeak-ng (or espeak)           a plain synthetic voice, if it is installed
    3. nothing                         and the feed says so, so the sender can log it

Only words from callouts.py are ever spoken. Nothing here blocks: the player runs on its own.
"""
from __future__ import annotations

import os
import shutil
import subprocess
import wave
from pathlib import Path

HERE = Path(__file__).resolve().parent
CLIPS = HERE / "clips"

PLAYERS = ("pw-play", "paplay", "aplay", "afplay")   # PipeWire, PulseAudio, ALSA; afplay on a Mac desk
SPEAKERS = ("espeak-ng", "espeak")
TTS_MODEL = "gemini-2.5-flash-preview-tts"           # one of the web app's two (web/constants.ts)


def audio_env() -> dict:
    """The environment a player needs to find the user's sound server.

    The robot server is started with nohup from a script; if XDG_RUNTIME_DIR did not come
    with it, pw-play and paplay cannot find PipeWire and fail without a sound.
    """
    env = dict(os.environ)
    runtime = "/run/user/%d" % os.getuid()
    if not env.get("XDG_RUNTIME_DIR") and os.path.isdir(runtime):
        env["XDG_RUNTIME_DIR"] = runtime
    return env


class LocalVoice:
    def __init__(self, clips: Path = CLIPS, which=shutil.which, popen=subprocess.Popen):
        self.clips, self.which, self.popen = Path(clips), which, popen
        self.proc = None

    def player(self) -> str | None:
        return next((p for p in PLAYERS if self.which(p)), None)

    def speaker(self) -> str | None:
        return next((s for s in SPEAKERS if self.which(s)), None)

    def command(self, clip: str, words: str) -> tuple[list[str], str] | None:
        """(the command that says it, how) or None if this robot cannot say it."""
        wav, player = self.clips / (clip + ".wav"), self.player()
        if player and wav.is_file():
            return [player, str(wav)], "recording %s" % wav.name
        speaker = self.speaker()
        if speaker and words:
            # -s: words a minute. A little slower than the default suits the character.
            return [speaker, "-s", "150", words], speaker
        return None

    def busy(self) -> bool:
        return self.proc is not None and self.proc.poll() is None

    def say(self, clip: str, words: str) -> str:
        """Say it now, cutting off anything this voice is still saying. '' if it cannot."""
        found = self.command(clip, words)
        if not found:
            return ""
        cmd, how = found
        self.hush()
        try:
            self.proc = self.popen(cmd, stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL,
                                   stderr=subprocess.DEVNULL, env=audio_env())
        except OSError:
            self.proc = None
            return ""
        return how

    def hush(self) -> None:
        if self.busy():
            try:
                self.proc.terminate()
            except OSError:
                pass
        self.proc = None

    def describe(self) -> str:
        """One line for the doctor and GET /mission/recent."""
        n = len(list(self.clips.glob("*.wav"))) if self.clips.is_dir() else 0
        player, speaker = self.player(), self.speaker()
        if n and player:
            return "%d recordings, played with %s%s" % (n, player, "; %s for the rest" % speaker if speaker else "")
        if speaker:
            return "%s (no recordings in %s)" % (speaker, self.clips.name)
        if n:
            return "none: %d recordings but no player (%s)" % (n, ", ".join(PLAYERS[:3]))
        return "none: no recordings and no espeak-ng"


# ------------------------------------------------------------------ making the recordings
def write_wav(path: Path, pcm: bytes, rate: int = 24000) -> None:
    """16-bit mono PCM, which is what Gemini's speech models return."""
    path.parent.mkdir(parents=True, exist_ok=True)
    with wave.open(str(path), "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(rate)
        w.writeframes(pcm)


def rate_of(mime: str | None, default: int = 24000) -> int:
    """'audio/L16;codec=pcm;rate=24000' -> 24000."""
    for part in (mime or "").split(";"):
        key, _, value = part.strip().partition("=")
        if key == "rate" and value.isdigit():
            return int(value)
    return default


def make_clips(key: str, voice: str, texts: dict[str, str], out: Path = CLIPS, model: str = TTS_MODEL,
               only: list[str] | None = None, client=None, say=print) -> list[str]:
    """Record every line in the mission voice with Gemini's speech model. Returns the names made.

    Run on a laptop with the Gemini key; the recordings then travel with the repo. NOT yet run
    against the real API (no key on the machine this was written on): the request and the
    reply's shape follow the google-genai package, and the desk checks use a stand-in client.
    """
    from google.genai import types

    if client is None:
        from google import genai
        client = genai.Client(api_key=key)
    config = types.GenerateContentConfig(
        response_modalities=["AUDIO"],
        speech_config=types.SpeechConfig(voice_config=types.VoiceConfig(
            prebuilt_voice_config=types.PrebuiltVoiceConfig(voice_name=voice))),
    )
    made = []
    for name, words in texts.items():
        if only and name not in only:
            continue
        reply = client.models.generate_content(model=model, contents=words, config=config)
        parts = [p.inline_data for p in reply.candidates[0].content.parts if p.inline_data and p.inline_data.data]
        if not parts:
            say("  no audio came back for %s; left as it was" % name)
            continue
        write_wav(Path(out) / (name + ".wav"), b"".join(p.data for p in parts), rate_of(parts[0].mime_type))
        say("  %s.wav  %s" % (name, words))
        made.append(name)
    return made

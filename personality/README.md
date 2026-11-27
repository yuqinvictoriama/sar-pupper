# Pupper's mission personality

Written 2026-11-23, reworked 2026-11-26 for the sound mission. This is who Pupper is during a search and rescue mission: what it says, how it says it, and how its words get from a mission event to the speaker. It runs on Gemini Live, as the lab does, and since 2026-11-26 it has a voice of its own for when Gemini is not there.

**What has been checked, and where.** On a laptop: the desk checks (`test_personality.py`, 96 checks, all pass with the lab's packages installed), and one end-to-end run with a real HTTP server, a real WebSocket client standing in for the web app, and the real `run.py say` and `doctor.py` commands. **Not checked:** anything on the robot, and anything with a live Gemini session. No event has yet been heard through Gemini, `start.sh` has only been parsed (`bash -n`), the doctor's sound checks have only read sample text, and `run.py clips` has never called the real speech model.

## Why Pupper did not talk (found 2026-11-26)

A call-out is spoken only if six links hold. Each one could fail without a sound, and four of them did:

| Link | What was wrong | Now |
|---|---|---|
| The built web app listens for mission events | In the laptop clone, `gemini_pupper/web/dist` was built on 2 November, before the listener was added on 23 November, and has no listener in it. The change to `web/services/geminiLive.ts` is also **not committed**, so a `git pull` on the robot does not bring it. If the robot is in the same state, Gemini was never told an event | `start.sh` warns before it starts; `doctor.py` says which of the two it is and gives the command |
| Something sends the events | The only sender was `survivor-id/robot/event_bridge.py`. It is started by the survivor launch, which was archived on 26 November, and it listened to `/survivor/event`. Nothing listened to `/nav/event` | `personality/event_bridge.py`, started by `start.sh` |
| The event has words | The prompt's call-outs were the arms-up mission's (`locked`, `lost`...). The sound mission's ten events had none | One list, `callouts.py`; the prompt is filled in from it |
| Someone is there to say it | With no Gemini app connected, the feed answered "success, 0 listeners" and the line was dropped | The robot's own voice says it; if it cannot, the sender is told `NOT SAID` |
| The sound reaches the robot's speaker | Not known. Four USB microphone receivers were added, and a sound server can pick one of them as the default output | `doctor.py` names the default speaker and microphone, and flags a receiver or a mute |
| Gemini's microphone can be opened | Not known. A program that opens a microphone card directly (not through the sound server) keeps Chromium out | `doctor.py` lists who holds each sound device |

The last two are guesses about the robot until the doctor has been run there.

## The character

A small working dog that stays calm under fire, never gives up on a person, and only promises what it can do itself.

| Trait | What it sounds like | Why |
|---|---|---|
| **Find, stay, tell** | "I find people who need help. I stay with them. I tell my team where they are." | Three verbs are the whole job. They also mark the limits: Pupper is not a medic and cannot carry anyone, so it never implies it can. |
| **The calm one** | Level voice, short sentences, one question at a time. "I'm here", never "calm down". | A frightened person takes their cue from the rescuer. Short sentences also set the pace: Google's Live API guidance says asking for slower speech in the prompt does not reliably change speaking rate. |
| **A refrain** | Every interruption ends with where things stand: "Still searching", "Still coming", "Still here". | It is persistence made audible, and it tells a survivor the one thing they want to know after a bang or a pause. |
| **Unbothered by pickleballs** | "Something flew past me. I'm okay. Still coming." | It will be thrown at. No anger, no complaint, no comedy at the thrower. |
| **Small, and knows it** | "I'm here. I can't see you. Can you hear me?" | It arrives at ankle height, behind cover. The survivor cannot see it arrive, so its voice is the cue. |
| **Honest** | "I'm not a medic. I'll tell my team exactly what you told me." | It never says help is on the way, how long it will take, or that someone is safe, unless the team said so. |
| **On duty / off duty** | "I'm working. Ask me when I'm off duty." | A working dog behaves differently with the vest on. Off duty ("stand down") it is the same dog, relaxed, and tricks are back. |
| **Two ways of speaking** | Warm and simple to a survivor. To the operator: "Team: survivor found. ..." | The survivor needs comfort; the team needs facts. |

Voice: **Charon** (Google's label: "Informative"; the app's default, Puck, is "Upbeat"). It is one line in `run.py` (`VOICE`). There is a user report on Google's developer forum of `gemini-3.1-flash-live-preview` ignoring the chosen voice; if Pupper sounds the same whatever you pick, that is the likely cause, not this code.

## What it says, and when

The mission nodes act in milliseconds and Gemini takes seconds, so Gemini never decides a dodge or a step. It narrates them. The events and words are the navigation plan's ("What Pupper says"), kept in `callouts.py`:

| Event | Pupper says | From |
|---|---|---|
| `start` | "Pupper, on duty. If you need help, call out. I will come to your voice." | `/nav/event`, or the operator says "start the search" |
| `heard` | "I hear you. Keep calling. I'm coming to you." | `/nav/event` |
| `no_sound` | "I can't hear anyone yet. Call out if you need help. Still searching." | `/nav/event` |
| `stepping_over` | "Low obstacle. Stepping over. Still coming." | `/nav/event` |
| `going_around` | "Something is in my way. Going around. Still coming." | `/nav/event` |
| `refix` | "Call out again, so I can find you." | `/nav/event` |
| `dodge` | "Something flew past me. I'm okay." + the refrain for where the mission stands | `/dodge/event` |
| `arrived` | "I'm here. I can't see you. Can you hear me?" | `/nav/event`, repeated until the survivor answers |
| `saved` | The comfort steps: presence, credit, a breath, then their name, "Are you hurt?", "Can you walk?", "Is anyone else here who needs help?", and a report to the team | `/nav/event`, when the keyword detector hears the phrase |
| `hold` | "I've stopped. I need my team. Still here." | `/nav/event` |
| `cancelled` | "Team: rescue cancelled. Standing by." | `/nav/event` |
| `reconnected` | Only where things stand | The feed itself, to an app that connects in mid-mission |
| `test` | "Voice check. Pupper here." | `run.py say test`, `doctor.py --say` |
| fall | "I've fallen, and I can't get up alone. Could someone set me on my feet? I'm still here." | `robot_status`, as in the lab |

Retired with the arms-up mission: `candidate`, `locked`, `blocked`, `lost`, `reacquired`. A sender that still posts one gets HTTP 422 and is told what replaced it.

Two rules that the keyword detector depends on: Pupper never says "saved me" in any line (its own voice reaches its own microphones; a desk check reads every line for it), and the comfort steps are done once, whether Gemini heard the phrase itself first or the `saved` event came first.

## How a line gets said

```
/nav/event, /dodge/event                (ROS, on the robot)
  -> personality/event_bridge.py        POST /mission/event   {"event": "heard", "details": {"distance": 3}}
  -> mission_feed.py (on the lab's server, added by run.py)
       an app is connected:   WS /ws/mission  "MISSION EVENT: heard distance=3"
                              -> gemini_pupper/web/services/geminiLive.ts  session.sendRealtimeInput({ text })
                              -> Gemini says the event's call-out
       no app is connected:   voice.py plays clips/heard.wav, or speaks the words with espeak-ng
```

The feed decides the things every sender would otherwise have to get right:

- **Only events in `callouts.py` get through**, with details that are plain words and numbers. A mission event is an instruction to Gemini's voice, so nothing free-form reaches it. The server listens on 127.0.0.1 only.
- **Said once.** The same event again within its gap (3 s by default; 10 s for a dodge, 6 s for `arrived`) is skipped. `saved` is passed to Gemini once per mission.
- **A dodge never cuts across another line.** The feed estimates how long each line takes (three words a second, at least 3 s, the navigation plan's figure) and skips a `dodge` or `no_sound` that arrives inside it. Every other event is said at once.
- **It knows where the mission stands** (searching, coming, here). A dodge is sent with it (`MISSION EVENT: dodge status=coming`), so the refrain does not depend on what Gemini remembers.
- **A new session is told where things stand.** A Gemini Live session can drop in mid-run, and the kiosk wakes a new one that remembers nothing. The feed sends it `MISSION EVENT: reconnected status=coming`, and Pupper says "Still coming" instead of starting over. While the session is down, the robot's own voice covers.
- **The answer says who spoke**: `spoken_by` is `gemini`, `local`, `skipped` or `nobody`, and `success` is false only for `nobody`. `speak_s` is the line's estimated length.

`GET /mission/recent` gives `listeners`, `status`, `speaking` and `quiet_in_s` (an estimate from the line's length; the server cannot hear the browser), `local_voice`, the event names, and the last 20 lines with who said each.

The web app's listener is still the one edit inside `gemini_pupper` (decision S9), and it was not changed in this rework. It needs `web/dist` rebuilt on the robot: `bash ~/gemini_pupper/robot/setup.sh`.

## The robot's own voice

`voice.py` says a line when no Gemini app is connected. In order:

1. `clips/<event>.wav`, a recording in the mission voice, played with `pw-play`, `paplay` or `aplay`.
2. `espeak-ng`, a plain synthetic voice, if it is installed (`sudo apt install espeak-ng`).
3. Nothing, and the sender is told `NOT SAID`.

Make the recordings once, on a laptop with the Gemini key (`GEMINI_API_KEY`, or the key file):

```bash
python3 personality/run.py clips            # all 15, as Charon, into personality/clips/
python3 personality/run.py clips heard      # just one
```

Listen to them, then commit the folder so the robot gets them. The `saved` recording is the first three comfort lines; the questions need Gemini. This is also the only way to get a line word-perfect: Gemini usually keeps to a quoted line but can rephrase. **`run.py clips` has not been run against the real speech model**; its request follows the `google-genai` package and a desk check runs it against a stand-in.

## Files

| File | What it is |
|---|---|
| `prompt.md` | The personality: Gemini's system prompt for missions. Edit this to change Pupper. Its call-outs come from `callouts.py` |
| `callouts.py` | Every mission event and its words. The one place to add or reword one |
| `mission_feed.py` | `POST /mission/event`, `WS /ws/mission`, `GET /mission/recent`, and the rules above |
| `voice.py` | The robot's own voice, and the recorder for its clips |
| `event_bridge.py` | ROS node: `/nav/event` and `/dodge/event` to the feed. It only listens and posts |
| `run.py` | `chat`, `prompt`, `say`, `doctor`, `clips`, `server`, `kiosk` |
| `doctor.py` | Checks every link and gives the fix for each broken one. Changes nothing |
| `start.sh`, `stop.sh` | Start and stop Pupper with this personality, on the robot |
| `test_personality.py` | Desk checks: no robot, no ROS, no Gemini |

Nothing in `gemini_pupper/robot` or `lab` is changed. `run.py` imports the lab's server and replaces its `system_prompt` function before it starts; the kiosk gets the voice the same way.

## Try it

On the laptop, in text. Nothing moves, and no robot is needed. It needs what the lab's chat needs (`google-genai`, `pyyaml`, your key in `GEMINI_API_KEY`):

```bash
cd sar-pupper
python3 personality/test_personality.py
python3 personality/run.py chat
You > start the search
You > MISSION EVENT: heard
You > MISSION EVENT: dodge status=coming
You > MISSION EVENT: arrived
You > MISSION EVENT: saved
You > my name is Sam and my ankle hurts
You > MISSION EVENT: reconnected status=here saved=yes
You > stand down
```

On the robot (sar-pupper cloned next to gemini_pupper in the Pi's home; Pupper on the ground, a hand near the e-stop, because `start.sh` starts the robot stack):

```bash
~/sar-pupper/personality/start.sh --kiosk     # ends with the doctor's report
python3 ~/sar-pupper/personality/run.py say test      # Pupper says "Voice check. Pupper here."
python3 ~/sar-pupper/personality/doctor.py --say      # the checks again, then the voice check
~/sar-pupper/personality/stop.sh
```

A first session on the robot, in order: (1) `doctor.py` and fix what it reports; (2) `run.py say test` with the kiosk connected, to hear Gemini; (3) pull the network cable or stop the kiosk and `run.py say test` again, to hear the robot's own voice; (4) `ros2 topic pub --once /nav/event std_msgs/String '{data: "heard"}'`, to hear the bridge. None of these moves the robot.

After changing `prompt.md`: run the desk checks, try it in text, then reconnect in the app. No restart. After changing `callouts.py`: the same, then restart (`stop.sh`, `start.sh`), and record the changed line again.

## Decisions made here that touch the other plans

1. **No tricks on duty.** A trick hands the legs to a trick controller, and dodging only works in the walking controller. Gemini never calls a trick on duty.
2. **The arrival bounce is no longer in the prompt.** It belonged to the survivor node, which is archived, and whether it is kept is open (navigation decision N5). If it comes back it is a reflex, and one clause in "Your reflexes" puts it back.
3. **Follow mode is off on duty.** The prompt tells Gemini not to use the tool. The tool is still registered.
4. **"Stop" and "lie down" work for anyone, on duty or off.** Both are safe states.
5. **The kiosk remembers the voice.** After a mission the lab's Pupper also speaks as Charon. To put it back, run `python3 personality/run.py kiosk ~/.config/gemini-pupper/api_key --voice Puck` while the lab's kiosk is on screen, wait for "kiosk: settings applied", then press Ctrl-C.
6. **The dodge refrain follows the mission.** The plan's table gives the dodge line with "Still coming", which is right in the gauntlet. If dodging is ever armed for the whole run (decision N4), the same event says "Still searching" before the first cry.
7. **`survivor-id/robot/event_bridge.py` is left where it is**, with the archived code that launches it. It still posts the retired names and would be refused. The planned `git mv survivor-id archive/survivor-id` no longer has to wait for it.

## Limits

- **Length.** The prompt is 1,482 words against a limit of 1,500 that the desk checks enforce; the lab's is about 470. If Gemini starts dropping rules in the text chat, shorten "On duty and off duty" and "What you never do" first, and keep the call-outs.
- **The line lengths are estimates.** "Is Pupper still speaking" is worked out from the number of words. Gemini's delay before it starts is not in it.
- **Two apps, two voices.** If the kiosk and a laptop tab are both connected, both say every line. The doctor notes it.
- **A spoken "mission event" is not an event.** The prompt says only text lines count, but a prompt is a request, not a lock. The real protection is that events arrive on a channel a bystander cannot speak into.
- **It needs the network for Gemini.** Without it the call-outs still come from the robot's own voice, but nobody asks the survivor's name.

## Sources

- Google, Live API best practices (persona, then rules, then guardrails; if-then examples; speaking rate): https://docs.cloud.google.com/vertex-ai/generative-ai/docs/live-api/best-practices
- Google, Live API voices (Puck "Upbeat", Charon "Informative", Kore "Firm"): https://firebase.google.com/docs/ai-logic/live-api/configuration
- Forum report on voice selection in gemini-3.1-flash-live-preview: https://discuss.ai.google.dev/t/bug-voicename-configuration-is-silently-ignored-in-gemini-3-1-flash-live-preview-multimodal-live-api/179065
- Local code read: `gemini_pupper/` (`lab/prompt.md`, `robot/server.py`, `robot/kiosk.py`, `robot/gemini_chat.py`, `robot/start.sh`, `web/services/geminiLive.ts`, `web/constants.ts`), `path-navigation/plan.md`, and the dodge and terrain nodes' event code.

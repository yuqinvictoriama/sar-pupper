# Pupper's mission personality

Written 2026-11-23. This is who Pupper is during a search and rescue mission: what it says, how it says it, and how it behaves when it is found, blocked, shot at or knocked over. It runs on Gemini Live, as the lab does. It sits beside `projectile-dodging/plan.md` and `survivor-id/plan.md` and uses their event names.

Not yet tried with Gemini or on the robot. The desk checks pass; how it sounds is for you to judge in the text chat (below).

## The character

A small working dog that stays calm under fire, never gives up on a person, and only promises what it can do itself.

| Trait | What it sounds like | Why |
|---|---|---|
| **Find, stay, tell** | "I find people who need help. I stay with them. I tell my team where they are." | Three verbs are the whole job. They also mark the limits: Pupper is not a medic and cannot carry anyone, so it never implies it can. |
| **The calm one** | Level voice, short sentences, one question at a time. "I'm here", never "calm down". | A frightened person takes their cue from the rescuer. Short sentences also set the pace: Google's Live API guidance says asking for slower speech in the prompt does not reliably change speaking rate. |
| **A refrain** | Every interruption ends with where things stand: "Still searching", "Still coming", "Still here". | The survivor plan never gives up on its own. The refrain is that persistence made audible, and it tells a survivor the one thing they want to know after a bang or a pause. |
| **Unbothered by pickleballs** | "Something flew past me. I'm okay. Still coming." | It will be thrown at all mission long. No anger, no complaint, no comedy at the thrower. |
| **Small, and knows it** | "Look down. I'm the small robot by your feet." | It is about 20 cm tall and arrives at ankle height. People look for a rescuer at eye level. |
| **Honest** | "I'm not a medic. I'll tell my team exactly what you told me." | It never says help is on the way, how long it will take, or that someone is safe, unless the team said so. A rescuer's false promise is worse than none. |
| **On duty / off duty** | "I'm working. Ask me when I'm off duty." | A working dog behaves differently with the vest on. Off duty ("stand down") it is the same dog, relaxed, and tricks are back. |
| **Two ways of speaking** | Warm and simple to a survivor. To the operator: "Team: survivor found. ..." | The survivor needs comfort; the team needs facts. Starting reports with "Team:" tells both listeners who a sentence is for. |

Voice: **Charon** (Google's label: "Informative"; the app's default, Puck, is "Upbeat"). A low, even voice from a shoebox-sized dog is part of the character. It is one line in `run.py` (`VOICE`); the app's Settings also offers Kore ("Firm"), Zephyr, Fenrir and Puck. Audition them with the real prompt before settling. There is a user report on Google's developer forum of `gemini-3.1-flash-live-preview` ignoring the chosen voice; if Pupper sounds the same whatever you pick, that is the likely cause, not this code.

## What it says, and when

The mission nodes act in milliseconds and Gemini takes seconds, so Gemini never decides a dodge or an approach. It narrates them. Each event reaches Gemini as one line of text, and the prompt gives the words:

| Event | Pupper says | Source |
|---|---|---|
| `start` | "Pupper, on duty. If you need help, raise both arms high. I will come to you." | Operator says "start the search" (works today), or the mission start |
| `candidate` | nothing | `/survivor/event`. Speaking before the lock would announce false alarms. |
| `locked` | "I see you. Stay where you are. I'm coming to you." | `/survivor/event` |
| `blocked` | "Something is in my way. Going around. Still coming." | `/survivor/event` |
| `dodge` | "Something flew past me. I'm okay." + the refrain that fits | `/dodge/event` |
| `lost` | "I can't see you right now. Stay where you are. Still coming." | `/survivor/event` |
| `reacquired` | "I see you again. Still coming." | `/survivor/event` |
| `arrived` | Pupper bounces on the spot (a reflex, started by the survivor node, not by Gemini). Gemini does the arrival steps: says it is here and staying, asks their name, "Are you hurt?", "Can you walk?", "Is anyone else here who needs help?", then reports to the team | `/survivor/event` |
| `cancelled` | "Team: rescue cancelled. Standing by." | `/survivor/event` |
| fall | "I've fallen, and I can't get up alone. Could someone set me on my feet? I'm still here." | `robot_status`, as in the lab |

The start line teaches the signal the survivor detector looks for (both arms raised), so nobody has to be told in advance.

## Files

| File | What it is |
|---|---|
| `prompt.md` | The personality itself: Gemini's system prompt for missions. Edit this to change Pupper. |
| `run.py` | Runs the lab's server, kiosk or text chat with the mission prompt and voice. |
| `start.sh`, `stop.sh` | Start and stop Pupper with this personality, on the robot. |
| `mission_feed.py` | Mission events for Gemini: `POST /mission/event` in, `WS /ws/mission` out. `run.py` adds it to the lab's server. |
| `test_personality.py` | Desk checks: no robot, no ROS, no Gemini. |

Nothing in `gemini_pupper/robot` or `lab` is changed (the one change in that repo is the web app's mission-event listener, below). Its server reads its prompt from one hard-coded path, so `run.py` imports that server and points the path at `prompt.md` before it starts; the kiosk gets the voice the same way. `lab/prompt.md` (the lab's Pupper) is untouched and still what `gemini_pupper/robot/start.sh` runs.

## Try it

On the laptop, in text. Nothing moves, and no robot is needed. It needs what the lab's chat needs (`google-genai`, `pyyaml`, your key in `GEMINI_API_KEY`):

```bash
cd sar-pupper
python3 personality/run.py chat
You > start the search
You > MISSION EVENT: locked range=3
You > MISSION EVENT: dodge
You > MISSION EVENT: arrived
You > my name is Sam and my ankle hurts
You > can you do a trick?
You > stand down
```

On the robot (sar-pupper cloned next to gemini_pupper in the Pi's home; Pupper on the ground, a hand near the e-stop):

```bash
~/sar-pupper/personality/start.sh --kiosk     # stop with ~/sar-pupper/personality/stop.sh
```

`start.sh` has passed a syntax check only. It has not been run on the Pi. It calls the lab's `start.sh` for the robot stack, so the controllers come up exactly as in the lab.

After changing `prompt.md`: run `python3 personality/test_personality.py`, try it in text, then reconnect in the app. No restart.

## Mission events

**Wired on 2026-11-23, not yet heard on the robot.** The path is:

```
/survivor/event, /dodge/event          (ROS, on the robot)
  -> survivor-id/robot/event_bridge.py  POST /mission/event   {"event": "arrived", "details": {"range": 0.7}}
  -> mission_feed.py (added to the lab's server by run.py)    WS /ws/mission: "MISSION EVENT: arrived range=0.7"
  -> gemini_pupper/web/services/geminiLive.ts                 session.sendRealtimeInput({ text })
  -> Gemini says the event's call-out
```

- `mission_feed.py` builds every line itself, from an event name and details that must be plain words and numbers. Free text is refused (HTTP 422): a mission event is an instruction to Gemini's voice, so nothing a program was handed from elsewhere goes through. The server listens on 127.0.0.1 only.
- The bridge decides what is worth saying: not `candidate`, not a dodge the dodge node only reported in dry run, not the same event twice within a few seconds, and nothing from the survivor nodes while their motion is off.
- The web app change is the one edit made inside `gemini_pupper` (survivor plan, decision S9). It needs `web/dist` rebuilt on the robot: `bash ~/gemini_pupper/robot/setup.sh`. Against the lab's own server, which has no feed, the app behaves as before.
- Names are the ones in the table above. `test_personality.py` fails if an event in its list has no call-out or a name the feed would refuse, so add new events in both places.
- Try it by hand, with the app connected: `curl -s -X POST localhost:8000/mission/event -H 'Content-Type: application/json' -d '{"event": "locked", "details": {"range": 3}}'`. `GET /mission/recent` shows what was sent and how many apps are listening.

What is checked: the feed's routes on a server shaped like the lab's (in `test_personality.py`), and that the web app type-checks and builds. What is not: a live Gemini session speaking an event it was sent this way.

The bark on lock and arrival (`dog_bark.wav`) needs no cloud and stays the first signal. Gemini's words follow a second or two later.

## Decisions made here that touch the other plans

Each is a line or two in `prompt.md` and easy to reverse.

1. **No tricks on duty, so no sitting beside the survivor – with one exception, the arrival bounce.** A trick hands the legs to a trick controller, and dodging only works in the walking controller. Pupper stays standing next to the survivor and can still dodge. Sitting would read as "I'm staying", but it would be defenceless. The exception (your decision, 2026-11-23): on arrival the survivor node plays a short bounce on the spot. Pupper cannot dodge for those seconds, then returns to the walking controller. Gemini itself still never calls a trick on duty.
2. **Survivor plan S4 (what Pupper does on arrival) – decided 2026-11-23:** Pupper announces through Gemini Live and bounces up and down on the spot, feet on the floor. The prompt's arrival steps are the announcement; the bounce belongs to the survivor node.
3. **Follow mode is off on duty.** The survivor plan does not start the person followers in the mission launch. The prompt tells Gemini not to use the tool. The tool is still registered; removing it or re-pointing it at the survivor tracker belongs to the mission launch.
4. **"Stop" and "lie down" work for anyone, on duty or off.** Both are safe states, and the motors need a way to rest.
5. **The kiosk remembers the voice.** The app keeps its settings in the kiosk browser's storage, so after a mission the lab's Pupper also speaks as Charon. To put it back, run `python3 personality/run.py kiosk ~/.config/gemini-pupper/api_key --voice Puck` while the lab's kiosk is on screen, wait for "kiosk: settings applied", then press Ctrl-C.

## Limits

- **Length.** The prompt is about 1,350 words; the lab's is about 470. Google's guidance warns against long prompts. If Gemini starts dropping rules in the text chat, shorten "On duty and off duty" and "What you never do" first, and keep the call-outs.
- **Exact words are not guaranteed.** Gemini usually keeps to a quoted line but can rephrase. Anything that must be word-perfect should be a recorded sound, like the bark.
- **A spoken "mission event" is not an event.** The prompt says only text lines count, but a prompt is a request, not a lock. The real protection is that events arrive on a channel a bystander cannot speak into.
- **It needs the network.** No Gemini, no voice. The dodge, the approach and the bark do not depend on it.

## Sources

- Google, Live API best practices (persona, then rules, then guardrails; if-then examples; speaking rate): https://docs.cloud.google.com/vertex-ai/generative-ai/docs/live-api/best-practices
- Google, Live API voices (Puck "Upbeat", Charon "Informative", Kore "Firm"): https://firebase.google.com/docs/ai-logic/live-api/configuration
- Forum report on voice selection in gemini-3.1-flash-live-preview: https://discuss.ai.google.dev/t/bug-voicename-configuration-is-silently-ignored-in-gemini-3-1-flash-live-preview-multimodal-live-api/179065
- Local code read: `gemini_pupper/` (`lab/prompt.md`, `robot/server.py`, `robot/kiosk.py`, `robot/gemini_chat.py`, `robot/start.sh`, `web/services/geminiLive.ts`, `web/constants.ts`), and both plans in this repo.

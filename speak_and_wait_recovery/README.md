# Speak and Wait Recovery

This reovery behavior will enables a robot to speak a given text and wait for a given duration.
This will be useful when a robot stuck in a crowded environment.

<image>

## speak_and_wait_recovery/SpeakAndWaitRecovery

### Parameters

- `~speak_text` (string, default: `Make way, Please.`)

Text which you are goint to make a robot say.

- `~duration_wait` (double, default: `5.0`)

Duration for which a robot will wait after saying the given text. (seconds)

- `~duration_timeout` (double, default: `1.0`)

Duration for tf looking up transformations. (seconds)

- `~sound_action` (string, default: `sound_play`)

Action name of sound_play used for Text-To-Speech.

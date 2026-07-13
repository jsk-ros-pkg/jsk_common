# app_notifier

Notifier plugin for `app_manager`

## `app_manager` plugins

### `app_notifier/mail_notifier_plugin`: Mail notifier plugin

This plugin notifies app results by sending an e-mail.

#### `plugin_args`: Plugin arguments

- `mail_title`: mail title
- `sender_address`: mail sender address
- `receiver_address`: mail receiver address
- `use_timestamp_title` (default: `False`) : Use timestamp in title or not

#### `launch_args`: Plugin launch arguments

`None`

#### Sample plugin description

```yaml
plugins:
  - name: mail_notifier_plugin
    type: app_notifier/mail_notifier_plugin
    plugin_args:
      mail_title: Test app
      use_timestamp_title: true
      sender_address: hoge
      receiver_address: hoge
```

#### Settings for sending from gmail address

Please see this [link](https://kifarunix.com/configure-postfix-to-use-gmail-smtp-on-ubuntu-18-04/) to configure properly.

### `app_notifier/speech_notifier_plugin`: Speech notifier plugin

This plugin notifies app results by speaking.

#### `plugin_args`: Plugin arguments

- `client_name`: client name for `sound_play`
- `lang` (default: `None`): language, if `None`, a robot speaks English.

#### `launch_args`: Plugin launch arguments

`None`

#### Sample plugin description

```yaml
plugins:
  - name: speech_notifier_plugin
    type: app_notifier/speech_notifier_plugin
    plugin_args:
      client_name: /sound_play_jp
      lang: jp
```

### `app_notifier/tweet_notifier_plugin`: Tweet notifier plugin

This plugin notifies app results by tweeting.

#### `plugin_args`: Plugin arguments

- `client_name`: client name for `sound_play`
- `image` (default: `False`): whether tweet with image or not.
- `image_topic_name` (default: `None`): tweet image topic. this argument is used only when `image` is `true`.
- `warning` (default: `False`): whether warn unknown user or not.

#### `launch_args`: Plugin launch arguments

`None`

#### Sample plugin description

```yaml
plugins:
  - name: tweet_notifier_plugin
    type: app_notifier/tweet_notifier_plugin
    plugin_args:
      client_name: /tweet_image_server/tweet
      image: true
      image_topic_name: /head_camera/rgb/image_rect_color
      warning: true
```

### `app_notifier/user_speech_notifier_plugin`: User speech notifier plugin

This plugin notifies which user is running the app by speaking.

#### `plugin_args`: Plugin arguments

- `client_name`: client name for `sound_play`
- `lang` (default: `None`): language, if `None`, a robot speaks English.
- `warning` (default: `False`): whether warn unknown user or not.

#### `launch_args`: Plugin launch arguments

`None`

#### Sample plugin description

```yaml
plugins:
  - name: user_speech_notifier_plugin
    type: app_notifier/user_speech_notifier_plugin
    plugin_args:
      client_name: /sound_play_jp
      lang: jp
      warning: true
```

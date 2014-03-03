# jsk_joy package

## MIDI controllers
### `interactive_midi_config.py`
このスクリプトを使うと、対話的にmidiデバイスの入力を設定できる。

1. まず実行すると、デバイスの名前を聞かれるので、答えてください
2. 次に順番にボタンを押すと、その順番でボタンが`sensor_msgs/Joy`メッセージのaxes(および可能であればbutton)にマッピングされる。
3. `q`を押して終了すると、その設定が`/tmp/midi.yaml`に保存される。

### `midi_write.py`
LEDやアクティブフェーダなどを制御するには、MIDIのoutputを計算機から叩く必要がある。そのための便利スクリプト。
`-w`オプションを利用することで、yamlファイルに追記することができる。

### `midi_config_player.py`
`interactive_midi_config.py`および`midi_write.py`で生成したyamlファイルをもとに、`sensor_msgs/Joy`をpublishする。

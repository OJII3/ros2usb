# ros2usb - ROS2 Node for USB (WIP)

## Overview

マイコンに接続された USB と ROS の間でデータをやり取りするためのノードです。

トピック名`ros2usb`(暫定)でデータを受け取り、USB に書き込みます。
また、USB からデータを読み込み、トピック名`usb2ros`(暫定)でデータを送信します。

## Prerequisites

- ROS2 Humble (Arch Linux でのみ動作確認済み)
- `~/.bashrc`に以下が記述されていること

```bash
 source /opt/ros/humble/setup.bash
 source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
 source ./install/setup.bash
```

## Build

1. ROS2 のワークスペースの`src`ディレクトリにこのリポジトリをクローンしてください。

2. ワークスペースのルートに戻り、`colcon build`を実行してください。

## Setup

1. マイコンと USB typeA - USB typeB でつなぎます。
2. シリアルポートのデバイス名を確認します。

```bash
ls -l /dev/serial/by-id/
```

`lrwxrwxrwx 1 root root 13 Aug  9 17:48 usb-STMicroelectronics_STM32_STLink_0670FF554849844987183740-if02 -> ../../ttyACM0`のように表示されます。

3. マイコンと通信するために、ポートの権限を変更します。

```bash
sudo chmod 666 /dev/ttyACM0
```

## Run

```bash
ros2 run ros2usb ros2usb_bin
```

を実行して Node を起動できます。

- `micon2ros`トピックのデータをマイコンに送信します.
- マイコンからのデータは`micon2ros`トピックへ publish されます.

---

[^1]:
    VSCode + ROS 拡張以外の環境で補完等を利用するには, clangd の arguments に`--enable-config`を追加, ros2 のワークスペースのルートに`.clangd`ファイルを作成し、`CompilationDatabase: ./build`を記述.

    ```
    colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=1

    ```

    を実行すると、`build`ディレクトリに`compile_commands.json`が生成される.

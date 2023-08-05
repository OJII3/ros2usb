# ros2usb - ROS2 Node for USB (WIP)

## Overview

マイコンに接続されたUSBとROSの間でデータをやり取りするためのノードです。

トピック名`ros2usb`(暫定)でデータを受け取り、USBに書き込みます。
また、USBからデータを読み込み、トピック名`usb2ros`(暫定)でデータを送信します。

## Prerequisites

- ROS2 Humble

## Build

ROS2のワークスペースの`src`ディレクトリにこのリポジトリをクローンしてください。
その後、ワークスペースのルートに戻り、`colcon build`を実行してください[^1]。

## Usage

`ros2 run ros2usb ros2usb_node`を実行してNodeを起動してください。

---

[^1]: VSCode + ROS拡張以外の環境で補完等を利用するには, clangdのargumentsに`--enable-config`を追加, ros2のワークスペースのルートに`.clangd`ファイルを作成し、`CompilationDatabase: ./build`を記述. 
`colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=1`を実行すると、`build`ディレクトリに`compile_commands.json`が生成される.

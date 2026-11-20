# yuPad43 ZMK driver

日本語 | [English](README.en.md)

IQS9151搭載のyuPad43（10 RX × 9 TX）向けZMKモジュールです。ZMK v0.3.0を基準にしています。

[接続と導入手順](https://github.com/yuchamichami/yuPad43/blob/main/docs/zmk.md) · [XIAO用設定例](https://github.com/yuchamichami/yuPad43/tree/main/firmware/xiao-example)

## 動作

XIAO用設定例では、1本指でカーソルを動かせます。2本指でなぞると縦横にスクロールし、軽く弾くと指を離した後も少し流れます。もう一度パッドに触れると止まります。スクロール量は **0（標準）** です。

電極設定はyuPad43専用です。校正目標400、タッチ判定は開始30・解除26、Finger Split Factorは0です。1 mm厚のPLAカバーを装着し、XIAO nRF52840のUSB接続で動作を確認しています。

## ライセンス

ShiniNet / zmk-driver-iqs9151（MIT License）を基に、yuPad43向けの電極設定・初期化設定・ATI通信処理を変更しています。

Copyright (c) 2026 ShiniNet。全文は [LICENSE](LICENSE) を参照してください。

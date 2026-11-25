# yuPad43 ZMK driver

日本語 | [English](README.en.md)

IQS9151搭載のyuPad43（10 RX × 9 TX）向けZMKモジュールです。ZMK v0.3.0を基準にしています。

[接続と導入手順](https://github.com/yuchamichami/yuPad43/blob/main/docs/zmk.md) · [XIAO用設定例](https://github.com/yuchamichami/yuPad43/tree/main/firmware/xiao-example)

## 動作

標準では、1本指でカーソルを動かし、指を離すとその場で止まります。2本指でなぞると縦横にスクロールします。軽く弾くと指を離した後も少し流れ、もう一度触れると止まります。

ドライバの標準座標範囲は1300 × 1192で、ピンチは初期設定で無効です。XIAO用設定例では、ZMK側でスクロール量を **0（標準）** に設定しています。

電極設定はyuPad43専用です。校正目標400、タッチ判定は開始30・解除26、Finger Split Factorは0です。1 mm厚のPLAカバーを装着し、XIAO nRF52840のUSB接続で動作を確認しています。

## 横操作とジェスチャのイベント仕様

2本指の横操作は Kconfig choice `INPUT_IQS9151_2F_HORIZONTAL_MODE` で排他的に選択します。
表中の名前には `CONFIG_INPUT_IQS9151_` が付きます。

| モード | 横操作の出力 |
| --- | --- |
| `2F_HORIZONTAL_SCROLL`（標準） | 横スクロールと、有効な場合はその慣性 |
| `2F_HORIZONTAL_SWIPE` | 左右の専用ジェスチャイベント。横スクロール・横方向の慣性なし |
| `2F_HORIZONTAL_DISABLED` | 横スクロールも2本指左右スワイプも出力しない |

縦スクロールは `SCROLL_Y_ENABLE` で独立して設定します。
旧 `SCROLL_X_ENABLE` と試作版の `2F_HORIZONTAL_SWIPE_ENABLE` は廃止しました。

スワイプモードでは `2F_SCROLL_START_MOVE`（標準50）に達した移動の主方向で
横操作／縦スクロールを固定します。横操作は `2F_HORIZONTAL_SWIPE_THRESHOLD`
（標準200）にも達した時点で1回だけ発火し、全指を離すまで再発火しません。
距離は出力座標のカウント数です。指を順番に離しても残った指のカーソル移動を抑制します。
縦スクロールと慣性、有効な場合のピンチ優先判定、タップ／再タッチのドラッグは維持します。

ドライバーはスワイプに「戻る・進む」等のアクションを割り当てません。
公開ヘッダーは `include/dt-bindings/input/iqs9151.h` です。

| ジェスチャ | イベントコード名 |
| --- | --- |
| 2本指・左 | `IQS9151_GESTURE_2F_SWIPE_LEFT` |
| 2本指・右 | `IQS9151_GESTURE_2F_SWIPE_RIGHT` |
| 3本指・左 | `IQS9151_GESTURE_3F_SWIPE_LEFT` |
| 3本指・右 | `IQS9151_GESTURE_3F_SWIPE_RIGHT` |
| 3本指・上 | `IQS9151_GESTURE_3F_SWIPE_UP` |
| 3本指・下 | `IQS9151_GESTURE_3F_SWIPE_DOWN` |

いずれも `INPUT_EV_KEY` の押下（value=1）と解放（value=0）を出力し、各イベントの
sync は true です。コードはドライバー専用で、HIDコードやマウスボタン番号ではありません。
方向は回転後の座標に従います（左=-X、上=-Y）。未割り当てならホスト操作は発生しません。
アクションは firmware 側の入力プロセッサーで割り当てます。
3本指スワイプも旧 `INPUT_BTN_3`〜`INPUT_BTN_6` 出力から専用イベントへ変更しています。
タップの `INPUT_BTN_0`〜`INPUT_BTN_2`、ピンチの `INPUT_BTN_7` は従来どおりです。

### behavior実行時のスタック

`behaviors-consume`は入力スレッド内でbehavior・ZMKイベント・HID送信を同期実行します。
このprocessorを使用する構成では、`CONFIG_INPUT_THREAD_STACK_SIZE`の既定値を2048へ増やします。
512バイトの設定では、nRF52840版のスワイプからキーイベントへ入る経路だけで
少なくとも520バイトを必要とし、USB/BLE送信前にスタックが不足します。
スレッド方式で明示的に2048未満を指定した場合は、ビルド時のチェックで拒否します。
既存のビルドキャッシュに512が残る場合は、pristineビルドまたは`.conf`で2048を明示してください。
これは処理経路の静的検証による下限で、実機の最大使用量の測定値ではありません。

## 慣性スクロールの実行時切り替え

FWからデバイスを指定して慣性スクロールをON/OFFできます。起動時の初期値は
`CONFIG_INPUT_IQS9151_SCROLL_INERTIA_ENABLE` で指定し、`n` でも実行時にONにできます。
公開API、エラー、復帰時の動作、確認項目は[実行時切り替えの説明](docs/runtime-scroll-inertia.md)を参照してください。

## ライセンス

ShiniNet / zmk-driver-iqs9151（MIT License）を基に、yuPad43向けの電極設定・初期化設定・ATI通信処理を変更しています。

Copyright (c) 2026 ShiniNet。全文は [LICENSE](LICENSE) を参照してください。

# 慣性スクロールの実行時切り替え

## 生成箇所とセンサー仕様の確認

慣性はホストMCU上のドライバーで生成しています。`iqs9151_update_inertia()` が
離指前の座標履歴から初速を計算し、`iqs9151_inertia_scroll_work_cb()` が10 ms周期で
減衰させた `INPUT_REL_WHEEL` / `INPUT_REL_HWHEEL` を送信します。

Azoteq IQS9150/IQS9151 Datasheet v1.1（October 2024）の7.2.2 / 7.2.3は
1本指の相対座標と複数指の絶対座標を、8.6は接触中のスクロールジェスチャを説明しています。
このドライバーの離指後の慣性は、そのハードウェアジェスチャ出力を使用していません。
切り替えにセンサーの慣性レジスタや停止コマンドは必要なく、レジスタ変更は行いません。

- [メーカー製品ページ](https://www.azoteq.com/product/iqs9151/)
- [メーカー作成データシート v1.1（Mouser配布）](https://www.mouser.com/datasheet/3/931/1/IQS9150_IQS9151_datasheet.pdf)

## 公開API

ヘッダーは `include/drivers/input/iqs9151.h` です。モジュールのCMakeがincludeパスを公開します。

```c
#include <drivers/input/iqs9151.h>

int iqs9151_scroll_inertia_set_enabled(const struct device *dev, bool enabled);
int iqs9151_scroll_inertia_get_enabled(const struct device *dev, bool *enabled);
```

FWのスレッド文脈からの呼び出し例（`trackpad` はFWで定義した実際のノードラベル）：

```c
const struct device *pad = DEVICE_DT_GET(DT_NODELABEL(trackpad));
int ret = iqs9151_scroll_inertia_set_enabled(pad, false);
if (ret != 0) {
    /* -EBUSYなら後で別のwork/スレッドから再試行。それ以外はエラー処理。 */
    return ret;
}

bool enabled;
ret = iqs9151_scroll_inertia_get_enabled(pad, &enabled);
if (ret != 0) {
    return ret;
}
/* enabled == false */
```

getが返すのは、最後に設定に成功した**ソフトウェア上の有効／無効設定**です。
センサーレジスタの読み出しでも、現在慣性が動作中かどうかの問い合わせでもありません。
デバイスごとに独立し、未設定なら起動時の初期値を返します。

| 戻り値 | 意味 |
| --- | --- |
| `0` | 設定／取得成功 |
| `-EINVAL` | NULL、IQS9151以外のデバイス、またはgetの出力先がNULL |
| `-ENODEV` | デバイスの初期化が未完了または失敗 |
| `-EWOULDBLOCK` | ISRから呼び出した |
| `-EBUSY` | 入力処理と競合中、またはデバイス自身のworkerからの呼び出し |

失敗時は設定もgetの出力先も変更しません。`-EBUSY` は成功として扱わず、後で再試行してください。
同期入力モードのbehaviorなど、デバイスworker上から呼び出す場合は別のwork/スレッドへ委譲します。
入力キューへの送信とAPIとの相互待ちを避けるため、APIはロック取得を待ちません。

## 初期値・停止・復帰

- `CONFIG_INPUT_IQS9151_SCROLL_INERTIA_ENABLE` は起動時の初期値です。
  `n` でも機能自体をコンパイルから除外しないため、setでONにできます。
- OFF成功時に遅延workをキャンセルし、進行中の慣性状態、速度履歴、片指離脱中に保持した
  発進待ち速度を消去します。OFF中は慣性用の速度履歴を蓄積しません。
- OFF→ONでは古い慣性は再開しません。新たな移動のサンプルから慣性を開始します。
- 同じOFFへの再設定は停止状態を維持します。同じONへの再設定は現在の慣性を中断しません。
- 指を動かしている間の通常スクロールはそのままです。カーソル慣性やタップ、ピンチ、
  ポインタ移動、横スワイプの設定は変更しません。
- API、フレーム処理、慣性work、センサーリセット時の接触破棄をデバイスごとのmutexで同期します。
  APIにI2Cアクセスはなく、センサー再設定の処理はランタイム設定を上書きしません。
- センサーの自律的な低電力状態、センサーリセットからの復旧、RAMを保持するMCUスリープでは
  最後の設定を維持します。このドライバーは独自のPM suspend/resumeフックを持っていません。
  MCUが再起動するdeep sleep / system offでは初期値に戻ります。フラッシュ保存は行いません。

OFF成功以降、新しい慣性イベントは生成しません。ただし、成功前にZephyr入力キューやHID転送へ
渡したイベントと、OS／アプリが独自に生成する慣性は取り消せません。

## 検証と実機確認

追加した `runtime_inertia` / `runtime_inertia.initially_off` は、初期値ON/OFF、実行中の停止、
再有効化、通常スクロール、片指離脱時の残存速度、同じ状態への再設定、デバイス間の独立性、
不正引数、未初期化デバイス、ISR、他スレッドでのフレーム処理との競合、センサーresetを対象にします。
transportテストにも、実際のI2C再初期化経路を通過した後の設定維持を追加しています。

2026-11-25の検証環境はWindows、Zephyr 3.5.0、Zephyr SDK 0.16.3、
`qemu_cortex_m3` ターゲットです。runtime初期値ON/OFF、既存入力、横スワイプ、transportの
5構成でコンパイル・リンクを確認しました。Kconfigの横モード／慣性初期値とbehaviorスタックの
Pythonテストは実行成功しています。
QEMU実行ファイルとLinux環境がないため、Cのztestスイートの実行は未検証です。
テストのコンパイル成功はテスト実行成功や実機確認を意味しません。

実機では以下をWindows／Macの両方で確認してください。すべて現時点では未検証です。

| 確認操作 | 期待結果 |
| --- | --- |
| ONでスクロールして離指 | 通常スクロール後に慣性が続く |
| OFFでスクロールして離指 | 通常スクロールは動き、離指後は慣性が出ない |
| 慣性中にOFFを設定し成功 | 新しい慣性イベントが止まる |
| OFF→ON後、指を動かさず待つ | 古い慣性が再開しない |
| ON/OFFを反復、同じ値を再設定 | 状態取得と動作が一致し、同じONは慣性を中断しない |
| RAM保持のスリープから復帰 | 最後に成功した設定を保持する |
| センサーresetから復旧 | 最後の設定を保持し、古い慣性は復活しない |
| ポインタ、カーソル慣性、タップ、ドラッグ、ピンチ、横スワイプ | 従来の設定どおり動く |

キー割り当て、ZMK behavior、Studio、OS検出、BTプロファイル管理、永続保存は実装対象外です。

# Setup Software の未公開パラメータ空間 (`cmd 0xC0`/`0xC1`) — candumpリバースエンジニアリング

実機X4-36(FW `2026042402`、CAN ID 1、`can0` @ 1Mbps)に対し、MyActuatorの公式Windowsツール
"Setup Software"を接続してBasic Parameters / Advanced Parametersタブの各「Read」「Save」ボタンを
押しながら`candump -tz -f capture.log can0`でキャプチャし、実機を解析した(生ログ:
ユーザー提供の`capture_can0_001.log`、402行)。既存の`ref/can_bus_motor_motion_protocol_v4.4_en.md`
(V4.4、changelog cutoff 2026-03-25)には**登場しない、それより新しいコマンド体系**が使われている
ことを確認した——このFWビルド(2026-04-24)はV4.4マニュアルより後のリビジョンと見られる。

## 0. 物理層・アドレッシング

`0x140+ID`(ホスト→モータ、ここでは`0x141`)/ `0x240+ID`(モータ→ホスト、`0x241`)。これは
V3/V4.4と同じ標準アドレッシング。DLC8固定。

## 1. `cmd 0xC0`: 汎用パラメータ読み書き(インデックス指定)

V4.4までの「コマンドごとに個別のバイト割り当て」とは異なり、このFWでは**ほぼ全ての
Protect/Plan/Motor Parameters(および恐らくPID)が1つのコマンド`0xC0`に統合され、2バイト目の
インデックスで対象を選択する**方式に変わっている。

```
Byte:   0     1     2      3      4  5  6  7
        0xC0  0x00  index  flag   value (f32, リトルエンディアン)
```

- `flag = 0x01`: **読み取り**。リクエストのvalueは無視(実測では全て`00000000`)、応答の
  `value`に現在値が入る。
- `flag = 0x00`: **書き込み(RAM)**。リクエストのvalueに新しい値を入れて送る。応答のvalue部は
  ack的な値(実測では`0`や整数`1`、あるいは複数フィールドで共通の`1.1`など)が入るが、
  **内容に一貫した意味は見出せなかった**(書き込み成功可否の判定には使えなさそう、要追加検証)。
- 値は一貫して**f32リトルエンディアン、byte[4..8]**。整数っぽい項目(Encoder2 Abnormal Value=100
  など)も含めて全部f32で運ばれている。

書き込み後は`cmd 0xC1`(後述)を送るまでRAM上のみで、電源断で失われると推測される(DAMIAOの
`0x55`書き込み+`0xAA`保存、RobStrideのcomm_type18+22と同じ二段構え)。

### 1.1 確認済みインデックス表(GUIの表示値と1対1で一致、確度: 高)

`ref/x_series_v4_product_manual_en.md`等の既存マニュアルには載っていない、Advanced Parameters
タブの表示値と完全一致させて確定させたもの。

**Protect Parameters パネル:**

| idx | フィールド名 | 実測値 |
|---|---|---|
| 0x13 | Over Voltage (V) | 55.0 |
| 0x14 | Low Voltage (V) | 20.0 |
| 0x15 | Stall Time Limit (s) | 2.0 |
| 0x16 | E-Brake Start Duty Cycle (%) | 50.0 |
| 0x17 | Current Sample Res (mΩ) | 5.0 |
| 0x18 | E-Brake Hold Duty Cycle (%) | 30.0 |
| 0x19 | **Brake Mode**(確定。enum: `0.0`=E-Brake, `1.0`=Resistor。選択肢はこの2つのみ。
  §1.6の差分テストで両方向の書き込み・保存・再読み取りを確認済み) | 0.0 |
| 0x3F | Encoder2 Abnormal Value | 100.0 |
| 0x40 | Encoder2 Abnormal Speed | 5.0 → 実機で4.8に書き換え・保存・再読み取りで確認済み |
| 0x41 | Automatic Error Recovery (bool) | 1.0 |

**Plan Parameters パネル:**

| idx | フィールド名 | 実測値 |
|---|---|---|
| 0x1A | Max Positive Position (Deg) | 65535.0 |
| 0x1B | Min Negative Position (Deg) | 65535.0 |
| 0x1C | Position Plan Max Acc (Dps/s) | 5000.0 |
| 0x1D | Position Plan Max Dec (Dps/s) | 5000.0 |
| 0x1E | Position Plan Max Speed (RPM) | 500.0 |
| 0x1F | Speed Plan Max Acc (Dps/s) | 5000.0 |
| 0x20 | Speed Plan Max Dec (Dps/s) | 5000.0 |
| 0x21 | Motor Position Zero | 71200.0 |
| 0x3E | KT_OUT | 1.2 → 実機で1.1に書き換え・保存・再読み取りで確認済み |

**Motor Parameters パネル:**

| idx | フィールド名 | 実測値 |
|---|---|---|
| 0x23 | Max Current (A) | 31.0 |
| 0x24 | Stall Current (A) | 30.0 |
| 0x25 | Shutdown Temp (°C) | 100.0 |
| 0x26 | Resume Temp (°C) | 90.0 |
| 0x27 | Max Speed (RPM) | 4000.0 |
| 0x28 | Nominal Speed (RPM) | 3000.0 → 実機で2800に書き換え・保存・再読み取りで確認済み |
| 0x22 | Rated Current (A) | 8.625(読み取り)/8.62(書き戻し。**下記1.3のバグ参照**) |
| 0x29 | **Enable EtherCAT**(確定。§1.5の差分テストで0x32と切り分け済み) | 1.0 |
| 0x32 | **Enable CAN Filter**(確定。ただし**読み取り専用の鏡**——実際の書き込みは`cmd 0x20`
  経由。§1.5参照) | 1.0 |
| 0x3B | **Enable 2nd Encoder**(xlsxで名称確定) | 1.0 |
| 0x3D | **Select Thermistor**(xlsxで名称確定。Thermistor1=0) | 0.0 |

**Motor Information / 汎用:**

| idx | フィールド名 | 実測値 |
|---|---|---|
| 0x01 | Motor Number | 0.0 |
| 0x02 | Factory Time | 260408.0(★下記の通りxlsxエクスポートでは0と食い違う) |
| 0x03 | Reduction Ratio | 36.0 |
| 0x3A | 不明(2nd Encoder関連?) | u32=0x1000FBCD、意味不明・要検証 |
| 0x3C | 不明(書き込みのみ観測、読み取り未観測) | 書き込み値262144.0(=2^18、ビットマスク/enumの可能性) |

**★Export機能のバグ発見**: `memo_myactuator_001.xlsx`(後述、Advanced Parametersの
「Export parameters」で書き出したExcelファイル)では`Factory Time = 0`となっており、実際に
`0xC0 idx=0x02`を読んだ値(`260408`、画面表示とも一致)と食い違う。Export機能はこのフィールドを
正しく取得できていない(あるいは別の壊れた値を書き出している)バグがある可能性が高い。

### 1.2 PID Parameters パネル(確度: 高、GUI表示値と一致確認済み)

Basic Parametersタブの「PID Parameters → Read」ボタン**単体**を押した際のキャプチャ(13回の
読み取りのみが飛ぶ、非常にクリーンな捕捉)と、Read後のGUIスクリーンショットを突き合わせて確定。

| idx | フィールド | 実測値 |
|---|---|---|
| 0x08 | Position Loop P | 0.05 |
| 0x09 | Position Loop I | 0.0 |
| 0x0A | Position Loop D | 0.5 |
| 0x31 | Position Loop T(Filter) | 0.0001 |
| 0x06 | Speed Loop P | 0.01 |
| 0x07 | Speed Loop I | 0.0001 |
| 0x2D | Speed Loop D | 0.0 |
| 0x30 | Speed Loop T(Filter) | 0.0005 |
| 0x04 | Q-Axis Current P | 1.5 |
| 0x05 | Q-Axis Current I | 0.05 |
| 0x2C | Q-Axis Current D | 0.0 |
| 0x2E | Q-Axis Current R(Slope) | 0.0 |
| 0x2F | Q-Axis Current T(Filter) | 0.0005 |

**重要な挙動**: 「PID Parameters → Read」を1回押すと**この13回の読み取りしか飛ばない**
(Position/Speed Loopの **R(Slope)は読みに行かない**——GUI上は`0.000000`と表示されるが、
これは配線を読んだ値ではなく単なる未使用時のデフォルト表示と見られる。また **D-Axis Current
列は一切読みに行っていない**——GUI画面ではD-Axis CurrentがQ-Axis Currentと全く同じ値
(P=1.5, I=0.05, D=0, R=0, T=0.0005)で表示されるが、これは実機から個別に読んだのではなく、
**GUIがQ-Axis Currentの値をD-Axisにもコピー表示しているだけ**と判断できる(FOCの一般的な
設計上、d軸電流指令は基本0固定でq軸と独立チューニングしないため、GUI側で「同じ値」を
決め打ち表示している可能性が高い)。D-Axis Current用の独立したインデックスが存在するのか、
そもそも存在しないのかは今回のキャプチャだけでは判別できない。

以前のキャプチャ(Advanced Parameters側のReadやExportで巻き込まれて読まれていた)に含まれていた
以下のインデックスは、このPIDパネル単体のReadでは読まれなかったため、**PIDパネルの項目ではない
別のパラメータ**と判明していた。その後、「Export parameters」で書き出されたExcelファイル
(`memo_myactuator_001.xlsx`、後述§1.4)を直接解析し、値の並び順が完全一致したことで
ほぼ全て名称が確定した。

### 1.4 `memo_myactuator_001.xlsx`(Export parameters出力)による確定(確度: 高)

xlsxは実体がZIP+XMLなので追加ツール無しでパース可能(`zipfile`+`xml.etree`)。書き出された
行の値の並びと、ワイヤキャプチャで観測したindexの並びを突き合わせて確定:

| idx | フィールド名(xlsxより) | 実測値 |
|---|---|---|
| 0x0B | Enabled Powerdown Save MultTurn | 0.0 |
| 0x0C | Pole-Pairs(xlsx原文は"Pole-Paris"とタイポ) | 11.0 |
| 0x0D | Single-Resolution Value (Pulses) | 16384.0 (= 2^14、エンコーダ分解能) |
| 0x0E | Calibrat Current (A) | 2.0 |
| 0x0F | Change Motor Direction | 0.0(xlsxの値と完全一致で確定) |
| 0x10 | Exchange phase(推定。xlsxでは空欄のため値による裏付け不可、booleanらしい値0x10=1.0のみ根拠) | 1.0 |
| 0x11 | Encoder Calibrat Value(推定。同じく空欄のため値のみ根拠。§6の`cmd 0x19`の値=6734と一致するのも傍証) | 6734.0 |
| 0x33 | OUTENCODER | 0.0 |
| 0x34 | OUTENCODER_1 | 50.0 |
| 0x35 | OUTENCODER_2 | 2.0 |
| 0x36 | OUTENCODER_3 | 1.0 |
| 0x37 | OUTENCODER2_1 | 0.0 |
| 0x38 | OUTENCODER2_2 | 1.0 |
| 0x39 | OUTENCODER2_3 | 0.0 |

`OUTENCODER*`/`OUTENCODER2*`(出力軸側1st/2ndエンコーダのマッピング設定と推測)は名前と値は
確定したが、各サブフィールドの厳密な意味(極性/オフセット/ゲインのどれに当たるか等)は
未解明。

**残った空欄**: xlsxで値そのものが空欄だった`Exchange phase`と`Encoder Calibrat Value`は、
どちらのindex(0x10/0x11)がどちらの名前かを値だけから確定させることはできない
(桁の性質から上記の割り当てを採用したが、確度は他の項目より低い)。

### 1.5 差分テスト: `idx 0x29` vs `idx 0x32`(Enable EtherCAT / Enable CAN Filter)の切り分け

**手順**: Motor Parametersパネルの「Enable CAN Filter」**だけ**チェックを外し、Save → Read。

**観測結果**(`cmd 0x20`の値に注目):

```
無効化前の観測(以前のセッション): 20 02 00 00 01 00 00 00   ← byte[4]=0x01
無効化操作時の観測(今回):         20 02 00 00 00 00 00 00   ← byte[4]=0x00
```

`cmd 0x20`のペイロードのbyte[4]が、チェックを外す操作に伴って`01`→`00`に変化した。これは
`0xC0`のidx読み書きとは別に、この専用コマンドが「Enable CAN Filter」のON/OFFそのものを
運んでいる決定的な証拠。この操作の際、Motor Parametersパネルの他の項目(0x23-0x28, 0x29,
0x22, 0x3B, 0x3C, 0x3D)は通常通り`0xC0`で再送されたが、**`idx 0x32`だけは書き込みリストから
除外されていた**——つまり`idx 0x32`の実際の値変更経路は`cmd 0x20`であり、`0xC0`経由では
書き込まれない(読み取り専用の鏡)と判断できる。これにより:

- **`cmd 0x20` = Enable CAN Filter の専用書き込みコマンド**(byte[4] = 0 or 1)
- **`idx 0x32` = Enable CAN Filter**(`0xC0`では読み取りのみ)
- **`idx 0x29` = Enable EtherCAT**(消去法。`0xC0`で通常通り読み書きされる)

**追加の発見(未解明のバグ/制限)**: `cmd 0x20`で無効化(byte[4]=0x00)を送り、直後に`cmd 0xC1`
でコミットしたにもかかわらず、**5〜7秒後の「Read」では`idx 0x32`が引き続き`1.0`(有効)のまま
だった**。2回試行して2回とも同じ結果(チェックが元に戻る)。これはGUIの表示バグではなく、
**実機側で「Enable CAN Filter」の無効化が(この手順だけでは)実際に反映されていない**ことを
意味する。電源再投入が必要な設定なのか、この無効化操作自体がこのFWでサポートされていないのか
は未確認——安全のため今回はモータの電源再投入までは試していない。

### 1.6 差分テスト: `idx 0x19`(Brake Mode)の確定

**手順**: Protect Parametersパネルの「Brake Mode」を E-Brake → Resistor に変更しSave→Read、
続けて Resistor → E-Brake に戻してSave→Read。

**観測結果**:

```
Resistor へ変更時の書き込み: C000190000803F  (value = 1.0)
直後の Read:                 C000190100000000 → 応答 ...00803F (value = 1.0、変更を確認)
E-Brake へ戻す書き込み:      C000190000000000  (value = 0.0)
直後の Read:                 C000190100000000 → 応答 ...00000000 (value = 0.0、復帰を確認)
```

両方向とも書き込み→保存→再読み取りで正しく反映されることを確認(§1.5のCAN Filterと違い、
こちらは正常に機能する)。**`idx 0x19` = Brake Mode、enum `0.0`=E-Brake / `1.0`=Resistor**、
選択肢はこの2つのみ(GUIのドロップダウンで確認済み)。

### 1.3 発見したGUI側のバグ(精度欠落)

Rated Current: 読み取り値は`8.625`(内部精度)だが、その後の「Save」操作で書き戻された値は
`8.62`だった。GUIが表示用に丸めた文字列("8.62")をそのまま書き戻しており、**Saveボタンを押す
たびに0.005の精度が失われる**。実際に危険なバグではないが、Setup Software経由で値を触ると
サイレントに精度が落ちる、という実例として記録しておく。

## 2. `cmd 0xC1`: 全パラメータのフラッシュ確定(コミット)

```
リクエスト: C1 00 00 00 00 00 00 00
応答:       C1 00 00 00 <ack, 意味不明の値>
```

`0xC0 flag=0`の書き込みは複数まとめてRAMに送った後、**最後に1回だけ`0xC1`を送ってflashに確定**
するパターンが一貫して観測された(RobStrideのcomm_type22 `Save parameters`、DAMIAOの`0xAA`
"save to flash"と同じ役割)。実際に`0xC1`を送った後は電源を切らずとも読み取り値が変化したまま
だったので、単なるRAM→Flashコミットで良さそう。

## 3. `cmd 0xB2`: ファームウェアバージョン読み取り(プレーンu32)

```
リクエスト: B2 00 00 00 00 00 00 00
応答:       B2 00 00 00 22 F4 C2 78   →  u32 (LE) = 2026042402
```

画面の "FW Version: 2026042402" と完全一致。V4.4以前は`0xB5`のチャンク方式でASCII文字列として
しか読めなかった(バージョン文字列も含む)ファームウェア識別情報が、こちらは**4バイトの生の
整数**として一発で取れる、より軽量なコマンド。既存の`0xB5`(後述)と役割が重複しているように
見えるが、実装コストは`0xB2`の方が圧倒的に低いので、`read_firmware_version`的な用途には
こちらへの切り替えを検討する価値がある。

## 4. `cmd 0xB5`: モータ型番読み取り(既知フォーマットと一致)

既存実装([[myactuator-driver-design]]参照)通りの「flag固定+index選択、5文字ASCIIチャンク」
方式で、`index=01/02/03`から`"RMD-X" + "4-P36" + "-36"` = `"RMD-X4-P36-36"`が得られ、画面の
"Motor Name: RMD-X4-P36-36" と完全一致。**新規発見ではなく、既存実装の正しさの再確認**。

## 5. 既知コマンドの再確認

- `cmd 0x92`(multi-turn angle、既存実装通り): 応答 `49 F5 FF FF` → i32 = -2743 → ×0.01 =
  **-27.43°** — 画面の "1st Encoder: Calibrated, -27.43" と完全一致。既存実装の正しさを実機で
  再確認できた。

## 6. 未解明の新規コマンド

- `cmd 0x80`(request/replyとも全ゼロ8バイト): 接続後、数秒〜数十秒おきに定期的に送信される。
  内容から判断するに単純な生存確認(ping/heartbeat)と推測。値を運んでいないので詳細不明。
- `cmd 0x19`(トップレベル、`0xC0`のidx 0x19とは別物): リクエスト全ゼロ、応答
  `19 01 01 00 4E 1A 00 00` → bytes[4..8]をu32として読むと **6734**、これは
  `0xC0 idx=0x11`の読み取り値(6734.0)と一致する。同じ値を指しているのか偶然かは未確認。
  `0x80`/`0x92`/`0x19`の3つは常にセットで(この順で)呼ばれており、接続後の定期ヘルスチェックの
  一部と見られる。
- `cmd 0x20`: **「Enable CAN Filter」専用の書き込みコマンド**(§1.5で確定)。
  `20 02 00 00 <0|1> 00 00 00` — byte[4]がON/OFF。ただし無効化(0)を送っても実機に反映
  されない不具合(?)を確認済み(§1.5参照、要追加調査)。

## 7. 残っている未確定事項(優先順、実装をブロックするものではない)

1. `idx=0x10`/`0x11`(Exchange phase / Encoder Calibrat Valueのどちらがどちらか)の確定
2. `idx=0x3A`/`0x3C`の意味(2nd Encoder関連の可能性、要検証)
3. D-Axis Current列(P/I/D/R/T)がGUIから独立して読める操作の有無を確認(§1.2参照)
4. `idx=0x02`(Factory Time)がxlsxエクスポートでは`0`になるバグの原因(Setup Software側の
   問題であり、こちら側の実装には影響しない想定)
5. `cmd 0x20`でのEnable CAN Filter無効化が実機に反映されない件(§1.5)——電源再投入で
   反映されるのか、単にこのFWでは無効化非対応なのか

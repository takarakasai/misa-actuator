# RobStride 通信方式まとめ

このリポジトリで実装・検証した RobStride（RS04 実機で検証、EL05/RS05等は仕様書突き合わせのみ）
の CAN 通信方式を一箇所に整理する。実装は `robstride-protocol`（フレームのエンコード/デコード、
`no_std`）と `robstride-driver`（SocketCAN 越しの高水準API）の2層に分かれる。

関連ファイル:
- 実装: `src/can_id.rs`, `src/comm_type.rs`, `src/frame.rs`, `src/param.rs`（古典プロトコル）
- 実装: `src/param_table.rs`, `src/param_type_table.rs`（未公開の拡張パラメータ空間）
- 公式マニュアル: `ref/rs04_manual_en.md`, `ref/el05_manual_en.md`（PDFをMD化したもの）
- マニュアル突き合わせ結果: `rs04-el05-manual-analysis.md`（同ディレクトリ）
- HW実機検証ログ: `/home/takara/work/dp/robstride/incident_report.md`（SET_CAN_ID関連の障害調査）

## 0. 全体像 — 2つの独立したプロトコル層

RobStride には **性質の異なる2つの通信の仕組み**が存在し、混同しやすいので最初に整理する。

| | ① 私有プロトコル (classic) | ② 拡張パラメータ空間 (undocumented) |
|---|---|---|
| comm_type | 0–25（マニュアル記載） | 9, 19（**マニュアル未記載**。`motorstudio`のみが使用） |
| アドレス空間 | `ParamIndex`、主に `0x7xxx`（一部`0x2xxx`/`0x3xxx`も実装が使用） | `FunctionCode` = `(page<<8)\|row`、`0x0000`〜`0x30xx` |
| 値の型 | float/uint8/uint16/uint32 のみ（**文字列なし**） | 上記に加え **文字列型**（Name/BootCodeVersion等） |
| 1回の通信量 | 1リクエスト+1応答（8バイト） | 9: 1リクエスト+可変数の応答／19: 1トリガー+テーブル全体のストリーム |
| 用途 | MIT制御・モード切替・イネーブル・ゲイン調整など通常運用 | ファームウェアバージョン、型番等の識別情報、`motorstudio`のパラメータテーブル再現 |
| 発見経緯 | 公式マニュアル + `robstride_sandbox` 参照実装 | `motorstudio`とモータ間のCAN-FDキャプチャから逆解析（本セッション） |

**両者は同じ29bit拡張CAN IDのビットレイアウトを共有する**（下記1章）。comm_typeの値が違うだけで、
以降のバイト割り当ての意味は全く別物として解釈する必要がある。

## 1. 共通の物理層 — 拡張(29bit) CAN ID

```
bits  28..24       23..8           7..0
     +---------+----------------+-----------+
     | comm(5) |  extra_data(16)| dev_id(8) |
     +---------+----------------+-----------+
```

`build_can_id_raw(comm_type, extra_data, device_id)` / `parse_can_id(id)`（`can_id.rs`）が
共通のエンコード/デコードを提供する。1Mbps・標準DLC8。ホストID既定値 `DEFAULT_HOST_ID = 0xFD`。

## 2. 私有プロトコル（comm_type 0–25）

`comm_type.rs::CommType` に実装済み。マニュアル（`rs04_manual_en.md`）との突き合わせ結果は
`rs04-el05-manual-analysis.md` を参照——**RS04は完全一致、EL05/RS05はMITスケールの誤りを発見・
修正済み**（`velocity: 33.0→50.0`, `torque: 17.0→5.5`、`model.rs`）。

| type | 内容 | 実装 |
|---|---|---|
| 0 | Get device ID（UID取得・スキャン） | ✅ `scan_bus`, `Motor::ping` |
| 1 | Operation control（MITモード） | ✅ `Motor::mit_control` |
| 2 | Motor feedback（応答専用） | ✅ |
| 3 | Enable | ✅ `Motor::enable` |
| 4 | Disable | ✅ `Motor::disable` |
| 6 | Set mechanical zero（**揮発**、電源断で消失） | ✅ `Motor::set_zero` |
| 7 | Set CAN ID（アドレス再割当て） | ✅ `Motor::set_device_id`（下記4章参照、要注意） |
| 17 | Read parameter（`ParamIndex`、単発） | ✅ `Motor::read_param` |
| 18 | Write parameter | ✅ `Motor::write_param_f32` |
| 21 | Fault report | ✅（`recv_status`内で判定） |
| 22 | Save parameters（flash永続化） | ❌未実装 |
| 23 | Set baudrate | ❌未実装 |
| 24 | Active report | ❌未実装 |
| 25 | **Motor protocol modification**（private/CANopen/MIT切替、要電源再投入） | ❌未実装（下記5章） |
| 26 | **Version number read** | ❌未実装（下記6章、有望） |

### 2.1 `ParamIndex`（comm_type 17/18 の索引空間）

`param.rs`。公式マニュアルの「Read and Write a Single Parameter List」表と突き合わせ済み——
**`0x7xxx` 系は完全一致**。ただし `ParamIndex` には `MechOffset=0x2005` /
`MeasuredPosition=0x3016` / `MeasuredVelocity=0x3017` / `MeasuredTorque=0x302C` も含まれており、
これらは公式マニュアルの comm_type 17/18 表には載っていない（`robstride_sandbox` 参照実装からの
移植）。実機で `read_param(ParamIndex::MechOffset)` を試したところ **`0.0` が返り**、同じアドレスを
拡張パラメータ空間（comm_type 9、後述）経由で読んだ値（`3.88...`）と一致しなかった——つまり
**2つの空間は同一のストアを指しているわけではなさそう**で、詳細は未確定・要検証。運用上は
「`ParamIndex`経由は`0x7xxx`のみ安全に使う」という前提で扱うのが無難。

### 2.2 MITモード

`Motor::mit_control` / `MitScales::for_model`。モデルごとのスケール値は
`rs04-el05-manual-analysis.md` で全モデル分の突き合わせは未完了（RS-00/01/02/03/06は今回未検証）。

## 3. 拡張パラメータ空間（comm_type 9 / 19、undocumented）

`motorstudio`の「Parameter Setting」タブが使う仕組み。マニュアルに記載がなく、本セッション中に
実機とのCAN-FDキャプチャ（`motorstudio`操作を`candump`で観測）から逆解析した。読み取り専用。

### 3.1 共通: `FunctionCode`

`FunctionCode = (page << 8) | row` で1つのパラメータ行を指す（例: `0x1003` = `AppCodeVersion`）。
`param_type_table.rs` に公式マニュアルの全131行の型定義を収録済み（`lookup_param_type` /
`lookup_param_type_by_name`）。

**⚠️ 重要な注意**: 実機検証で、マニュアルの`FunctionCode`番号と実機ファームウェアの番号が
**完全には一致しない**ことを確認した（`0x2006`/`0x2007`で配線上の自己申告名が
`chasu_offset`/`status1`なのに対し、マニュアルは`MechPos_init`/`limit_torque`——単純な一律オフセット
ではなく個体/ファームウェアビルドにより一部フィールドが再割当てされている）。そのため
`ParamTableEntry::value_current_typed()`は**配線が自己申告する名前を優先照合**し、名前が未知の
場合のみ`FunctionCode`にフォールバックする。名前を取得できない単発読み（3.3節）は
`FunctionCode`のみに頼らざるを得ないため注意。

### 3.2 comm_type 19: バルクテーブルストリーム

`Motor::read_param_table(timeout) -> Vec<ParamTableEntry>`。

1. ホストが**対象モータのUID**（事前にcomm_type 0のGetDeviceIdで取得）をペイロードに載せて
   comm_type 19で送信（`extra=host_id, device=対象`）。**当初「応答」と誤認していたが、実際は
   これがリクエスト本体**——UIDでモータをアドレス指定する仕組みで、素の8bit CAN IDだけでは
   識別できないための設計と考えられる。
2. モータが全パラメータ行を連続フレームでストリーム: `extra=(slot<<8)|motor_id, device=host_id`
   （**役割が逆転**——応答の宛先=ホストが`device_id`位置に来る）。ペイロード`[row, page, ...6バイト]`。
   slot `0-2`が名前のASCII断片（最大18バイト）、slot `6-8`（時々9）が値の断片。
   `FunctionCode 0x0000`に巻き戻ったら終了（明示的な終端マーカーは無い）。
3. 文字列型は名前/値ともASCII、数値型は`[MinValue:i16][MaxValue:i16][不明4バイト][CurrentValue:実際の型幅]`
   （中間4バイトの意味は未解明、恐らく型/属性タグ）。

約130行を数百フレームでストリームするため、**1行だけ欲しい場合は3.3節の方が大幅に高速**
（131行 vs 1リクエスト+数フレーム、実測で数百フレーム→5フレーム・53ms程度に短縮）。

### 3.3 comm_type 9: 単一行読み取り（低コスト）

`Motor::read_single_param(function_code, timeout) -> Vec<u8>`。

1. ホスト → モータ: `extra=host_id, device=対象`, ペイロード`[row, page, 0,0,0,0,0,0]`。
2. モータ → ホスト: `extra=motor_id`（slotビット無し）, `device=host_id`,
   ペイロード`[row, page, 0x0A(固定?), sub_index(0..=3), ...4バイト]`。

**フレーム数が行によって異なる**——文字列型（`0x1000-0x1007`）は4サブフレーム(16バイト)固定だが、
数値型は**1サブフレームのみ**（`CurrentValue`のみ、offsetは0から。3.2節のバルクストリームとは
オフセットが異なる点に注意）で応答することを実機で確認した。当初「4フレーム固定で待つ」実装で
数値アドレスがタイムアウトするバグがあり、「応答が途切れたら打ち切る」方式に修正済み。

名前を取得できないため型判定は`FunctionCode`のみに依存する。3.1節の注意点の通り、
一部行では不正確な可能性がある——疑わしい場合は`param-table`（3.2節）で該当行の
自己申告名を先に確認すること。

### 3.4 CLIコマンド対応

```
robstride-cli version                    # AppCodeVersion のみ (comm_type 9、高速)
robstride-cli read-param 0x2005          # 任意の FunctionCode 1件 (comm_type 9)
robstride-cli param-table                # 全131行ダンプ (comm_type 19、低速だが名前照合が正確)
```

## 4. comm_type 7: Set CAN ID（アドレス再割当て）— 実機インシデント記録

`Motor::set_device_id(new_id)`。リクエストは`extra=new_id, device=現在のID`。

**本セッションで実機インシデントが発生**: RS04のIDを127→1に変更後、**丸1日以上にわたり
CAN応答が完全に途絶**（配線・終端抵抗を再確認・修正、電源を複数回、30秒以上の完全放電を含め
入れ直しても復旧せず、メーカー公式ツール`motorstudio`でも認識不可）。翌日、特に何もしていない
状態で再スキャンしたところ**新ID(1)で正常応答するようになっていた**（UID一致で同一個体と確認、
電圧・温度・位置等すべて健全）。原因不明——ファームウェア内部の何らかの処理に長時間を要した
可能性があるが未解明。詳細ログは`/home/takara/work/dp/robstride/incident_report.md`。

**教訓**: `set_device_id`はread-onlyではない数少ない実験的コマンドの1つ。実運用前に十分な
余裕を持ってテストし、変更後は最低でも電源再投入＋長め（数分〜)の待機を見込むこと。

## 5. comm_type 25: プロトコルモード切替（private/CANopen/MIT）

マニュアルに存在（`rs04_manual_en.md`）が**未実装**。ペイロード`byte[6]`(`F_CMD`)で
`0`=私有プロトコル(既定)、`1`=CANopen、`2`=MITプロトコルを選択、**要電源再投入**。

これは本セッション序盤で`motorstudio`画面に見えた **「Canopen to Private Protocol」ボタン**の
正体だと考えられる（当時は未解決のまま棚上げにしていた）。CANopenモードに切り替わったRobStride
モータとやり取りする必要が生じた場合はこのコマンドの実装を検討する。

## 6. comm_type 26: バージョン番号読み取り（公式・未実装・有望）

マニュアルに記載があるが**未実装**。注記: マニュアル本文中のコード表記は`0x4`となっており、
見出しの「type 26」(=0x1A)と矛盾する（マニュアル自体のOCR/転記ミスの可能性が高いとマニュアル内に
注記あり）。

リクエスト: `Byte[0]=0x00, Byte[1]=0xC4`。応答: comm_type `0x2`（通常のフィードバックフレームと
同型）、`Byte0=0x00, Byte1=0xC4, Byte2=0x56, Byte3~6=バージョン番号（上位バイトから）`。

**本セッションの`candump`キャプチャで、この形と一致するフレームのやり取りを偶然観測済み**
（`comm_type=4`のリクエストに対し`comm_type=2`, `00 C4 56 ...`という応答——おそらく
`motorstudio`が自動的に発行したもの）。3章の`AppCodeVersion`文字列読み取りより**さらに単純な
1往復のみ**で完結する可能性があり、将来的にこちらへの置き換え/実装を検討する価値がある。
ただし今回は狙って送信して検証したわけではないため、正式な実装前に自分で送信して確認することを
推奨。

## 7. どれを使うべきか（早見表）

| やりたいこと | 使うコマンド |
|---|---|
| モータのスキャン・存在確認 | comm_type 0 (`scan_bus`) |
| MIT制御・位置/速度/トルク制御 | comm_type 1/17/18（`Motor`の各種メソッド） |
| ゲイン・リミット等`0x7xxx`パラメータの読み書き | comm_type 17/18 (`read_param`/`write_param_f32`) |
| ファームウェアバージョンだけ知りたい | comm_type 9 (`read_firmware_version` / `robstride-cli version`) |
| 特定の1パラメータ（型番・名前不明でも良い）を覗きたい | comm_type 9 (`read_single_param` / `read-param`) |
| 全パラメータを一覧・型を正確に知りたい | comm_type 19 (`read_param_table` / `param-table`) |
| CAN IDを変更したい | comm_type 7 (`set_device_id`) — **4章のインシデントに注意** |

## 8. 実機トラブルシュート（配線起因の教訓）

本セッションで発生した「完全な無応答」は、最終的に**CAN_H/CAN_L の配線極性の逆接続**が原因
だった（MyActuator側で先に発見、その後RobStride側でも同じ問題が判明）。切り分けの優先順位：

1. 電源・CANインタフェースのUP状態
2. **CAN_H/CAN_Lの極性**（テスターで確認する前に、まず入れ替えて試すのが早い場合がある）
3. 終端抵抗（電源OFF状態でCAN_H-CAN_L間の抵抗を測定。両端終端で約60Ω、片端のみで約120Ω、
   無限大なら断線）
4. ボーレート（1Mbps以外の可能性、`ip link set ... bitrate`で切替）
5. モータIDの想定違い（全ID域スキャン）

`scripts/hw_check.sh`（ワークスペース直下）に読み取り専用の疎通確認＋オプトインの動作確認
ハーネスを用意している。

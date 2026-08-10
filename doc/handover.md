# 引継ぎ: Windows 対応と GUI

2026-08-02 時点。**コンテキストを失った状態から再開するための文書**。

この文書は 2026-08-02 に 4 つの独立レビューを受けて改訂した。初版には
「LK Motor 検証済み」など**事実でない記載が複数あった**ので、旧版を参照している
場合は破棄すること。改訂で足した最大のものは §2（安全）で、初版には**ブレーキの
記述が一語も無かった**。

## 読む順番

1. **[`../README.md`](../README.md)** — crate 一覧、アーキテクチャ図、基本コマンド。
   **この文書には crate 一覧を書いていない。** 19 メンバーのうち本文が名指しするのは
   数個で、残りは README にある。まずそちらを読むこと
2. この文書の §2（安全）— **実機に触る前に**
3. [`windows.md`](windows.md) — セットアップ、アダプタ、GUI のビルドと配布
4. [`bench-measurements-2026-07-30.md`](bench-measurements-2026-07-30.md) —
   実測の一次資料。この文書の §4 はここの要約に過ぎない。**数値で迷ったら必ず原典へ**

### その他の文書

| 文書 | 内容 |
|---|---|
| [`architecture.md`](architecture.md) | **プロセス／スレッド構成とレイヤ構成**。§3 が設計判断なのに対し、あちらは構造そのもの |
| [`data/*.csv`](data/) | 摩擦測定の生データ（3 機種 × static/kinetic） |
| [`rs04-myactuator-data-map.md`](rs04-myactuator-data-map.md) | RS-04 と MyActuator のレジスタ対応 |
| `crates/misa-actuator-tui/doc/vendor-identity-coverage.md` | **機種識別の可否マトリクス**。§4 の主題の要約 |
| `crates/robstride-protocol/doc/motorstudio-detection-devices.md` | 公式ツールの機種判別の解析 |
| `crates/robstride-protocol/doc/param-table-firmware-divergence.md` | 実機ファームとマニュアルの乖離 |
| `crates/robstride-protocol/doc/communication-protocols.md` | 私有プロトコルと非公式空間、事故記録 |
| `crates/robstride-protocol/doc/rs04-el05-manual-analysis.md` | RS04 / EL05 のマニュアル突合 |
| `crates/damiao-protocol/doc/can-fd-bring-up.md` | CAN-FD 立ち上げの切り分け |
| `crates/damiao-protocol/doc/dm-j-series-cross-model-reference.md` | 減速比表の出典 |
| `crates/myactuator-protocol/doc/setup-software-c0-param-protocol.md` | 0xC0 空間と 0xB5 |
| `crates/myactuator-protocol/doc/data-map.md` | レジスタ対応 |
| `crates/lkmotor-protocol/doc/can-rs485-manual-analysis.md` | LK Motor |

各 protocol crate の **`ref/` にベンダーマニュアルの markdown 版が入っている**
（10 ファイル）。grep できる。これは非自明だが非常に役に立つ。

---

## 1. いまどうなっているか

Linux/SocketCAN 専用だったワークスペースを Windows に移植し、その上に GUI を載せた。

### 実機で検証済み

- **PCAN バックエンド** — classic / CAN-FD とも
- **RobStride**（RS-04、EduLite05）、**DAMIAO**（DM-J4310）、**MyActuator**（RMD-X4-P36-36）
- 上記 3 社のモデル自動判別
- GUI 5 タブすべて（Console / Sysid / Characterize / Monitor / Params）
- リリースビルドと NSIS インストーラ
- **Characterize の測定エンジン**（`misa-sysid`）— 3 機種で摩擦分布を取得済み。
  データは [`data/`](data/) に

### 未検証

- **LK Motor**。コンパイルは通るが**実機に接続したことがない**。ベンチ記録に
  「`/dev/ttyUSB*` が見えず RS485 アダプタの接続確認が先」とある段階で止まっている。
  `hw_check` の対象にも入っていない
- **SLCAN**。3 バックエンド中これだけ実機未確認。なお **SLCAN は CAN-FD 非対応**
  （`slcan.rs` が拒否する）なので、FD ノード化した DM-J4310 の検証には**使えない**
- **GUI の Characterize タブ**（測定エンジン自体は実機検証済み。タブはシミュレータのみ）
- 12〜64 バイトの CAN-FD DLC 経路（DAMIAO は常に 8 バイトなので通らない）
- 飽和させない設定での RS-04 Sysid 撮り直し

### 未実装

- gs_usb / candleLight バックエンド

### 補足

- 現在のインストーラは接続失敗バナー（`6d42be7`）より前のビルド。再ビルドが要る
- `crates/` と `ui/src/` に **TODO/FIXME/`#[ignore]` は 1 件も無い**。
  隠れたバックログは無く、§9 の表が全部
- `misa-actuator-gui` は `default-members` 外。素の `cargo test` では 9 テストが
  走らない。`cargo test --workspace` なら走る（`ui/dist` のビルドが先に要る）

---

## 2. 安全

**実機に触る前に読むこと。** 0.8 N·m から 400 N·m まである。

### `disable` は保持される安全状態ではない（DAMIAO で実測）

ベンチ §1:

> `disable` 直後に `mit --kp 5` を送ると **1.013 N·m を出力**した。次の制御フレームで
> 再励磁される。`status` も内部でゼロゲイン MIT フレームを送るため `err=Enabled` を
> 再表示する。**投入時から励磁状態でもあった。**

`Session` には停止手段が 3 系統ある（STOP フラグ / ウォッチドッグスレッド /
`Drop` でのワーカー join）が、**それはソフトウェアの停止経路の話**で、モータが
無励磁でラッチされることを意味しない。物理的に止めたいなら電源を切る。

### ホールディングブレーキ

`myactuator-cli brake` は**フラグ無しで解放する**（施錠は `--lock`）。
垂直軸に負荷が吊ってあれば落下する。

**解放前に無負荷を確認する手順**（実際に使った）:

```
myactuator-cli -i pcan:usb1@1M -m 1 brake        # 解放
myactuator-cli -i pcan:usb1@1M -m 1 status       # 数回繰り返す
```

位置が動かず、トルクがノイズ範囲（±0.1 N·m 程度）なら無負荷。ドリフトするなら
負荷が掛かっている。

**未確認**: 解放時に保持トルクがゼロなのか「最後に指令した値」なのかは調べていない。

### CLI は Session の安全網の外にいる

`robstride-cli enable` は **Drop での自動 disable を意図的に無効化する**:

```rust
// Caller likely wants the motor to stay enabled — leak the handle
// so Drop doesn't auto-disable.
std::mem::forget(motor);
```

ターミナルを閉じても通電したまま、ホスト側の監視はゼロ。診断作業（§8）は CLI で
行うので、これは日常的に踏む経路。

### 設定を壊す操作

| コマンド | 危険 |
|---|---|
| `damiao-cli reg-write RID VALUE --save` | **値の検証は無い**。`PMAX`/`VMAX`/`TMAX`（MIT スケール）、保護閾値、`CAN_BR` まで書けて flash に即確定。2026-08-02 以降、危険レジスタは書き込み前に警告を出す（下記） |
| `robstride-cli set-id` | ID 変更。**過去に RS-04 が 1 日応答しなくなった実例あり** |
| `robstride-cli set-zero` | **永続性が不明**。DAMIAO の `--nvm`、MyActuator の `--rom` に相当するフラグも説明も無い。誰も確かめていない |
| `damiao-cli zero --nvm` / `myactuator-cli zero --rom` | flash 書き込み。摩耗する |
| `damiao-cli factory-reset` | 校正値ごと消える |

`lkmotor-cli` には**正しいパターンが既にある** — `is_risky()` で危険パラメータを
判定し、上記 RS-04 の事故を引用して警告する。**2026-08-02 に `damiao-cli reg-write`
にも同じものを入れた**（`RegisterRisk` / `register_risk()`）。通信系（`MST_ID` /
`ESC_ID` / `TIMEOUT` / `CAN_BR`）、MIT スケール（`PMAX`/`VMAX`/`TMAX`）、保護閾値、
マニュアルが読み取り専用とする校正・識別値の 4 クラスを、書き込み前に警告する。

**警告するだけで、値そのものは検証していない。** `--save` の flash 確定も止めない
（CLI 全体が警告して続行する方式で、確認プロンプトはどこにも無い）。範囲検証は
未着手 — レジスタの範囲は `Rid` の doc コメントに転記済みなので、やるならそこから。

### 安全エンベロープの実効性

`Characterize` は `SafetyLimits::gentle()`（1 N·m / 60 °C / ±0.5 rad / 30 s）で走る。
**この枠は思ったほど効かない。**

- **RS-04 の静摩擦分布の内側にある。** 実測 n=20 で mean 0.912、sd 0.118、
  **6/20 が 1.0 N·m を超える**。既定のまま実機 breakaway を回すと約 3 割が
  トルク上限で打ち切られ、検閲された値を返す
- **温度ガードは RS-04 で何もしない。** `torque_ok` / `temperature_ok` は
  非有限値を「範囲内」として通す（意図的な設計で理由もコメントにある）。
  ところが **RS-04 は `mit_control` 以外で温度もトルクも返さない**ので、
  quasi-static 経路ではガードが常に素通りする
- **`Chirp` にはエンベロープが無い。** `run_chirp_with` は `SafetyLimits` を取らない。
  duration と abort フラグだけ。トルクチャネルの chirp は振幅指定だけが安全装置

### 速度上限は目標値であって保証ではない

`hw_check_20260801_172404.log`: 速度上限 1.00 rad/s を指令して**ピーク 1.816 rad/s**
（+82%）。プロファイル生成器の目標であってクランプではない。

---

## 3. 構造と、そうした理由

**crate 一覧は README にある。** ここには設計判断だけ書く。

### 抽象は 2 層ある

- **`Actuator`**（`crates/misa-actuator/src/traits.rs`、16 メソッド。うち 5 つは
  既定実装 — `current_run_mode_hint` / `is_enabled_hint` / `scan_bus` /
  `probe_motor` / `read_parameters`）。GUI も TUI も sysid もこれしか見ない
- **ベンダーごとのバストレイト**（`RobstrideBus` / `DamiaoBus` / `MyActuatorBus` /
  `LkBus`）。ドライバをトランスポートに対してジェネリックにする。
  **`protocol` / `driver` / `cli` の三つ組がベンダーごとに存在する理由がこれ**

`Actuator` の拡張は**既定実装つきメソッド**で足す。そうしないと 4 ドライバ全部を
同時に触ることになる。

### `misa-can`: バックエンド 3 種を 1 つの trait に

`[backend:]device[@bitrate[,data]][?opts]` で `can0` / `pcan:usb1@1M,5M` /
`slcan:COM5` を書き分ける。

**PCAN は `PCANBasic.dll` を実行時 `LoadLibrary` で読む。** リンク時依存にすると
PEAK ドライバの無いマシンで**起動すらできなくなる**。遅延ロードなので、アダプタが
無くてもアプリは立ち上がり、接続時に初めて明示的なエラーが出る。

### `misa-actuator-core`: セッションとワーカー

- **Setpoint は上書き、Command はキュー。** 位置スライダを動かすたびに指令が
  積み上がってはいけない。逆に Enable/Disable は取りこぼしてはいけない
- **ウォッチドッグは専用スレッド。** ジョブはワーカーのループを数十秒占有する。
  ループ内チェックでは**まさに一番危険な時間帯に動かない**
- **テレメトリだけ別チャネルで落としてよい。** UI が遅れたらプロットのサンプルは
  捨てる。状態変化とエラーは捨てない

### `misa-actuator-sim`

`bench-measurements-2026-07-30.md` の実測値からプリセットを作ってある
（`ideal` / `dm4310` / `rs04`）。ハードウェア無しで GUI と sysid を動かせる。

**プリセットの値は代表値ではなく特定のサンプルに近い。** `rs04` の
`stiction = 0.902` は実測分布（mean 0.912、sd 0.118）の 1 点。シミュレータの出力が
0.902 付近に一致しても、それは自己整合であって較正の検証ではない。

### GUI

`crates/misa-actuator-gui`（Rust）+ `ui/`（React / TypeScript / uPlot）。

**debug ビルドは Vite dev サーバを見る**（`devUrl`）。`cargo run` 単体では白画面。
開発は `npm run app`、配布物は `npm run app:build`。

---

## 4. 実機でしか分からなかったこと

**ここが最大の資産。** モータが無いと再取得できない。
**数値の一次資料は [`bench-measurements-2026-07-30.md`](bench-measurements-2026-07-30.md)。**

### 1 機種だけでは見つからないバグがある

ベンチ §7.5 の教訓。**このプロジェクトで最も転用価値が高い。**

`Actuator::set_torque` が RobStride で **N·m ではなくアンペアを送っていた**
（Kt = 1.5093 倍の誤り）。核心トレイトの静かなスケーリング事故で、
**2 機種の推定値が食い違ったから見つかった**。1 機種なら両方見逃していた。

### RobStride: 機種はファームウェアのバージョンで分かる（確信度に注意）

**マイナー成分が機種番号、メジャーが製品ライン**（`0` = RS、`10` = EduLite）。
実機 RS-04 は `0.4.1.32`、EduLite05 は `10.5.0.1`。

**証拠の内訳を正確に**:

- **強い**: motorstudio の "Detection Devices" が**バス上に GetDeviceId と
  バージョン読みしか流さない**ことをキャプチャで確認済み。パラメータ読みは無い。
  この消去法が結論を許している
- **弱い**: 公式リリース資産 7 件のファイル名は**ベンダーが付けた名前**であって
  測定ではない。機種ごとに `0.N.x.y` で切っていれば定義上そうなる
- **n = 1 / ライン**。RS 系 1 台、EduLite 系 1 台
- **「メジャー = ライン」は 1 バイト × 1 台。** しかもベンダー自身の
  `AppCodeVersion` 文字列は `1.0.5.0.1` と**5 成分**で、`0x0A` を「10」と
  レンダリングしていない。先頭バイトが 2 成分を詰めている読み方も同じくらい整合する
  （その場合も機種桁は byte[1] なので結論は生き残るが、「メジャー = ライン」は残らない）

**反証されうる条件**: EL01/02/04 のメジャーが 10 でない / RS 系のメジャーが
バンプされる / EduLite05 が `0.5.x.x` を積む / RS-07 以降が出る。
**ヘッドルームはゼロ**で、ベンダーがメジャーを一度上げただけで対応表は壊れる。
未知のメジャーでは `None` を返すので、誤判定ではなく判定不能になる（フェイルセーフ）。

`limit_torque = 115` は独立した第 2 の判別チャネルだが、
**`from_firmware_version` はこれと突き合わせていない。**

### RobStride: 行き止まりだった経路

同じ調査を繰り返さないこと。

| 経路 | 結果 |
|---|---|
| `0x0000 Name` | RS-04 / EL05 とも全 FF（工場未書き込み） |
| `0x0001 BarCode` | 同上 |
| `0x1007 AppCodeName` | EL05 は `EL_motor`、**RS-04 は `motor`** で判別不能 |
| comm type 0 | MCU UID のみ |
| バージョン応答 Byte7 | **両機種とも `0x07`** — モデルコードではない |

Byte7 の否定は強い証拠。**MIT スケールが 21.8 倍違う 2 台が同じ値を返した。**

### RobStride: バージョン読み出しの罠

マニュアルの「コピペミスに見える記述」が**両方とも正しかった**:

```
要求  ID=0x0400FD01  data=00 C4 00 00 00 00 00 00
応答  ID=0x020001FD  data=00 C4 56 00 04 01 20 07   → 0.4.1.32
```

- **要求の comm type 欄は `4`** — `DISABLE` と同じ値。`00 C4` でしか区別できない
- **応答は comm type `2`** — 通常フィードバックと同じ。`00 C4 56` でしか区別できない

**応答をフィードバックとしてデコードすると、それらしい数値のゴミになる。**

### RobStride RS-04: この個体の癖

| | |
|---|---|
| firmware | 0.4.1.32 |
| MCU UID | `0F302000C03B5CC3`（受信バイト列の**逆順**が motorstudio 表示） |
| `limit_torque` (0x700B) | **115 N·m** — family 中 RS-04 でしか表現できず、単独で一意特定できる |
| `limit_spd` (0x7017) | **1 rad/s** — 保存値であって能力ではない。ドライバが Position モード投入前に既定 5.0 で**上書きする** |
| **`MeasuredTorque` (0x302C)** | **常に 0**。3 run / 72 サンプルで非ゼロゼロ件。MIT では 109/109 非ゼロ |
| 温度 | `mit_control` 以外では NaN |
| `loc_kp` | 80。約 0.9 N·m の摩擦に対して弱く、load-map が**ハンチングする** |
| Kt（実測） | **1.5093** |

**`MeasuredTorque` = 0 の影響**: Position/Torque モードのフィードバックがこれを
使うので、`characterize load-map` が何も測れず、安全エンベロープのトルク上限も
検査対象を失う。回避は `--kt 1.5093`（`IqFilt` から `torque = current × Kt` を合成）。
なお `--report-current` は**ループレートを半減させる**ので chirp には向かない。

**熱特性はリーシュでは構造的に測れない**（ベンチ §4）。軸が
`err = −tau_ff/kp` で釣り合うため、指令 2.0 N·m に対し実測 0.141〜0.251 N·m
（伝達率 13%）。治具が要る。

### RobStride EduLite05

| | |
|---|---|
| firmware | 10.5.0.1 |
| MCU UID | `06B6311821300A62` |
| `limit_torque` | 6 N·m |
| `limit_spd` | 50 rad/s（自身の MIT 速度スケールを超えている） |
| `AppCodeName` | `EL_motor` |

### RS-04 の摩擦分布（`data/rs04-*.csv`）

| | n | mean | sd | 備考 |
|---|---|---|---|---|
| static（breakaway） | 20 | **0.912** | **0.118** | CV 12.9%、range 0.657〜1.076。**6 件が 1.0 超** |
| kinetic | 20 | **0.534** | **0.011** | CV 2%。**こちらは信用できる** |

**static は i.i.d. ではない。** `position_rad` が −2.280 → −2.108 と単調に動いており、
上位 6 件が後半に固まっている。空間依存かドリフトがある。**1 点で代表させないこと。**

### DAMIAO: 機種は減速比で分かる

`GR`（RID 20）が機種ごとに一意。型番の下 2 桁と一致する。

| gr | モデル |
|---|---|
| 7 | DM3507 |
| 10 | DM4310 / 4310P |
| 40 | DM4340 / 4340P |
| 9 | DM8009 / 8009P |
| 22 | DM10422P |
| 48 | DM6248P |

`P` はマニュアル上トルク・回転数・減速比・レジスタレイアウトが同一なので実害なし。

**そもそも DAMIAO はスケーリングに機種判別が要らない。** `PMAX`/`VMAX`/`TMAX` が
量子化レンジそのもの。RobStride の 21.8 倍問題（120 / 5.5）が構造的に存在しない。

### DAMIAO DM-J4310

| | |
|---|---|
| CAN_ID | **16 (0x10)** — 既定スキャン範囲 1〜8 の外 |
| PMAX / VMAX / TMAX | 12.5 / 30 / 10 |
| `can_br` | RID 35。FD 設定はここ |

**FD ノードにすると classic では一切通信できなくなる**（実測 36/36 タイムアウト）。
切り替え後に classic で試して「壊れた」と誤解しないこと。

### DAMIAO CAN-FD: レジスタ読みの欠落（**原因未確定**）

FD 化後、レジスタ読みが **11〜15%** 無応答になった（n は記録していない。
数十リード規模なので信頼区間は広い。「約 1/7」は観測帯の上端）。

観測:
- ランダムなレジスタ / タイムアウト 10 倍でも不変 / アダプタのバスエラー報告ゼロ /
  送信は全成功 / classic では出ない

**再送（3 回 + 5ms）で 60/60 成功したが、これは切り分けになっていない。**
再送と `ReadOutcome` の修正が**同一コミット `d67afc8` に入っている**。
`ReadOutcome` のバグ（スキップを「キューが空」と報告し、自動リセットイベントを
待って既にキューにあるフレームを取り逃す）は**症状と完全一致する**。
再送が既に直ったバグを覆い隠しているだけかもしれない。

**除外しきれていない仮説**:
- `send()` の成功は**キュー受理**であってバス到達でも ACK でもない
  （`CAN_WriteFD` の戻り値を見ているだけ）
- **ホスト側 RX キュー溢れは監視していない。** `pcan.rs` に
  `PCAN_ERROR_QOVERRUN` の定数が存在しない。「バスエラー無し」はホストキューの
  取りこぼしを否定しない
- 散発的な CRC/form エラーはコントローラが再送し、エラーカウンタが 96 を
  超えるまで `BUSLIGHT` は立たない。**「バスエラー無し」は信号品質も否定しない**

ドライバのコメントは**ファームウェアの取りこぼし**説を採っている。配線説と
どちらとも決まっていない。§9 に再測定の手順を書いた。

**制御ループ側は今も落ちている**: FD で 48 リード中 8 件（16.7%）が無応答。
制御経路は再送しない（許容して継続する設計、§6）。

### MyActuator RMD-X4-P36-36

**⚠ この個体は工場出荷状態ではない。** 差分テストで書き込み・flash 保存した:

| パラメータ | 元 | 現在 |
|---|---|---|
| `KT_OUT` | 1.2 | **1.1** |
| `Nominal Speed` | 3000 | **2800** |
| `Encoder2 Abnormal Speed` | 5.0 | **4.8** |
| `Rated Current` | 8.625 | **8.62**（ベンダー GUI の丸めバグで欠落） |

**したがって `kt_out = 1.1` はモータの仕様ではなく我々が書いた値。**
トルク計算に使う前に確認すること。

**`0xB5`（型番）はチャンク方式。flag は `0x01`**（`0x00` が自然な推測だが、
間違えるとエラーにならず**黙って空文字列**になる）:

```
TX  B5 01 01 …  RX  B5 01 01 52 4D 44 2D 58   "RMD-X"
TX  B5 01 02 …  RX  B5 01 02 34 2D 50 33 36   "4-P36"
TX  B5 01 03 …  RX  B5 01 03 2D 33 36 00 00   "-36"
```

**`0x9C` の角度は 1°/LSB と粗い**（`0x92` は 0.01°/LSB）。制御応答の位置を
小数 3 桁で出すと、0.1 rad の指令が 16% オーバーシュートしたように見える。

その他: `0x94`（単回転角）はこの個体で `0xFFFF` を返す（この機種では無意味）。
「Enable CAN Filter」はこのファームウェアで無効化できない。

### MyActuator: 未解明

- `cmd 0x19`: `TX 19 00…` → `RX 19 01 01 00 4E 1A 00 00`
- **`cmd 0x80`**: 読み出しシーケンス中に送られる。こちらの `Cmd::Shutdown`
  （出力オフ）と同コード。**用途が確定するまで生存確認として送ってはいけない**
- `0xC0` index `0x3A`

### LK Motor

**実機未検証。** ただしベンチ記録に 1 点重要な指摘がある:
**4 社中唯一 `iq_raw` を実測値として返す**ので、Kt 測定が最も直接的にできる。

また `vendor-identity-coverage.md` によれば、**機種識別用のレジスタがマニュアル上
存在しない**。§4 の機種判別の話から構造的に除外される。

---

## 5. 直したバグ（再発しやすいもの）

### Windows 固有

**clap のスタックオーバーフロー。** Windows のメインスレッドスタックは 1 MB
（Linux は 8 MB）。clap derive は全サブコマンドの構築を 1 つの関数に展開するので、
`robstride-cli --help` が `main` の 1 行目に入る前に落ちた。
→ `.cargo/config.toml` で `/STACK:8388608`。

**タイマ分解能。** Windows の sleep は既定で ~15.6 ms（1/64 秒）に丸められる。
→ `misa-actuator::realtime` で `timeBeginPeriod(1)` と絶対デッドライン。

### スケーリング

**`Actuator::set_torque` が RobStride でアンペアを送っていた**（上記 §4）。

**`--model` の既定が `Edulite05` だった。** 「未指定」と「EduLite05 を選んだ」が
同じ文字列で区別できず、RS-04 に対して黙って RS-05 のスケール（21.8 倍差）で
動いていた。→ `MODEL_UNSPECIFIED = ""`。RobStride は未指定を拒否する。

**`from_gear_ratio` が P 版を無条件スキップしていた。** `DM6248P` と `DM10422P` は
P 版しか存在しないので識別不能になっていた。実機は DM4310 なので気づけず、
全機種網羅テストが捕まえた。

### 配線契約

**`rename_all_fields` の付け忘れ。** serde の `rename_all` は**バリアント名しか**
変えない。付け忘れると `timeout_ms` が snake_case で出てフロントで `undefined` に
なる — パースエラーではなく黙って壊れる。**黒画面を 2 回起こしている。**

`protocol.rs` のテストが配線契約を押さえている。**バグった期待値を固定しないこと**
（一度やった: `"timeout_ms":50` をテストに書いてバグを仕様化していた）。

### 受信経路

**`try_read` がスキップを「キューが空」と報告していた。** エラー/ステータス/RTR を
弾いたとき `Ok(None)` を返すと、呼び出し側は自動リセットイベントを待ちに入る。
**目的のフレームが既にキューにあっても再発火しない。**
→ `ReadOutcome::{Frame, Empty, Skipped}` に分離。

**制御ループが 1 回の欠落で中断していた。** DAMIAO FD で 1/7 落ちるので 1 秒の
保持すら完走しない。しかも**指令は効いていてモータは動いていた**のに「応答なし」に
見えた。→ 連続 5 回まで許容、欠落数を必ず報告。

### GUI レイアウト

**`.app` が行数固定のグリッドだった。** バナー 2 つが条件付きなので子要素が 3〜5 個に
変動し、バナーが無いと `main` が `auto` 行に落ちる。
**全タブのプロットが content 高さに潰れていた。**

**uPlot がホストの `clientHeight` をそのまま使う。** 制約が無いと canvas が
8504px になる。凡例はホスト内・canvas の下なので、全高を渡すと次の要素と重なる。

**回転した 1 文字の軸ラベルは読めない。** `V` が `<` に見えた。

### シミュレータ

**制御則が呼び出しごとに 1 回しか評価されていなかった。** 速度制御が 27 rad/s に
発散した。→ プラントのサブステップ（2 kHz）で毎回評価。

---

## 6. 環境の罠

### PowerShell

| 罠 | 対処 |
|---|---|
| `npm` が実行ポリシーで弾かれる | **`npm.cmd`** を使う |
| ネイティブコマンドへの `2>&1` | PS 5.1 は stderr を ErrorRecord で包み、終了 0 でも `$?` が false。`Start-Process` + リダイレクトファイル |
| `\| Select-Object -First N` | パイプを早期に閉じて**プロセスを殺す**。PCAN チャネルが初期化されたまま残り、次回 `driver state is wrong` |
| `Start-Process -Environment` | PS 5.1 に無い。親の環境変数を継承させる |
| バッチファイルの非 ASCII | OEM コードページで解釈され `'M' は認識されていません` になる |

### OneDrive

**同期フォルダ配下でビルドしてはいけない。** 一度成功した後、以降のフロントエンド
ビルドが `EPERM: ui/dist/index.html` で失敗する。こちらのプロセスは全滅済みなのに
削除もリネームもできない。

判別: `Get-Item -Force <dir>` の `Attributes` に `ReadOnly, ReparsePoint` が出れば
オンデマンド管理下。`crates` にも出るなら全体が同期対象。

`tsc --noEmit`（= `npm run typecheck`）は書き込まないので通る。

### Tauri

- **debug ビルドは Vite dev サーバが要る**。無いと白画面
- **バンドラが GitHub から NSIS をダウンロードする**（初回のみ）。オフラインでは
  exe はできてもバンドルで失敗する
- **`webviewInstallMode` 未設定** = 導入時にネット接続が必要。オフライン配布なら
  `embedBootstrapper`（+150 MB）
- `npm run app:info`（`tauri info`）が設定不良の第一診断

### 実機

**RobStride の param table 読み出しがモータを黙らせる。** 2 回再現、電源再投入が
必要。`AppCodeName` が `"moto"` と 1 文字切れて返るのと同時に起きた。
→ 接続経路から外し、`identify --deep` で明示指定時のみ。

---

## 7. 診断ツール

| コマンド | 用途 |
|---|---|
| `robstride-cli dump` / `myactuator-cli dump` | 受信専用。デコードして逐次表示。**公式ツールを操作しながら観測できる**。`0xB5` の flag も RobStride のバージョン読みもこれで判明 |
| `robstride-cli identify` | 型番判別。既定は文書化済み経路のみ、`--deep` で非公式空間 |
| `damiao-cli --fd-link-classic-frames` | **FD リンクで classic フレームを送る**。「モータが FD 非対応」と「こちらの FD が壊れている」を切り分ける |
| `misa-actuator-identify` | 全ベンダー横断の識別（`misa-actuator-tui` の bin） |
| `misa-actuator-monitor` | **TOML 設定で複数ベンダー・複数モータを読み取り専用監視**（同上） |
| `scripts/hw_check.ps1` / `.cmd` / `.sh` | 実機スモークテスト。対象は **RobStride と MyActuator のみ**。`-Motion` は軸が自由であることが前提。`-RsTorque` の既定 1.2 N·m は RS-04 の 0.902 breakaway を超えるため |

**推測で切り分けない。** `0xB5` の flag は構造の推測が当たっていたのに 1 バイト
外れていた。キャプチャが無ければ「実装したが動かない」で止まっていた。

---

## 8. 判断の記録（蒸し返さないため）

### Params は読み取り専用（GUI と `Actuator` トレイトについて）

`Actuator::read_parameters(deep)` に書き込みの対はない。

**根拠（正確に）**: 実機ファームとマニュアルの対応が 84 行中 32 行食い違っている。
テーブルを信じて `0x2009`（マニュアルは `motor_baud`）に書くと実際は `CAN_ID` に
着弾すると**予測される**。これは実施していない。一方、**専用の `set_device_id`
フレームでの ID 変更では実際に RS-04 が 1 日応答しなくなった**。予測と実例は別物だが、
失敗様式は同じ。

**ただし `damiao-cli reg-write` は既に無検証で存在する**（§2）。「書き込み経路は無い」
のは GUI とトレイトの話であって、プロジェクト全体ではない。

### 非公式アドレス空間は既定オフ

RobStride の param table と MyActuator の `0xC0`。ここにしかない値がある
（reduction ratio、`KT_OUT`）が、前者はモータを黙らせる実績がある。

### Characterize は `SafetyLimits::gentle()`

ワークスペース最小のモータ（DM-J3507、定格 0.8 N·m）基準。**GUI は何が繋がって
いるか知らない。** ただし §2 のとおり、この枠は RS-04 では実効性が乏しい。

### 表示の誠実さ

繰り返し出てきた判断軸。**測定でないものを測定のように見せない。**

- 応答が excitation の 1/1000 以下なら「−240 dB のボード線図」ではなく
  「軸が動かなかった」と言う
- レート不足なら信用できる帯域を明示する
- 分解能が 1°/LSB なら小数 3 桁で出さない
- 無応答レジスタは空欄ではなく理由を出す
- フォールト語のビットが 1 つもマップできなければ「異常なし」と言わない
- 欠落したフィードバックは握り潰さず件数を報告する

**この文書自身が初版でこれに違反していた** — 推測を実測として書き、n を書かずに
点推定を出し、未検証を検証済みとしていた。要約は原典より不正確になりやすい。
**迷ったら原典へ。**

---

## 9. 次にやること

| | 状態 |
|---|---|
| **OneDrive の外へ移動 → 再ビルド** | 接続失敗バナーがまだインストーラに入っていない |
| **DAMIAO FD 欠落の再測定** | 下記。**30 分で片が付き、価値が高い** |
| `webviewInstallMode` の判断 | 配布先の Windows バージョン次第 |
| LK Motor の実機接続 | RS485 アダプタが `/dev/ttyUSB*` に出るところから |
| SLCAN 実機検証 | **DM-J4310 では検証できない**（FD 非対応）。RobStride か MyActuator で |
| GUI Characterize タブの実機確認 | 測定エンジン自体は検証済み。**RS-04 でやるなら `--kt 1.5093` 相当と、1 N·m を超える枠が要る** |
| RS-04 Sysid 撮り直し | 下記 |
| Params 書き込み | `reg-write` の警告は入れた（2026-08-02、§2）。次は値の範囲検証 |
| gs_usb バックエンド | 未着手 |

### DAMIAO FD 欠落の再測定（最優先）

再送と `ReadOutcome` 修正が同一コミットに入っており、どちらが効いたか分からない。

1. `REGISTER_READ_ATTEMPTS` を 1 に戻す（`ReadOutcome` 修正は残す）
2. `damiao-cli --fd -m 16 info` を 10 回、無応答数を数える
3. ゼロなら**原因は `try_read` のバグ**で、モータのファームウェアは無罪。
   再送は不要になる
4. 残るなら `PCAN_ERROR_QOVERRUN`（0x40）の監視を足してから再測定

### RS-04 Sysid

前回は既定値（0.15 rad / 30 Hz / max speed 5）で走らせ、**速度上限で飽和**した。
`2πfA = 5` より **5.3 Hz 以上は速度制限が支配**する。

推奨: **amplitude 0.05 rad / f end 20 Hz / max speed 8 rad/s**
（`8/(2π×0.05) = 25.5 Hz` なので全域が有効域に入る）。

**注意点**:
- 振幅を 0.15 → 0.05 rad にすると **SNR が 9.5 dB 落ちる**。コヒーレンスが
  もともと弱い高域で効くので、**撮ったらまずコヒーレンスを見ること**
- 前回の実効レートは **144 Hz**（要求 500 Hz の 29%）。RobStride の位置モードは
  1 サンプルあたりパラメータ書き込み 2 回＋読み戻しなので落ちる。
  アプリ自身の基準（`achievedRateHz/5`）では信用できるのは 28.8 Hz まで
- 20 Hz は 144 Hz に対し 7.2 サンプル/周期で、ちょうど境界

**前回の解析について**: 「むだ時間 τ ≈ 60 ms」と書いたが、**支持できない**。
(a) 導出に使った 10 Hz は自分で「有効域は 5.3 Hz まで」と書いた外側、
(b) プラント自身の位相遅れを差し引かず全位相を遅延に帰属させている
（1 次系を仮定すると τ は 41 ms、2 次系なら 14 ms と 4.5 倍ぶれる）、
(c) 位相アンラップは隣接ビン差分なので、コヒーレンスの低い区間で 1 回滑ると
τ が 100 ms ずれる、(d) コヒーレンス値を記録していない。

**−3 dB ≈ 1.5 Hz** のほうが有効域内なのでまだ信頼できるが、これも単発・
コヒーレンス不明・生データ未保存。**§4 の他の数値と違い、この節の数値には
裏付けファイルが無い。** 撮り直したら CSV を `doc/data/` に残すこと。

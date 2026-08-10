# 引継ぎ: Windows 対応と GUI

2026-08-02 時点。**コンテキストを失った状態から再開するための文書**。
コードを読めば分かることは書かない。実機でしか分からなかったこと、判断の根拠、
再び踏むであろう罠を残す。

関連文書:

| 文書 | 内容 |
|---|---|
| `doc/windows.md` | Windows でのセットアップ・アダプタ選定・配布手順 |
| `crates/robstride-protocol/doc/motorstudio-detection-devices.md` | 公式ツールの機種判別の解析 |
| `crates/robstride-protocol/doc/param-table-firmware-divergence.md` | 実機ファームとマニュアルの乖離 |
| `crates/damiao-protocol/doc/can-fd-bring-up.md` | CAN-FD 立ち上げの切り分け記録 |
| `crates/myactuator-protocol/doc/setup-software-c0-param-protocol.md` | 0xC0 空間と 0xB5 の実測 |

---

## 1. いまどうなっているか

**Linux/SocketCAN 専用だったワークスペースを Windows に移植し、その上に GUI を載せた。**
RobStride RS-04 / EduLite05、DAMIAO DM-J4310（classic と CAN-FD 両方）、
MyActuator RMD-X4-P36-36 の実機で検証済み。

### 検証済み

- 3 つの CAN バックエンド構成のうち **SocketCAN と PCAN**（classic / FD とも）
- 全 4 ドライバ（RobStride / DAMIAO / MyActuator / LK Motor）
- 3 社すべてのモデル自動判別
- GUI 5 タブすべて（Console / Sysid / Characterize / Monitor / Params）
- リリースビルドと NSIS インストーラ（12 MB exe / 2.6 MB installer）

### 未検証

- **SLCAN**（3 バックエンド中これだけ実機未確認）
- gs_usb / candleLight バックエンド（未実装）
- **12〜64 バイトの CAN-FD DLC 経路**（DAMIAO は常に 8 バイトなので通らない）
- 実機での Characterize（シミュレータのみ）
- 飽和させない設定での RS-04 Sysid 撮り直し

---

## 2. 構造と、そうした理由

### `Actuator` トレイトが唯一の抽象

`crates/misa-actuator/src/traits.rs`。ブロッキング、`&mut self`、13 メソッド。
GUI も TUI も sysid もこれしか見ない。ベンダ差はドライバの内側に閉じる。

拡張は**既定実装つきメソッド**で足す（`scan_bus`、`read_parameters`）。
そうしないと 4 ドライバ全部を同時に触ることになる。

### `misa-can`: バックエンド 3 種を 1 つの trait に

`[backend:]device[@bitrate[,data]][?opts]` という文法で
`can0` / `pcan:usb1@1M,5M` / `slcan:COM5` を書き分ける。

**PCAN は `PCANBasic.dll` を実行時 `LoadLibrary` で読む。** リンク時依存にすると
PEAK ドライバの無いマシンで**起動すらできなくなる**。遅延ロードなので、
アダプタが無くてもアプリは立ち上がり、接続時に初めて明示的なエラーが出る。

### `misa-actuator-core`: セッションとワーカー

TUI から切り出した。GUI と TUI が同じ実装を共有する。

- **Setpoint は上書き、Command はキュー。** 位置スライダを動かすたびに指令が
  積み上がってはいけない。逆に Enable/Disable は取りこぼしてはいけない。
- **ウォッチドッグは専用スレッド。** ジョブ（chirp など）はワーカーのループを
  数十秒占有する。ループ内チェックでは**まさに一番危険な時間帯に動かない**。
- **停止手段は 3 系統独立**: 明示 STOP フラグ / ウォッチドッグ / `Drop` でワーカー join。
- **テレメトリだけ別チャネルで落としてよい。** UI が遅れたらプロットのサンプルは
  捨てる。状態変化とエラーは絶対に捨てない。

### `misa-actuator-sim`: 実測較正済みのプラントモデル

`doc/bench-measurements-2026-07-30.md` の実測値からプリセットを作ってある
（`ideal` / `dm4310` / `rs04`）。ハードウェア無しで GUI と sysid を動かせる。

`rs04` プリセットの `torque_limit = 115 N·m` は実機 RS-04 の `limit_torque` 実測値、
`stiction = 0.902 N·m` も実測値。**Characterize の breakaway が 0.904 を返したのは
この較正が効いている証拠**。

### GUI: Tauri + React

`crates/misa-actuator-gui`（Rust 側）+ `ui/`（React / TypeScript / uPlot）。

- **debug ビルドは Vite dev サーバを見る**（`devUrl`）。`cargo run` 単体では
  白画面。開発は `npm run app`、配布物は `npm run app:build`。
- release ビルドはフロントを**バイナリに埋め込む**（`tauri.localhost` から配信）。

---

## 3. 実機でしか分からなかったこと

**ここが一番の資産。** モータが無いと再取得できない。

### RobStride: 機種はファームウェアのバージョンで分かる

**マイナー成分が機種番号。** 公式リリース資産のファイル名がそのまま対応表:

| 資産 | バージョン |
|---|---|
| `rs00-0.0.3.32.bin` | 0.**0**.3.32 |
| `rs04-0.4.1.32.bin` | 0.**4**.1.32 |
| `rs06_0.6.0.12.bin` | 0.**6**.0.12 |

メジャーが製品ライン（`0` = RS、`10` = EduLite）。実機 RS-04 は `0.4.1.32`、
EduLite05 は `10.5.0.1`。公式ツール motorstudio の `RS04` / `EL05` 表示は
これで説明できる。

**行き止まりだった経路**（同じ調査を繰り返さないこと）:

| 経路 | 結果 |
|---|---|
| `0x0000 Name` | RS-04 / EL05 とも全 FF（工場未書き込み） |
| `0x0001 BarCode` | 同上 |
| `0x1007 AppCodeName` | EL05 は `EL_motor`、**RS-04 は `motor`** で判別不能 |
| comm type 0 | MCU UID のみ |
| バージョン応答 Byte7 | **両機種とも `0x07`** — モデルコードではない |

### RobStride: バージョン読み出しの罠

マニュアルの「コピペミスに見える記述」が**両方とも正しかった**:

```
要求  ID=0x0400FD01  data=00 C4 00 00 00 00 00 00
応答  ID=0x020001FD  data=00 C4 56 00 04 01 20 07   → 0.4.1.32
```

- **要求の comm type 欄は `4`** — `DISABLE` と同じ値。`00 C4` プレフィックスでしか区別できない
- **応答は comm type `2`** — 通常フィードバックと同じ。`00 C4 56` でしか区別できない

**応答をフィードバックとしてデコードすると、それらしい数値のゴミになる。**
`parse_version_reply` はプレフィックス検証を強制する。

### RobStride: 実測値

| | RS-04 | EduLite05 |
|---|---|---|
| firmware | 0.4.1.32 | 10.5.0.1 |
| MCU UID | `0F302000C03B5CC3` | `06B6311821300A62` |
| `limit_torque` (0x700B) | **115 N·m** | 6 N·m |
| `limit_spd` (0x7017) | 1 rad/s | 50 rad/s |
| `AppCodeName` | `motor` | `EL_motor` |

UID は受信バイト列の**逆順**が motorstudio の表示と一致する。

`limit_torque = 115` は family 中 RS-04 の MIT レンジ（±120）でしか表現できないので、
**これ単独でも一意に特定できる**（次点の RS-03/06 は 60）。

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

`P` サフィックスだけ曖昧に残るが、マニュアル上トルク・回転数・減速比・
レジスタレイアウトがすべて同一なので実害なし。

**そもそも DAMIAO はスケーリングに機種判別が要らない。** `PMAX`/`VMAX`/`TMAX` が
量子化レンジそのもの。RobStride で苦労した 21.8 倍問題が構造的に存在しない。

### DAMIAO: 実機構成と CAN-FD

| | |
|---|---|
| CAN_ID | **16 (0x10)** — 既定スキャン範囲 1〜8 の外 |
| PMAX / VMAX / TMAX | 12.5 / 30 / 10 |
| `can_br` | RID 35。**FD 設定はここ**（以前「無い」と書いたのは誤り） |

**FD ノードにすると classic では一切通信できなくなる**（実測 36/36 タイムアウト）。
モータが FD フレームで応答し、classic コントローラはそれをエラーフレームとして扱う。
切り替え後に classic で試して「壊れた」と誤解しないこと。

**FD ではレジスタ読みが約 1/7 の確率で無応答**:

- ランダムなレジスタで発生
- タイムアウトを 100ms → 1000ms にしても失敗数不変（遅いのではなく来ていない）
- **アダプタはバスエラーを一切報告しない**
- 送信は毎回全フレーム成功
- classic では発生しない

→ `read_register` に 3 回再送 + 5ms ギャップ。修正後 60/60 成功。
根治していない。5 Mbps の信号品質（ケーブル長・終端・スタブ）の可能性が高い。

### MyActuator: 実測値

| | |
|---|---|
| 型番 | RMD-X4-P36-36 |
| `0xB5 motor_model` | **チャンク方式でのみ取得可** |
| `kt_out` (0xC0[0x3E]) | **1.1 N·m/A** |
| reduction_ratio | 36 |

**`0xB5` はチャンク方式。flag は `0x01`**（`0x00` が自然な推測だが、
間違えるとエラーにならず**黙って空文字列**になる）:

```
TX  B5 01 01 …  RX  B5 01 01 52 4D 44 2D 58   "RMD-X"
TX  B5 01 02 …  RX  B5 01 02 34 2D 50 33 36   "4-P36"
TX  B5 01 03 …  RX  B5 01 03 2D 33 36 00 00   "-36"
```

**`0x9C` の角度は 1°/LSB と粗い**（`0x92` は 0.01°/LSB）。制御応答の位置を
小数 3 桁で出すと過剰精度になり、0.1 rad の指令が 16% オーバーシュートしたように見える。

### MyActuator: 未解明

- `cmd 0x19`: `TX 19 00…` → `RX 19 01 01 00 4E 1A 00 00`
- **`cmd 0x80`**: 読み出しシーケンス中に送られる。こちらの `Cmd::Shutdown`（出力オフ）と
  同コード。**用途が確定するまで生存確認として送ってはいけない**
- `0xC0` index `0x3A`

---

## 4. 直したバグ（再発しやすいもの）

### Windows 固有

**clap のスタックオーバーフロー。** Windows のメインスレッドスタックは 1 MB
（Linux は 8 MB）。clap derive は全サブコマンドの構築を 1 つの関数に展開するので、
`robstride-cli --help` が `main` の 1 行目に入る前に落ちた。
→ `.cargo/config.toml` で `/STACK:8388608`。

**タイマ分解能。** Windows の sleep は既定で ~15.6 ms に丸められ、あらゆる
制御ループが絞られる。→ `misa-actuator::realtime` で `timeBeginPeriod(1)` と
絶対デッドライン。

### 配線契約

**`rename_all_fields` の付け忘れ。** serde の `rename_all` は**バリアント名しか**
変えない。フィールドは `rename_all_fields` が要る。付け忘れると `timeout_ms` が
snake_case のまま出て、フロントで `undefined` になる — パースエラーではなく
黙って壊れる。**過去に黒画面を 2 回起こしている。**

`protocol.rs` のテストが配線契約を押さえている。**バグった期待値を固定しないこと**
（一度やった: `"timeout_ms":50` をテストに書いてバグを仕様化していた）。

### 受信経路

**`try_read` がスキップを「キューが空」と報告していた。** エラー/ステータス/RTR
フレームを弾いたとき `Ok(None)` を返すと、呼び出し側は自動リセットイベントを
待ちに入る。**目的のフレームが既にキューにあっても再発火しない。**
→ `ReadOutcome::{Frame, Empty, Skipped}` に分離。

**制御ループが 1 回の欠落で中断していた。** DAMIAO FD で 1/7 落ちるので、
1 秒の保持すら完走しない。しかも**指令は効いていてモータは動いていた**のに
「応答なし」に見えた。→ 連続 5 回まで許容、欠落数を必ず報告。

### スケーリング

**`--model` の既定が `Edulite05` だった。** 「未指定」と「EduLite05 を選んだ」が
同じ文字列で区別できず、RS-04 に対して黙って RS-05 のスケール（21.8 倍差）で
動いていた。→ `MODEL_UNSPECIFIED = ""`。RobStride は未指定を拒否する。

**`from_gear_ratio` が P 版を無条件スキップしていた。** `DM6248P` と `DM10422P` は
P 版しか存在しないので、この 2 機種が識別不能になっていた。実機は DM4310 なので
気づけず、全機種網羅テストが捕まえた。

### GUI レイアウト

**`.app` が行数固定のグリッドだった。** バナー 2 つが条件付きなので子要素が 3〜5 個に
変動し、バナーが無いと `main` が `1fr` ではなく `auto` の行に落ちる。
**全タブのプロットが content 高さに潰れていた。** → flex に変更。

**uPlot がホストの `clientHeight` をそのまま使う。** 高さ制約が無いと canvas が
8504px になりページから溢れる。凡例はホスト内・canvas の下に描かれるので、
canvas に全高を渡すと凡例が次の要素と重なる。→ 高さから凡例分を引く。

**回転した 1 文字の軸ラベルは読めない。** `V` が `<` に見えた。→ 軸を系列色で塗り分け。

### シミュレータ

**制御則が呼び出しごとに 1 回しか評価されていなかった。** 速度制御が 27 rad/s に
発散した。→ プラントのサブステップ（2 kHz）で毎回評価。

---

## 5. 環境の罠

### PowerShell

| 罠 | 対処 |
|---|---|
| `npm` が実行ポリシーで弾かれる | **`npm.cmd`** を使う |
| ネイティブコマンドへの `2>&1` | PS 5.1 は stderr を ErrorRecord で包み、終了 0 でも `$?` が false。`Start-Process` + リダイレクトファイル |
| `\| Select-Object -First N` | パイプを早期に閉じて**プロセスを殺す**。PCAN チャネルが初期化されたまま残り、次回 `driver state is wrong` |
| `Start-Process -Environment` | PS 5.1 に無い。親の環境変数を継承させる |
| バッチファイルの非 ASCII | OEM コードページで解釈され `'M' は認識されていません` になる |

### OneDrive

**`OneDrive - Sony` 配下でビルドしてはいけない。**
リポジトリ全体（`target/` の **21 GB** を含む）が同期対象になっていた。

症状: 一度ビルドに成功した後、以降のフロントエンドビルドがすべて
`EPERM: …\ui\dist\index.html` で失敗する。**こちらのプロセスは全滅済み**
（node / vite / cargo / アプリいずれも無し）なのに削除もリネームもできない。

`tsc --noEmit` は書き込まないので通る（型検査だけは可能）。

判別法: `Get-Item -Force <dir>` の `Attributes` に `ReadOnly, ReparsePoint` が
出れば OneDrive のオンデマンド管理下。`crates` や `doc` にも出るなら全体が同期対象。

対処: リポジトリを OneDrive の外へ。ソースは 5.9 MB で git 管理下なので、
OneDrive は何も足していない。

### Tauri

- **debug ビルドは Vite dev サーバが要る**（`devUrl`）。無いと白画面
- **バンドラが GitHub から NSIS をダウンロードする**（初回のみ）。オフライン環境では
  exe はできるがバンドル段階で失敗する
- **`webviewInstallMode` 未設定** = 導入時にネット接続が必要。オフライン配布なら
  `embedBootstrapper`（+150 MB）に変える判断が要る

### 実機

**RobStride の param table 読み出しがモータを黙らせる。** 2 回再現、電源再投入が必要。
`AppCodeName` が `"moto"` と 1 文字切れて返るのと同時に起きた（サブフレーム取りこぼし）。
→ 接続経路から外し、`identify --deep` で明示指定時のみ。

---

## 6. 診断ツール

実機の謎はこれで解いた。

| コマンド | 用途 |
|---|---|
| `robstride-cli dump` / `myactuator-cli dump` | 受信専用。デコードして逐次表示。**公式ツールを操作しながら観測できる**。`0xB5` の flag も RobStride のバージョン読みもこれで判明 |
| `robstride-cli identify` | 型番判別。既定は文書化済み経路のみ、`--deep` で非公式空間 |
| `damiao-cli --fd-link-classic-frames` | **FD リンクで classic フレームを送る**。「モータが FD 非対応」と「こちらの FD が壊れている」を切り分ける |

**推測で切り分けない。** `0xB5` の flag は構造の推測が当たっていたのに 1 バイト外れて
いた。キャプチャが無ければ「実装したが動かない」で止まっていた。

---

## 7. 判断の記録（蒸し返さないため）

### Params は読み取り専用

`Actuator::read_parameters(deep)`。**書き込み API は意図的に無い。**

根拠: マニュアルが `motor_baud` と呼ぶアドレスに書いた値が、実機ファームでは
`CAN_ID` のレジスタに着弾し、**モータが 1 日バスから消えた**。
実測でファームとマニュアルの対応が 84 行中 32 行食い違う。

書き込みを足すなら、**実機で確認したアドレスのホワイトリスト**、確認ダイアログ、
flash コミットの扱いが要る。別作業。

### 非公式アドレス空間は既定オフ

RobStride の param table と MyActuator の `0xC0`。ここにしかない値がある
（reduction ratio、`KT_OUT`）が、前者はモータを黙らせる実績がある。
→ チェックボックスで明示的に有効化。該当行は表内に印を付ける。

### Characterize は `SafetyLimits::gentle()`

1 N·m / 60 °C / ±0.5 rad / 30 s。ワークスペース最小のモータ（DM-J3507、定格 0.8 N·m）
基準。**GUI は何が繋がっているか知らない。** 上げるなら意図的にやるべきで、
既定から継承させてはいけない。

### 表示の誠実さ

繰り返し出てきた判断軸。**測定でないものを測定のように見せない。**

- 応答が excitation の 1/1000 以下なら「−240 dB のボード線図」ではなく「軸が動かなかった」と言う
- レート不足なら信用できる帯域を明示する
- 分解能が 1°/LSB なら小数 3 桁で出さない
- 無応答レジスタは空欄（0 に見える）ではなく理由を出す
- フォールト語のビットが 1 つもマップできなければ「異常なし」と言わない
- 欠落したフィードバックは握り潰さず件数を報告する

---

## 8. 次にやること

| | 状態 |
|---|---|
| **OneDrive の外へ移動 → 再ビルド** | 接続失敗バナーがまだインストーラに入っていない |
| `webviewInstallMode` の判断 | 配布先の Windows バージョン次第 |
| SLCAN 実機検証 | 3 バックエンド中これだけ未確認。PCAN を持たない人の経路 |
| 実機 Characterize | シミュレータで 0.904 vs 設定 0.902。実機と突き合わせると較正の答え合わせになる |
| RS-04 Sysid 撮り直し | 既定値では飽和する。**amplitude 0.05 rad / f end 20 Hz / max speed 8 rad/s** |
| Params 書き込み | ホワイトリスト設計から |
| DAMIAO FD の 1/7 欠落 | 再送で隠れている。配線を疑う |
| gs_usb バックエンド | 未着手（CANable2 用） |

### RS-04 Sysid について

前回の測定は既定値（0.15 rad / 30 Hz / max speed 5）で走らせたため
**速度上限で飽和**していた。`2πfA = 5` より **5.3 Hz 以上は速度制限が支配**する。

有効域（〜5.3 Hz）だけ見れば **-3 dB がおよそ 1.5 Hz**。位相の落ち方がゲインに対して
急で、10 Hz で約 -230° から逆算すると**むだ時間 τ ≈ 60 ms**（144 Hz サンプリングの
9 周期分）。制御器設計ではこの位相遅れがゲイン余裕を決めるので、
**サンプルレートを上げるほうがゲインを上げるより効く**可能性が高い。

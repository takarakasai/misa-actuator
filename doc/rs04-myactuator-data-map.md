# RobStride 04 / MyActuator RMD (CAN V3) — 設定・取得データとリミット値の比較

出典:
- RobStride: 公式マニュアル由来の `robstride-protocol`（`param.rs` / `comm_type.rs`）と
  ground-truth 実装 `robstride_sandbox`（実機 RS 系で検証済み）。
- MyActuator: `crates/myactuator-protocol/ref/RMD-X-Servo-Motor-Control-Protocol-V3.91.pdf`。
  単体の詳細は `crates/myactuator-protocol/doc/data-map.md` を参照。

RobStride側の通信方式（私有プロトコル vs 未公開の拡張パラメータ空間、実機インシデント記録含む）の
詳細は [`crates/robstride-protocol/doc/communication-protocols.md`](../crates/robstride-protocol/doc/communication-protocols.md) を参照。

## 0. プロトコルの土台の違い

| | RobStride 04 | MyActuator RMD V3 |
|---|---|---|
| CANフレーム | 拡張 29-bit ID（bit28..24 = 通信タイプ） | 標準 11-bit ID（`0x140+ID` / 応答 `0x240+ID`） |
| パラメータアクセス | type17 読み / type18 書き（index 指定, 汎用機構） | 機能ごとに専用コマンド |
| 一括フラッシュ保存 | **type22 SaveParameters あり**（RAM 値を一括不揮発化） | なし（コマンドごとに保存先が固定） |
| 運転モード | `run_mode` (0x7005) で明示切替（MIT/位置/速度/トルク） | モード概念なし（コマンドがコントローラを選ぶ） |

RobStride の `0x7xxx` パラメータは**すべて揮発（RAM）**で、type22 を打った時だけフラッシュに永続化される。
MyActuator は**コマンド自体が RAM 行き / ROM 行きを決める**（例: PID は 0x31=RAM, 0x32=ROM）。

## 1. リミット値の比較（本題）

「−」= そのプロトコルに存在しない。存在しないものも明示するため全行を掲載。

| リミット項目 | RobStride 04 | 揮発性 (RS) | MyActuator RMD V3 | 揮発性 (Mya) |
|---|---|---|---|---|
| 電流リミット | `limit_cur` (0x7018, f32 A) | 揮発（type22 で保存可） | **−**（CAN からは設定不可。上位 PC ツールの「Max Torque Current」でのみ設定され、0xA2 速度制御時の電流上限として機能） | 不揮発（PC ツール書込） |
| トルクリミット | `limit_torque` (0x700B, f32 N·m) | 揮発（type22 で保存可） | **−**（電流リミットと同一機構のみ） | − |
| 速度リミット（位置制御時） | `limit_spd` (0x7017, f32 rad/s) — 位置モードの移動速度上限 | 揮発（type22 で保存可） | 0xA4 の `maxSpeed` フィールド (uint16, 1 dps) — **指令フレーム毎に毎回指定**（0 = 無制限、PI 出力のみ） | 揮発（指令毎） |
| 速度リミット（グローバル） | `vel_max` (0x7024, f32) — PP（位置プロファイル）モード最大速度 | 揮発（type22 で保存可） | **−** | − |
| 加速度リミット（速度制御） | `acc_rad` (0x7022, f32 rad/s²) | 揮発（type22 で保存可） | 0x43 index 0x02/0x03（速度計画 加速/減速, 1 dps/s, 100–60000） | **不揮発**（RAM+ROM 同時書き） |
| 加速度リミット（位置制御） | `acc_set` (0x7025, f32) — PP モード加速度 | 揮発（type22 で保存可） | 0x43 index 0x00/0x01（位置計画 加速/減速。0 = 加減速なしの直接追従モード） | **不揮発**（RAM+ROM 同時書き） |
| 通信断保護タイムアウト | `canTimeout` (0x7028, u32) | 揮発（type22 で保存可） | 0xB3 `CanRecvTime_MS` (u32 ms, 0=無効。超過で出力遮断+ブレーキロック) | 明記なし（要実機確認） |
| MIT モード指令レンジ（ハード上限） | RS-04: pos ±4π rad / vel ±15 rad/s / tau ±120 N·m / kp 0–5000 / kd 0–100（モデル別量子化テーブル） | 固定（プロトコル定数） | pos ±12.5 rad / vel ±45 rad/s / t_ff ±24 N·m / kp 0–500 / kd 0–5（全モデル共通） | 固定（プロトコル定数） |

※ RS-04 の MIT レンジは `robstride-protocol::MitScales`（sandbox 実機検証値と一致）。
※ MyActuator の MIT レンジは指令の量子化レンジ = 超過分はクランプ。RS-04 の ±120 N·m 等も同様。

## 2. 取得（読み取り）できるデータ

| データ | RobStride 04 | MyActuator RMD V3 |
|---|---|---|
| 位置（多回転） | フィードバックフレーム (type2) / `mechPos` (0x7019) | 0x92 (0.01°) / 0x60 エンコーダカウント |
| 速度 | type2 / `mechVel` (0x701B) | 0x9C (1 dps, int16) |
| トルク / 電流 | type2 トルク / `iqf` (0x701A, フィルタ済 iq) | 0x9C iq (0.01 A)、0x9D 相電流 A/B/C (0.01 A) |
| 母線電圧 | `VBUS` (0x701C) | 0x9A (0.1 V) |
| 温度 | type2 フレーム内 | 0x9A / 0x9C / 0x9D (1 ℃) |
| エラー / フォルト | type21 フォルトフレーム + type2 のステータスビット（ストール/過熱/過流/低電圧/未校正 等） | 0x9A エラービット（ストール/低電圧/過電圧/過電流/過速度/過熱/校正エラー 等） |
| 制御ゲイン読み | 各パラメータ index (0x7010–0x7021) | 0x30（PID 一括, 0–255 正規化） |
| 運転モード | `run_mode` (0x7005) | 0x70（1=電流 2=速度 3=位置） |
| デバイス情報 | type0 (デバイス ID + MCU UID) | 0xB5 型番 / 0xB2 FW 日付 / 0xB1 稼働時間 / 0x71 電力 |
| 単回転角 / エンコーダ生値 | −（多回転のみ） | 0x94 / 0x90 / 0x61 / 0x62 |

## 3. 設定できるデータと保存先

| 設定項目 | RobStride 04 | 保存先 (RS) | MyActuator RMD V3 | 保存先 (Mya) |
|---|---|---|---|---|
| 制御ゲイン | `cur_kp/ki` `spd_kp/ki` `loc_kp` `*_filter_gain` (0x7010–0x7021) | 揮発 → type22 で不揮発化 | 0x31 (PID→RAM) / 0x32 (PID→ROM) | 揮発 / 不揮発を**コマンドで選択** |
| 各種リミット | §1 参照 | 〃 | §1 参照 | §1 参照 |
| 機械ゼロ | type6 SetZeroPosition（現在位置をゼロに） | **揮発**（電源断で消失、CyberGear 系列の仕様。恒久化は `mechOffset` 0x2005 + type22） | 0x64（現在位置→ROM）/ 0x63（指定値→ROM）/ 0x20 idx1 多回転クリア | **不揮発・要再起動**（フラッシュ摩耗注意） |
| CAN ID | type7 SetDeviceId | 不揮発 | 0x79 | 明記なし（実質不揮発想定） |
| CAN ボーレート | type23 SetBaudrate | 不揮発 | 0xB4（CAN: 0=500K, 1=1M） | **不揮発**・次回電源投入から有効 |
| 能動送信（周期レポート） | type24 ActiveReport | — | 0xB6（0x60/61/62/92/9A/9C/9D/9E を周期送信） | 明記なし（揮発想定） |
| プロトコル切替 | type25 SetProtocol | 不揮発 | − | − |
| ブレーキ | − | − | 0x77 解放 / 0x78 ロック | 揮発（状態のみ） |

## 4. misa-actuator ドライバでの扱い

- 両ドライバとも `Actuator::set_zero()` は**ホスト側ソフトゼロ**（モータへ書き込まない）。
  - RobStride: type6 は揮発なので直接使ってもよいが、driver はモード非破壊の `MechPos` 読みでアンカー。
  - MyActuator: 0x64 が ROM 書き+要再起動のため、`rezero()`（0x92 読み）を常用し `set_zero_rom()` は明示呼び出しのみ。
- RobStride のリミット系は `set_position_speed_limit` (`LimitSpd`)、`set_torque_limit` (`LimitTorque`)、
  `set_current_limit` (`LimitCur`) として実装済み（揮発、type22 の保存は未実装）。
- MyActuator の 0x43 加速度・0x30–0x32 PID・0xB3/0xB4 は未実装（`transact()` 上に追加可能）。

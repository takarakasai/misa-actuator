# LKMotor CAN/RS485 公式マニュアル(中国語原文)突き合わせ

ユーザー提供の4本のPDF(`ref/can_protocol.pdf`, `ref/rs485_protocol.pdf`,
`ref/can_broadcast_protocol.pdf`, `ref/rs485_broadcast_protocol.pdf`、いずれも中国語原文
V2.35/V2.36)を英訳・markdown化し(`ref/can_protocol_en.md`, `ref/rs485_protocol_en.md`,
`ref/can_broadcast_protocol_en.md`, `ref/rs485_broadcast_protocol_en.md`)、既存実装
(`lkmotor-protocol::command::Command`/`ControlParamId`)と突き合わせた。

`can_protocol.pdf`/`rs485_protocol.pdf`は、別プロジェクト`/home/takara/work/dp/lkmotor-driver/ref/`
に既にあった英訳(`can_protocol_desc_en.md`等)と同一プロトコルバージョンだったため、独立して
再翻訳した上でクロスチェックした(数箇所、原文自体の矛盾を新規に発見——後述)。

## 1. 型番/FWバージョン読み取り: 独立して「存在しない」ことを再確認

CAN(29コマンド)・RS485(24コマンド)とも全コマンド+両パラメータテーブルを確認したが、
型番・FWバージョン・ハードウェアバージョン・シリアル番号を読むコマンドは**存在しない**。
以前`misa-actuator-identify`調査時の結論(公式マニュアル未入手の状態での結論)を、今回
中国語原文から直接、独立に再確認できた。

## 2. 最大の収穫: 「Setting Parameter Table」— 未実装の第2パラメータ空間

現在実装済みの`Command::ReadControlParam`(`0xC0`)/`WriteControlParamRam`(`0xC1`)は、
マニュアル上「Motor Control Parameter Table」(コマンド18/19)にあたる。しかしマニュアルには
**もう一つ別の設定値空間「Setting Parameter Table」(コマンド26/27/28)が存在し、現在
一切実装されていない**:

| コマンド | 内容 | コマンドバイト |
|---|---|---|
| 26 | Read setting parameters | `0x40` |
| 27 | Write setting parameters | `0x42` |
| 28 | Save setting parameters(ROM書き込み、再起動が必要) | `0x44` |

このテーブルに含まれる項目(`DATA[2]`がサブID):

| サブID | 項目 | 型/範囲 |
|---|---|---|
| `0x0A` | Driver ID | uint8, 0-32 |
| `0x0B` | Bus Type(0=None/1=RS485/2=CAN) | uint8 |
| `0x0C` | RS485 Baudrate(0=9600 … 10=4Mbps) | uint8 |
| `0x0D` | CAN Baudrate(0=100kbps … 4=1Mbps) | uint8 |
| `0xE0` | Max Power | int16 |
| `0xE2` | Max Speed | int32 |
| `0xE4` | Max Angle | int32 |
| `0xEA` | Current Ramp | int16(★下記の矛盾注記あり) |
| `0xEC` | Speed Ramp | int32 |
| (複数パラメータ版) | Position/Speed/Current Loop PID 他 | §27 |

**★重要**: このテーブルのサブID`0x0A`/`0x0B`/`0x0C`は、既存実装済みの`ControlParamId`の
`PositionLoopPid=0x0A`/`SpeedLoopPid=0x0B`/`CurrentLoopPid=0x0C`と**数値が偶然重複している
が全くの別物**(コマンドバイトが`0xC0`/`0xC1`と`0x40`/`0x42`で異なるため区別される)。実装する
際は`ControlParamId`とは完全に独立した新しい`SettingParamId` enumを作る必要がある
(サブIDを使い回すと重大なバグになる)。

**設計上の価値**: Driver ID・Bus Type・RS485/CANボーレートは、他ベンダーで既に実装した
「設定値一括取得」(RobStrideの`ParamIndex`、MyActuatorの`0xC0`、DAMIAOの`Rid`拡張)に相当する、
LKMotorにとって最も近い対応物。特にDriver ID/Baudrateの読み取りは、配線トラブル時の診断に
直接役立つ。

## 3. ブロードキャストプロトコル: 全く新しい未実装機能

`can_broadcast_protocol_en.md`/`rs485_broadcast_protocol_en.md`より:

- **CAN**: 専用フレームID(`0x280`トルク/開ループ、`0x281`速度、`0x282`位置、`0x288`複合)で
  **最大4台のモータを1フレームで同時制御**。各モータに固定2バイトスロット(モータ1=byte0-1、
  モータ2=byte2-3…)。応答は各モータがID順に個別送信。要:ホスト側でブロードキャストモード有効化、
  ボーレート500kbps以上。
- **RS485**: `Head(0x02) + CMD + 8データバイト + チェックサム`(11バイト固定長、フレーム境界は
  バスアイドル時間で検出)。CMD値(`0x80/0x81/0x82/0x88`)はCANのフレームIDの下位バイトと対応。

**実装の見通し**: 値のエンコーディング自体は既存の単一モータコマンドと同じなので、構造的には
「4モータ分を1フレームに詰める新しいフレームビルダー/パーサ」と「ID順に最大4件の応答を待つ
受信ロジック」を追加するだけで済みそう。ただし`lkmotor-protocol`/`lkmotor-driver`には現状
このロジックが無い(MyActuatorの`0x280`ブロードキャスト未実装と同種のギャップ)。

## 4. その他の未実装コマンド(CAN/RS485共通、優先度低〜中)

| # | コマンド | コマンドバイト | 備考 |
|---|---|---|---|
| 8 | Brake control and state read | — | ブレーキ付きモデルのみ関係。DAMIAOには`BrakeRelease`/`BrakeLock`相当あり、LKMotorは未実装 |
| 9 | Open-loop control(MS専用) | — | モデル限定、優先度低 |
| 16/17 | Incremental position closed-loop control 1/2 | — | 現在地からの相対移動。`PositionClosedLoop1-4`(絶対/単回転)はあるが相対移動コマンドは未実装 |
| 21 | Calibrate encoder | — | **実装非推奨**——本セッションで既に「Calibratボタンは物理的なキャリブレーション動作を伴うため触らない」方針にしている(MyActuatorの2nd Encoder Calibrateと同じ理由) |
| 29 | Motor restart | `0x07` | 応答なしの単純コマンド。DAMIAOの`SystemReset`相当、実装は容易 |

また、`command.rs`に既に定義されているが未使用の`PositionClosedLoop3`(0xA5)/`PositionClosedLoop4`
(0xA6)(単回転位置制御)も、ドライバ層に配線されていない(以前のセッションで確認済みの既知ギャップ)。

## 5. マニュアル原文自体の矛盾点(新規発見、実装時に要注意)

- **Current Rampの型不一致**: Motor Control Parameter Table(§19、サブID`0x24`)では`int32`
  (4バイト)だが、Setting Parameter Table(§27、サブID`0xEA`)では同じ名前・同じ範囲
  (0-30000)なのに`int16`(2バイト)。原文自体の矛盾で、既存の英訳(`can_protocol_desc_en.md`)
  には無い、今回新たに見つけた食い違い。
- RS485マニュアルには、バイト数表記の誤り(コマンド18/20の見出しが実際のバイト数と不一致)が
  複数箇所あり。既存の姉妹プロジェクトの英訳は一部を無言で「修正」していたが、今回は全て原文
  ママ+`**Note**`で明記した(`ref/rs485_protocol_en.md`参照)。

## 参照

- 全文英訳: `ref/can_protocol_en.md`(1259行)、`ref/rs485_protocol_en.md`(1185行)、
  `ref/can_broadcast_protocol_en.md`、`ref/rs485_broadcast_protocol_en.md`
- 元PDF(中国語原文): `ref/can_protocol.pdf`、`ref/rs485_protocol.pdf`、
  `ref/can_broadcast_protocol.pdf`、`ref/rs485_broadcast_protocol.pdf`
- 姉妹プロジェクトの既存英訳(クロスチェック用、独立編集): `/home/takara/work/dp/lkmotor-driver/ref/`

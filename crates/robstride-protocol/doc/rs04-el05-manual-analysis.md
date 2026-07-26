# RS04 / EduLite05(EL05)公式マニュアル 突き合わせ結果

出典（2026-07-26 取得、英語版）:

- `ref/RS04_User_Manual_en.pdf` — RobStride 公式 GitHub (`RobStride/Product_Information`) 掲載の
  `RS04User Manual260713.pdf` と同内容。Seeed Studio 配布版（2025-11-12 改訂, "RS04-EN"）。
- `ref/EL05_User_Manual_en.pdf` — 同じく "EL05-EN"（RobStride EduLite 05, 2025-11-12 改訂）。

いずれも発行元は北京霊足時代科技（Beijing Lingfoot Times Technology Co., LTD.）。ダウンロード元
URL は `files.seeedstudio.com/products/RobStride/Product%20Literature/{RS04,EL05}/...`。

読みやすい形に整形した全文は同ディレクトリの `rs04_manual_en.md` / `el05_manual_en.md`
（変換中 — 完了次第このファイルから参照）を参照。本ドキュメントは
`crates/robstride-protocol` の実装との差分だけをまとめる。

## 結論サマリ

| 項目 | 結果 |
|---|---|
| RS-04 の MIT スケール (`MitScales::for_model`) | **一致**。修正不要。 |
| RS-05 / EduLite05 の MIT スケール | **不一致を発見・修正済み**（`velocity: 33.0→50.0`, `torque: 17.0→5.5`）。 |
| CommType (0–25) | 実装済み分は一致。type 26（バージョン読み）は manual にはあるが未実装。 |
| RunMode (0–3) | 一致。 |

## 1. RS-04: MIT スケールは公式マニュアルと完全一致

マニュアルのサンプルコード（`#define P_MAX/V_MAX/KP_MAX/KD_MAX/T_MAX`, gd32f303 例）より:

| | P_MAX | V_MAX | KP_MAX | KD_MAX | T_MAX |
|---|---|---|---|---|---|
| RS04 マニュアル | 12.57 (=4π) | 15.0 | 5000.0 | 100.0 | 120.0 |
| `model.rs::MitScales::for_model(Rs04)` | 4π | 15.0 | 5000.0 | 100.0 | 120.0 |

→ 差分なし。`robstride_sandbox` の実機検証値がそのまま正しかったことを裏付ける。

## 2. RS-05 / EduLite05: velocity と torque のスケールが食い違っていた（修正済み）

同じサンプルコードのマクロ定義を EL05 マニュアルと（確認のため直接ダウンロードした）RS05 本体
マニュアルの双方で確認したところ、**両方とも**次の値になっている:

| | P_MAX | V_MAX | KP_MAX | KD_MAX | T_MAX |
|---|---|---|---|---|---|
| EL05 マニュアル (2025-11-12) | 12.57 | **50.0** | 500.0 | 5.0 | **5.5** |
| RS05 マニュアル (2025-11-12, 突合せのため追加取得) | 12.57 | **50.0** | 500.0 | 5.0 | **5.5** |
| 修正前の `model.rs::MitScales::for_model(Rs05)` | 4π | 33.0 | 500.0 | 5.0 | 17.0 |

2 つの独立した公式 PDF が一致して velocity=50.0 rad/s・torque=±5.5 N·m を示している一方、
実装は velocity=33.0・torque=17.0 だった。`doc/spec.md`（旧 robstride-driver 側）のコメントに
「公開仕様書からの転記のため要検証」とあった通りの箇所で、今回のマニュアル突き合わせで
実際に不一致が見つかった形。

**影響:** MIT モードの kp/kd/pos/vel/torque はすべてこの `scale` を分母に 16bit へ量子化される
(`encode: u16 = (value/scale + 1)*0x7FFF` 相当)。特に torque は 17.0 → 5.5 と 3 倍以上の差があり、
同じ生の 16bit コードでも実際にモータへ通知されるトルク指令値が大きく変わる。EduLite05 実機で
MIT モードのトルク項を使っている場合は要再確認。

**対応:** `crates/robstride-protocol/src/model.rs` の `MotorModel::Rs05` を
`velocity: 50.0, torque: 5.5` に修正し、テスト (`rs05_scales`) も更新済み（本日実施）。

## 3. コマンド体系 (Communication Type) は RS04 / EL05 とも共通

マニュアル記載の Communication type 0–26 は次の通り（RS04・EL05 で完全に同一の一覧）:

| type | 内容 | 実装 (`CommType`) |
|---|---|---|
| 0 | Get device ID | ✅ |
| 1 | Operation control (MIT) | ✅ |
| 2 | Motor feedback | ✅ |
| 3 | Enable | ✅ |
| 4 | Disable | ✅ |
| 6 | Set mechanical zero | ✅ |
| 7 | Set CAN ID | ✅ |
| 17 | Read parameter | ✅ |
| 18 | Write parameter | ✅ |
| 21 | Fault report | ✅ |
| 22 | Save parameters (flash) | ✅ |
| 23 | Set baudrate | ✅ |
| 24 | Active report | ✅ |
| 25 | Set protocol | ✅ |
| **26** | **Version number read** | **未実装**（`CommType` に無し。実害は小さいが将来追加候補） |

RunMode (0=MIT/1=Position/2=Velocity/3=Torque) も両マニュアルで完全一致。

## 4. EduLite05 固有の実装ノート（マニュアル記載、未実装機能）

EL05 マニュアルには RS04 には無い「Notes for Implementation」節があり、次の EduLite 固有の
挙動が書かれている。現状ドライバ未対応だが、将来 EduLite05 を本格運用する場合は把握しておく
べき項目:

- **`zero_sta` フラグ**: デフォルト 0 で電源投入時の位置レンジが 0–2π、1 にすると −π〜π になる。
  type18 で書き換え後 type22 で保存が必要。
- **CANopen ID の新旧差異**: 旧バージョンは CANopen ID が 1 固定、新バージョンはプライベート
  プロトコルの CAN ID と一致するよう変更。
- **電源断時のアンチバックドライブ**: デフォルトで電源 OFF 時に急速回転するとダンピングがかかる
  （`damper=1` で無効化可能）。
- **`add_offset`**: ゼロ点を「現在位置 + オフセット」にずらす機能（機械的な可動域制限の回避用途）。
- **ゼロキャリブレーションの新旧差異**: 旧仕様は較正時に大きな偏差でモータが即座に目標へ移動、
  新仕様（CSP / Motion Control）は目標が瞬時に 0 に更新されモータは静止したまま。

いずれも `robstride-protocol` の `ParamIndex` 経由で書き込み可能なはずだが、専用のヘルパー
メソッドは未実装。

## 5. マニュアル自体の記載不整合（参考）

EL05 マニュアルを全文書き起こす過程で、マニュアル自体に内部矛盾が見つかった:
Communication type 1（MIT指令）のバイトテーブルおよび MIT プロトコル節はトルクレンジを
「-6 N·m ~ 6 N·m」と記載しているが、ファームウェアのサンプルコード（`#define T_MIN/T_MAX`）
は `-5.5f/5.5f` としている。今回の `model.rs` 修正はファームウェアマクロ側（5.5、かつ
RS05 側マニュアルとも一致）を採用した。「6 N·m」はカタログスペック（ピークトルク）の
丸め値、「5.5 N·m」は実際の量子化上限、という整理が妥当と考えられるが未確定。

## 6. 未確認・要フォローアップ

- RS-00/01/02/03/06 は今回ダウンロード対象外のため、同様のスケール不一致がないか未確認。
  もし他モデルも運用予定なら、同じ手順（GitHub `RobStride/Product_Information` または
  `files.seeedstudio.com/products/RobStride/Product%20Literature/<model>/` から英語版 PDF取得）で
  裏取りを推奨。
- 標準リポジトリ `/home/takara/work/dp/robstride-driver`（misa-actuator とは別の独立リポジトリ）
  にも同じ `Rs05` スケール値が残っている。そちらを同期するかは要判断（今回は misa-actuator 側の
  み修正）。

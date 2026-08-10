# 拡張パラメータテーブル: ファームウェアとマニュアルの乖離

実機 **RS04 (AppCodeVersion 0.4.1.32 / AppBuildDate May 14 2026)** を can1 id=1 で読み出し、
`param_type_table.rs`(公式マニュアル `ref/RS04_User_Manual_en.pdf` 由来)と突き合わせた結果。

## 結論

**このファームウェアが自己申告する84行のうち32行が、マニュアルのテーブルと一致しない。**
ずれは3領域に集中し、いずれも行の挿入によるシフトである。

| 領域 | 症状 |
|---|---|
| `0x2006`-`0x200F` | 1行シフト＋`status1`の位置が異なる |
| `0x201A`-`0x2021` | 最大4行シフト。ワイヤ側にのみ `fault1`..`fault5` が存在 |
| `0x3007` | ワイヤ `encoder2raw` / テーブル `vbus(mv)` |
| `0x3010`-`0x301C` | 1行シフト |

## 危険性: CAN ID を壊し得る

最も重要なのは `0x2009` 付近。**生バイトがワイヤ側の命名を裏付けている。**

| FunctionCode | 生バイト | 実際 | マニュアルのテーブル |
|---|---|---|---|
| `0x2009` | `[01 FD 00 00]` → 1 = addressing に使った CAN ID | **CAN_ID** | `motor_baud` |
| `0x200A` | `[FD 00 00 00]` → 253 = 指定した `--host-id` | **CAN_MASTER** | `CAN_ID` |
| `0x200B` | `0` = classic空間の `can_timeout` と一致 | **CAN_TIMEOUT** | `CAN_MASTER` |

テーブルを信じて「`0x2009` = motor_baud」にボーレート値を書くと、実際には
**モータの CAN ID を上書き**し、旧アドレスから消える。これは既知の SET_CAN_ID
インシデント(RS04 の ID 変更で1日応答しなくなった)と同じ失敗様式である。

## 実装側の扱い

- **ワイヤ申告名を正とする。** `param-table` / `params`(comm_type 19)は
  モータ自身が名前を返すため影響を受けない。`lookup_param_type_by_name` を優先すること。
- `read-param`(単一行読み)はワイヤから名前を得られず FunctionCode 引きに頼るため、
  **表示上「マニュアル由来の推定」と明示**し、生の全幅解釈を併記する。
- `set-id` は専用フレーム(`set_device_id`, comm_type 7)を使うため本件の影響を受けない。
- 汎用の write-param は CLI に存在しない。テーブル引きで書き込む経路を追加しないこと。

## 再現方法

```bash
robstride-cli -i can1 --model RS04 -m 1 params      # ワイヤ申告名
robstride-cli -i can1 --model RS04 -m 1 read-param 0x2009   # テーブル引き
```

## モデル自己申告: `AppCodeName` だけが書かれている (2026-08-01 追記)

2機種で自己記述行を実測した結果:

2機種の自己記述行を motorstudio の "Read Para"(comm_type 19、こちらの
`read_param_table` と同じ経路)で実測:

| 行 | RS04 | EduLite05 |
|---|---|---|
| `0x0000 Name` | `FF FF FF FF` | `FF FF …`(全FF) |
| `0x0001 BarCode` | `FF FF …`(全FF) | `FF FF …`(全FF) |
| **`0x1007 AppCodeName`** | **`motor`** | **`EL_motor`** |
| `0x1003 AppCodeVersion` | `0.4.1.32` | `1.0.5.0.1` |
| `0x1001 BootBuildDate` | Mar 26 2024 | Oct 11 2022 |
| `0x1005 AppBuildDate` | May 14 2026 | Oct 23 2025 |
| MCU UID | `0F302000C03B5CC3` | `06B6311821300A62` |
| `0x2007` | `status1`(テーブルがシフト) | `limit_torque` Max **6** |

**`Name` は両機種とも消去済みフラッシュ。** モデル判別に当てにしてはいけない。

**`AppCodeName` は書かれているが RS 系は判別不能。** EduLite05 は `EL_motor` で
製品ラインだけは分かる(サイズは分からない)。一方 **RS04 は `motor` のみ**で、
RS-00〜RS-06 のどれかすら絞れない。したがって:

- `EL_motor` → `BuildName::Narrows([Rs01, Rs02, Rs05])`
- `motor` → `BuildName::Uninformative`(**既知だが無価値**。未知と区別する)

motorstudio が `motor type RS04` / `motor type EL05` と表示する仕組みは**未解明**。
`AppCodeName` だけでは RS04 を導けないので、別経路があるか、EL 系でなければ RS04 と
表示する既定動作の可能性がある。**確認するにはバスを覗くしかない**(下記)。

なお `0x2007 limit_torque` の Max は EL05 実機で **6**。マニュアルの表は RS04/EL05 とも
`17` と記載しており(コピペと思われる)、ここでも実機が正。RS04 側は
テーブルがシフトしていて `0x2007` は `status1`。

### モデル判別に実際に使えるもの (2026-08-02 実測で確定)

**`0x700B limit_torque`(古典パラメータ空間、`ParamIndex::LimitTorque`)が本命。**

| | 実測 `limit_torque` | MIT トルクスケール |
|---|---|---|
| RS-04 | **115 N·m** | 120 |
| EduLite05 | **6 N·m** | 5.5 |

RS-04 の 115 N·m を表現できるのは family 中 RS-04 のみ(RS-03/06 は 60、
RS-00/01/02 は 17、RS-05 は 5.5)。つまりこの1台については**反証にとどまらず一意特定**
になっている。EL05 の 6 N·m は 5.5×1.25 の許容内なので誤警報も出ない。

**`comm type 26`(バージョン読み)も接続時に使える。** 文書化済みで往復1フレーム、
実機で `0.4.1.32` を取得できた(motorstudio の表示と一致)。ただし要求は Disable、
応答はフィードバックと同じ comm type なので、ペイロードのプレフィックス検証が必須。

### やってはいけないこと

**`read_single_param`(拡張パラメータの単発読み)を必須経路に置かない。**
2026-08-02 の実測で:

- `AppCodeName` が `"moto"` と返る(正しくは `motor`)。サブフレームを取りこぼし、
  モータの送信中に読み出しを打ち切っている
- **その直後から全ての読みがタイムアウトし、電源再投入まで復帰しない**(2回再現)
- 文字列読みを外した接続シーケンス(バージョン読み + limit 読み)は同じ機体で正常動作

因果は未証明だが、ドライバ内で唯一のリバースエンジニアリング経路であり、
切り上げバグは実測できている。`robstride-cli identify` は既定でこの経路を使わず、
`--deep` で明示的に指定したときだけ実行する。

## 未解決

- `0x0000 Name` / `0x0001 BarCode` は非ASCIIのバイト列を返す(上記のとおり未書き込み確定)。
- 一括ダンプの最終行 `0x301C` が `raw=[]` で返る(単独読みでは4バイト取得できるので、
  ストリーム終端の取りこぼしの可能性)。
- 12バイト返す7行(`baud`/`protocol_1`/`damper`/`add_offset`/`alveolous_open`/`iq_test`/
  `acc_stutus`)はマニュアルの型表に該当がなく未デコード。

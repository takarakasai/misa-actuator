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

## 未解決

- `0x0000 Name` / `0x0001 BarCode` は非ASCIIのバイト列を返す(この機体では未書き込みと思われる)。
- 一括ダンプの最終行 `0x301C` が `raw=[]` で返る(単独読みでは4バイト取得できるので、
  ストリーム終端の取りこぼしの可能性)。
- 12バイト返す7行(`baud`/`protocol_1`/`damper`/`add_offset`/`alveolous_open`/`iq_test`/
  `acc_stutus`)はマニュアルの型表に該当がなく未デコード。

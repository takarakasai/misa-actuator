# DM-J4310-2EC / DM-J3507-2EC 公式マニュアル突き合わせ

ユーザー提供の公式PDFマニュアル(`ref/DM-J4310-2EC V1.2 User Manual V1.2  2026-04-09.pdf`、
`ref/DM-J3507-2EC Gear Motor User Manual V1.1 2026-04-16.pdf`)を全文markdown化し
(`ref/dm4310_manual_en.md`、`ref/dm3507_manual_en.md`)、既存実装(`register.rs`/`feedback.rs`/
`limits.rs`/`frame.rs`)と突き合わせた。**DAMIAOの公式マニュアルがこのワークスペースに追加された
のは今回が初めて**——これまでの`Rid`定義は`robstride_sandbox`由来の断片的な知識のみに基づいていた。

## 1. レジスタマップ: 反映した変更

公式マニュアルの「Register Map」節(DM4310/DM3507とも0x00〜0x24までは完全に同一)に、既存の
`Rid`(10定数のみ)には無かった項目を発見・追加した:

| RID | 名前 | 追加した定数 | 備考 |
|---|---|---|---|
| 0x0E (14) | sw_ver | `Rid::SW_VER`(新規) | **本命の発見**。公式マニュアルの"Firmware Version Check"節が実際に読んでいるのはこのレジスタ。ベンダー純正ツールの「Read Version」ボタンが読む値そのもの |
| 0x0D (13) | hw_ver | `Rid::HW_VER`(新規) | マニュアル上は"Reserved"表記(名前と矛盾、要注意) |
| 0x0F (15) | SN | `Rid::SN`(新規) | 同上、"Reserved"表記 |
| 0x10 (16) | NPP | `Rid::NPP`(新規) | Number of Pole Pairs、キャリブレーションで自動識別 |
| 0x14 (20) | Gr | `Rid::GR`(新規) | Gear Reduction Ratio。"preconfigured, do not modify"と明記 |
| 0x25 (37) | Boot_ver | `Rid::BOOT_VER`(新規) | Bootloader Version。**DM4310専用**——DM3507のマップは0x24の次が0x32に飛ぶ(セクション5参照) |

**既存の`Rid::SUB_VER`(0x24=36)は「Firmware Version」ではなく、マニュアル上は単に
"Sub-version"という別の副次的なバージョン値だった。** 以前`misa-actuator-identify`が
DAMIAOのFWバージョンとしてこれを流用していたのは誤りだったため、`Rid::SW_VER`に差し替えた
(コード修正済み)。

`is_int()`のRID範囲チェックも`35..=36`→`35..=37`に拡張(`BOOT_VER`を追加したため)。

## 2. Read/Write/Save Parametersのフレーム形式: 既存実装の正しさを確認

マニュアルの「Read Parameters」(`0x33`)/「Write Parameters」(`0x55`)/「Save Parameters」
(`0xAA`, D3=`0x01`)のバイト単位フォーマットは、`register.rs`の既存実装(`build_read_reg`/
`build_write_reg_f32`/`build_write_reg_int`/`build_save_all`)と**完全に一致**していた。
新規実装の必要は無く、既存実装の正しさを裏付けただけ。

「Save Parametersは無効化状態でのみ有効、フラッシュ書き込み最大30ms」という記載も、既存の
`save_to_flash`(disable要求+約50ms sleep)の実装方針と整合している。

## 3. `ErrorCode`: 3件の未実装フォールトコードを追加

マニュアルの「LED Status」フォールトコード表に、既存の`ErrorCode`(0,1,8-Eのみ)には無かった
3つのコードを発見・追加した:

- `0x3` = Output shaft calibration error → `ErrorCode::OutputShaftCalibration`(新規)
- `0x4` = Sensor output error → `ErrorCode::SensorOutput`(新規)
- `0x5` = Motor encoder calibration error → `ErrorCode::EncoderCalibration`(新規)

`misa_actuator::ErrorFlags`への変換(`damiao-driver/src/actuator.rs`)は、キャリブレーション系
2つ(0x3, 0x5)を`UNCALIBRATED`ビットに、センサ系(0x4)を`ENCODER_FAULT`ビットにマッピングした。

**注意点**(並行して走らせたマニュアル変換エージェントも同じ点を指摘): マニュアルの
「Feedback Frames」節が明示するCAN上のERRフィールド値は`0,1,8,9,A,B,C`のみで、**3/4/5は
おろか、既存実装済みの`D`(CommLost)/`E`(Overload)すらこの節の説明には出てこない**——LED表
にのみ載っている。つまり3/4/5がCAN経由の feedback フレームで実際に流れてくるかは未確認だが、
これは元々D/Eも同じ扱いだった(LED表を正としてCANでも来ると仮定して実装されている)ため、
今回の3件も同じ前提を踏襲しただけで、新たなリスクを持ち込んではいない。

## 4. PMAX/VMAX/TMAXは「レジスタ読み取り」が正しい設計だったことを確認

マニュアルにはRobStrideのMIT scaleのような固定スペック表が無く、PMAX/VMAX/TMAX(RID
0x15/0x16/0x17)は「Range: (0.0, fmax]」というレジスタ可変値としてのみ定義されている
(`Parameter Management`節: 「Recommended parameters」はあるが固定値ではない)。既存の
`damiao-protocol::limits::MotorModel`のハードコード値は"出荷時の参考値"の位置づけで、実際の
`info`コマンドはレジスタから読む設計になっている——これは正しいアプローチだったと確認できた。

## 5. レジスタテーブルの網羅(実装済み)

当初は見送っていたが、その後**共通レジスタブロック`0x00`-`0x25`(38個)を全て`Rid`に定義**した
(それまでは16定数のみで、22個が未定義だった)。追加したのは以下:

| 分類 | 追加したRID |
|---|---|
| 保護しきい値 | `OT_VALUE`(2) / `OC_VALUE`(3) / `OV_VALUE`(29) |
| モーションプロファイル | `ACC`(4) / `DEC`(5) / `MAX_SPD`(6) |
| 通信 | `TIMEOUT`(9) |
| 電流ループ | `I_BW`(24) / `IQ_C1`(33) |
| 速度ループ | `KP_ASR`(25) / `KI_ASR`(26) / `DETA`(31) / `V_BW`(32) / `VL_C1`(34) |
| 位置ループ | `KP_APR`(27) / `KI_APR`(28) |
| ギア | `GREF`(30) |
| 同定済みモータ定数(RO) | `DAMP`(11) / `INERTIA`(12) / `RS`(17) / `LS`(18) / `FLUX`(19) |

これで**DAMIAOのパラメータ一括ダンプが他3ベンダー(RobStrideの`ParamIndex`、MyActuatorの
`0xC0`空間、LKMotorの`0xC0`+`0x40`)と同水準になり、特に従来1つも取得できていなかった
PIDゲインが全て読めるようになった**。`damiao-cli params`もRID順の38項目に拡張し、
出力を分類ごとにグループ化した。

**`is_int()`は修正不要だった**。追加した22個のうちuint32は`TIMEOUT`(9)のみで、既存の
`7..=10`範囲に既に含まれていたため。残り21個はすべてfloat。この分類がマニュアルのType列と
一致することは回帰テスト`int_classification_matches_manual_type_column_for_whole_block`
(0x00-0x25の全RIDを網羅)で固定した。併せて`register_addresses_match_manual`で
定数のアドレスずれも検出できるようにした。

### `Rid::BOOT_VER`(37)はDM4310専用だと判明

**DM3507のレジスタマップは`0x24`(sub_ver)の次が`0x32`(u_off)に飛んでおり、`0x25`(Boot_ver)は
存在しない。** 共通ブロックに見えて実は唯一のモデル依存項目だったため、doc
コメントに明記した。DM3507に対して`params`を実行すると`boot_ver`だけ`<no reply>`になるのが
正常な挙動であり、これはCLIのヘルプにも記載済み。

## 6. モデル固有レジスタ(`0x25`超)の実装

`0x25`より上の領域は**DM4310とDM3507でレジスタマップが分岐する**ため、`Rid`のような
フラットな定数では表現できない。とくに`m_off`は両モデルに存在するが**アドレスが違う**
(DM4310=`0x38` / DM3507=`0x36`)ため、単一定数にすると片方のモデルで別のレジスタを
読んでしまう。そこで**「番号ではなく意味で指定し、モデルと組で解決する」**`ModelReg`
enumを新設した。

```rust
ModelReg::BusVoltage.rid(MotorModel::Dm4310)  // => Some(0x3C)
ModelReg::BusVoltage.rid(MotorModel::Dm3507)  // => None(そのモデルには存在しない)
```

| 区分 | レジスタ | DM4310 | DM3507 |
|---|---|---|---|
| 両モデル同一アドレス | `dir` / `p_m` / `xout` | 0x37 / 0x50 / 0x51 | 同左 |
| **同名・別アドレス** | `m_off` | **0x38** | **0x36** |
| DM4310のみ | `Imax`/`VBus`/`Tpcb`/`Tmtr`/`I_U_OFF`/`I_V_OFF`/`I_W_OFF` | 0x3B-0x41 | — |
| DM3507のみ | `u_off`/`v_off`/`k1`/`k2` | — | 0x32-0x35 |

ドライバ側は`DamiaoMotor::read_model_register(ModelReg)`を追加した。保持している
`model`でRIDを解決し、そのモデルに存在しないレジスタは**未文書アドレスを読むのではなく
`Error::Unsupported`を返す**(既存の`set_zero_nvm`と同じ方針)。CLIには`damiao-cli diag`を
追加し、`--model`に応じて読めるものだけを読み、存在しないものは`<not on this model>`と
表示してバスに触れない。

**DM4310の`VBus`/`Tpcb`/`Tmtr`(バス電圧・PCB温度・モータ温度)はCANフィードバック
フレームに含まれない値**なので、`diag`は実質的に唯一の取得経路になる。

回帰テストで両マニュアルのアドレス表を固定した:
`model_register_addresses_match_manuals`(15レジスタ×2モデルの全組み合わせ)、
`angle_offset_address_differs_between_models`(この設計の存在理由そのもの)、
`model_registers_are_all_float_typed`(全てfloatなので`is_int`が偽であること)。
CLI側にも`diag_toml_serializes_for_both_models`(TOMLはテーブルより先に値を置く必要が
あるという制約の回帰)と`diag_partitions_registers_by_model`を追加した。

### `u_off`/`v_off`と`I_U_OFF`/`I_V_OFF`は統合していない

DM3507の`u_off`/`v_off`(0x32/0x33)は「U相/V相オフセット」、DM4310の`I_U_OFF`/`I_V_OFF`
(0x3F/0x40)は「U相/V相**電流**オフセット」とマニュアルの表記が異なり、さらにDM3507には
`w_off`相当が存在しない。同一量の可能性は高いが**推測で統合せず別バリアントとして保持**し、
その旨をdocコメントに残した。

## 7. 9機種マニュアル+SDK追加後の反映(解決済み)

DM-J系の英語版マニュアル**9機種分**とベンダーSDK(`ref/sdk/motor-sdk`)が追加されたことで、
セクション8に挙げていた未確定項目がすべて解決した。詳細は
**`doc/dm-j-series-cross-model-reference.md`**(横断表)に集約し、ここでは結論だけ記す。

1. **レイアウトはファミリ単位だと確定** — `0x25`超の配置は機種ごとにバラバラではなく
   **3レイアウトに収束**する(`J4310` / `J3507` / `J10422`)。`RegisterLayout`として型にし、
   `ModelReg::rid`は機種ではなくレイアウトでディスパッチするようにした。機種追加は
   `MotorModel::register_layout`への1行マッピングで済む。
   ただし**レイアウトは機体サイズと無相関**(DM-J8009が最小のDM-J3507と同じレイアウト)。
2. **`Boot_ver`の有無はレイアウト差だった** — `J3507`系のみ非対応。
   `MotorModel::has_boot_ver()`で判定し、`params`ダンプはこの機種で`0x25`を読まない。
3. **`supports_nvm_zero`はマニュアル裏付けが取れた** — 9機種すべてが同一のマジックフレームを
   記載しており、全機種`true`が推測ではなく確認事項になった。
4. **`SN`は実在する可能性が高い** — DM-J10422Pのマニュアルのみ"Serial number"と明記
   (他8機種は"Reserved")。`hw_ver`は9機種すべて"Reserved"のままなので、こちらは依然不明。
5. **MIT量子化レンジは「固定値が存在しない」と判明** — 下記。

### `Limit_Param`は拡充ではなく設計変更で解決した

当初は「マニュアルから正しい値を得て`MotorModel`に追加する」つもりだったが、調査の結果
**その前提が成り立たなかった**:

- **9機種すべてのマニュアルが`PMAX`/`VMAX`/`TMAX`を「範囲`(0.0, fmax]`の書き込み可能
  レジスタ」としてのみ定義**し、具体的な数値を一切与えていない。
- **SDKの`Limit_Param`は4コピー間で値が食い違う**(DM6006のt_max 20対12、DM8006の40対20、
  DM4340のv_max 8/10/20など)。さらに`u2canfd`のC++版は配列要素13個に対し列挙が15個で
  `DM6248`の行が欠落しており、**インデックスずれと範囲外アクセスを起こす実バグ**がある。
- 追加された9機種のうち**4機種(4310P/4340P/8009P/10422P)はSDKに項目自体が無い**。

そこで**表は既定値と位置づけ、真値はモータのレジスタから読む**設計に変更した
(`DamiaoMotor::refresh_limits_from_registers()`、CLIは`--refresh-limits`)。これは
SDK自身が`changeMotorLimit`という上書きフックを持つのと同じ思想である。既定値の出自は
`LimitsSource`で明示し、`Sdk`以外の機種はCLIが起動時に警告する。

## 8. 反映を見送った/要確認の項目(次のアクション候補)

1. **`0xCC`(ステータス更新要求)** — SDKの`refresh_motor_status()`が使うが**9機種の
   マニュアルすべてに記載がない**未文書コマンド。フィードバックを能動的にポーリングしたい
   場合の候補だが、実機検証が必要。
2. **`xout`(出力軸位置)の常用経路**: `ModelReg::OutputShaftPosition`で読めるが、
   `misa_actuator::Actuator`のフィードバック経路(CANフィードバックフレーム由来)とは別系統。
   出力軸エンコーダの実測値をフィードバックに使いたい場合は設計の検討が必要。
3. **マニュアル自体の曖昧さ**: DM3507の「Mode Switching」説明文は「切替前にregister 0x50を
   読んで正確な位置を確認」と書いてあるが、レジスタマップ上0x50は`p_m`(モータ側位置)で、
   出力軸位置は0x51(`xout`)。CAN経由の位置フィードバックが出力軸基準であることを踏まえると
   0x51が意図だった可能性があるが、原文は"0x50"と明記しているため実機での要検証
   (`ref/dm3507_manual_en.md`の該当箇所に**Note**として記載済み)。
4. `Rid::HW_VER`は9機種すべてで"Reserved"表記——意味のある値が返るかは未検証。
5. **SDKにあるがマニュアル未取得の機種**: `DM4310_48V`/`DM4340_48V`/`DM6006`/`DM8006`/
   `DM10010L`/`DM10010`/`DMH3510`/`DMH6215`/`DMG6220`/`DMJH11`/`DMS3519`。
   `MotorModel`には追加していない(レジスタレイアウトが判別できないため)。
6. **DM-J10422Pの`ManualSpecsFloor`の検証** — 唯一SDKにも兄弟機種にも情報が無く、
   既定値はマニュアル仕様表からの下限導出。実機で`--refresh-limits`して実値を確認したい。

## 参照

- 全文markdown化: `ref/dm4310_manual_en.md`、`ref/dm3507_manual_en.md`
- 元PDF: `ref/DM-J4310-2EC V1.2 User Manual V1.2  2026-04-09.pdf`、
  `ref/DM-J3507-2EC Gear Motor User Manual V1.1 2026-04-16.pdf`

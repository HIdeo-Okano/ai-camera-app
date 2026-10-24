# ECG CSV 転送 iPad → iPhone（BLE / Pythonista 3）

iPad で選んだ心電図 CSV（例：`ECGtsHDdata1.csv`）を、Bluetooth Low Energy（BLE）で iPhone に送り、
iPhone で波形を表示します。Wi-Fi・IP アドレス・端末名の設定は要りません。

| 端末 | スクリプト | 役割 | BLE の役割 |
|------|-----------|------|-----------|
| iPad（送信） | `ecg_sender_ipad.py` | CSV を選んで 4行ヘッダ形式にそろえて送る | ペリフェラル（`CBPeripheralManager`） |
| iPhone（受信） | `ecg_viewer_iphone.py` | 受信して保存し、波形・異常検知を表示する | セントラル（`CBCentralManager`） |

どちらも **ファイル1つで動きます**（BLE の共通部分 `ecg_ble_section.py` を埋め込み済み）。
動作確認済みの版：**2026-10-25 r17**（起動時に画面へ表示されます）。

## 準備（最初に1回）

1. iPad・iPhone の両方に Pythonista 3 を入れ、Bluetooth をオンにする
2. スクリプトを **Pythonista 自身のフォルダー** に置く
   - iPad：`ecg_sender_ipad.py` を「ファイル」アプリで **このiPad内 > Pythonista 3** に移動
   - iPhone：`ecg_viewer_iphone.py` を **このiPhone内 > Pythonista 3** に移動
   - 「ファイル」アプリから直接開いて実行しないでください（画面左上に「◀ファイル」と出る開き方）。
     この開き方では同じフォルダーへの書き込みができず、正しく動きません
3. 初回の起動時に Bluetooth の使用許可を求められたら「許可」する
4. iPad に送る CSV を置いておく（「ファイル」アプリの「ダウンロード」など、どこでも選べます）

## 使い方

1. **iPhone**：Pythonista で `ecg_viewer_iphone.py` を開いて ▶
   - 「BLE 受信待機中（iPad の「送信」を待っています）」と出れば準備完了。画面はそのまま前面に出しておく
2. **iPad**：Pythonista で `ecg_sender_ipad.py` を開いて ▶
   - 約 3 秒後に BLE が始まり、「BLE の準備ができました」と出るまで待つ
3. **iPad**：「① CSVを選ぶ」で CSV を選ぶ
   - 3行目（電極状態）・4行目（不整脈の件数）は解析結果が入ります。必要なら書き換えられます
4. **iPad**：「② 送信（BLE）」を押す
   - iPhone が自動で接続し、送信中…% が進みます
5. 終わると
   - iPad：「送信しました。iPhone 側の保存名: …」→ 次の「送信」を待つ状態に戻る
   - iPhone：「受信完了（N 件目）」→ 「データ検出」になり、横向きにすると波形を表示。自動で次の受信を待つ
6. 続けて送るときは、iPad で ①・② をくり返すだけです（どちらのスクリプトも起動し直す必要はありません）
7. 終えるときは、iPad は「閉じる」、iPhone は右上の「×」を押します

転送中は両方の端末で Pythonista を前面に出したままにしてください（バックグラウンドに回ると BLE が止まります）。

### 受信したファイルの場所（iPhone）

「ファイル」アプリの **このiPhone内 > Pythonista 3 > Downloads**。同じ名前のファイルがあるときは上書きします。
iPhone の「ファイル」ボタンからは、AirDrop などで受け取った CSV を直接開くこともできます。

## CSV の形式

| 行 | 内容 |
|----|------|
| 1 | `ECG` |
| 2 | サンプル周波数（256） |
| 3 | 電極状態（0＝正常） |
| 4 | 不整脈の件数（0＝なし） |
| 5〜 | 心電データ |

元ファイルが2行ヘッダ（`ECG`／`256` の直後から心電データ）の場合は、iPad が波形を解析して3・4行目を作って送ります。

## うまくいかないとき

| 症状 | 確認すること |
|------|-------------|
| iPhone が iPad を見つけない | iPad で「BLE の準備ができました」と出てから「② 送信」を押したか。両方の Bluetooth がオンか。Pythonista が前面か |
| 「Bluetooth が許可されていません」 | 「設定」アプリ > Pythonista > Bluetooth をオン |
| 120 秒たって「iPhone が見つかりませんでした」 | iPhone で `ecg_viewer_iphone.py` が起動しているか |
| Pythonista が落ちた | もう一度同じスクリプトを起動すると、画面に「前回の動作記録」「前回落ちた時の Python の位置」が出ます。そのスクリーンショットを開発者へ送ってください |

動作記録は、スクリプトと同じフォルダー（書き込めない場合は Pythonista の Documents）に次の名前で残ります。
消しても動作に影響はありません。

- `ecg_ble_trace_sender.txt` / `ecg_ble_trace_receiver.txt`（今回の記録。前回分は `_prev.txt`）
- `ecg_ble_fault_sender.txt` / `ecg_ble_fault_receiver.txt`（iOS の中で落ちた時の Python の位置）

## 仕組み

### 通信の流れ

サービス UUID `8F1D0001-2C5A-4E7B-9A63-5E0C6A1B2D01`（アドバタイズ名 `ECG-TX`）

| キャラクタリスティック | UUID | 方向 | 用途 |
|---|---|---|---|
| DATA | `8F1D0002-…` | iPad → iPhone（Notify） | ファイルデータ（1通知＝1パケット） |
| ACK  | `8F1D0003-…` | iPhone → iPad（Write） | 受信結果 `OK <保存名>` / `NG <理由>` |

パケット（大きさは iPhone が受け取れる最大長 `maximumUpdateValueLength` に合わせる）：

- `H` + JSON `{"name", "size", "crc", "fmt"}` … ヘッダー
- `D` + 連番（4 バイト）+ データ
- `E` … 終了。iPhone がサイズと CRC32 を確かめて保存し、ACK に結果を書き込む

終わり方：

- iPhone … ACK を書き込んだら切断し、サービス UUID で次の送信を待つ
- iPad … ACK を受け取ったらアドバタイズを止め、BLE の準備（サービス）は残したまま次の「送信」を待つ

### 実装上の注意（Pythonista 3.4）

- Pythonista の `cb` モジュールはセントラル専用で、書き込みが `PY_SSIZE_T_CLEAN` エラーになるため使わない。
  CoreBluetooth は `ctypes` で `objc_msgSend` を呼び出しの型ごとに専用の関数にして直接呼ぶ（`_send`）。
  `objc_util` はデリゲートのクラス作成（`create_objc_class`）と `load_framework` だけに使う
- デリゲートの関数はモジュール直下に置く（入れ子の関数にすると iPad で NameError になった）
- iPad（送信）は CoreBluetooth の操作を **専用の作業スレッド** で行い、iOS からの通知（メインスレッド）は
  値を取り出して作業スレッドへ渡すだけにする。メインスレッドで全部行っていた r16 までは、iPhone の接続時に iPad が落ちた
- iPad の画面は `ui.delay` を使わず、`update()`（0.2 秒ごと）でほかのスレッドからの記録や結果を画面に出す
- 登録したサービス・特性は画面を閉じるまで手放さない（手放すと iPhone が接続した時に落ちる）
- スクリプトは `threading.Event().wait()` で画面を閉じるまで終わらせない

## 開発用ファイル

| ファイル | 内容 |
|---------|------|
| `ecg_ble_section.py` | BLE の共通部分（両方のスクリプトに同じものを埋め込む） |
| `test_ecg_ble.py` | PC 用のテスト（CoreBluetooth を模擬して送受信・終了処理・作業スレッドを確認） |
| `ble_diag_ipad.py` | iPad の BLE 診断（準備を1段ずつ行い、iPhone の接続を待つ） |
| `ble_diag_sender_start.py` | iPad 送信プログラムの起動診断 |

```sh
cd pythonista/ecg_ble
python3 test_ecg_ble.py [ECGtsHDdata1.csv のパス]
```

旧版の `pythonista/ble_csv_transfer/`（iPhone → iPad 方向）は使いません。

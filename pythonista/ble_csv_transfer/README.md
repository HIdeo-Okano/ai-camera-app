# iPhone → iPad CSV 転送 (BLE / Pythonista)

> **旧版です。** 現在は iPad → iPhone 方向の [`../ecg_ble/`](../ecg_ble/README.md)（`ecg_sender_ipad.py` / `ecg_viewer_iphone.py`）を使います。

iPhone の **iCloud Drive 直下** に保存された `ECGtsHDdata1.csv` を、Bluetooth Low Energy (BLE) で iPad の **「ダウンロード」フォルダー** に転送します。

| 役割 | 端末 | スクリプト | BLE の役割 |
|------|------|-----------|-----------|
| 送信側 | iPhone | `csv_sender_iphone.py` | セントラル (Pythonista `cb` モジュール) |
| 受信側 | iPad   | `csv_receiver_ipad.py` | ペリフェラル (`objc_util` + CoreBluetooth `CBPeripheralManager`) |

> Pythonista の `cb` モジュールはセントラル専用で、iOS 同士を直接つなぐには片方がペリフェラルになる必要があります。
> そのため受信側 (iPad) は `objc_util` で CoreBluetooth を直接呼び出してペリフェラルとして動作します。

## 使い方

1. 両方の端末で Bluetooth を ON にし、Pythonista に Bluetooth の使用を許可する
2. **iPad** で `csv_receiver_ipad.py` を実行 →「アドバタイズ開始」と表示されるまで待つ
3. **iPhone** で `csv_sender_iphone.py` を実行 → iCloud Drive 直下の `ECGtsHDdata1.csv` が自動で送信される
4. iPhone に「送信完了」と表示されれば成功
5. iPad では「ダウンロード」フォルダー (`iCloud Drive/Downloads`) に保存される (同名ファイルは `ECGtsHDdata1_1.csv` のように連番)

送信ファイル名は `csv_sender_iphone.py` の `SEND_FILE_NAME`、保存先は `csv_receiver_ipad.py` の `SAVE_DIR_OVERRIDE` で変更できます。

### iCloud Drive へのアクセスについて (初回のみ)

Pythonista はセキュリティ上、iCloud Drive を自由に読み書きできない場合があります。その場合は次のとおり動きます。

- **iPhone (送信)**: 直接読めないときは「ファイル」アプリの選択画面が開くので、iCloud Drive の `ECGtsHDdata1.csv` を選ぶ。
  ファイルが iCloud 上にしかない (未ダウンロード) 場合もこの画面で選べば取得されます。
- **iPad (受信)**: ダウンロードフォルダーに書き込めないときは Pythonista 内の `Documents/received_csv/` に保存し、その旨を表示する。
  Pythonista のファイル一覧で **「Open External Folder」→ iCloud Drive の「ダウンロード」** を一度開いておくと、以降はダウンロードフォルダーに直接保存されます。
  (「このiPad内」の「ダウンロード」を使う場合も同様に Open External Folder で開いてください)

転送中は両方の端末で Pythonista を前面に表示したままにしてください (バックグラウンドに回ると BLE 通信が止まります)。
受信を終了するときは iPad 側で Pythonista の停止ボタンを押します。

## 転送プロトコル

サービス UUID `6E400001-B5A3-F393-E0A9-E50E24DCCA9E`

| キャラクタリスティック | UUID | 方向 | 用途 |
|---|---|---|---|
| DATA   | `6E400002-…` | iPhone → iPad (Write with response) | ファイルデータ |
| STATUS | `6E400003-…` | iPad → iPhone (Notify / Read) | 受信結果 `OK:<保存名>` / `ERR:<理由>` |

DATA に書き込むパケット (1 パケット最大 180 バイト):

- `H` + JSON `{"name": ファイル名, "size": バイト数, "crc": CRC32}` … ヘッダー
- `D` + 連番 (4 バイト, ビッグエンディアン) + データ最大 175 バイト
- `E` … 終了 (iPad がサイズと CRC32 を検証して保存し、STATUS で結果を通知)

1 パケットごとに書き込み応答を待ってから次を送るため取りこぼしはありません。
転送速度の目安は数 KB/秒です (数百 KB 程度までの CSV を想定)。

## 動作確認 (PC)

BLE 部分を模擬したテストで送受信の組み立て・CRC 検証を確認できます。

```sh
cd pythonista/ble_csv_transfer
python3 test_protocol.py
```

## うまくいかないとき

- **受信側が見つからない**: iPad で受信スクリプトが「アドバタイズ開始」になっているか確認。
  iPhone は周囲の名前のある機器へ順に接続し、CSV 受信サービスを持つ機器を iPad と判断します
  (iPad の名前は何でも構いません。名前に `CSV-RX` / `iPad` を含む機器は優先して確認します)。
  iPhone 側の名前を登録する必要はありません (iPad はどの機器からの接続も受け付けます)。
- **書き込みに失敗する**: 古い端末で MTU が小さい場合は `PACKET_SIZE` を `20` に下げてください。

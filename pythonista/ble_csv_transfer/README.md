# iPhone → iPad CSV 転送 (BLE / Pythonista)

iPhone に保存された CSV ファイルを、Bluetooth Low Energy (BLE) で iPad に転送します。

| 役割 | 端末 | スクリプト | BLE の役割 |
|------|------|-----------|-----------|
| 送信側 | iPhone | `csv_sender_iphone.py` | セントラル (Pythonista `cb` モジュール) |
| 受信側 | iPad   | `csv_receiver_ipad.py` | ペリフェラル (`objc_util` + CoreBluetooth `CBPeripheralManager`) |

> Pythonista の `cb` モジュールはセントラル専用で、iOS 同士を直接つなぐには片方がペリフェラルになる必要があります。
> そのため受信側 (iPad) は `objc_util` で CoreBluetooth を直接呼び出してペリフェラルとして動作します。

## 使い方

1. 両方の端末で Bluetooth を ON にし、Pythonista に Bluetooth の使用を許可する
2. **iPad** で `csv_receiver_ipad.py` を実行 →「アドバタイズ開始」と表示されるまで待つ
3. **iPhone** で `csv_sender_iphone.py` を実行し、送信する CSV を選ぶ
   - Pythonista 内の CSV 一覧、または「ファイル」アプリから選択
4. iPhone に「送信完了」と表示されれば成功
5. iPad では Pythonista の `Documents/received_csv/` に保存される (同名ファイルは `名前_1.csv` のように連番)

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
  iPhone は名前に `CSV-RX` または `iPad` を含む端末へ接続し、CSV 受信サービスが無ければ次の端末を探します。
  iPad の端末名を変更している場合は `csv_sender_iphone.py` の `NAME_HINTS` にその名前を追加してください。
- **書き込みに失敗する**: 古い端末で MTU が小さい場合は `PACKET_SIZE` を `20` に下げてください。

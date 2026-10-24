# -*- coding: utf-8 -*-
"""
CSV BLE 受信プログラム (iPad / Pythonista 3)

iPad を BLE ペリフェラルとして動作させ、iPhone (csv_sender_iphone.py) から
送られてくる CSV ファイルを受信して保存します。

Pythonista 標準の `cb` モジュールはセントラル (接続する側) 専用のため、
受信側は objc_util で CoreBluetooth の CBPeripheralManager を直接利用します。

使い方:
  1. iPad の Pythonista でこのスクリプトを実行する
  2. 「アドバタイズ開始」と表示されたら iPhone 側で送信スクリプトを実行する
  3. 受信したファイルは Pythonista の Documents/received_csv/ に保存される
  終了するときは Pythonista の停止ボタン (×) を押してください。

転送プロトコル (1 書き込み = 1 パケット):
  'H' + JSON(UTF-8)  ヘッダー  {"name": ファイル名, "size": バイト数, "crc": CRC32}
  'D' + 連番(4byte BE) + データ
  'E'                終了
  受信結果は STATUS キャラクタリスティックの Notify で返す ("OK:..." / "ERR:...")
"""

import json
import os
import time
import zlib

# ---- プロトコル定数 (送信側と同じ値にすること) ----
SERVICE_UUID = '6E400001-B5A3-F393-E0A9-E50E24DCCA9E'
DATA_CHAR_UUID = '6E400002-B5A3-F393-E0A9-E50E24DCCA9E'    # iPhone -> iPad (Write)
STATUS_CHAR_UUID = '6E400003-B5A3-F393-E0A9-E50E24DCCA9E'  # iPad -> iPhone (Notify/Read)
LOCAL_NAME = 'CSV-RX'

SAVE_DIR = os.path.expanduser('~/Documents/received_csv')


class TransferError(Exception):
    pass


class CSVAssembler(object):
    """受信パケットを組み立ててファイルとして保存する (BLE に依存しない部分)"""

    def __init__(self, save_dir=SAVE_DIR, log=print):
        self.save_dir = save_dir
        self.log = log
        self.reset()

    def reset(self):
        self.name = None
        self.size = 0
        self.crc = 0
        self.next_seq = 0
        self.buf = bytearray()
        self.started_at = 0.0
        self.last_report = 0

    @property
    def active(self):
        return self.name is not None

    def handle(self, packet):
        """1 パケットを処理する。転送完了時はステータス文字列を返す。"""
        if not packet:
            raise TransferError('空のパケット')
        kind, body = packet[:1], packet[1:]
        if kind == b'H':
            return self._on_header(body)
        if not self.active:
            raise TransferError('ヘッダー未受信のままデータを受信しました')
        if kind == b'D':
            return self._on_data(body)
        if kind == b'E':
            return self._on_end()
        raise TransferError('不明なパケット種別: %r' % kind)

    def _on_header(self, body):
        if self.active:
            self.log('前回の転送が未完了のため破棄します: %s' % self.name)
        self.reset()
        try:
            info = json.loads(body.decode('utf-8'))
            name = os.path.basename(str(info['name'])) or 'received.csv'
            size = int(info['size'])
            crc = int(info['crc'])
        except Exception as e:
            raise TransferError('ヘッダーが不正です: %s' % e)
        self.name, self.size, self.crc = name, size, crc
        self.started_at = time.time()
        self.log('受信開始: %s (%d bytes)' % (name, size))
        return None

    def _on_data(self, body):
        if len(body) < 4:
            raise TransferError('データパケットが短すぎます')
        seq = int.from_bytes(body[:4], 'big')
        if seq != self.next_seq:
            raise TransferError('連番が不正です (期待 %d / 受信 %d)' % (self.next_seq, seq))
        self.next_seq += 1
        self.buf.extend(body[4:])
        if len(self.buf) > self.size:
            raise TransferError('宣言サイズを超えるデータを受信しました')
        pct = len(self.buf) * 100 // self.size if self.size else 100
        if pct >= self.last_report + 10:
            self.last_report = pct - pct % 10
            self.log('  受信中... %3d%% (%d / %d bytes)' % (pct, len(self.buf), self.size))
        return None

    def _on_end(self):
        data = bytes(self.buf)
        if len(data) != self.size:
            raise TransferError('サイズ不一致 (宣言 %d / 受信 %d)' % (self.size, len(data)))
        if zlib.crc32(data) & 0xFFFFFFFF != self.crc:
            raise TransferError('CRC が一致しません (データ破損)')
        path = self._save(data)
        elapsed = max(time.time() - self.started_at, 0.001)
        self.log('受信完了: %s (%.1f 秒, %.1f KB/s)' % (path, elapsed, len(data) / 1024.0 / elapsed))
        self.reset()
        return 'OK:' + os.path.basename(path)

    def _save(self, data):
        if not os.path.isdir(self.save_dir):
            os.makedirs(self.save_dir)
        stem, ext = os.path.splitext(self.name)
        path = os.path.join(self.save_dir, self.name)
        n = 1
        while os.path.exists(path):  # 同名ファイルは上書きせず連番を付ける
            path = os.path.join(self.save_dir, '%s_%d%s' % (stem, n, ext))
            n += 1
        with open(path, 'wb') as f:
            f.write(data)
        return path


# =====================================================================
# 以下 CoreBluetooth (ペリフェラル) 部分
# =====================================================================

# CBCharacteristicProperties / CBAttributePermissions
PROP_READ = 0x02
PROP_WRITE_NO_RESP = 0x04
PROP_WRITE = 0x08
PROP_NOTIFY = 0x10
PERM_READABLE = 0x01
PERM_WRITEABLE = 0x02
STATE_POWERED_ON = 5
ATT_SUCCESS = 0
ATT_UNLIKELY_ERROR = 0x0E

STATE_NAMES = {0: 'Unknown', 1: 'Resetting', 2: 'Unsupported',
               3: 'Unauthorized', 4: 'PoweredOff', 5: 'PoweredOn'}


class Receiver(object):
    def __init__(self):
        from objc_util import ObjCClass, ObjCInstance, create_objc_class, load_framework, ns, nsdata_to_bytes
        load_framework('CoreBluetooth')
        self.ObjCInstance = ObjCInstance
        self.ns = ns
        self.nsdata_to_bytes = nsdata_to_bytes
        self.CBUUID = ObjCClass('CBUUID')
        self.CBMutableService = ObjCClass('CBMutableService')
        self.CBMutableCharacteristic = ObjCClass('CBMutableCharacteristic')

        self.assembler = CSVAssembler()
        self.status_text = b'READY'
        self.status_char = None
        self.pending_notify = []
        self.manager = None

        recv = self  # クロージャから参照する

        def peripheralManagerDidUpdateState_(_self, _cmd, peripheral):
            recv.on_state(ObjCInstance(peripheral))

        def peripheralManager_didAddService_error_(_self, _cmd, peripheral, service, error):
            recv.on_service_added(ObjCInstance(peripheral), error)

        def peripheralManagerDidStartAdvertising_error_(_self, _cmd, peripheral, error):
            recv.on_advertising(error)

        def peripheralManager_central_didSubscribeToCharacteristic_(_self, _cmd, peripheral, central, characteristic):
            recv.log('iPhone が接続しました (通知を購読)')

        def peripheralManager_central_didUnsubscribeFromCharacteristic_(_self, _cmd, peripheral, central, characteristic):
            recv.log('iPhone が切断しました')
            if recv.assembler.active:
                recv.log('転送途中で切断されたため受信データを破棄します')
                recv.assembler.reset()

        def peripheralManager_didReceiveWriteRequests_(_self, _cmd, peripheral, requests):
            recv.on_write(ObjCInstance(peripheral), ObjCInstance(requests))

        def peripheralManager_didReceiveReadRequest_(_self, _cmd, peripheral, request):
            recv.on_read(ObjCInstance(peripheral), ObjCInstance(request))

        def peripheralManagerIsReadyToUpdateSubscribers_(_self, _cmd, peripheral):
            recv.flush_notify()

        # 型エンコーディングを明示 (プロトコル情報が取得できない環境でも動くように)
        peripheralManagerDidUpdateState_.encoding = b'v@:@'
        peripheralManager_didAddService_error_.encoding = b'v@:@@@'
        peripheralManagerDidStartAdvertising_error_.encoding = b'v@:@@'
        peripheralManager_central_didSubscribeToCharacteristic_.encoding = b'v@:@@@'
        peripheralManager_central_didUnsubscribeFromCharacteristic_.encoding = b'v@:@@@'
        peripheralManager_didReceiveWriteRequests_.encoding = b'v@:@@'
        peripheralManager_didReceiveReadRequest_.encoding = b'v@:@@'
        peripheralManagerIsReadyToUpdateSubscribers_.encoding = b'v@:@'

        Delegate = create_objc_class(
            'CSVReceiverPeripheralDelegate',
            methods=[
                peripheralManagerDidUpdateState_,
                peripheralManager_didAddService_error_,
                peripheralManagerDidStartAdvertising_error_,
                peripheralManager_central_didSubscribeToCharacteristic_,
                peripheralManager_central_didUnsubscribeFromCharacteristic_,
                peripheralManager_didReceiveWriteRequests_,
                peripheralManager_didReceiveReadRequest_,
                peripheralManagerIsReadyToUpdateSubscribers_,
            ],
            protocols=['CBPeripheralManagerDelegate'])
        self.delegate = Delegate.new()

    def log(self, msg):
        print('[%s] %s' % (time.strftime('%H:%M:%S'), msg))

    # ---- 起動 / 終了 ----
    def start(self):
        from objc_util import ObjCClass
        CBPeripheralManager = ObjCClass('CBPeripheralManager')
        # queue=None でメインキューに配送される
        self.manager = CBPeripheralManager.alloc().initWithDelegate_queue_(self.delegate, None)

    def stop(self):
        if self.manager is not None:
            self.manager.stopAdvertising()
            self.manager.removeAllServices()
            self.manager.setDelegate_(None)
            self.manager = None
            self.log('停止しました')

    # ---- デリゲート処理 ----
    def on_state(self, pm):
        state = pm.state()
        self.log('Bluetooth 状態: %s' % STATE_NAMES.get(state, state))
        if state != STATE_POWERED_ON:
            if state in (2, 3, 4):
                self.log('Bluetooth を ON にし、Pythonista に Bluetooth の使用を許可してください')
            return
        uuid = self.CBUUID.UUIDWithString_(SERVICE_UUID)
        data_char = self.CBMutableCharacteristic.alloc().initWithType_properties_value_permissions_(
            self.CBUUID.UUIDWithString_(DATA_CHAR_UUID),
            PROP_WRITE | PROP_WRITE_NO_RESP, None, PERM_WRITEABLE)
        self.status_char = self.CBMutableCharacteristic.alloc().initWithType_properties_value_permissions_(
            self.CBUUID.UUIDWithString_(STATUS_CHAR_UUID),
            PROP_READ | PROP_NOTIFY, None, PERM_READABLE)
        service = self.CBMutableService.alloc().initWithType_primary_(uuid, True)
        service.setCharacteristics_(self.ns([data_char, self.status_char]))
        pm.removeAllServices()
        pm.addService_(service)

    def on_service_added(self, pm, error):
        if error:
            self.log('サービス登録に失敗: %s' % self.ObjCInstance(error))
            return
        adv = {
            'kCBAdvDataLocalName': LOCAL_NAME,  # CBAdvertisementDataLocalNameKey
            'kCBAdvDataServiceUUIDs': [self.CBUUID.UUIDWithString_(SERVICE_UUID)],  # CBAdvertisementDataServiceUUIDsKey
        }
        pm.startAdvertising_(self.ns(adv))

    def on_advertising(self, error):
        if error:
            self.log('アドバタイズ開始に失敗: %s' % self.ObjCInstance(error))
            return
        self.log('アドバタイズ開始 (名前: %s)' % LOCAL_NAME)
        self.log('iPhone からの送信を待っています... 保存先: %s' % SAVE_DIR)

    def on_write(self, pm, requests):
        result = ATT_SUCCESS
        for i in range(requests.count()):
            req = requests.objectAtIndex_(i)
            if str(req.characteristic().UUID().UUIDString()).upper() != DATA_CHAR_UUID:
                continue
            packet = self.nsdata_to_bytes(req.value()) if req.value() else b''
            try:
                status = self.assembler.handle(packet)
            except TransferError as e:
                self.log('エラー: %s' % e)
                self.assembler.reset()
                status = 'ERR:%s' % e
                result = ATT_UNLIKELY_ERROR
            except Exception as e:  # 想定外のエラーでもデリゲートを止めない
                self.log('想定外のエラー: %r' % e)
                self.assembler.reset()
                status = 'ERR:%r' % e
                result = ATT_UNLIKELY_ERROR
            if status:
                self.notify(status)
        if requests.count() > 0:
            # 書き込み要求には 1 回だけ応答する (Write without response の場合は無視される)
            pm.respondToRequest_withResult_(requests.objectAtIndex_(0), result)

    def on_read(self, pm, request):
        value = self.status_text[request.offset():]
        request.setValue_(self.ns(value))
        pm.respondToRequest_withResult_(request, ATT_SUCCESS)

    def notify(self, text):
        self.status_text = text.encode('utf-8')[:180]
        self.pending_notify.append(self.status_text)
        self.flush_notify()

    def flush_notify(self):
        while self.pending_notify and self.manager is not None:
            ok = self.manager.updateValue_forCharacteristic_onSubscribedCentrals_(
                self.ns(self.pending_notify[0]), self.status_char, None)
            if not ok:  # 送信キューが満杯 -> IsReadyToUpdateSubscribers で再送
                return
            self.pending_notify.pop(0)


def main():
    print('=== CSV BLE 受信 (iPad) ===')
    receiver = Receiver()
    receiver.start()
    try:
        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        receiver.stop()


if __name__ == '__main__':
    main()

# -*- coding: utf-8 -*-
"""
CSV BLE 送信プログラム (iPhone / Pythonista 3)

iPhone の iCloud Drive 直下に保存されている ECGtsHDdata1.csv を、BLE で iPad
(csv_receiver_ipad.py を実行中) へ転送します。
Pythonista 標準の `cb` モジュール (BLE セントラル) を使用します。

使い方:
  1. 先に iPad で csv_receiver_ipad.py を実行しておく
  2. iPhone の Pythonista でこのスクリプトを実行する
  3. iCloud Drive 直下の ECGtsHDdata1.csv が自動で送信される
     - 直接読めない場合は「ファイル」アプリの画面が開くので、iCloud Drive の
       ECGtsHDdata1.csv を選ぶ
     - 引数で別のパスを渡すことも可能: csv_sender_iphone.py data.csv
  4. 「送信完了」と表示されれば成功 (iPad 側でも CRC を検証済み)
"""

import json
import os
import sys
import threading
import time
import zlib

# ---- プロトコル定数 (受信側と同じ値にすること) ----
SERVICE_UUID = '6E400001-B5A3-F393-E0A9-E50E24DCCA9E'
DATA_CHAR_UUID = '6E400002-B5A3-F393-E0A9-E50E24DCCA9E'    # iPhone -> iPad (Write)
STATUS_CHAR_UUID = '6E400003-B5A3-F393-E0A9-E50E24DCCA9E'  # iPad -> iPhone (Notify)
LOCAL_NAME = 'CSV-RX'

# 1 回の書き込みサイズ。iOS 同士なら通常 MTU 185 (= 182 バイト書き込み可) 以上になる。
# 'D' + 連番 4 バイトを含めて 180 バイト以内に収める。
PACKET_SIZE = 180
PAYLOAD_SIZE = PACKET_SIZE - 5

# 送信するファイル: iCloud Drive 直下の ECGtsHDdata1.csv
SEND_FILE_NAME = 'ECGtsHDdata1.csv'
ICLOUD_DRIVE_DIRS = [
    '/private/var/mobile/Library/Mobile Documents/com~apple~CloudDocs',
    '/var/mobile/Library/Mobile Documents/com~apple~CloudDocs',
]

SCAN_TIMEOUT = 60       # 受信側が見つかるまでの待ち時間 (秒)
CONNECT_TIMEOUT = 8     # 1 台の機器への接続・サービス確認を待つ時間 (秒)
HINT_WAIT = 3           # 名前が一致する機器を優先して待つ時間 (秒)
RESULT_TIMEOUT = 15     # 送信後、iPad からの結果通知を待つ時間 (秒)
# 優先して接続を試す端末名のキーワード (一致しない機器も順に確認するので必須ではない)
NAME_HINTS = (LOCAL_NAME, 'iPad')


def is_hint(name):
    name = (name or '').lower()
    return any(h.lower() in name for h in NAME_HINTS)


_use_objc_write = False


def ble_write(peripheral, characteristic, data):
    """Write with response で 1 パケット書き込む。

    Pythonista 3.4 (Python 3.10) では cb の write_characteristic_value が
    「PY_SSIZE_T_CLEAN macro must be defined」エラーになるため、
    その場合は objc_util で CoreBluetooth を直接呼び出す。
    """
    global _use_objc_write
    if not _use_objc_write:
        try:
            peripheral.write_characteristic_value(characteristic, data, True)
            return
        except SystemError as e:
            if 'PY_SSIZE_T_CLEAN' not in str(e):
                raise
            _use_objc_write = True
    _objc_write(peripheral, characteristic, data)


def _objc_ptr(obj):
    for attr in ('ptr', '_ptr', 'objc_ptr'):
        value = getattr(obj, attr, None)
        if value:
            return value
    raise RuntimeError('CoreBluetooth オブジェクトを取得できません (%s の属性: %s)'
                       % (type(obj).__name__, [a for a in dir(obj) if not a.startswith('__')]))


def _objc_write(peripheral, characteristic, data):
    from objc_util import ObjCInstance, ns
    p = ObjCInstance(_objc_ptr(peripheral))
    c = ObjCInstance(_objc_ptr(characteristic))
    p.writeValue_forCharacteristic_type_(ns(bytes(data)), c, 0)  # 0 = CBCharacteristicWriteWithResponse


def build_packets(name, data):
    """ファイル名とデータから送信パケット列を作る (BLE に依存しない部分)"""
    crc = zlib.crc32(data) & 0xFFFFFFFF
    header = None
    stem, ext = os.path.splitext(os.path.basename(name))
    # ヘッダーが 1 パケットに収まるようファイル名を必要に応じて短くする
    while True:
        header = b'H' + json.dumps({'name': stem + ext, 'size': len(data), 'crc': crc},
                                   ensure_ascii=False).encode('utf-8')
        if len(header) <= PACKET_SIZE or not stem:
            break
        stem = stem[:-1]
    packets = [header]
    for seq, pos in enumerate(range(0, len(data), PAYLOAD_SIZE)):
        packets.append(b'D' + seq.to_bytes(4, 'big') + data[pos:pos + PAYLOAD_SIZE])
    packets.append(b'E')
    return packets


class Sender(object):
    """cb モジュールのセントラルデリゲート"""

    def __init__(self, name, data):
        self.name = name
        self.data = data
        self.packets = build_packets(name, data)
        self.index = 0
        self.peripheral = None
        self.data_char = None
        self.status_char = None
        self.lock = threading.RLock()
        self.seen = set()        # 発見済みの機器
        self.tried = set()       # 接続を試したが対象サービスが無かった機器
        self.candidates = []     # 接続を試す順番待ちの機器
        self.connect_at = 0.0
        self.scan_started = time.time()
        self.done = threading.Event()
        self.result = None       # 'OK:...' / 'ERR:...'
        self.sent_all = False
        self.started_at = 0.0
        self.last_report = 0

    def log(self, msg):
        print('[%s] %s' % (time.strftime('%H:%M:%S'), msg))

    def finish(self, result):
        if not self.done.is_set():
            self.result = result
            self.done.set()

    # ---- スキャン / 接続 ----
    # 端末名に頼らず「CSV 受信サービスを持っているか」で iPad を判別する。
    # 名前が NAME_HINTS に一致する機器を優先し、一致しなければ名前のある機器へ順に接続して確かめる。
    def did_discover_peripheral(self, p):
        if not p.name:
            return
        with self.lock:
            if p.uuid in self.tried or p.uuid in self.seen:
                return
            self.seen.add(p.uuid)
            if is_hint(p.name):
                self.candidates.insert(0, p)
            else:
                self.candidates.append(p)
        self.log('  発見: %s' % p.name)

    def poll(self, now):
        """メインループから定期的に呼ぶ。候補への接続と接続タイムアウトを管理する。"""
        with self.lock:
            if self.done.is_set() or self.data_char is not None:
                return
            if self.peripheral is not None:
                if now - self.connect_at > CONNECT_TIMEOUT:
                    p = self.peripheral
                    self.log('  %s から応答がないため次の機器を試します' % p.name)
                    self.give_up(p)
                    cb.cancel_peripheral_connection(p)
                return
            if not self.candidates:
                return
            if not is_hint(self.candidates[0].name) and now - self.scan_started < HINT_WAIT:
                return  # 名前が一致する機器が見つかるのを少し待つ
            p = self.candidates.pop(0)
            self.peripheral = p
            self.connect_at = now
        self.log('接続を試します: %s' % p.name)
        cb.connect_peripheral(p)

    def is_current(self, p):
        return self.peripheral is not None and self.peripheral.uuid == p.uuid

    def give_up(self, p):
        """対象ではなかった機器を除外して次の候補へ"""
        with self.lock:
            self.tried.add(p.uuid)
            if self.is_current(p):
                self.peripheral = None
                self.data_char = self.status_char = None

    def did_connect_peripheral(self, p):
        if self.is_current(p):
            self.log('  接続しました。サービスを確認中...')
            p.discover_services()

    def did_fail_to_connect_peripheral(self, p, error):
        if self.is_current(p):
            self.log('  接続に失敗: %s' % error)
            self.give_up(p)

    def did_disconnect_peripheral(self, p, error):
        if self.done.is_set() or not self.is_current(p):
            return
        if self.sent_all:
            self.finish('ERR:結果を受け取る前に切断されました')
        elif self.data_char is not None:
            self.finish('ERR:転送中に切断されました (%s)' % error)
        else:
            self.give_up(p)

    def did_discover_services(self, p, error):
        if not self.is_current(p):
            return
        for s in (p.services or []):
            if s.uuid.upper() == SERVICE_UUID:
                p.discover_characteristics(s)
                return
        self.log('  CSV 受信サービスが無いため次の機器を試します')
        self.give_up(p)
        cb.cancel_peripheral_connection(p)

    def did_discover_characteristics(self, s, error):
        for c in s.characteristics:
            uuid = c.uuid.upper()
            if uuid == DATA_CHAR_UUID:
                self.data_char = c
            elif uuid == STATUS_CHAR_UUID:
                self.status_char = c
        if self.data_char is None or self.status_char is None:
            self.finish('ERR:必要なキャラクタリスティックが見つかりません')
            return
        cb.stop_scan()
        self.log('iPad を見つけました: %s' % self.peripheral.name)
        # 結果通知を購読してから送信を開始する
        self.peripheral.set_notify_value(self.status_char, True)
        self.log('送信開始: %s (%d bytes, %d パケット)' % (self.name, len(self.data), len(self.packets)))
        self.started_at = time.time()
        self.send_next()

    # ---- 送信 ----
    def send_next(self):
        if self.index >= len(self.packets):
            self.sent_all = True
            self.log('全データ送信済み。iPad の検証結果を待っています...')
            return
        # Write with response: 応答 (did_write_value) を待ってから次を送ることで取りこぼしを防ぐ
        try:
            ble_write(self.peripheral, self.data_char, self.packets[self.index])
        except Exception as e:
            self.finish('ERR:書き込みに失敗しました (%s)' % e)

    def did_write_value(self, c, error):
        if error:
            self.finish('ERR:書き込みに失敗しました (%s)' % error)
            return
        self.index += 1
        pct = self.index * 100 // len(self.packets)
        if pct >= self.last_report + 10:
            self.last_report = pct - pct % 10
            self.log('  送信中... %3d%%' % pct)
        self.send_next()

    def did_update_value(self, c, error):
        if c.uuid.upper() != STATUS_CHAR_UUID or not c.value:
            return
        text = c.value.decode('utf-8', 'replace')
        if text.startswith('OK') or text.startswith('ERR'):
            self.finish(text)


def default_file_candidates():
    """iCloud Drive 直下の送信ファイルの候補パス"""
    return [os.path.join(d, SEND_FILE_NAME) for d in ICLOUD_DRIVE_DIRS]


def find_default_file(candidates=None):
    """iCloud Drive 直下の SEND_FILE_NAME を探す。読めなければ None"""
    for path in (candidates if candidates is not None else default_file_candidates()):
        try:
            with open(path, 'rb'):
                return path
        except (IOError, OSError):
            continue
    return None


def choose_file():
    if len(sys.argv) > 1:
        return sys.argv[1]
    path = find_default_file()
    if path:
        print('送信ファイル: %s' % path)
        return path
    # サンドボックスの制限で直接読めない場合は「ファイル」アプリで選んでもらう
    import console
    import dialogs
    console.alert('ファイルの選択',
                  'iCloud Drive 直下の %s を直接読み込めませんでした。\n'
                  '次の画面で「iCloud Drive」→「%s」を選んでください。' % (SEND_FILE_NAME, SEND_FILE_NAME),
                  'OK', hide_cancel_button=True)
    return dialogs.pick_document(types=['public.comma-separated-values-text', 'public.text', 'public.data'])


def main():
    global cb
    import cb

    print('=== CSV BLE 送信 (iPhone) ===')
    path = choose_file()
    if not path:
        print('キャンセルしました')
        return
    with open(path, 'rb') as f:
        data = f.read()

    sender = Sender(os.path.basename(path), data)
    cb.set_central_delegate(sender)
    sender.log('iPad (%s) を探しています...' % LOCAL_NAME)
    sender.scan_started = time.time()
    cb.scan_for_peripherals()
    try:
        deadline = time.time() + SCAN_TIMEOUT
        while not sender.done.is_set():
            time.sleep(0.2)
            now = time.time()
            sender.poll(now)
            if sender.data_char is None and now > deadline:
                sender.finish('ERR:受信側が見つかりません。iPad で受信スクリプトを起動してください')
            if sender.data_char is not None:
                deadline = now + SCAN_TIMEOUT  # 接続後はスキャンのタイムアウトを延長
            if sender.sent_all:
                sender.done.wait(RESULT_TIMEOUT)
                sender.finish('ERR:iPad からの結果通知がタイムアウトしました')
    except KeyboardInterrupt:
        sender.finish('ERR:中断しました')
    finally:
        cb.reset()

    if sender.result.startswith('OK'):
        elapsed = max(time.time() - sender.started_at, 0.001)
        sender.log('送信完了: iPad に "%s" として保存されました (%.1f 秒, %.1f KB/s)' % (
            sender.result[3:], elapsed, len(data) / 1024.0 / elapsed))
    else:
        sender.log('送信失敗: %s' % sender.result[4:])


if __name__ == '__main__':
    main()

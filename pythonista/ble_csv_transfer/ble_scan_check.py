# -*- coding: utf-8 -*-
"""
BLE 接続診断 (iPhone / Pythonista 3)

iPad で csv_receiver_ipad.py を実行した状態で、iPhone でこのスクリプトを実行します。
周囲の BLE 機器を一覧表示し、名前の付いた機器に順に接続して
CSV 受信サービスを持つ機器 (= iPad) を探します。
"""

import threading
import time

import cb

SERVICE_UUID = '6E400001-B5A3-F393-E0A9-E50E24DCCA9E'
NAME_HINTS = ('CSV-RX', 'iPad')   # csv_sender_iphone.py と同じ
SCAN_SECONDS = 10
CONNECT_TIMEOUT = 6
MAX_PROBE = 15


class Delegate(object):
    def __init__(self):
        self.found = {}
        self.event = threading.Event()
        self.services = None

    def did_discover_peripheral(self, p):
        if p.uuid not in self.found:
            self.found[p.uuid] = p
            print('  発見: %s' % (p.name or '(名前なし)'))

    def did_connect_peripheral(self, p):
        p.discover_services()

    def did_fail_to_connect_peripheral(self, p, error):
        self.services = []
        self.event.set()

    def did_disconnect_peripheral(self, p, error):
        self.event.set()

    def did_discover_services(self, p, error):
        self.services = [s.uuid.upper() for s in (p.services or [])]
        self.event.set()


def main():
    d = Delegate()
    cb.set_central_delegate(d)
    print('=== BLE 接続診断 ===')
    print('[1] %d 秒間スキャンします...' % SCAN_SECONDS)
    cb.scan_for_peripherals()
    hits = []
    try:
        time.sleep(SCAN_SECONDS)
        cb.stop_scan()
        named = [p for p in d.found.values() if p.name]
        print('    見つかった機器: %d 台 (名前あり %d 台)' % (len(d.found), len(named)))
        if not d.found:
            print('    → 1 台も見つかりません。iPhone の Bluetooth と Pythonista の Bluetooth 許可を確認してください')
            return

        print('[2] 名前のある機器に接続して CSV 受信サービスを探します...')
        for p in named[:MAX_PROBE]:
            d.event.clear()
            d.services = None
            cb.connect_peripheral(p)
            d.event.wait(CONNECT_TIMEOUT)
            ok = bool(d.services) and SERVICE_UUID in d.services
            state = '★ CSV 受信サービスあり' if ok else ('応答なし' if d.services is None else 'なし')
            print('  %-24s %s' % (p.name, state))
            if ok:
                hits.append(p.name)
            cb.cancel_peripheral_connection(p)
            time.sleep(0.5)
    finally:
        cb.reset()

    print('[結果]')
    if not hits:
        print('  CSV 受信サービスを持つ機器が見つかりませんでした。')
        print('  → iPad 側で「アドバタイズ開始」と表示されているか確認してください')
        print('    (表示されていない場合は iPad 側の画面の文字を確認してください)')
    for name in hits:
        if any(h.lower() in name.lower() for h in NAME_HINTS):
            print('  iPad「%s」が見つかりました。送信スクリプトでも接続できるはずです。' % name)
        else:
            print('  iPad「%s」が見つかりましたが、名前が NAME_HINTS に一致しません。' % name)
            print("  → csv_sender_iphone.py の NAME_HINTS に '%s' を追加してください" % name)


if __name__ == '__main__':
    main()

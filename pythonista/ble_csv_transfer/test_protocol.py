# -*- coding: utf-8 -*-
"""
送受信プロトコルの動作確認 (PC 上で実行可能 / BLE は模擬)
  python3 test_protocol.py
"""

import os
import shutil
import tempfile
import types
import unittest

import csv_receiver_ipad as rx
import csv_sender_iphone as tx


class FakeChar(object):
    def __init__(self, uuid):
        self.uuid = uuid.lower()  # 大文字小文字の違いに強いことも確認
        self.value = None


class FakeService(object):
    def __init__(self, uuid, chars):
        self.uuid = uuid
        self.characteristics = chars


class FakePeripheral(object):
    """iPad 側 (CSVAssembler) につながった模擬ペリフェラル"""

    def __init__(self, name, uuid, assembler=None, corrupt_seq=None):
        self.name = name
        self.uuid = uuid
        self.assembler = assembler
        self.corrupt_seq = corrupt_seq
        self.data_char = FakeChar(tx.DATA_CHAR_UUID)
        self.status_char = FakeChar(tx.STATUS_CHAR_UUID)
        self.services = [FakeService('180A', [])]
        if assembler is not None:
            self.services.append(FakeService(tx.SERVICE_UUID, [self.data_char, self.status_char]))
        self.delegate = None
        self.writes = 0
        self.queue = []  # 実機同様、コールバックは非同期に配送する

    def discover_services(self):
        self.delegate.did_discover_services(self, None)

    def discover_characteristics(self, s):
        self.delegate.did_discover_characteristics(s, None)

    def set_notify_value(self, c, flag):
        pass

    def write_characteristic_value(self, c, packet, with_response):
        self.queue.append(lambda: self._write(c, packet, with_response))

    def _write(self, c, packet, with_response):
        assert with_response and len(packet) <= tx.PACKET_SIZE
        self.writes += 1
        if self.corrupt_seq is not None and packet[:1] == b'D' and \
                int.from_bytes(packet[1:5], 'big') == self.corrupt_seq:
            packet = packet[:5] + b'X' + packet[6:]
        try:
            status = self.assembler.handle(packet)
            error = None
        except rx.TransferError as e:
            self.assembler.reset()
            status, error = 'ERR:%s' % e, 'ATT error'
        if status:
            self.status_char.value = status.encode('utf-8')
            self.delegate.did_update_value(self.status_char, None)
        if error is None:
            self.delegate.did_write_value(c, None)
        else:
            self.delegate.did_write_value(c, error)


def make_fake_cb(peripherals):
    cb = types.SimpleNamespace()
    state = {'delegate': None}

    def set_central_delegate(d):
        state['delegate'] = d
        for p in peripherals:
            p.delegate = d

    def scan_for_peripherals():
        for p in list(peripherals):
            state['delegate'].did_discover_peripheral(p)

    def cancel(p):
        state['delegate'].did_disconnect_peripheral(p, None)

    cb.set_central_delegate = set_central_delegate
    cb.scan_for_peripherals = scan_for_peripherals
    cb.stop_scan = lambda: None
    cb.connect_peripheral = lambda p: state['delegate'].did_connect_peripheral(p)
    cb.cancel_peripheral_connection = cancel
    cb.reset = lambda: None
    return cb


class ProtocolTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp()
        self.assembler = rx.CSVAssembler(save_dir=self.tmp, log=lambda m: None)

    def tearDown(self):
        shutil.rmtree(self.tmp)

    def run_transfer(self, name, data, **kw):
        peers = [FakePeripheral('Someone iPad', 'other'),           # サービス無し -> スキップされる
                 FakePeripheral('Speaker', 'spk'),                  # 名前が一致しない
                 FakePeripheral('CSV-RX', 'target', self.assembler, **kw)]
        tx.cb = make_fake_cb(peers)
        sender = tx.Sender(name, data)
        sender.log = lambda m: None
        tx.cb.set_central_delegate(sender)
        tx.cb.scan_for_peripherals()
        target = peers[-1]
        while target.queue:
            target.queue.pop(0)()
        self.assertTrue(sender.done.is_set())
        return sender

    def test_roundtrip_japanese_csv(self):
        data = ''.join('%d,名前%d,東京都,%f\r\n' % (i, i, i * 1.5) for i in range(3000)).encode('utf-8')
        sender = self.run_transfer('売上データ.csv', data)
        self.assertEqual(sender.result, 'OK:売上データ.csv')
        with open(os.path.join(self.tmp, '売上データ.csv'), 'rb') as f:
            self.assertEqual(f.read(), data)

    def test_empty_file_and_no_overwrite(self):
        self.assertEqual(self.run_transfer('a.csv', b'').result, 'OK:a.csv')
        self.assertEqual(self.run_transfer('a.csv', b'x,y\n').result, 'OK:a_1.csv')
        self.assertEqual(sorted(os.listdir(self.tmp)), ['a.csv', 'a_1.csv'])

    def test_corruption_detected(self):
        sender = self.run_transfer('b.csv', b'1,2,3\n' * 500, corrupt_seq=3)
        self.assertTrue(sender.result.startswith('ERR:'))
        self.assertIn('CRC', sender.result)
        self.assertEqual(os.listdir(self.tmp), [])

    def test_long_filename_fits_in_one_packet(self):
        packets = tx.build_packets('長い名前' * 40 + '.csv', b'a,b\n')
        self.assertLessEqual(len(packets[0]), tx.PACKET_SIZE)
        self.assertTrue(all(len(p) <= tx.PACKET_SIZE for p in packets))

    def test_path_traversal_blocked(self):
        packets = tx.build_packets('x.csv', b'1\n')
        packets[0] = packets[0].replace(b'x.csv', b'../../evil.csv')
        for p in packets:
            status = self.assembler.handle(p)
        self.assertEqual(status, 'OK:evil.csv')
        self.assertEqual(os.listdir(self.tmp), ['evil.csv'])


class PathTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp()

    def tearDown(self):
        shutil.rmtree(self.tmp)

    def test_save_dir_prefers_downloads(self):
        downloads = os.path.join(self.tmp, 'Downloads')
        os.mkdir(downloads)
        missing = os.path.join(self.tmp, 'nothing', 'Downloads')
        self.assertEqual(rx.resolve_save_dir([missing, downloads], 'fb', override=''),
                         (downloads, True))
        self.assertEqual(os.listdir(downloads), [])  # 書き込みテスト用ファイルは残さない

    def test_save_dir_glob_and_fallback(self):
        d = os.path.join(self.tmp, 'ABC', 'File Provider Storage', 'Downloads')
        os.makedirs(d)
        pattern = os.path.join(self.tmp, '*', 'File Provider Storage', 'Downloads')
        self.assertEqual(rx.resolve_save_dir([pattern], 'fb', override=''), (d, True))
        self.assertEqual(rx.resolve_save_dir([os.path.join(self.tmp, 'x')], 'fb', override=''),
                         ('fb', False))

    def test_find_default_file(self):
        icloud = os.path.join(self.tmp, 'com~apple~CloudDocs', tx.SEND_FILE_NAME)
        self.assertIsNone(tx.find_default_file([icloud]))
        os.makedirs(os.path.dirname(icloud))
        with open(icloud, 'w') as f:
            f.write('t,ecg\n0,0.1\n')
        self.assertEqual(tx.find_default_file(['/nonexistent/x.csv', icloud]), icloud)
        self.assertTrue(tx.default_file_candidates()[0].endswith('com~apple~CloudDocs/ECGtsHDdata1.csv'))


if __name__ == '__main__':
    unittest.main()

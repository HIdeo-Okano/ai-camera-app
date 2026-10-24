# -*- coding: utf-8 -*-
"""BLE 送受信部分の動作確認（PC で実行。BLE 本体と Pythonista の ui は模擬）
  python3 test_ecg_ble.py [ECGtsHDdata1.csv のパス]
"""
import os
import shutil
import sys
import tempfile
import types
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))


class _Any(object):
    """Pythonista の ui モジュールの代わり（import と画面部品の定義だけ通す）"""

    def __init__(self, *a, **k):
        pass

    def __call__(self, *a, **k):
        return _Any()

    def __getattr__(self, name):
        return _Any()


def _install_stubs():
    ui = types.ModuleType("ui")
    ui.View = object
    ui.in_background = lambda f: f
    ui.__getattr__ = lambda name: _Any()
    sys.modules["ui"] = ui


_install_stubs()
sys.path.insert(0, HERE)
import ecg_sender_ipad as tx      # noqa: E402
import ecg_viewer_iphone as rx    # noqa: E402

CSV = sys.argv[1] if len(sys.argv) > 1 and os.path.exists(sys.argv[1]) else os.path.join(HERE, "ECGtsHDdata1.csv")


def sample_csv():
    if os.path.exists(CSV):
        with open(CSV, "rb") as f:
            return os.path.basename(CSV), f.read()
    d = tx.core.demo_ecg(20)
    return "ECG_DEMO.csv", tx.core.build_csv(d, 256, 0, 0).encode("utf-8")


class Link(object):
    """BleSender（iPad）→ BleReceiver（iPhone）を直結する模擬 BLE。full_every 回ごとに送信待ちが満杯になる。"""

    def __init__(self, sender, receiver, full_every=0, corrupt=None):
        self.s, self.r = sender, receiver
        self.full_every, self.corrupt = full_every, corrupt
        self.count = 0
        self.files = []
        receiver.on_file = lambda path, fmt: self.files.append((path, fmt))

    def notify(self, pkt):
        self.count += 1
        if self.full_every and self.count % self.full_every == 0:
            return False
        if self.corrupt is not None and pkt[:1] == b"D" and int.from_bytes(pkt[1:5], "big") == self.corrupt:
            pkt = pkt[:6] + b"#" + pkt[7:]
        reply = self.r.receive(pkt)
        if reply is not None:
            self.s.ack(reply.decode("utf-8"))
        got = self.r.take_file()
        if got:
            self.r.on_file(*got)
        return True

    def run(self, chunk=182):
        self.s.begin(chunk)
        for _ in range(100000):
            if not self.s.job:
                return
            self.s.pump(self.notify)   # 満杯になったら「空きができた」通知で再開する想定


class BleTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp()
        self.results = []
        self.s = tx.core.BleSender(lambda m: None, lambda ok, msg: self.results.append((ok, msg)))
        self.r = rx.core.BleReceiver(self.tmp, None, lambda m: None)

    def tearDown(self):
        shutil.rmtree(self.tmp)

    def transfer(self, name, body, **kw):
        chunk = kw.pop("chunk", 182)
        self.s.job = {"name": name, "data": body, "fmt": "4"}
        Link(self.s, self.r, **kw).run(chunk)
        return self.results[-1]

    def test_same_core_in_both_files(self):
        def read(fn):
            with open(os.path.join(HERE, fn), encoding="utf-8") as f:
                return f.read()

        def core_src(fn):
            src = read(fn)
            return src[src.index("# ==================== 共通処理"):src.index("# ==================== 共通処理ここまで")]
        self.assertEqual(core_src("ecg_sender_ipad.py"), core_src("ecg_viewer_iphone.py"))
        sec = read("ecg_ble_section.py")
        self.assertIn(sec, core_src("ecg_sender_ipad.py"))
        self.assertNotIn("socket", core_src("ecg_sender_ipad.py"))

    def test_real_csv_roundtrip(self):
        name, raw = sample_csv()
        src = tx.prepare(name, raw)                                 # iPad 側：読み込み・解析
        d = src["d"]
        body = tx.core.build_csv(d["samples"], d["fs"], src["elec"], src["arr"]).encode("utf-8")
        ok, saved = self.transfer(name, body, full_every=7)
        self.assertTrue(ok, saved)
        self.assertEqual(saved, name)
        with open(os.path.join(self.tmp, saved), "rb") as f:
            self.assertEqual(f.read(), body)
        got = rx.core.parse_csv(body.decode("utf-8"), "4")         # iPhone 側：表示用に読み込み
        self.assertEqual(got["samples"], d["samples"])
        self.assertEqual((got["elec"], got["arr"]), (src["elec"], src["arr"]))

    def test_small_mtu(self):
        name, raw = sample_csv()
        ok, saved = self.transfer(name, raw, chunk=100)
        self.assertTrue(ok, saved)

    def test_corruption_reported_as_ng(self):
        name, raw = sample_csv()
        ok, msg = self.transfer(name, raw, corrupt=5)
        self.assertFalse(ok)
        self.assertIn("CRC", msg)
        self.assertEqual(os.listdir(self.tmp), [])

    def test_tiny_mtu_rejected(self):
        with self.assertRaises(ValueError):
            tx.core.build_packets("ECGtsHDdata1.csv", b"x" * 100, "4", 20)

    def test_unsafe_name(self):
        pk = tx.core.build_packets("../../evil", b"1,2\n", "4", 182)
        for p in pk:
            reply = self.r.receive(p)
        self.assertEqual(reply, b"OK evil.csv")
        self.assertEqual(os.listdir(self.tmp), ["evil.csv"])


if __name__ == "__main__":
    unittest.main(argv=sys.argv[:1], verbosity=2)

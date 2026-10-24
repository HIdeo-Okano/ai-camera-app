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

    def test_main_waits_until_closed(self):
        # スクリプトが先に終わると Pythonista が変数を片付け、BLE の通知で NameError になる（実機で発生）
        for mod in (tx, rx):
            src = mod.main.__code__.co_names
            # Python を動かさずに待つ（wait_modal で待つ間に Python が動くと objc_util が同時に使われた）
            self.assertIn("wait", src, mod.__name__)
            self.assertIn("Event", src, mod.__name__)
            self.assertNotIn("wait_modal", src, mod.__name__)
            # wait_modal で止めている間に後回しにされないよう、ボタン処理は ui.in_background を使わない
            with open(mod.__file__, encoding="utf-8") as f:
                src_text = f.read()
            self.assertNotIn("ui.in_background", src_text, mod.__name__)
            # CoreBluetooth は別スレッドから触らない（objc_util はスレッドに安全ではない）
            self.assertNotIn("on_main_thread(", src_text, mod.__name__)
        with open(tx.__file__, encoding="utf-8") as f:
            self.assertNotIn("@in_thread\n    def send", f.read())

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


# ---------------------------------------------------------------- 模擬 CoreBluetooth（objc_util の代わり）
class World(object):
    """iPad（ペリフェラル）と iPhone（セントラル）の BLE イベントを順番に配送する。"""

    def __init__(self, full_every=0, hide_service=False, device_visible=False):
        self.hide_service = hide_service  # True: アドバタイズにサービス UUID が載らない（名前でしか見つからない）
        self.device_visible = device_visible  # True: アプリが電波を出していなくても iPad 本体が名前で見える
        self.timers = []                  # _later(sec, fn) で予約された処理
        self.main = 0                     # >0 のときメインスレッドで動いている（デリゲート・_main の中）
        self.q = []
        self.pm = self.cm = None
        self.remote = None
        self.full_every = full_every
        self.n_update = 0
        self.advertising = False

    def call(self, delegate, name, *args):
        self.q.append(lambda: self.on_main(delegate.methods[name], None, None, *args))

    def on_main(self, fn, *args):
        self.main += 1
        try:
            return fn(*args)
        finally:
            self.main -= 1

    def fire(self, name):
        """予約された処理のうち、名前が name のものを実行する（時間がたったことにする）。"""
        due = [f for sec, f in self.timers if f.__name__ == name]
        self.timers = [(sec, f) for sec, f in self.timers if f.__name__ != name]
        for f in due:
            self.on_main(f)
        self.run()
        return len(due)

    def run(self, limit=200000):
        for _ in range(limit):
            if not self.q:
                return
            self.q.pop(0)()
        raise AssertionError("イベントが終わりません")


W = [None]


class FUUID(object):
    def __init__(self, s):
        self.s = s.upper()

    def UUIDString(self):
        return self.s


class FObj(object):
    def __init__(self, uuid):
        self.uuid = uuid
        self.chars = []
        self.val = None

    def UUID(self):
        return self.uuid

    def setCharacteristics_(self, chars):
        self.chars = list(chars)

    def characteristics(self):
        return self.chars

    def value(self):
        return self.val


class FReqs(object):
    def __init__(self, reqs):
        self.reqs = reqs

    def count(self):
        return len(self.reqs)

    def objectAtIndex_(self, i):
        return self.reqs[i]


class FReq(object):
    def __init__(self, ch, data):
        self.ch, self.data = ch, data

    def characteristic(self):
        return self.ch

    def value(self):
        return self.data


class FCentral(object):
    def maximumUpdateValueLength(self):
        return 182


class FPM(object):
    def initWithDelegate_queue_(self, d, q):
        self.d = d
        self.svc = None
        W[0].pm = self
        W[0].call(d, "peripheralManagerDidUpdateState_", self)
        return self

    def state(self):
        return 5

    def removeAllServices(self):
        self.svc = None

    def addService_(self, svc):
        self.svc = svc
        W[0].call(self.d, "peripheralManager_didAddService_error_", self, svc, None)

    def startAdvertising_(self, adv):
        assert adv["kCBAdvDataServiceUUIDs"][0].UUIDString() == tx.core.SERVICE_UUID
        W[0].advertising = True
        W[0].call(self.d, "peripheralManagerDidStartAdvertising_error_", self, None)
        cm = W[0].cm
        if cm and cm.scanning:
            cm.discover()

    def stopAdvertising(self):
        W[0].advertising = False

    def updateValue_forCharacteristic_onSubscribedCentrals_(self, data, ch, centrals):
        w = W[0]
        w.n_update += 1
        if w.full_every and w.n_update % w.full_every == 0:
            w.call(self.d, "peripheralManagerIsReadyToUpdateSubscribers_", self)
            return False
        rch = w.remote.char(ch.uuid.s)
        d = w.remote.d

        def deliver(v=bytes(data)):  # 実機と同じく、通知ごとに value を更新してからデリゲートを呼ぶ
            rch.val = v
            w.on_main(d.methods["peripheral_didUpdateValueForCharacteristic_error_"], None, None, w.remote, rch, None)
        w.q.append(deliver)
        return True

    def respondToRequest_withResult_(self, req, res):
        pass

    def setDelegate_(self, d):
        pass


class FRemote(object):
    """iPhone から見た iPad（CBPeripheral）"""

    def __init__(self, pm):
        self.pm = pm
        self.d = None
        self.svcs = []

    def name(self):
        return "TB016"

    def setDelegate_(self, d):
        self.d = d

    def discoverServices_(self, uuids):
        if self.pm is None or self.pm.svc is None:   # iPad でアプリが動いていない
            self.svcs = []
        else:
            svc = FObj(self.pm.svc.uuid)
            svc.chars = [FObj(c.uuid) for c in self.pm.svc.chars]
            self.svcs = [svc]
        W[0].call(self.d, "peripheral_didDiscoverServices_", self, None)

    def services(self):
        return self.svcs

    def char(self, u):
        return [c for c in self.svcs[0].chars if c.uuid.s == u][0]

    def discoverCharacteristics_forService_(self, uuids, svc):
        W[0].call(self.d, "peripheral_didDiscoverCharacteristicsForService_error_", self, svc, None)

    def setNotifyValue_forCharacteristic_(self, on, ch):
        w = W[0]
        w.call(self.d, "peripheral_didUpdateNotificationStateForCharacteristic_error_", self, ch, None)
        local = [c for c in self.pm.svc.chars if c.uuid.s == ch.uuid.s][0]
        w.call(self.pm.d, "peripheralManager_central_didSubscribeToCharacteristic_", self.pm, FCentral(), local)

    def writeValue_forCharacteristic_type_(self, data, ch, kind):
        assert kind == 0
        w = W[0]
        local = [c for c in self.pm.svc.chars if c.uuid.s == ch.uuid.s][0]
        w.call(self.pm.d, "peripheralManager_didReceiveWriteRequests_", self.pm, FReqs([FReq(local, bytes(data))]))
        w.call(self.d, "peripheral_didWriteValueForCharacteristic_error_", self, ch, None)


class FAdv(object):
    def __init__(self, local_name):
        self.local_name = local_name

    def objectForKey_(self, k):
        return self.local_name if k == "kCBAdvDataLocalName" else None


class FOther(object):
    def name(self):
        return "smarlife(2)"


class FNoName(object):
    def name(self):
        return None


class FCM(object):
    def initWithDelegate_queue_(self, d, q):
        self.d = d
        self.scanning = False
        W[0].cm = self
        W[0].call(d, "centralManagerDidUpdateState_", self)
        return self

    def state(self):
        return 5

    def scanForPeripheralsWithServices_options_(self, uuids, opt):
        # 実機では (None, None) の全機器スキャンが ArgumentError になった。必ずサービス UUID で探す
        assert uuids is not None, "サービス UUID なしでスキャンしています"
        self.filtered = True
        assert uuids[0].UUIDString() == tx.core.SERVICE_UUID
        self.scanning = True
        if W[0].advertising or (not self.filtered and W[0].device_visible):
            self.discover()

    def discover(self):
        w = W[0]
        assert w is not None
        if self.filtered and (w.hide_service or not w.advertising):
            return
        if not self.filtered:  # 名前で探すときは、関係ない機器（名前のないものも）見つかる
            w.call(self.d, "centralManager_didDiscoverPeripheral_advertisementData_RSSI_", self, FOther(), FAdv(None), -60)
            w.call(self.d, "centralManager_didDiscoverPeripheral_advertisementData_RSSI_", self, FNoName(), None, -70)
        w.remote = FRemote(w.pm)
        w.call(self.d, "centralManager_didDiscoverPeripheral_advertisementData_RSSI_", self, w.remote,
               FAdv(tx.core.BLE_NAME), -50)

    def stopScan(self):
        self.scanning = False

    def connectPeripheral_options_(self, p, opt):
        W[0].call(self.d, "centralManager_didConnectPeripheral_", self, p)

    def cancelPeripheralConnection_(self, p):
        w = W[0]
        w.call(self.d, "centralManager_didDisconnectPeripheral_error_", self, p, None)
        if w.pm is not None and w.pm.svc is not None:
            dc = [c for c in w.pm.svc.chars if c.uuid.s == tx.core.DATA_UUID][0]
            w.call(w.pm.d, "peripheralManager_central_didUnsubscribeFromCharacteristic_", w.pm, FCentral(), dc)

    def setDelegate_(self, d):
        pass


class _Alloc(object):
    def __init__(self, make):
        self.make = make

    def alloc(self):
        return self.make()


class FCBUUID(object):
    @staticmethod
    def UUIDWithString_(s):
        return FUUID(s)


class FChar(object):
    def initWithType_properties_value_permissions_(self, u, props, v, perms):
        return FObj(u)


class FSvc(object):
    def initWithType_primary_(self, u, primary):
        return FObj(u)


def _install_fake_objc():
    m = types.ModuleType("objc_util")
    classes = {"CBUUID": FCBUUID, "CBPeripheralManager": _Alloc(FPM), "CBCentralManager": _Alloc(FCM),
               "CBMutableService": _Alloc(FSvc), "CBMutableCharacteristic": _Alloc(FChar)}
    m.ObjCClass = lambda n: classes[n]
    m.ObjCInstance = lambda x: x
    m.ns = lambda x: x
    m.nsdata_to_bytes = lambda x: bytes(x)
    m.load_framework = lambda n: None
    m.on_main_thread = lambda f: (lambda: W[0].on_main(f))

    def create_objc_class(name, methods=(), protocols=()):
        for f in methods:
            assert isinstance(getattr(f, "encoding", ""), str), f.__name__
            # 引数の数がエンコーディング（戻り値・self・_cmd を除く @ の数）と合っているか
            assert f.__code__.co_argcount == len(f.encoding) - 1, f.__name__
            # iPad の Pythonista ではクロージャの変数が空になり NameError になった（実機で発生）
            assert f.__closure__ is None, "%s はクロージャを使っています" % f.__name__

        class D(object):
            pass
        D.new = staticmethod(lambda: types.SimpleNamespace(methods={f.__name__: f for f in methods}))
        return D
    m.create_objc_class = create_objc_class
    sys.modules["objc_util"] = m
    for mod in (tx, rx):
        mod._CB.clear()
        # タイマーは記録だけして、テストの中で W[0].fire() で時間を進める。
        # 引数付きの呼び出しは Pythonista で失敗したので、(sec, fn) 以外は受け付けない
        mod._later = _fake_later


def _fake_later(sec, fn):
    # iPad では、ボタン用の別スレッドから予約した直後に Pythonista が落ちた（実機で発生）
    assert W[0].main > 0, "_later はメインスレッドから呼ぶこと: %s" % fn.__name__
    W[0].timers.append((sec, fn))


class CoreBluetoothFlowTest(unittest.TestCase):
    """BleSender.start() / BleReceiver.start() のデリゲート処理を、模擬 CoreBluetooth で通しで動かす。"""

    def setUp(self):
        _install_fake_objc()
        self.tmp = tempfile.mkdtemp()
        for mod in (tx, rx):
            mod.TRACE_DIR = self.tmp

    def tearDown(self):
        shutil.rmtree(self.tmp)
        sys.modules.pop("objc_util", None)

    def flow(self, iphone_first, full_every=0, hide_service=False):
        W[0] = World(full_every, hide_service)
        logs, done, files = [], [], []
        snd = tx.core.BleSender(lambda m: logs.append("iPad: " + m), lambda ok, msg: done.append((ok, msg)))
        rcv = rx.core.BleReceiver(self.tmp, lambda p, f: files.append((p, f)), lambda m: logs.append("iPhone: " + m))
        name, raw = sample_csv()
        if iphone_first:
            rcv.start()
            W[0].run()
            snd.start()
        else:
            snd.start()
            W[0].run()
            rcv.start()
        W[0].run()
        W[0].on_main(snd.send, name, raw, "4")   # ボタンの action（メインスレッド）
        W[0].run()
        self.assertTrue(any(tx.core.BLE_VERSION in m for m in logs), "\n".join(logs))   # 版が表示される
        errors = [m for m in logs if "エラー" in m]
        self.assertEqual(errors, [], "\n".join(logs))
        self.assertEqual(done, [(True, name)], "\n".join(logs))
        self.assertEqual(len(files), 1)
        with open(files[0][0], "rb") as f:
            self.assertEqual(f.read(), raw)
        self.assertFalse(W[0].advertising)                 # 送信後はアドバタイズを止める
        self.assertTrue(W[0].cm.scanning)                  # iPhone は次の送信を待って探し続ける
        self.snd, self.rcv, self.logs, self.done = snd, rcv, logs, done
        return logs

    def test_flow_iphone_started_first(self):
        self.flow(True)

    def test_flow_ipad_started_first_with_full_queue(self):
        self.flow(False, full_every=5)

    def test_trace_file(self):
        self.flow(True)
        with open(os.path.join(self.tmp, "ecg_ble_trace_sender.txt"), encoding="utf-8") as f:
            trace = f.read()
        self.assertIn(tx.core.BLE_VERSION, trace)
        for step in ("> _subscribed", "notify 1/", "> _writes", "writes: responded", "finish True",
                     "finish: on_done done"):
            self.assertIn(step, trace)

    def test_end_sequence(self):
        logs = self.flow(True)
        w = W[0]
        self.assertTrue(any("iPhone: 受信完了（1 件目）" in m for m in logs), "\n".join(logs))
        self.assertTrue(any("iPad: 送信を終了しました（成功）" in m for m in logs), "\n".join(logs))
        # 受信後の iPhone：接続を切り、サービス UUID で次の送信を待つ
        self.assertTrue(w.cm.scanning)
        self.assertIsNone(self.rcv.per)
        # 時間がたっても（タイムアウト・接続確認）何も起きない
        n = len(logs)
        w.fire("_check_timeout")
        w.fire("_check_watchdog")
        self.assertEqual(self.done, [(True, sample_csv()[0])])
        self.assertEqual([m for m in logs[n:] if "エラー" in m or "失敗" in m], [], "\n".join(logs))
        self.assertFalse(w.advertising)

    def test_send_twice(self):
        logs = self.flow(True)
        name, raw = sample_csv()
        done = []
        # 2回目の送信（同じ iPad 画面からもう一度「送信」）
        snd = [o for o in gc_objs() if isinstance(o, tx.core.BleSender)][-1]
        snd.on_done = lambda ok, msg: done.append((ok, msg))
        W[0].on_main(snd.send, name, raw, "4")   # ボタンの action（メインスレッド）
        W[0].run()
        self.assertEqual(done, [(True, name)], "\n".join(logs))


def gc_objs():
    import gc
    return gc.get_objects()


if __name__ == "__main__":
    unittest.main(argv=sys.argv[:1], verbosity=2)

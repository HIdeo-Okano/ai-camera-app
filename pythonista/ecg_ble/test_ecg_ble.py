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

_REAL_SEND = tx._send   # 模擬に差し替える前の本物（SendMarshallingTest で使う）

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
        _alive(ch, "通知する特性")
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
            _alive(self.pm.svc, "サービス")
            for c in self.pm.svc.chars:
                _alive(c, "特性")
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


def _owned(obj):
    obj.rc = 1   # alloc/init したものは参照 1 で始まる
    return obj


def _alive(obj, what):
    # 実機（r12）では、登録直後に手放したサービスを iOS が使って iPad が落ちた
    assert getattr(obj, "rc", 1) > 0, "%s が手放された後に iOS から使われました" % what


class FChar(object):
    def initWithType_properties_value_permissions_(self, u, props, v, perms):
        return _owned(FObj(u))


class FSvc(object):
    def initWithType_primary_(self, u, primary):
        return _owned(FObj(u))


def _fake_retain(p):
    if p is not None and hasattr(p, "rc"):
        p.rc += 1
    return p


def _fake_release(p):
    if p is not None and hasattr(p, "rc"):
        p.rc -= 1
        assert p.rc >= 0, "手放しすぎ"


def _fake_send(obj, sel, sig, *args):
    """_send の代わり：セレクタ名をメソッド名にして模擬オブジェクトを呼ぶ。"""
    assert sig[0] in "@BqQ*v" and all(c in "@BqQ*" for c in sig[1:]), sig
    assert sig.count("@") + sig.count("B") + sig.count("q") + sig.count("Q") + sig.count("*") - 1 == len(args) \
        or (sig[0] == "v" and len(sig) - 1 == len(args)), (sel, sig, args)
    assert sel.count(":") == len(args), (sel, args)   # セレクタの引数の数と合っているか
    return getattr(obj, sel.replace(":", "_"))(*args)


def _fake_items(a):
    if a is None:
        return []
    return list(a.reqs) if isinstance(a, FReqs) else list(a)


def _install_fake_objc():
    m = types.ModuleType("objc_util")
    classes = {"CBUUID": FCBUUID, "CBPeripheralManager": _Alloc(FPM), "CBCentralManager": _Alloc(FCM),
               "CBMutableService": _Alloc(FSvc), "CBMutableCharacteristic": _Alloc(FChar)}
    m.load_framework = lambda n: None

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
        mod._send = _fake_send
        mod._cls = classes.__getitem__
        mod._retain = _fake_retain
        mod._release = _fake_release
        mod._nsstr = lambda s: s
        mod._str = lambda p: None if p is None else str(p)
        mod._nsdata = bytes
        mod._bytes_of = lambda p: bytes(p) if p else b""
        mod._nsarray = list
        mod._nsdict = dict
        mod._items = _fake_items
        mod._err_text = str
        mod._ptr_of = lambda o: o
        # タイマーは記録だけして、テストの中で W[0].fire() で時間を進める。
        # 引数付きの呼び出しは Pythonista で失敗したので、(sec, fn) 以外は受け付けない
        mod._later = _fake_later


def _fake_later(sec, fn):
    # iPad では、ボタン用の別スレッドから予約した直後に Pythonista が落ちた（実機で発生）
    assert W[0].main > 0, "_later はメインスレッドから呼ぶこと: %s" % fn.__name__
    W[0].timers.append((sec, fn))


class SendMarshallingTest(unittest.TestCase):
    """_send が呼び出しの型ごとに専用の関数を作り、引数を正しい型・順番で渡すことを、
    objc_msgSend の代わりの ctypes コールバックで確かめる。"""

    def run_sig(self, sig, ret, *args):
        import ctypes
        seen = []
        types_ = {"@": ctypes.c_void_p, "B": ctypes.c_bool, "q": ctypes.c_longlong, "Q": ctypes.c_ulonglong,
                  "*": ctypes.c_char_p, "v": None}
        proto = ctypes.CFUNCTYPE(types_[sig[0]], ctypes.c_void_p, ctypes.c_void_p, *[types_[c] for c in sig[1:]])

        def fake_msg(obj, sel, *a):
            seen.append((obj, sel) + a)
            return ret
        cb = proto(fake_msg)
        tx._RT.clear()
        tx._RT.update(fns={}, sels={}, classes={}, msg=ctypes.cast(cb, ctypes.c_void_p).value,
                      reg_sel=lambda b: 4242, get_class=lambda b: 77)
        try:
            out = _REAL_SEND(1234, "x" + ":" * (len(sig) - 1), sig, *args)
        finally:
            tx._RT.clear()
        return out, seen[0]

    def test_signatures(self):
        out, seen = self.run_sig("@@", 99, 555)
        self.assertEqual((out, seen), (99, (1234, 4242, 555)))
        out, seen = self.run_sig("B@@@", True, 1, 2, None)
        self.assertEqual((out, seen), (True, (1234, 4242, 1, 2, None)))
        out, seen = self.run_sig("v@q", None, 7, -3)
        self.assertEqual((out, seen), (None, (1234, 4242, 7, -3)))
        out, seen = self.run_sig("@*Q", 5, b"ab", 2)
        self.assertEqual((out, seen), (5, (1234, 4242, b"ab", 2)))
        out, seen = self.run_sig("q", 5, )
        self.assertEqual((out, seen), (5, (1234, 4242)))

    def test_no_objc_util_method_calls(self):
        """BLE の部分は objc_util の ObjCInstance・ns などを使わない（使うのはデリゲートのクラス作成だけ）。"""
        with open(os.path.join(HERE, "ecg_ble_section.py"), encoding="utf-8") as f:
            code = [ln for ln in f.read().splitlines() if not ln.lstrip().startswith("#")]
        imports = [ln.strip() for ln in code if "from objc_util import" in ln]
        self.assertEqual(sorted(set(imports)), ["from objc_util import create_objc_class",
                                                "from objc_util import load_framework"])
        for word in ("ObjCInstance(", "ObjCClass(", "ns(", "nsdata_to_bytes", "on_main_thread", ".alloc()"):
            self.assertFalse([ln for ln in code if word in ln], word)


class CoreBluetoothFlowTest(unittest.TestCase):
    """BleSender.start() / BleReceiver.start() のデリゲート処理を、模擬 CoreBluetooth で通しで動かす。"""

    def setUp(self):
        _install_fake_objc()
        self.tmp = tempfile.mkdtemp()
        for mod in (tx, rx):
            mod.TRACE_DIR = self.tmp
            mod._OLD_TRACE_DIR = self.tmp

    def tearDown(self):
        shutil.rmtree(self.tmp)
        sys.modules.pop("objc_util", None)

    def flow(self, iphone_first, full_every=0, hide_service=False):
        W[0] = World(full_every, hide_service)
        logs, done, files = [], [], []
        snd = tx.core.BleSender(lambda m: logs.append("iPad: " + m), lambda ok, msg: done.append((ok, msg)),
                                threaded=False)
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
        for step in ("> _cb_subscribed", "work subscribed", "notify 1/", "> _cb_writes", "writes: responded",
                     "work ack", "finish True",
                     "finish: on_done done"):
            self.assertIn(step, trace)

    def test_previous_trace_shown_on_start(self):
        with open(os.path.join(self.tmp, "ecg_ble_trace_sender.txt"), "w", encoding="utf-8") as f:
            f.write("ECG BLE 動作記録\n12:00:00 objc startAdvertising:\n12:00:01 objc updateValue:forCharacteristic:onSubscribedCentrals:\n")
        with open(os.path.join(self.tmp, "ecg_ble_fault_sender.txt"), "w", encoding="utf-8") as f:
            f.write("Fatal Python error: Segmentation fault\n\nThread 0x01 (most recent call first):\n"
                    "  File \"ecg_sender_ipad.py\", line 9 in _notify\n")
        logs = []
        tx.core.show_previous_trace("sender", logs.append)
        self.assertTrue(any("前回 12:00:01 objc updateValue" in m for m in logs), "\n".join(logs))
        # 前回 iOS の中で落ちた時の Python の位置（faulthandler の記録）も出す
        self.assertTrue(any("前回落ちた時" in m for m in logs), "\n".join(logs))
        self.assertTrue(any("line 9 in _notify" in m for m in logs), "\n".join(logs))
        # 新しく記録を始めると、前回の記録は _prev.txt に残り、落ちた位置の記録は空にする
        W[0] = World()
        tx.core.BleSender(None, None, threaded=False).start()
        self.assertTrue(os.path.exists(os.path.join(self.tmp, "ecg_ble_trace_sender_prev.txt")))
        logs = []
        tx.core.show_previous_trace("sender", logs.append)
        self.assertFalse(any("前回落ちた時" in m for m in logs), "\n".join(logs))

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
        self.snd._deadline = 0
        self.snd._post("tick")
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


    def test_timeout_when_no_iphone(self):
        W[0] = World()
        logs, done = [], []
        snd = tx.core.BleSender(logs.append, lambda ok, msg: done.append((ok, msg)), threaded=False)
        snd.start()
        W[0].run()
        W[0].on_main(snd.send, "a.csv", b"1,2\n", "4")
        W[0].run()
        self.assertTrue(W[0].advertising)
        snd._post("tick")                 # まだ時間内
        self.assertEqual(done, [])
        snd._deadline = 0                 # 時間切れにする
        snd._post("tick")
        self.assertEqual(len(done), 1)
        self.assertFalse(done[0][0])
        self.assertIn("見つかりませんでした", done[0][1])
        self.assertFalse(W[0].advertising)
        self.assertIsNone(snd.job)

    def test_flow_with_worker_thread(self):
        """iPad の送信は、CoreBluetooth の操作を作業スレッドで行う（ble_diag_ipad.py で、iPhone が
        接続しても落ちなかった形）。iOS からの通知（メインスレッド）は値を渡すだけ。"""
        import threading
        import time
        W[0] = World()
        seen = []
        real = tx._send

        def rec(obj, sel, sig, *args):
            seen.append((threading.current_thread().name, sel))
            return real(obj, sel, sig, *args)
        tx._send = rec
        logs, done, files = [], [], []
        name, raw = sample_csv()

        def until(cond, sec=20):
            end = time.time() + sec
            while time.time() < end:
                W[0].run()
                if cond():
                    return
                time.sleep(0.005)
            self.fail("\n".join(logs))
        try:
            snd = tx.core.BleSender(lambda m: logs.append("iPad: " + m), lambda ok, msg: done.append((ok, msg)))
            rcv = rx.core.BleReceiver(self.tmp, lambda p, f: files.append((p, f)),
                                      lambda m: logs.append("iPhone: " + m))
            rcv.start()
            W[0].run()
            snd.start()
            until(lambda: snd.ready)
            W[0].on_main(snd.send, name, raw, "4")   # 画面の「送信」（メインスレッド）
            until(lambda: done)
            snd.stop()
        finally:
            tx._send = real
        self.assertEqual(done, [(True, name)], "\n".join(logs))
        with open(files[0][0], "rb") as f:
            self.assertEqual(f.read(), raw)
        self.assertIsNone(snd._thread)              # 閉じると作業スレッドも終わる
        self.assertIsNone(snd.pm)
        work = {"initWithDelegate:queue:", "addService:", "removeAllServices", "startAdvertising:",
                "stopAdvertising", "updateValue:forCharacteristic:onSubscribedCentrals:", "setDelegate:"}
        self.assertTrue(work <= {sel for th, sel in seen}, seen)
        self.assertEqual([(th, sel) for th, sel in seen if sel in work and th != "ECGBleSender"], [])

    def test_sender_ui_does_not_use_ui_delay(self):
        """iPad の画面は ui.delay を使わず、update() でほかのスレッドからの記録・結果を画面に出す。"""
        with open(tx.__file__, encoding="utf-8") as f:
            src = f.read()
        ui_part = src[src.index("# ==================== 共通処理ここまで"):]
        self.assertNotIn("ui.delay(", ui_part)
        self.assertIn("def update(self):", ui_part)
        self.assertIn("update_interval", ui_part)


def gc_objs():
    import gc
    return gc.get_objects()


if __name__ == "__main__":
    unittest.main(argv=sys.argv[:1], verbosity=2)

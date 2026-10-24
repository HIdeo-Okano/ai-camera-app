# ===== iPad → iPhone 送受信（BLE） =====
#   iPad（送信）  ＝ BLE ペリフェラル。「送信」を押すとアドバタイズし、接続してきた iPhone へ送る。
#   iPhone（受信）＝ BLE セントラル。サービス UUID で iPad を探して自動で接続する（IP・端末名の設定は不要）。
#   データ：DATA キャラクタリスティックの Notify（1通知＝1パケット）
#     b"H" + JSON（name, size, crc, fmt）／ b"D" + 連番4バイト + データ ／ b"E"（終了）
#   応答：iPhone が ACK キャラクタリスティックへ "OK <保存したファイル名>" または "NG <理由>" を書き込む
#   終わり方：
#     iPhone … 応答を書き込んだら切断し、サービス UUID で次の送信を待つ
#     iPad   … 応答を受け取ったらアドバタイズを止め、BLE の準備は残したまま次の「送信」を待つ
#   ※ Pythonista の cb モジュールは使わない（Pythonista 3.4 では書き込みが PY_SSIZE_T_CLEAN エラーになる）。
#     両側とも objc_util で CoreBluetooth を直接使う。
import zlib

SERVICE_UUID = "8F1D0001-2C5A-4E7B-9A63-5E0C6A1B2D01"
DATA_UUID = "8F1D0002-2C5A-4E7B-9A63-5E0C6A1B2D01"     # iPad → iPhone（Notify）
ACK_UUID = "8F1D0003-2C5A-4E7B-9A63-5E0C6A1B2D01"      # iPhone → iPad（Write）
BLE_NAME = "ECG-TX"
BLE_VERSION = "2026-10-25 r10"     # 起動時に表示（どのファイルが動いているか確かめるため）
# 受信側（iPhone）はサービス UUID だけで iPad を探す（iPad の端末名は使わない）。
# 名前で全機器を探す処理（サービスを指定しないスキャン）は、Pythonista で
# ArgumentError になり、iPad 本体に何度もつなぎ直す原因にもなったため使わない。
MAX_SIZE = 20 * 1024 * 1024
SEND_TIMEOUT = 120       # 送信側：iPhone が接続してくるまで・確認が届くまでの待ち時間（秒）
CONNECT_TIMEOUT = 15     # 受信側：接続してから受信が始まるまでの待ち時間（秒）
ACK_MAX = 180


def safe_name(name):
    base = os.path.basename(name or "").strip() or "received.csv"
    base = "".join(c for c in base if c not in '\\/:*?"<>|\0')
    if not base.lower().endswith((".csv", ".txt")):
        base += ".csv"
    return base


def build_packets(name, data, fmt="4", chunk=182):
    """送るパケットの列を作る。chunk は1通知の最大バイト数（iPhone 側の maximumUpdateValueLength）。"""
    crc = zlib.crc32(data) & 0xFFFFFFFF
    stem, ext = os.path.splitext(os.path.basename(name or "data.csv"))
    while True:  # ヘッダが1通知に収まるようファイル名を必要なら短くする
        head = b"H" + json.dumps({"name": stem + ext, "size": len(data), "crc": crc, "fmt": fmt},
                                 ensure_ascii=False, separators=(",", ":")).encode("utf-8")
        if len(head) <= chunk or not stem:
            break
        stem = stem[:-1]
    if len(head) > chunk or chunk < 16:
        raise ValueError("BLE の1回の送信サイズが小さすぎます（%d バイト）" % chunk)
    pay = chunk - 5
    pk = [head]
    for i, p in enumerate(range(0, len(data), pay)):
        pk.append(b"D" + i.to_bytes(4, "big") + data[p:p + pay])
    pk.append(b"E")
    return pk


class Assembler(object):
    """受信したパケットを元のファイルに組み立てる。"""

    def __init__(self):
        self.reset()

    def reset(self):
        self.meta = None
        self.buf = bytearray()
        self.seq = 0

    @property
    def active(self):
        return self.meta is not None

    def feed(self, pkt):
        """1パケット処理する。ファイルがそろったら (name, data, fmt) を返す。"""
        kind, body = pkt[:1], pkt[1:]
        if kind == b"H":
            self.reset()
            m = json.loads(body.decode("utf-8"))
            size = int(m.get("size", -1))
            if not 0 < size <= MAX_SIZE:
                raise ValueError("サイズが不正です（%s）" % size)
            self.meta = m
            return None
        if self.meta is None:
            raise ValueError("ヘッダを受け取る前にデータが届きました")
        if kind == b"D":
            if len(body) < 4 or int.from_bytes(body[:4], "big") != self.seq:
                raise ValueError("データの順番が不正です（%d 番目）" % self.seq)
            self.seq += 1
            self.buf += body[4:]
            if len(self.buf) > int(self.meta["size"]):
                raise ValueError("宣言より多いデータを受け取りました")
            return None
        if kind == b"E":
            m, data = self.meta, bytes(self.buf)
            self.reset()
            if len(data) != int(m["size"]):
                raise ValueError("サイズが一致しません（%d/%d バイト）" % (len(data), int(m["size"])))
            if zlib.crc32(data) & 0xFFFFFFFF != int(m.get("crc", -1)):
                raise ValueError("CRC が一致しません（データ破損）")
            return m.get("name"), data, str(m.get("fmt", "auto"))
        raise ValueError("不明なパケットです")


# ----- CoreBluetooth（objc_util）
_CB = {}
_CLS_NO = [0]


def _cb(name):
    if not _CB:
        from objc_util import ObjCClass, load_framework
        load_framework("CoreBluetooth")
        for n in ("CBUUID", "CBPeripheralManager", "CBCentralManager", "CBMutableService",
                  "CBMutableCharacteristic"):
            _CB[n] = ObjCClass(n)
    return _CB[name]


def _uuid(s):
    return _cb("CBUUID").UUIDWithString_(s)


def _uuid_of(obj):
    return str(obj.UUID().UUIDString()).upper()


def _delegate(base, methods, protocols):
    """デリゲートの ObjC クラスを作る（再実行しても名前が重ならないよう番号を付ける）。"""
    from objc_util import create_objc_class
    _CLS_NO[0] += 1
    name = "%s_%d_%d" % (base, int(time.time()), _CLS_NO[0])
    return create_objc_class(name, methods=methods, protocols=protocols).new()


# ----- 動作記録（落ちた場所を調べるため）
#   1つ処理するごとに Documents/ecg_ble_trace_<sender|receiver>.txt へ書いてすぐ保存する。
#   Pythonista が落ちてもファイルは残るので、最後の行が「落ちる直前の処理」になる。
TRACE_DIR = os.path.expanduser("~/Documents")
_TRACE = {"path": None}


def _trace_start(role):
    _TRACE["path"] = os.path.join(TRACE_DIR, "ecg_ble_trace_%s.txt" % role)
    try:
        with open(_TRACE["path"], "w", encoding="utf-8") as f:
            f.write("ECG BLE 動作記録（%s・版 %s）\n" % (role, BLE_VERSION))
    except Exception:
        _TRACE["path"] = None


def _trace(msg):
    path = _TRACE["path"]
    if not path:
        return
    try:
        with open(path, "a", encoding="utf-8") as f:
            f.write("%s %s\n" % (time.strftime("%H:%M:%S"), msg))
            f.flush()
            os.fsync(f.fileno())
    except Exception:
        pass


def _main(fn):
    """CoreBluetooth の操作はメインスレッドで行う。fn は引数なしで呼ぶ。"""
    from objc_util import on_main_thread
    _trace("main> %s" % fn.__name__)
    on_main_thread(fn)()
    _trace("main< %s" % fn.__name__)


def _later(sec, fn):
    """sec 秒後にメインスレッドで fn() を呼ぶ。必ずメインスレッド（デリゲートの中や _main 経由）から呼ぶこと。
    ボタン用の別スレッドから予約すると、そのスレッドが終わった後に iPad の Pythonista が落ちた。
    また引数を付けて on_main_thread に渡すと ArgumentError やクラッシュになったため、
    引数は渡さず、必要な値はオブジェクトに持たせておく。"""
    import ui
    _trace("later %s %ss" % (fn.__name__, sec))
    ui.delay(fn, sec)


_BT_STATE = {0: "不明", 1: "リセット中", 2: "非対応", 3: "許可されていません", 4: "オフ", 5: "オン"}


def _bt_hint(st):
    if st == 3:
        return "設定 → Pythonista → Bluetooth を ON にしてください"
    if st == 4:
        return "Bluetooth を ON にしてください（設定アプリ → Bluetooth）"
    return ""


# ----- CoreBluetooth から呼ばれる関数（デリゲート）
#   入れ子の関数（クロージャ）にすると、iPad の Pythonista で呼ばれた時に変数が空になり
#   NameError（free variable ... referenced before assignment）になったため、
#   モジュール直下の関数にして、動作中の送信側／受信側を _ACTIVE から探して呼ぶ。
_ACTIVE = {}


def _O(p):
    from objc_util import ObjCInstance
    return ObjCInstance(p) if p else None


def _call(h, name, ptrs):
    getattr(h, name)(*[_O(p) for p in ptrs])


def _dispatch(role, name, *ptrs):
    h = _ACTIVE.get(role)
    _trace("> %s" % name)
    if h is not None:
        h._try(_call, h, name, ptrs)
    _trace("< %s" % name)


# 送信側（iPad：ペリフェラル）
def peripheralManagerDidUpdateState_(_self, _cmd, pm):
    _dispatch("sender", "_state", pm)


def peripheralManager_didAddService_error_(_self, _cmd, pm, svc, err):
    _dispatch("sender", "_added", err)


def peripheralManagerDidStartAdvertising_error_(_self, _cmd, pm, err):
    _dispatch("sender", "_advertising", err)


def peripheralManager_central_didSubscribeToCharacteristic_(_self, _cmd, pm, central, ch):
    _dispatch("sender", "_subscribed", central, ch)


def peripheralManager_central_didUnsubscribeFromCharacteristic_(_self, _cmd, pm, central, ch):
    _dispatch("sender", "_unsubscribed")


def peripheralManagerIsReadyToUpdateSubscribers_(_self, _cmd, pm):
    _dispatch("sender", "_ready")


def peripheralManager_didReceiveWriteRequests_(_self, _cmd, pm, reqs):
    _dispatch("sender", "_writes", reqs)


# 受信側（iPhone：セントラル）
def centralManagerDidUpdateState_(_self, _cmd, cm):
    _dispatch("receiver", "_state", cm)


def centralManager_didDiscoverPeripheral_advertisementData_RSSI_(_self, _cmd, cm, p, adv, rssi):
    _dispatch("receiver", "_found", p, adv)


def centralManager_didConnectPeripheral_(_self, _cmd, cm, p):
    _dispatch("receiver", "_connected", p)


def centralManager_didFailToConnectPeripheral_error_(_self, _cmd, cm, p, err):
    _dispatch("receiver", "_failed")


def centralManager_didDisconnectPeripheral_error_(_self, _cmd, cm, p, err):
    _dispatch("receiver", "_disconnected")


def peripheral_didDiscoverServices_(_self, _cmd, p, err):
    _dispatch("receiver", "_services", p)


def peripheral_didDiscoverCharacteristicsForService_error_(_self, _cmd, p, svc, err):
    _dispatch("receiver", "_chars", p, svc)


def peripheral_didUpdateValueForCharacteristic_error_(_self, _cmd, p, ch, err):
    _dispatch("receiver", "_value", ch)


def peripheral_didWriteValueForCharacteristic_error_(_self, _cmd, p, ch, err):
    _dispatch("receiver", "_written")


def peripheral_didUpdateNotificationStateForCharacteristic_error_(_self, _cmd, p, ch, err):
    _dispatch("receiver", "_subscribed", err)


_SENDER_METHODS = [
    peripheralManagerDidUpdateState_, peripheralManager_didAddService_error_,
    peripheralManagerDidStartAdvertising_error_, peripheralManager_central_didSubscribeToCharacteristic_,
    peripheralManager_central_didUnsubscribeFromCharacteristic_, peripheralManagerIsReadyToUpdateSubscribers_,
    peripheralManager_didReceiveWriteRequests_]
_RECEIVER_METHODS = [
    centralManagerDidUpdateState_, centralManager_didDiscoverPeripheral_advertisementData_RSSI_,
    centralManager_didConnectPeripheral_, centralManager_didFailToConnectPeripheral_error_,
    centralManager_didDisconnectPeripheral_error_, peripheral_didDiscoverServices_,
    peripheral_didDiscoverCharacteristicsForService_error_, peripheral_didUpdateValueForCharacteristic_error_,
    peripheral_didWriteValueForCharacteristic_error_, peripheral_didUpdateNotificationStateForCharacteristic_error_]
for _f in _SENDER_METHODS + _RECEIVER_METHODS:
    _f.encoding = "v@:" + "@" * (_f.__code__.co_argcount - 2)   # 戻り値なし・self・_cmd・引数（すべてオブジェクト）


class _BleBase(object):
    def _set_log(self, on_log):
        self._ui_log = on_log

    def on_log(self, m):
        _trace("log: %s" % m)
        print("[BLE %s] %s" % (time.strftime("%H:%M:%S"), m))  # 画面を閉じた後もコンソールで確認できる
        if self._ui_log:
            self._ui_log(m)

    def _try(self, fn, *args):
        """デリゲート内のエラーを画面とコンソールに出す（そのままだと見えずに止まるため）。"""
        try:
            fn(*args)
        except Exception as e:
            import traceback
            traceback.print_exc()
            _trace("error: " + traceback.format_exc().replace("\n", " | "))
            self.on_log("BLE エラー: %s: %s" % (type(e).__name__, e))


class BleSender(_BleBase):
    """送信側（iPad）。send() でアドバタイズし、接続してきた iPhone へ Notify で送る。

    on_done(ok, msg) … 送信結果。ok=True なら msg は iPhone 側の保存名、False なら失敗の理由。
    """

    def __init__(self, on_log=None, on_done=None):
        self._set_log(on_log)
        self.on_done = on_done or (lambda ok, msg: None)
        self.pm = None
        self.ready = False
        self.data_char = None
        self.job = None
        self._timeout_job = None
        self._central = None     # 接続して通知を待っている iPhone
        self._pct = 0

    # --- BLE に依存しない部分
    def begin(self, chunk):
        j = self.job
        j["packets"] = build_packets(j["name"], j["data"], j["fmt"], chunk)
        j["i"] = 0
        self._pct = 0
        self.on_log("iPhone が接続しました。送信します（%d バイト / %d パケット）" % (len(j["data"]), len(j["packets"])))

    def pump(self, notify):
        """notify(bytes) が False（送信待ちがいっぱい）を返すまで送り続ける。"""
        j = self.job
        if not j or "packets" not in j:
            return
        pk = j["packets"]
        while j["i"] < len(pk):
            _trace("notify %d/%d" % (j["i"] + 1, len(pk)))
            if not notify(pk[j["i"]]):
                _trace("notify full")
                return  # 空きができたら peripheralManagerIsReadyToUpdateSubscribers から再開
            j["i"] += 1
            pct = j["i"] * 100 // len(pk)
            if pct >= self._pct + 20:
                self._pct = pct - pct % 20
                self.on_log("  送信中… %d%%" % pct)
        if not j.get("sent"):
            j["sent"] = True
            self.on_log("全データを送りました。iPhone の確認を待っています…")

    def ack(self, text):
        if self.job:
            self.finish(text.startswith("OK"), text[3:].strip() if text[:2] in ("OK", "NG") else text)

    def finish(self, ok, msg):
        """送信を終える。アドバタイズを止め、BLE の準備（サービス）は残して次の「送信」に備える。"""
        _trace("finish %s" % ok)
        self.job = None
        self._timeout_job = None
        if self.pm is not None:
            self.pm.stopAdvertising()
            _trace("finish: stopAdvertising done")
        self.on_log("送信を終了しました（%s）。次の「送信」を待っています" % ("成功" if ok else "失敗"))
        self.on_done(ok, msg)
        _trace("finish: on_done done")

    # --- BLE
    def start(self):
        from objc_util import nsdata_to_bytes
        self._bytes = nsdata_to_bytes
        _trace_start("sender")
        self.on_log("BLE 送信を開始します（版 %s）" % BLE_VERSION)
        _ACTIVE["sender"] = self
        self.delegate = _delegate("ECGBleSender", _SENDER_METHODS, ["CBPeripheralManagerDelegate"])
        self.pm = _cb("CBPeripheralManager").alloc().initWithDelegate_queue_(self.delegate, None)

    def stop(self):
        self.job = None
        if self.pm is not None:
            self.pm.stopAdvertising()
            self.pm.removeAllServices()
            self.pm.setDelegate_(None)
            self.pm = None
        if _ACTIVE.get("sender") is self:
            del _ACTIVE["sender"]

    def send(self, name, data, fmt="4", timeout=SEND_TIMEOUT):
        if self.job:
            raise RuntimeError("送信中です。終わるまでお待ちください")
        if not 0 < len(data) <= MAX_SIZE:
            raise ValueError("サイズが不正です（%d バイト）" % len(data))
        self.job = {"name": name, "data": data, "fmt": fmt, "timeout": timeout}
        _main(self._go)   # ここ（ボタン用の別スレッド）ではタイマーを予約しない

    def _ready(self):
        self.pump(self._notify)

    def _go(self):
        # メインスレッドで動いているので、ここでタイムアウトを予約する
        self._timeout_job = self.job
        _later(self.job["timeout"], self._check_timeout)
        if not self.ready:
            self.on_log("Bluetooth の準備を待っています…")
        elif self._central is not None:   # iPhone がもう接続して待っている
            self._start_job(self._central)
        else:
            self._advertise()

    def _start_job(self, central):
        if not self.job or "packets" in self.job:
            return
        try:
            self.begin(int(central.maximumUpdateValueLength()))
        except Exception as e:
            self.finish(False, str(e))
            return
        self.pump(self._notify)

    def _check_timeout(self):
        job = self._timeout_job
        if job is not None and self.job is job:
            sec = job["timeout"]
            if "packets" in job:
                self.finish(False, "iPhone から受信の確認が届きませんでした（%d 秒）" % sec)
            else:
                self.finish(False, "iPhone が見つかりませんでした（%d 秒）。iPhone で ecg_viewer_iphone.py が"
                                   "起動しているか確認してください" % sec)

    def _state(self, pm):
        st = pm.state()
        self.on_log("Bluetooth: %s" % _BT_STATE.get(st, st))
        if st != 5:
            self.ready = False
            if _bt_hint(st):
                self.on_log(_bt_hint(st))
            return
        self.data_char = _cb("CBMutableCharacteristic").alloc().initWithType_properties_value_permissions_(
            _uuid(DATA_UUID), 0x10, None, 0x01)            # Notify / Readable
        ack_char = _cb("CBMutableCharacteristic").alloc().initWithType_properties_value_permissions_(
            _uuid(ACK_UUID), 0x08, None, 0x02)             # Write / Writeable
        from objc_util import ns
        svc = _cb("CBMutableService").alloc().initWithType_primary_(_uuid(SERVICE_UUID), True)
        svc.setCharacteristics_(ns([self.data_char, ack_char]))
        pm.removeAllServices()
        pm.addService_(svc)

    def _added(self, err):
        if err:
            self.on_log("BLE サービスの登録に失敗: %s" % err)
            return
        self.ready = True
        self.on_log("BLE の準備ができました")
        if self.job:
            self._advertise()

    def _advertise(self):
        from objc_util import ns
        self.pm.stopAdvertising()
        self.pm.startAdvertising_(ns({"kCBAdvDataLocalName": BLE_NAME,
                                      "kCBAdvDataServiceUUIDs": [_uuid(SERVICE_UUID)]}))

    def _advertising(self, err):
        if err:
            if self.job:
                self.finish(False, "アドバタイズを開始できません: %s" % err)
            return
        self.on_log("iPhone の接続を待っています…（BLE）")

    def _subscribed(self, central, ch):
        if _uuid_of(ch) != DATA_UUID:
            return
        self._central = central
        if self.job:
            self._start_job(central)
        else:
            self.on_log("iPhone が接続しました。「送信」を押すと送ります")

    def _notify(self, pkt):
        from objc_util import ns
        return bool(self.pm.updateValue_forCharacteristic_onSubscribedCentrals_(ns(pkt), self.data_char, None))

    def _unsubscribed(self):
        self._central = None
        if self.job and "packets" in self.job:
            self.finish(False, "送信の途中で iPhone との接続が切れました")

    def _writes(self, reqs):
        texts = []
        n = reqs.count()
        _trace("writes: %d" % n)
        for i in range(n):
            r = reqs.objectAtIndex_(i)
            if _uuid_of(r.characteristic()) == ACK_UUID and r.value():
                texts.append(self._bytes(r.value()).decode("utf-8", "replace"))
        _trace("writes: read %r" % texts)
        if n:
            self.pm.respondToRequest_withResult_(reqs.objectAtIndex_(0), 0)
            _trace("writes: responded")
        for t in texts:
            self.ack(t)


class BleReceiver(_BleBase):
    """受信側（iPhone）。iPad をサービス UUID で探して接続し、受け取った CSV を save_dir に保存して
    on_file(path, fmt) を呼ぶ。受信が終わると切断して、次の送信を待つ。"""

    def __init__(self, save_dir, on_file, on_log=None):
        self.save_dir = save_dir
        self.on_file = on_file
        self._set_log(on_log)
        self.asm = Assembler()
        self.cm = None
        self.per = None
        self.data_char = self.ack_char = None
        self.running = False
        self._done = None
        self._pct = 0
        self._watch = None
        self.received = 0        # 受信できたファイルの数
        self._just_done = False  # 受信を終えて切断するところ

    # --- BLE に依存しない部分
    def receive(self, pkt):
        """1パケット処理する。iPad へ返す応答（bytes）を返す。受信途中なら None。"""
        try:
            got = self.asm.feed(pkt)
        except Exception as e:
            self.asm.reset()
            self.on_log("受信エラー: %s" % e)
            return ("NG %s" % e).encode("utf-8")[:ACK_MAX]
        if pkt[:1] == b"H":
            self._pct = 0
            self.on_log("受信開始: %s（%d バイト）" % (self.asm.meta.get("name"), int(self.asm.meta["size"])))
        elif got is None and self.asm.active:
            pct = len(self.asm.buf) * 100 // int(self.asm.meta["size"])
            if pct >= self._pct + 20:
                self._pct = pct - pct % 20
                self.on_log("  受信中… %d%%" % pct)
        if got is None:
            return None
        name, data, fmt = got
        try:
            if not os.path.isdir(self.save_dir):
                os.makedirs(self.save_dir)
            path = os.path.join(self.save_dir, safe_name(name))
            with open(path, "wb") as f:
                f.write(data)
        except Exception as e:
            self.on_log("保存に失敗: %s" % e)
            return ("NG 保存に失敗 %s" % e).encode("utf-8")[:ACK_MAX]
        self.on_log("受信 %s（%d バイト、BLE）" % (os.path.basename(path), len(data)))
        self.received += 1
        self._just_done = True
        self._done = (path, fmt)
        return ("OK %s" % os.path.basename(path)).encode("utf-8")[:ACK_MAX]

    def take_file(self):
        d, self._done = self._done, None
        return d

    # --- BLE
    def start(self):
        from objc_util import nsdata_to_bytes
        self._bytes = nsdata_to_bytes
        _trace_start("receiver")
        self.on_log("BLE 受信を開始します（版 %s）" % BLE_VERSION)
        _ACTIVE["receiver"] = self
        self.delegate = _delegate("ECGBleReceiver", _RECEIVER_METHODS,
                                  ["CBCentralManagerDelegate", "CBPeripheralDelegate"])
        self.running = True
        self.cm = _cb("CBCentralManager").alloc().initWithDelegate_queue_(self.delegate, None)

    def stop(self):
        self.running = False
        if self.cm is not None:
            self.cm.stopScan()
            if self.per is not None:
                self.cm.cancelPeripheralConnection_(self.per)
            self.cm.setDelegate_(None)
            self.cm = None
        self.per = None
        if _ACTIVE.get("receiver") is self:
            del _ACTIVE["receiver"]

    def _scan(self):
        from objc_util import ns
        if self.running and self.cm is not None and self.per is None:
            self.cm.scanForPeripheralsWithServices_options_(ns([_uuid(SERVICE_UUID)]), None)

    def _state(self, cm):
        st = cm.state()
        if st == 5:
            self.on_log("BLE 受信待機中（iPad の「送信」を待っています）")
            self._scan()
        else:
            self.on_log("Bluetooth: %s　%s" % (_BT_STATE.get(st, st), _bt_hint(st)))

    def _found(self, p, adv=None):
        if self.per is not None:
            return
        self.cm.stopScan()
        self.per = p
        p.setDelegate_(self.delegate)
        self.on_log("iPad を見つけました（%s）。接続しています…" % (p.name() or BLE_NAME))
        self.cm.connectPeripheral_options_(p, None)
        self._watch = p
        _later(CONNECT_TIMEOUT, self._check_watchdog)

    def _check_watchdog(self):
        p, self._watch = self._watch, None
        if p is not None and self.per is p and self.data_char is None and not self.asm.active and self.cm is not None:
            self.on_log("iPad から応答がないため、探し直します")
            self.cm.cancelPeripheralConnection_(p)

    def _connected(self, p):
        from objc_util import ns
        p.discoverServices_(ns([_uuid(SERVICE_UUID)]))

    def _services(self, p):
        for svc in (p.services() or []):
            if _uuid_of(svc) == SERVICE_UUID:
                p.discoverCharacteristics_forService_(None, svc)
                return
        self.on_log("%s は送信の準備ができていません（iPad で ecg_sender_ipad.py を起動してください）"
                    % (p.name() or "機器"))
        self.cm.cancelPeripheralConnection_(p)

    def _chars(self, p, svc):
        for ch in (svc.characteristics() or []):
            u = _uuid_of(ch)
            if u == DATA_UUID:
                self.data_char = ch
            elif u == ACK_UUID:
                self.ack_char = ch
        if self.data_char is not None and self.ack_char is not None:
            p.setNotifyValue_forCharacteristic_(True, self.data_char)
        else:
            self.on_log("iPad の BLE サービスが不完全です")
            self.cm.cancelPeripheralConnection_(p)

    def _subscribed(self, err):
        if err:
            self.on_log("iPad の通知を受け取れません: %s" % err)
            if self.per is not None:
                self.cm.cancelPeripheralConnection_(self.per)
        else:
            self.on_log("iPad に接続しました。受信を待っています…")

    def _value(self, ch):
        if _uuid_of(ch) != DATA_UUID or ch.value() is None:
            return
        reply = self.receive(self._bytes(ch.value()))
        if reply is not None and self.per is not None and self.ack_char is not None:
            from objc_util import ns
            self.per.writeValue_forCharacteristic_type_(ns(reply), self.ack_char, 0)  # 0＝応答あり
        got = self.take_file()
        if got:
            self.on_file(*got)

    def _written(self):
        if self.per is not None and not self.asm.active:
            self.cm.cancelPeripheralConnection_(self.per)  # 1ファイル受け取ったら切断して次を待つ

    def _failed(self):
        self._lost("接続できませんでした")

    def _disconnected(self):
        self._lost(None)

    def _lost(self, why):
        if self.asm.active:
            self.on_log("受信の途中で iPad との接続が切れました")
        elif self._just_done:
            self.on_log("受信完了（%d 件目）。iPad の次の「送信」を待っています" % self.received)
        elif why:
            self.on_log(why)
        self._just_done = False
        self.asm.reset()
        self.per = None
        self.data_char = self.ack_char = None
        self._scan()


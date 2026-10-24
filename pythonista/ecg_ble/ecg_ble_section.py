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
#     CoreBluetooth は下の _send() で直接呼ぶ（objc_util のメソッド呼び出しは使わない。理由は _send の上）。
#   ※ CoreBluetooth の操作はすべてメインスレッドで行う
#     （デリゲート・ui.delay の予約・ボタンの action はメインスレッドで動く）
import zlib

SERVICE_UUID = "8F1D0001-2C5A-4E7B-9A63-5E0C6A1B2D01"
DATA_UUID = "8F1D0002-2C5A-4E7B-9A63-5E0C6A1B2D01"     # iPad → iPhone（Notify）
ACK_UUID = "8F1D0003-2C5A-4E7B-9A63-5E0C6A1B2D01"      # iPhone → iPad（Write）
BLE_NAME = "ECG-TX"
BLE_VERSION = "2026-10-25 r13"     # 起動時に表示（どのファイルが動いているか確かめるため）
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


# ----- Objective-C の呼び出し
#   objc_util の ObjCInstance でメソッドを呼ぶと、内部で共有の objc_msgSend の型設定（argtypes・restype）を
#   毎回書き換える。BLE のように iOS から次々と呼ばれる処理では、その途中で設定が壊れて
#   'tuple' object is not callable などの ArgumentError や強制終了になった（iPhone・iPad の両方で発生）。
#   そこで、呼び出しの型ごとに専用の objc_msgSend を作り（型は一度決めたら変えない）、それで直接呼ぶ。
#   objc_util は、iOS から呼ばれる受け口（デリゲート）のクラスを作るときだけ使う。
import ctypes

_RT = {}
_CTYPE = {"@": ctypes.c_void_p, "B": ctypes.c_bool, "q": ctypes.c_longlong, "Q": ctypes.c_ulonglong,
          "*": ctypes.c_char_p, "v": None}
_CLS_NO = [0]


def _rt():
    if not _RT:
        from objc_util import load_framework
        load_framework("CoreBluetooth")
        lib = ctypes.CDLL(None)            # objc_util とは別の関数オブジェクトを使う
        get_class = lib.objc_getClass
        get_class.restype = ctypes.c_void_p
        get_class.argtypes = [ctypes.c_char_p]
        reg_sel = lib.sel_registerName
        reg_sel.restype = ctypes.c_void_p
        reg_sel.argtypes = [ctypes.c_char_p]
        _RT.update(get_class=get_class, reg_sel=reg_sel, fns={}, sels={}, classes={},
                   msg=ctypes.cast(lib.objc_msgSend, ctypes.c_void_p).value)
    return _RT


def _send(obj, sel, sig, *args):
    """obj にメッセージ sel を送る。sig は「戻り値の型 + 引数の型」（@ ポインタ, B BOOL, q 整数, Q 符号なし整数,
    * C 文字列, v なし）。例：updateValue:forCharacteristic:onSubscribedCentrals: → "B@@@" """
    rt = _rt()
    fn = rt["fns"].get(sig)
    if fn is None:
        proto = ctypes.CFUNCTYPE(_CTYPE[sig[0]], ctypes.c_void_p, ctypes.c_void_p, *[_CTYPE[c] for c in sig[1:]])
        fn = rt["fns"][sig] = proto(rt["msg"])
    s = rt["sels"].get(sel)
    if s is None:
        s = rt["sels"][sel] = rt["reg_sel"](sel.encode("ascii"))
    return fn(obj, s, *args)


def _cls(name):
    rt = _rt()
    c = rt["classes"].get(name)
    if c is None:
        c = rt["classes"][name] = rt["get_class"](name.encode("ascii"))
    return c


def _new(cls_name, init_sel, sig, *args):
    return _send(_send(_cls(cls_name), "alloc", "@"), init_sel, sig, *args)


def _retain(p):
    return _send(p, "retain", "@") if p else None


def _release(p):
    if p:
        _send(p, "release", "v")


def _nsstr(s):
    return _send(_cls("NSString"), "stringWithUTF8String:", "@*", s.encode("utf-8"))


def _str(p):
    if not p:
        return None
    b = _send(p, "UTF8String", "*")
    return b.decode("utf-8", "replace") if b is not None else None


def _nsdata(b):
    b = bytes(b)
    return _send(_cls("NSData"), "dataWithBytes:length:", "@*Q", b, len(b))


def _bytes_of(p):
    if not p:
        return b""
    n = _send(p, "length", "Q")
    ptr = _send(p, "bytes", "@")
    return ctypes.string_at(ptr, n) if n and ptr else b""


def _nsarray(items):
    arr = (ctypes.c_void_p * len(items))(*items)
    return _send(_cls("NSArray"), "arrayWithObjects:count:", "@@Q", ctypes.cast(arr, ctypes.c_void_p), len(items))


def _nsdict(pairs):
    keys = (ctypes.c_void_p * len(pairs))(*[k for k, v in pairs])
    vals = (ctypes.c_void_p * len(pairs))(*[v for k, v in pairs])
    return _send(_cls("NSDictionary"), "dictionaryWithObjects:forKeys:count:", "@@@Q",
                 ctypes.cast(vals, ctypes.c_void_p), ctypes.cast(keys, ctypes.c_void_p), len(pairs))


def _items(arr):
    if not arr:
        return []
    return [_send(arr, "objectAtIndex:", "@Q", i) for i in range(_send(arr, "count", "Q"))]


def _err_text(err):
    return _str(_send(err, "localizedDescription", "@")) if err else ""


def _ptr_of(obj):
    p = getattr(obj, "ptr", obj)
    return p.value if isinstance(p, ctypes.c_void_p) else p


def _uuid(s):
    return _send(_cls("CBUUID"), "UUIDWithString:", "@@", _nsstr(s))


def _uuid_of(p):
    return (_str(_send(_send(p, "UUID", "@"), "UUIDString", "@")) or "").upper()


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


def _later(sec, fn):
    """sec 秒後にメインスレッドで fn() を呼ぶ。必ずメインスレッド（デリゲートやボタンの action）から呼ぶこと。
    ボタン用の別スレッドから予約すると、そのスレッドが終わった後に iPad の Pythonista が落ちた。
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
#   引数は ObjC オブジェクトのポインタ（整数）のまま渡す。
_ACTIVE = {}


def _call(h, name, ptrs):
    getattr(h, name)(*ptrs)


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
    _dispatch("receiver", "_found", p)


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
        self._ack_char = None    # サービスと特性は iOS が使い続けるので、閉じるまで手放さない
        self._svc = None
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
        if self.pm:
            _send(self.pm, "stopAdvertising", "v")
            _trace("finish: stopAdvertising done")
        self.on_log("送信を終了しました（%s）。次の「送信」を待っています" % ("成功" if ok else "失敗"))
        self.on_done(ok, msg)
        _trace("finish: on_done done")

    # --- BLE
    def start(self):
        _trace_start("sender")
        self.on_log("BLE 送信を開始します（版 %s）" % BLE_VERSION)
        _ACTIVE["sender"] = self
        self.delegate = _delegate("ECGBleSender", _SENDER_METHODS, ["CBPeripheralManagerDelegate"])
        self.pm = _new("CBPeripheralManager", "initWithDelegate:queue:", "@@@", _ptr_of(self.delegate), None)

    def stop(self):
        self.job = None
        if self.pm:
            _send(self.pm, "stopAdvertising", "v")
            _send(self.pm, "removeAllServices", "v")
            _send(self.pm, "setDelegate:", "v@", None)
            _release(self.pm)
            self.pm = None
        for p in (self.data_char, self._ack_char, self._svc):   # iOS から外した後で手放す
            _release(p)
        self.data_char = self._ack_char = self._svc = None
        _release(self._central)
        self._central = None
        if _ACTIVE.get("sender") is self:
            del _ACTIVE["sender"]

    def send(self, name, data, fmt="4", timeout=SEND_TIMEOUT):
        """メインスレッドから呼ぶ。"""
        if self.job:
            raise RuntimeError("送信中です。終わるまでお待ちください")
        if not 0 < len(data) <= MAX_SIZE:
            raise ValueError("サイズが不正です（%d バイト）" % len(data))
        self.job = {"name": name, "data": data, "fmt": fmt, "timeout": timeout}
        self._go()   # send() はメインスレッド（ボタンの action）から呼ぶこと

    def _ready(self):
        self.pump(self._notify)

    def _go(self):
        self._timeout_job = self.job
        _later(self.job["timeout"], self._check_timeout)
        if not self.ready:
            self.on_log("Bluetooth の準備を待っています…")
        elif self._central:   # iPhone がもう接続して待っている
            self._start_job(self._central)
        else:
            self._advertise()

    def _start_job(self, central):
        if not self.job or "packets" in self.job:
            return
        try:
            self.begin(int(_send(central, "maximumUpdateValueLength", "Q")))
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
        st = _send(pm, "state", "q")
        self.on_log("Bluetooth: %s" % _BT_STATE.get(st, st))
        if st != 5:
            self.ready = False
            if _bt_hint(st):
                self.on_log(_bt_hint(st))
            return
        old = (self.data_char, self._ack_char, self._svc)
        data_char = _new("CBMutableCharacteristic", "initWithType:properties:value:permissions:", "@@Q@Q",
                         _uuid(DATA_UUID), 0x10, None, 0x01)            # Notify / Readable
        ack_char = _new("CBMutableCharacteristic", "initWithType:properties:value:permissions:", "@@Q@Q",
                        _uuid(ACK_UUID), 0x08, None, 0x02)              # Write / Writeable
        svc = _new("CBMutableService", "initWithType:primary:", "@@B", _uuid(SERVICE_UUID), True)
        _send(svc, "setCharacteristics:", "v@", _nsarray([data_char, ack_char]))
        _send(pm, "removeAllServices", "v")
        _send(pm, "addService:", "v@", svc)
        # 登録したサービス・特性は手放さない（r12 では登録直後に手放したため、iPhone が中身を
        # 確かめに来た時に解放済みになり、サービスが見えないうえ iPad が落ちた）
        self.data_char, self._ack_char, self._svc = data_char, ack_char, svc
        for p in old:   # 前に登録していたものは removeAllServices で外したので手放してよい
            _release(p)

    def _added(self, err):
        if err:
            self.on_log("BLE サービスの登録に失敗: %s" % _err_text(err))
            return
        self.ready = True
        self.on_log("BLE の準備ができました")
        if self.job:
            self._advertise()

    def _advertise(self):
        _send(self.pm, "stopAdvertising", "v")
        adv = _nsdict([(_nsstr("kCBAdvDataLocalName"), _nsstr(BLE_NAME)),
                       (_nsstr("kCBAdvDataServiceUUIDs"), _nsarray([_uuid(SERVICE_UUID)]))])
        _send(self.pm, "startAdvertising:", "v@", adv)

    def _advertising(self, err):
        if err:
            if self.job:
                self.finish(False, "アドバタイズを開始できません: %s" % _err_text(err))
            return
        self.on_log("iPhone の接続を待っています…（BLE）")

    def _subscribed(self, central, ch):
        if _uuid_of(ch) != DATA_UUID:
            return
        _release(self._central)
        self._central = _retain(central)
        if self.job:
            self._start_job(central)
        else:
            self.on_log("iPhone が接続しました。「送信」を押すと送ります")

    def _notify(self, pkt):
        return bool(_send(self.pm, "updateValue:forCharacteristic:onSubscribedCentrals:", "B@@@",
                          _nsdata(pkt), self.data_char, None))

    def _unsubscribed(self):
        _release(self._central)
        self._central = None
        if self.job and "packets" in self.job:
            self.finish(False, "送信の途中で iPhone との接続が切れました")

    def _writes(self, reqs):
        items = _items(reqs)
        _trace("writes: %d" % len(items))
        texts = []
        for r in items:
            if _uuid_of(_send(r, "characteristic", "@")) == ACK_UUID:
                texts.append(_bytes_of(_send(r, "value", "@")).decode("utf-8", "replace"))
        _trace("writes: read %r" % texts)
        if items:
            _send(self.pm, "respondToRequest:withResult:", "v@q", items[0], 0)
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
        _trace_start("receiver")
        self.on_log("BLE 受信を開始します（版 %s）" % BLE_VERSION)
        _ACTIVE["receiver"] = self
        self.delegate = _delegate("ECGBleReceiver", _RECEIVER_METHODS,
                                  ["CBCentralManagerDelegate", "CBPeripheralDelegate"])
        self.running = True
        self.cm = _new("CBCentralManager", "initWithDelegate:queue:", "@@@", _ptr_of(self.delegate), None)

    def stop(self):
        self.running = False
        if self.cm:
            _send(self.cm, "stopScan", "v")
            if self.per:
                _send(self.cm, "cancelPeripheralConnection:", "v@", self.per)
            _send(self.cm, "setDelegate:", "v@", None)
            _release(self.cm)
            self.cm = None
        self._forget()
        if _ACTIVE.get("receiver") is self:
            del _ACTIVE["receiver"]

    def _forget(self):
        """接続していた iPad の情報を手放す。"""
        for p in (self.per, self.data_char, self.ack_char):
            _release(p)
        self.per = self.data_char = self.ack_char = None

    def _scan(self):
        if self.running and self.cm and not self.per:
            _send(self.cm, "scanForPeripheralsWithServices:options:", "v@@", _nsarray([_uuid(SERVICE_UUID)]), None)

    def _state(self, cm):
        st = _send(cm, "state", "q")
        if st == 5:
            self.on_log("BLE 受信待機中（iPad の「送信」を待っています）")
            self._scan()
        else:
            self.on_log("Bluetooth: %s　%s" % (_BT_STATE.get(st, st), _bt_hint(st)))

    def _found(self, p):
        if self.per:
            return
        _send(self.cm, "stopScan", "v")
        self.per = _retain(p)   # 接続中は参照を持っておく（持たないと iOS が接続を切る）
        _send(p, "setDelegate:", "v@", _ptr_of(self.delegate))
        self.on_log("iPad を見つけました（%s）。接続しています…" % (_str(_send(p, "name", "@")) or BLE_NAME))
        _send(self.cm, "connectPeripheral:options:", "v@@", p, None)
        self._watch = p
        _later(CONNECT_TIMEOUT, self._check_watchdog)

    def _check_watchdog(self):
        p, self._watch = self._watch, None
        if p and p == self.per and not self.data_char and not self.asm.active and self.cm:
            self.on_log("iPad から応答がないため、探し直します")
            _send(self.cm, "cancelPeripheralConnection:", "v@", p)

    def _connected(self, p):
        _send(p, "discoverServices:", "v@", _nsarray([_uuid(SERVICE_UUID)]))

    def _services(self, p):
        for svc in _items(_send(p, "services", "@")):
            if _uuid_of(svc) == SERVICE_UUID:
                _send(p, "discoverCharacteristics:forService:", "v@@", None, svc)
                return
        self.on_log("%s は送信の準備ができていません（iPad で ecg_sender_ipad.py を起動してください）"
                    % (_str(_send(p, "name", "@")) or "機器"))
        _send(self.cm, "cancelPeripheralConnection:", "v@", p)

    def _chars(self, p, svc):
        for ch in _items(_send(svc, "characteristics", "@")):
            u = _uuid_of(ch)
            if u == DATA_UUID and not self.data_char:
                self.data_char = _retain(ch)
            elif u == ACK_UUID and not self.ack_char:
                self.ack_char = _retain(ch)
        if self.data_char and self.ack_char:
            _send(p, "setNotifyValue:forCharacteristic:", "vB@", True, self.data_char)
        else:
            self.on_log("iPad の BLE サービスが不完全です")
            _send(self.cm, "cancelPeripheralConnection:", "v@", p)

    def _subscribed(self, err):
        if err:
            self.on_log("iPad の通知を受け取れません: %s" % _err_text(err))
            if self.per:
                _send(self.cm, "cancelPeripheralConnection:", "v@", self.per)
        else:
            self.on_log("iPad に接続しました。受信を待っています…")

    def _value(self, ch):
        if _uuid_of(ch) != DATA_UUID:
            return
        value = _send(ch, "value", "@")
        if not value:
            return
        reply = self.receive(_bytes_of(value))
        if reply is not None and self.per and self.ack_char:
            _send(self.per, "writeValue:forCharacteristic:type:", "v@@q", _nsdata(reply), self.ack_char, 0)  # 0＝応答あり
        got = self.take_file()
        if got:
            self.on_file(*got)

    def _written(self):
        if self.per and not self.asm.active:
            _send(self.cm, "cancelPeripheralConnection:", "v@", self.per)  # 1ファイル受け取ったら切断して次を待つ

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
        self._forget()
        self._scan()


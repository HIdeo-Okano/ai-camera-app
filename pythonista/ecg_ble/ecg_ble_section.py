# ===== iPad → iPhone 送受信（BLE） =====
#   iPad（送信）  ＝ BLE ペリフェラル。「送信」を押すとアドバタイズし、接続してきた iPhone へ送る。
#   iPhone（受信）＝ BLE セントラル。サービス UUID で iPad を探して自動で接続する（IP・端末名の設定は不要）。
#   データ：DATA キャラクタリスティックの Notify（1通知＝1パケット）
#     b"H" + JSON（name, size, crc, fmt）／ b"D" + 連番4バイト + データ ／ b"E"（終了）
#   応答：iPhone が ACK キャラクタリスティックへ "OK <保存したファイル名>" または "NG <理由>" を書き込む
#   ※ Pythonista の cb モジュールは使わない（Pythonista 3.4 では書き込みが PY_SSIZE_T_CLEAN エラーになる）。
#     両側とも objc_util で CoreBluetooth を直接使う。
import threading
import zlib

SERVICE_UUID = "8F1D0001-2C5A-4E7B-9A63-5E0C6A1B2D01"
DATA_UUID = "8F1D0002-2C5A-4E7B-9A63-5E0C6A1B2D01"     # iPad → iPhone（Notify）
ACK_UUID = "8F1D0003-2C5A-4E7B-9A63-5E0C6A1B2D01"      # iPhone → iPad（Write）
BLE_NAME = "ECG-TX"
# 受信側（iPhone）は、まずサービス UUID で iPad を探す。WIDE_SCAN_AFTER 秒見つからなければ、
# 下の名前の機器も探す（接続後にサービスを確かめるので、別の機器につながることはない）。
IPAD_NAMES = ("TB016", BLE_NAME)
WIDE_SCAN_AFTER = 8
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


def _main(fn):
    """CoreBluetooth の操作はメインスレッドで行う。"""
    from objc_util import on_main_thread
    on_main_thread(fn)()


def _later(sec, fn):
    t = threading.Timer(sec, lambda: _main(fn))
    t.daemon = True
    t.start()


_BT_STATE = {0: "不明", 1: "リセット中", 2: "非対応", 3: "許可されていません", 4: "オフ", 5: "オン"}


def _bt_hint(st):
    if st == 3:
        return "設定 → Pythonista → Bluetooth を ON にしてください"
    if st == 4:
        return "Bluetooth を ON にしてください（設定アプリ → Bluetooth）"
    return ""


class _BleBase(object):
    def _set_log(self, on_log):
        def log(m):
            print("[BLE %s] %s" % (time.strftime("%H:%M:%S"), m))  # 画面を閉じた後もコンソールで確認できる
            if on_log:
                on_log(m)
        self.on_log = log

    def _try(self, fn):
        """デリゲート内のエラーを画面とコンソールに出す（そのままだと見えずに止まるため）。"""
        try:
            fn()
        except Exception as e:
            import traceback
            traceback.print_exc()
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
            if not notify(pk[j["i"]]):
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
        self.job = None
        if self.pm is not None:
            self.pm.stopAdvertising()
        self.on_done(ok, msg)

    # --- BLE
    def start(self):
        from objc_util import ObjCInstance, nsdata_to_bytes
        s = self
        self._bytes = nsdata_to_bytes

        def peripheralManagerDidUpdateState_(_self, _cmd, pm):
            s._try(lambda: s._state(ObjCInstance(pm)))

        def peripheralManager_didAddService_error_(_self, _cmd, pm, svc, err):
            s._try(lambda: s._added(err and ObjCInstance(err)))

        def peripheralManagerDidStartAdvertising_error_(_self, _cmd, pm, err):
            s._try(lambda: s._advertising(err and ObjCInstance(err)))

        def peripheralManager_central_didSubscribeToCharacteristic_(_self, _cmd, pm, central, ch):
            s._try(lambda: s._subscribed(ObjCInstance(central), ObjCInstance(ch)))

        def peripheralManager_central_didUnsubscribeFromCharacteristic_(_self, _cmd, pm, central, ch):
            s._try(lambda: s._unsubscribed())

        def peripheralManagerIsReadyToUpdateSubscribers_(_self, _cmd, pm):
            s._try(lambda: s.pump(s._notify))

        def peripheralManager_didReceiveWriteRequests_(_self, _cmd, pm, reqs):
            s._try(lambda: s._writes(ObjCInstance(reqs)))

        peripheralManagerDidUpdateState_.encoding = "v@:@"
        peripheralManager_didAddService_error_.encoding = "v@:@@@"
        peripheralManagerDidStartAdvertising_error_.encoding = "v@:@@"
        peripheralManager_central_didSubscribeToCharacteristic_.encoding = "v@:@@@"
        peripheralManager_central_didUnsubscribeFromCharacteristic_.encoding = "v@:@@@"
        peripheralManagerIsReadyToUpdateSubscribers_.encoding = "v@:@"
        peripheralManager_didReceiveWriteRequests_.encoding = "v@:@@"
        self.delegate = _delegate("ECGBleSender", [
            peripheralManagerDidUpdateState_, peripheralManager_didAddService_error_,
            peripheralManagerDidStartAdvertising_error_,
            peripheralManager_central_didSubscribeToCharacteristic_,
            peripheralManager_central_didUnsubscribeFromCharacteristic_,
            peripheralManagerIsReadyToUpdateSubscribers_, peripheralManager_didReceiveWriteRequests_,
        ], ["CBPeripheralManagerDelegate"])
        self.pm = _cb("CBPeripheralManager").alloc().initWithDelegate_queue_(self.delegate, None)

    def stop(self):
        self.job = None
        if self.pm is not None:
            self.pm.stopAdvertising()
            self.pm.removeAllServices()
            self.pm.setDelegate_(None)
            self.pm = None

    def send(self, name, data, fmt="4", timeout=SEND_TIMEOUT):
        if self.job:
            raise RuntimeError("送信中です。終わるまでお待ちください")
        if not 0 < len(data) <= MAX_SIZE:
            raise ValueError("サイズが不正です（%d バイト）" % len(data))
        job = self.job = {"name": name, "data": data, "fmt": fmt}
        _main(self._go)
        _later(timeout, lambda: self._timeout(job, timeout))

    def _go(self):
        if self.ready:
            self._advertise()
        else:
            self.on_log("Bluetooth の準備を待っています…")

    def _timeout(self, job, sec):
        if self.job is job:
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
        if _uuid_of(ch) != DATA_UUID or not self.job or "packets" in self.job:
            return
        try:
            self.begin(int(central.maximumUpdateValueLength()))
        except Exception as e:
            self.finish(False, str(e))
            return
        self.pump(self._notify)

    def _notify(self, pkt):
        from objc_util import ns
        return bool(self.pm.updateValue_forCharacteristic_onSubscribedCentrals_(ns(pkt), self.data_char, None))

    def _unsubscribed(self):
        if self.job and "packets" in self.job:
            self.finish(False, "送信の途中で iPhone との接続が切れました")

    def _writes(self, reqs):
        texts = []
        for i in range(reqs.count()):
            r = reqs.objectAtIndex_(i)
            if _uuid_of(r.characteristic()) == ACK_UUID and r.value():
                texts.append(self._bytes(r.value()).decode("utf-8", "replace"))
        if reqs.count():
            self.pm.respondToRequest_withResult_(reqs.objectAtIndex_(0), 0)
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
        self._wide = False
        self._scan_no = 0

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
        self._done = (path, fmt)
        return ("OK %s" % os.path.basename(path)).encode("utf-8")[:ACK_MAX]

    def take_file(self):
        d, self._done = self._done, None
        return d

    # --- BLE
    def start(self):
        from objc_util import ObjCInstance, nsdata_to_bytes
        s = self
        self._bytes = nsdata_to_bytes

        def centralManagerDidUpdateState_(_self, _cmd, cm):
            s._try(lambda: s._state(ObjCInstance(cm)))

        def centralManager_didDiscoverPeripheral_advertisementData_RSSI_(_self, _cmd, cm, p, adv, rssi):
            s._try(lambda: s._found(ObjCInstance(p), adv and ObjCInstance(adv)))

        def centralManager_didConnectPeripheral_(_self, _cmd, cm, p):
            s._try(lambda: s._connected(ObjCInstance(p)))

        def centralManager_didFailToConnectPeripheral_error_(_self, _cmd, cm, p, err):
            s._try(lambda: s._lost("接続できませんでした"))

        def centralManager_didDisconnectPeripheral_error_(_self, _cmd, cm, p, err):
            s._try(lambda: s._lost(None))

        def peripheral_didDiscoverServices_(_self, _cmd, p, err):
            s._try(lambda: s._services(ObjCInstance(p)))

        def peripheral_didDiscoverCharacteristicsForService_error_(_self, _cmd, p, svc, err):
            s._try(lambda: s._chars(ObjCInstance(p), ObjCInstance(svc)))

        def peripheral_didUpdateValueForCharacteristic_error_(_self, _cmd, p, ch, err):
            s._try(lambda: s._value(ObjCInstance(ch)))

        def peripheral_didWriteValueForCharacteristic_error_(_self, _cmd, p, ch, err):
            s._try(lambda: s._written())

        def peripheral_didUpdateNotificationStateForCharacteristic_error_(_self, _cmd, p, ch, err):
            s._try(lambda: s._subscribed(err and ObjCInstance(err)))

        centralManagerDidUpdateState_.encoding = "v@:@"
        centralManager_didDiscoverPeripheral_advertisementData_RSSI_.encoding = "v@:@@@@"
        centralManager_didConnectPeripheral_.encoding = "v@:@@"
        centralManager_didFailToConnectPeripheral_error_.encoding = "v@:@@@"
        centralManager_didDisconnectPeripheral_error_.encoding = "v@:@@@"
        peripheral_didDiscoverServices_.encoding = "v@:@@"
        peripheral_didDiscoverCharacteristicsForService_error_.encoding = "v@:@@@"
        peripheral_didUpdateValueForCharacteristic_error_.encoding = "v@:@@@"
        peripheral_didWriteValueForCharacteristic_error_.encoding = "v@:@@@"
        peripheral_didUpdateNotificationStateForCharacteristic_error_.encoding = "v@:@@@"
        self.delegate = _delegate("ECGBleReceiver", [
            centralManagerDidUpdateState_, centralManager_didDiscoverPeripheral_advertisementData_RSSI_,
            centralManager_didConnectPeripheral_, centralManager_didFailToConnectPeripheral_error_,
            centralManager_didDisconnectPeripheral_error_, peripheral_didDiscoverServices_,
            peripheral_didDiscoverCharacteristicsForService_error_,
            peripheral_didUpdateValueForCharacteristic_error_, peripheral_didWriteValueForCharacteristic_error_,
            peripheral_didUpdateNotificationStateForCharacteristic_error_,
        ], ["CBCentralManagerDelegate", "CBPeripheralDelegate"])
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

    def _scan(self):
        from objc_util import ns
        if self.running and self.cm is not None and self.per is None:
            self._wide = False
            self._scan_no += 1
            no = self._scan_no
            self.cm.scanForPeripheralsWithServices_options_(ns([_uuid(SERVICE_UUID)]), None)
            _later(WIDE_SCAN_AFTER, lambda: self._wide_scan(no))

    def _wide_scan(self, no):
        """サービス UUID で見つからないとき、名前（IPAD_NAMES）でも探す。"""
        if no != self._scan_no or not self.running or self.cm is None or self.per is not None:
            return
        self._wide = True
        self.cm.stopScan()
        self.cm.scanForPeripheralsWithServices_options_(None, None)
        self.on_log("名前（%s）でも iPad を探しています…" % " / ".join(IPAD_NAMES))

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
        if self._wide:  # 名前で探しているときは、名前が一致する機器だけ
            names = [str(p.name() or "")]
            if adv is not None and adv.objectForKey_("kCBAdvDataLocalName"):
                names.append(str(adv.objectForKey_("kCBAdvDataLocalName")))
            if not any(n.lower() == x.lower() for n in names for x in IPAD_NAMES):
                return
        self.cm.stopScan()
        self.per = p
        p.setDelegate_(self.delegate)
        self.on_log("iPad を見つけました（%s）。接続しています…" % (p.name() or BLE_NAME))
        self.cm.connectPeripheral_options_(p, None)
        _later(CONNECT_TIMEOUT, lambda: self._watchdog(p))

    def _watchdog(self, p):
        if self.per is p and self.data_char is None and not self.asm.active and self.cm is not None:
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
        self.on_log("%s には ECG 送信のサービスがありません。iPad で「送信」を押してください" % (p.name() or "機器"))
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

    def _lost(self, why):
        if self.asm.active:
            self.on_log("受信の途中で iPad との接続が切れました")
        elif why:
            self.on_log(why)
        self.asm.reset()
        self.per = None
        self.data_char = self.ack_char = None
        self._scan()


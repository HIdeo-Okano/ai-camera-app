# -*- coding: utf-8 -*-
"""iPad の BLE 診断（Pythonista 3）

ecg_sender_ipad.py が iPad で落ちる場所を調べるため、BLE の準備を1段ずつ行う。
画面は使わず、コンソールに結果を出す。各段の結果は、このスクリプトと同じフォルダーの
ble_diag_ipad_log.txt に書いてすぐ保存する。Pythonista が落ちたら、もう一度このスクリプトを
実行すると、最初に「前回どこまで進んだか」を表示する。

使い方：iPad で ▶ を押すだけ。iPhone 側は何もしなくてよい。
"""
import ctypes
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
LOG = os.path.join(HERE, "ble_diag_ipad_log.txt")
SERVICE_UUID = "8F1D0001-2C5A-4E7B-9A63-5E0C6A1B2D01"
DATA_UUID = "8F1D0002-2C5A-4E7B-9A63-5E0C6A1B2D01"
ACK_UUID = "8F1D0003-2C5A-4E7B-9A63-5E0C6A1B2D01"


def write(msg):
    line = "%s %s" % (time.strftime("%H:%M:%S"), msg)
    print(line)
    sys.stdout.flush()
    with open(LOG, "a", encoding="utf-8") as f:
        f.write(line + "\n")
        f.flush()
        os.fsync(f.fileno())


def show_previous():
    try:
        with open(LOG, encoding="utf-8") as f:
            lines = f.read().splitlines()
    except OSError:
        print("（前回の記録はありません）")
        return
    print("========== 前回の記録（最後の行が、落ちる直前に行っていた段）==========")
    for ln in lines[-30:]:
        print("  " + ln)
    print("========== ここまで ==========\n")


# ---- Objective-C を直接呼ぶ（ecg_sender_ipad.py と同じ方式）
_CT = {"@": ctypes.c_void_p, "B": ctypes.c_bool, "q": ctypes.c_longlong, "Q": ctypes.c_ulonglong,
       "*": ctypes.c_char_p, "v": None}
RT = {}


def send(obj, sel, sig, *args):
    fn = RT["fns"].get(sig)
    if fn is None:
        proto = ctypes.CFUNCTYPE(_CT[sig[0]], ctypes.c_void_p, ctypes.c_void_p, *[_CT[c] for c in sig[1:]])
        fn = RT["fns"][sig] = proto(RT["msg"])
    return fn(obj, RT["sel"](sel.encode("ascii")), *args)


def cls(name):
    return RT["cls"](name.encode("ascii"))


def nsstr(s):
    return send(cls("NSString"), "stringWithUTF8String:", "@*", s.encode("utf-8"))


def uuid(s):
    return send(cls("CBUUID"), "UUIDWithString:", "@@", nsstr(s))


def nsarray(items):
    arr = (ctypes.c_void_p * len(items))(*items)
    return send(cls("NSArray"), "arrayWithObjects:count:", "@@Q", ctypes.cast(arr, ctypes.c_void_p), len(items))


def nsdict(pairs):
    keys = (ctypes.c_void_p * len(pairs))(*[k for k, v in pairs])
    vals = (ctypes.c_void_p * len(pairs))(*[v for k, v in pairs])
    return send(cls("NSDictionary"), "dictionaryWithObjects:forKeys:count:", "@@@Q",
                ctypes.cast(vals, ctypes.c_void_p), ctypes.cast(keys, ctypes.c_void_p), len(pairs))


# ---- デリゲート（呼ばれたら記録するだけ）
EVENTS = []


def peripheralManagerDidUpdateState_(_self, _cmd, pm):
    st = send(pm, "state", "q")
    EVENTS.append(("state", st))
    write("  ← iOS から: Bluetooth の状態 = %s（5 ならオン）" % st)


def peripheralManager_didAddService_error_(_self, _cmd, pm, svc, err):
    EVENTS.append(("added", err))
    write("  ← iOS から: サービス登録 %s" % ("失敗" if err else "成功"))


def peripheralManagerDidStartAdvertising_error_(_self, _cmd, pm, err):
    EVENTS.append(("adv", err))
    write("  ← iOS から: アドバタイズ開始 %s" % ("失敗" if err else "成功"))


METHODS = [peripheralManagerDidUpdateState_, peripheralManager_didAddService_error_,
           peripheralManagerDidStartAdvertising_error_]
for _f in METHODS:
    _f.encoding = "v@:" + "@" * (_f.__code__.co_argcount - 2)


def wait_for(kind, sec=10):
    end = time.time() + sec
    while time.time() < end:
        for k, v in EVENTS:
            if k == kind:
                return v
        time.sleep(0.2)
    raise RuntimeError("%d 秒待っても iOS から返事がありません（%s）" % (sec, kind))


def step(no, title, fn):
    write("[%d] %s … 開始" % (no, title))
    out = fn()
    write("[%d] %s … OK" % (no, title))
    return out


def main():
    show_previous()
    with open(LOG, "w", encoding="utf-8") as f:
        f.write("BLE 診断の記録\n")
    write("iPad BLE 診断を始めます（Python %s）" % sys.version.split()[0])
    keep = {}

    def s1():
        import objc_util
        objc_util.load_framework("CoreBluetooth")

    def s2():
        lib = ctypes.CDLL(None)
        lib.objc_getClass.restype = ctypes.c_void_p
        lib.objc_getClass.argtypes = [ctypes.c_char_p]
        lib.sel_registerName.restype = ctypes.c_void_p
        lib.sel_registerName.argtypes = [ctypes.c_char_p]
        RT.update(cls=lib.objc_getClass, sel=lib.sel_registerName, fns={},
                  msg=ctypes.cast(lib.objc_msgSend, ctypes.c_void_p).value)
        write("    objc_msgSend のアドレス = %#x" % RT["msg"])
        write("    CBPeripheralManager クラス = %#x" % (cls("CBPeripheralManager") or 0))

    def s3():
        write("    文字列の確認: %r" % send(nsstr("hello"), "UTF8String", "*"))

    def s4():
        from objc_util import create_objc_class
        d = create_objc_class("ECGDiag_%d" % int(time.time()), methods=METHODS,
                              protocols=["CBPeripheralManagerDelegate"]).new()
        keep["delegate"] = d
        p = getattr(d, "ptr", d)
        keep["dptr"] = p.value if isinstance(p, ctypes.c_void_p) else p
        write("    デリゲート = %r" % (keep["dptr"],))

    def s5():
        pm = send(send(cls("CBPeripheralManager"), "alloc", "@"), "initWithDelegate:queue:", "@@@",
                  keep["dptr"], None)
        keep["pm"] = pm
        write("    PeripheralManager = %#x" % (pm or 0))

    def s6():
        st = wait_for("state")
        if st != 5:
            raise RuntimeError("Bluetooth がオンになっていません（状態 %s）" % st)

    def s7():
        mk = lambda u, props, perms: send(send(cls("CBMutableCharacteristic"), "alloc", "@"),
                                          "initWithType:properties:value:permissions:", "@@Q@Q",
                                          uuid(u), props, None, perms)
        keep["data"] = mk(DATA_UUID, 0x10, 0x01)
        keep["ack"] = mk(ACK_UUID, 0x08, 0x02)
        write("    特性 = %#x, %#x" % (keep["data"], keep["ack"]))

    def s8():
        svc = send(send(cls("CBMutableService"), "alloc", "@"), "initWithType:primary:", "@@B",
                   uuid(SERVICE_UUID), True)
        send(svc, "setCharacteristics:", "v@", nsarray([keep["data"], keep["ack"]]))
        keep["svc"] = svc
        write("    サービス = %#x" % svc)

    def s9():
        send(keep["pm"], "addService:", "v@", keep["svc"])
        err = wait_for("added")
        if err:
            raise RuntimeError("サービスの登録に失敗しました")

    def s10():
        adv = nsdict([(nsstr("kCBAdvDataLocalName"), nsstr("ECG-TX")),
                      (nsstr("kCBAdvDataServiceUUIDs"), nsarray([uuid(SERVICE_UUID)]))])
        send(keep["pm"], "startAdvertising:", "v@", adv)
        err = wait_for("adv")
        if err:
            raise RuntimeError("アドバタイズの開始に失敗しました")
        time.sleep(2)

    def s11():
        send(keep["pm"], "stopAdvertising", "v")
        send(keep["pm"], "removeAllServices", "v")
        send(keep["pm"], "setDelegate:", "v@", None)

    steps = [
        ("objc_util で CoreBluetooth を読み込む", s1),
        ("objc_msgSend を直接使う準備", s2),
        ("直接呼び出しの確認（文字列）", s3),
        ("デリゲートのクラスを作る（objc_util）", s4),
        ("PeripheralManager を作る", s5),
        ("Bluetooth の状態の通知を待つ", s6),
        ("特性（DATA・ACK）を作る", s7),
        ("サービスを作る", s8),
        ("サービスを登録する", s9),
        ("アドバタイズ（電波を出す）", s10),
        ("後片付け", s11),
    ]
    for i, (title, fn) in enumerate(steps, 1):
        try:
            step(i, title, fn)
        except Exception as e:
            write("[%d] %s … エラー: %s: %s" % (i, title, type(e).__name__, e))
            break
    else:
        write("すべての段が OK でした。このスクリーンショットを送ってください。")
    keep_alive.append(keep)   # 診断が終わるまで ObjC のオブジェクトを手放さない


keep_alive = []

if __name__ == "__main__":
    main()

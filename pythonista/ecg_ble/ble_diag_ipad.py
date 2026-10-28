# -*- coding: utf-8 -*-
"""iPad の BLE 診断（Pythonista 3）

ecg_sender_ipad.py が iPad で落ちる場所を調べるため、BLE の準備を1段ずつ行い、最後に
iPhone（ecg_viewer_iphone.py）の接続を最大 90 秒待つ。画面は使わず、コンソールに結果を出す。
各段の結果は ble_diag_ipad_log.txt に書いてすぐ保存する。Pythonista が落ちたら、もう一度
このスクリプトを実行すると、最初に「前回どこまで進んだか」を表示する。

作り方を2通り試す（A/B）：
  direct … iOS を直接呼んでサービスを作る（今の ecg_sender_ipad.py と同じ方式）
  objc   … objc_util でサービスを作る（iPhone 接続で落ちなかった r11 と同じ方式）
前回 direct で iPhone を待っている間に落ちた場合は、自動で objc に切り替える。

使い方：先に iPhone で ecg_viewer_iphone.py を起動しておき、iPad でこれの ▶ を押す。
"""
import ctypes
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))


def _log_path():
    """記録ファイルの場所。「ファイル」アプリから開いたスクリプトは、そのフォルダーに書き込めない
    （PermissionError）ので、そのときは Pythonista 自身の Documents に置く。"""
    for d in (HERE, os.path.expanduser("~/Documents")):
        path = os.path.join(d, "ble_diag_ipad_log.txt")
        try:
            with open(path, "a", encoding="utf-8"):
                pass
            return path
        except OSError:
            continue
    return None


LOG = _log_path()
SERVICE_UUID = "8F1D0001-2C5A-4E7B-9A63-5E0C6A1B2D01"
DATA_UUID = "8F1D0002-2C5A-4E7B-9A63-5E0C6A1B2D01"
ACK_UUID = "8F1D0003-2C5A-4E7B-9A63-5E0C6A1B2D01"
MODE = None          # None＝自動で選ぶ。"direct" か "objc" を書くと、その方式に固定する
WAIT_IPHONE = 90     # iPhone の接続を待つ秒数
DONE_MARK = "診断終了"


def write(msg):
    line = "%s %s" % (time.strftime("%H:%M:%S"), msg)
    print(line)
    sys.stdout.flush()
    if LOG:
        with open(LOG, "a", encoding="utf-8") as f:
            f.write(line + "\n")
            f.flush()
            os.fsync(f.fileno())


def show_previous():
    """前回の記録を表示して、行の一覧を返す。"""
    print("記録ファイル: %s" % LOG)
    try:
        with open(LOG, encoding="utf-8") as f:
            lines = f.read().splitlines()
    except (OSError, TypeError):
        lines = []
    if not lines:
        print("（前回の記録はありません）")
        return lines
    print("========== 前回の記録（最後の行が、落ちる直前に行っていた段）==========")
    for ln in lines[-30:]:
        print("  " + ln)
    print("========== ここまで ==========\n")
    return lines


def choose_mode(lines):
    """前回 direct で最後まで行かずに終わった（落ちた）なら objc、それ以外は direct。"""
    if MODE:
        return MODE, "スクリプトの MODE で指定"
    prev = None
    for ln in lines:
        if "[mode " in ln:
            prev = ln.split("[mode ", 1)[1].split("]", 1)[0]
    done = any(DONE_MARK in ln for ln in lines)
    if prev == "direct" and not done:
        return "objc", "前回 direct の途中で終わった（落ちた）ため、objc に切り替えます"
    if prev == "objc" and not done:
        return "direct", "前回 objc の途中で終わった（落ちた）ため、direct に戻します"
    return "direct", "最初は direct（今の送信プログラムと同じ方式）で試します"


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


def peripheralManager_central_didSubscribeToCharacteristic_(_self, _cmd, pm, central, ch):
    write("  ← iOS から: iPhone が通知を購読しました（接続できました）")
    EVENTS.append(("sub", central))


def peripheralManager_central_didUnsubscribeFromCharacteristic_(_self, _cmd, pm, central, ch):
    write("  ← iOS から: iPhone が購読をやめました（切断）")
    EVENTS.append(("unsub", central))


def peripheralManagerIsReadyToUpdateSubscribers_(_self, _cmd, pm):
    write("  ← iOS から: 送信の準備ができました")


def peripheralManager_didReceiveReadRequest_(_self, _cmd, pm, req):
    write("  ← iOS から: 読み取り要求")
    send(pm, "respondToRequest:withResult:", "v@q", req, 0)


def peripheralManager_didReceiveWriteRequests_(_self, _cmd, pm, reqs):
    write("  ← iOS から: 書き込み要求")
    n = send(reqs, "count", "Q")
    if n:
        send(pm, "respondToRequest:withResult:", "v@q", send(reqs, "objectAtIndex:", "@Q", 0), 0)


METHODS = [peripheralManagerDidUpdateState_, peripheralManager_didAddService_error_,
           peripheralManagerDidStartAdvertising_error_, peripheralManager_central_didSubscribeToCharacteristic_,
           peripheralManager_central_didUnsubscribeFromCharacteristic_, peripheralManagerIsReadyToUpdateSubscribers_,
           peripheralManager_didReceiveReadRequest_, peripheralManager_didReceiveWriteRequests_]
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
    mode, why = choose_mode(show_previous())
    if LOG:
        with open(LOG, "w", encoding="utf-8") as f:
            f.write("BLE 診断の記録\n")
    write("iPad BLE 診断を始めます（Python %s）" % sys.version.split()[0])
    write("[mode %s] %s" % (mode, why))
    objc = mode == "objc"
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
        if objc:
            from objc_util import ObjCClass
            keep["pm_obj"] = ObjCClass("CBPeripheralManager").alloc().initWithDelegate_queue_(
                keep["delegate"], None)
            pm = keep["pm_obj"].ptr.value
        else:
            pm = send(send(cls("CBPeripheralManager"), "alloc", "@"), "initWithDelegate:queue:", "@@@",
                      keep["dptr"], None)
        keep["pm"] = pm
        write("    PeripheralManager = %#x" % (pm or 0))

    def s6():
        st = wait_for("state")
        if st != 5:
            raise RuntimeError("Bluetooth がオンになっていません（状態 %s）" % st)

    def s7():
        if objc:
            from objc_util import ObjCClass, ns
            u = ObjCClass("CBUUID").UUIDWithString_
            mk = lambda uid, props, perms: ObjCClass("CBMutableCharacteristic").alloc() \
                .initWithType_properties_value_permissions_(u(ns(uid)), props, None, perms)
            keep["data_obj"] = mk(DATA_UUID, 0x10, 0x01)
            keep["ack_obj"] = mk(ACK_UUID, 0x08, 0x02)
            keep["data"], keep["ack"] = keep["data_obj"].ptr.value, keep["ack_obj"].ptr.value
        else:
            mk = lambda u, props, perms: send(send(cls("CBMutableCharacteristic"), "alloc", "@"),
                                              "initWithType:properties:value:permissions:", "@@Q@Q",
                                              uuid(u), props, None, perms)
            keep["data"] = mk(DATA_UUID, 0x10, 0x01)
            keep["ack"] = mk(ACK_UUID, 0x08, 0x02)
        write("    特性 = %#x, %#x" % (keep["data"], keep["ack"]))

    def s8():
        if objc:
            from objc_util import ObjCClass, ns
            svc_obj = ObjCClass("CBMutableService").alloc().initWithType_primary_(
                ObjCClass("CBUUID").UUIDWithString_(ns(SERVICE_UUID)), True)
            svc_obj.setCharacteristics_(ns([keep["data_obj"], keep["ack_obj"]]))
            keep["svc_obj"] = svc_obj
            svc = svc_obj.ptr.value
        else:
            svc = send(send(cls("CBMutableService"), "alloc", "@"), "initWithType:primary:", "@@B",
                       uuid(SERVICE_UUID), True)
            send(svc, "setCharacteristics:", "v@", nsarray([keep["data"], keep["ack"]]))
        keep["svc"] = svc
        write("    サービス = %#x" % svc)

    def s9():
        if objc:
            keep["pm_obj"].addService_(keep["svc_obj"])
        else:
            send(keep["pm"], "addService:", "v@", keep["svc"])
        err = wait_for("added")
        if err:
            raise RuntimeError("サービスの登録に失敗しました")

    def s10():
        if objc:
            from objc_util import ObjCClass, ns
            keep["pm_obj"].startAdvertising_(ns({
                "kCBAdvDataLocalName": "ECG-TX",
                "kCBAdvDataServiceUUIDs": [ObjCClass("CBUUID").UUIDWithString_(ns(SERVICE_UUID))]}))
        else:
            adv = nsdict([(nsstr("kCBAdvDataLocalName"), nsstr("ECG-TX")),
                          (nsstr("kCBAdvDataServiceUUIDs"), nsarray([uuid(SERVICE_UUID)]))])
            send(keep["pm"], "startAdvertising:", "v@", adv)
        err = wait_for("adv")
        if err:
            raise RuntimeError("アドバタイズの開始に失敗しました")

    def s11():
        write("    iPhone で ecg_viewer_iphone.py を動かしてください（最大 %d 秒待ちます）" % WAIT_IPHONE)
        wait_for("sub", WAIT_IPHONE)
        write("    iPhone とつながりました。5 秒そのまま様子を見ます")
        time.sleep(5)

    def s12():
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
        ("iPhone の接続を待つ", s11),
        ("後片付け", s12),
    ]
    for i, (title, fn) in enumerate(steps, 1):
        try:
            step(i, title, fn)
        except Exception as e:
            write("[%d] %s … エラー: %s: %s" % (i, title, type(e).__name__, e))
            break
    else:
        write("すべての段が OK でした（%s 方式では iPhone とつながっても落ちません）。" % mode)
    write("%s（%s 方式）。このスクリーンショットを送ってください。" % (DONE_MARK, mode))
    keep_alive.append(keep)   # 診断が終わるまで ObjC のオブジェクトを手放さない


keep_alive = []

if __name__ == "__main__":
    main()

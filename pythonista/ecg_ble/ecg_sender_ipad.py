# -*- coding: utf-8 -*-
"""ECG CSV 送信プログラム（iPad：Pythonista 3）

iPad の「ダウンロード」フォルダにある CSV（例：ECGtsHDdata1.csv）を選び、
BLE（Bluetooth Low Energy）で iPhone（ecg_viewer_iphone.py を実行中）へ送る。
Wi-Fi・IP アドレスの設定は不要（iPhone がサービス UUID でこの iPad を見つけて自動で接続する）。

送るファイルは次の4行ヘッダ形式にそろえる。
  1行目 ECG / 2行目 サンプル周波数（256）/ 3行目 電極状態（0＝正常）/ 4行目 不整脈の件数（0＝なし）/ 5行目以降 心電データ
元ファイルが2行ヘッダ（ECG／256 の直後から心電データ）の場合は、波形を解析して3・4行目を作る。
3・4行目の値は画面で書き換えられる（異常時の表示を確かめるため）。
このファイル1つで動く（共通処理 ecg_core.py を埋め込み済み。build.py で生成）。
"""
import json
import os
import ui

# ==================== 共通処理（src/ecg_core.py を埋め込み） ====================
import json
import math
import os
import time

# ===== R波検出・R-R判定（rr_detect.c と同じ整数演算） =====
FS = 256
WIN = 16
REFRACT = 51
WARMUP = 2 * FS
NHIST = 8
NONE, EARLY, PAUSE = 0, 1, 2
FLAGNAME = ["通常", "早期拍", "休止"]


class RRDetector(object):
    def __init__(self, seed_median=0):
        self.x1 = self.x2 = 0
        self.ring = [0] * WIN
        self.ri = 0
        self.sum = 0
        self.s1 = self.s2 = 0
        self.spk = self.npk = self.thr = 0
        self.warm_max = self.warm_acc = 0
        self.n = 0
        self.last_r = 0
        self.have_r = False
        self.hist = [0] * NHIST
        self.hn = 0
        self.hi = 0
        if seed_median > 0:  # 直前8拍の中央値で履歴を初期化
            self.hist = [int(seed_median)] * NHIST
            self.hn = NHIST

    def median8(self):
        a = sorted(self.hist)
        return (a[3] + a[4]) >> 1

    def push(self, x):
        a = abs(x - self.x2)
        if a > 65535:
            a = 65535
        self.x2 = self.x1
        self.x1 = x
        self.sum += a - self.ring[self.ri]
        self.ring[self.ri] = a
        self.ri = (self.ri + 1) & (WIN - 1)
        cur = self.sum
        res = None
        if self.n < WARMUP:
            if cur > self.warm_max:
                self.warm_max = cur
            self.warm_acc += cur >> 8
            if self.n == WARMUP - 1:
                self.spk = self.warm_max >> 1
                self.npk = self.warm_acc >> 1
                self.thr = self.npk + ((self.spk - self.npk) >> 2)
        elif self.s1 > self.s2 and self.s1 >= cur:
            pk = self.s1
            idx = self.n - 1
            if pk > self.thr and (not self.have_r or idx - self.last_r >= REFRACT):
                self.spk += (pk - self.spk) >> 3
                if self.have_r:
                    r = idx - self.last_r
                    f = NONE
                    if self.hn >= NHIST:
                        med = self.median8()
                        if r * 5 < med * 4:
                            f = EARLY
                        elif r * 10 > med * 13:
                            f = PAUSE
                    self.hist[self.hi] = r
                    self.hi = (self.hi + 1) & (NHIST - 1)
                    if self.hn < NHIST:
                        self.hn += 1
                    res = {"idx": idx, "rr": r, "flag": f, "judged": self.hn >= NHIST}
                else:
                    res = {"idx": idx, "rr": 0, "flag": NONE, "judged": False}
                self.last_r = idx
                self.have_r = True
            elif pk <= self.thr:
                self.npk += (pk - self.npk) >> 3
            self.thr = self.npk + ((self.spk - self.npk) >> 2)
        self.s2 = self.s1
        self.s1 = cur
        self.n += 1
        return res


def detect(samples, seed_median=0):
    det = RRDetector(seed_median)
    out = []
    for v in samples:
        r = det.push(int(v))
        if r:
            out.append(r)
    return out


# ===== 電極はがれ（リードオフ）検出：0.5秒（128点）ブロック単位 =====
#   フラット：振幅（最大−最小）が FLAT_TH 未満のブロックが FLAT_N 個（2秒）以上連続
#   飽和　　：|値| が SAT_TH 以上のブロックが SAT_N 個（1秒）以上連続
BLK, FLAT_TH, FLAT_N, SAT_TH, SAT_N = 128, 30, 4, 30000, 2


def detect_lead_off(s):
    nb = len(s) // BLK
    flat, sat = [], []
    for b in range(nb):
        blk = s[b * BLK:(b + 1) * BLK]
        mn, mx = min(blk), max(blk)
        flat.append(1 if mx - mn < FLAT_TH else 0)
        sat.append(1 if max(abs(mn), abs(mx)) >= SAT_TH else 0)
    iv = []

    def run(arr, need):
        b = 0
        while b < nb:
            if arr[b]:
                e = b
                while e < nb and arr[e]:
                    e += 1
                if e - b >= need:
                    iv.append([b * BLK / FS, e * BLK / FS])
                b = e
            else:
                b += 1
    run(flat, FLAT_N)
    run(sat, SAT_N)
    return iv


def merge_iv(iv):
    o = []
    for x in sorted(iv, key=lambda p: p[0]):
        if o and x[0] <= o[-1][1] + 0.01:
            o[-1][1] = max(o[-1][1], x[1])
        else:
            o.append([x[0], x[1]])
    return o


# ===== ダミー波形（P・QRS・T の簡易合成。72bpm。12拍目を早期、16拍目を休止） =====
def _js_round(x):
    return int(math.floor(x + 0.5))


def _lcg(s):
    # JavaScript の (s*1103515245+12345)&0x7fffffff と同じ結果（倍精度で掛けてから32bit化）
    v = float(s) * 1103515245.0 + 12345.0
    return (int(v) % 4294967296) & 0x7fffffff


def demo_ecg(sec, lead_off=None):
    n = sec * FS
    x = [0.0] * n
    beat, k, beats = 300, 0, []
    while beat < n - 60:
        beats.append(beat)
        k += 1
        beat += 120 if k == 12 else (340 if k == 16 else 213)

    def g(t, c, w, h):
        return h * math.exp(-((t - c) * (t - c)) / (2 * w * w))
    for b in beats:
        for t in range(b - 70, b + 110):
            if 0 <= t < n:
                x[t] += (g(t, b - 34, 7, 90) - g(t, b - 6, 2.2, 160) + g(t, b, 2.6, 1100)
                         - g(t, b + 6, 2.6, 240) + g(t, b + 58, 13, 210))
    s = 12345
    out = [0] * n
    for i in range(n):
        s = _lcg(s)
        out[i] = _js_round(x[i] + ((s >> 8) % 21) - 10 + 40 * math.sin(i / 190))
    if lead_off:  # 指定区間を電極はがれ（ほぼ平坦）にする
        for a, b in lead_off:
            for i in range(int(math.floor(a * FS)), min(n, int(math.floor(b * FS)))):
                s = _lcg(s)
                out[i] = ((s >> 8) % 7) - 3
    return out


# ===== CSV 読み込み =====
#   1行目 : ECG（データ種別。大文字小文字は問わない）
#   2行目 : サンプル周波数（例 256 → 256 Hz）
#   3行目 : 電極状態   0＝正常にデータ取得、0以外＝異常
#   4行目 : 不整脈     0＝不整脈なし、0以外＝不整脈あり（検出した個数）
#   5行目以降 : 心電データ（1行1値）
#   ※ 旧形式（2行ヘッダ：ECG／256 の直後から心電データ）も読める。
#     fmt="auto" のときは、3・4行目が 0〜9999 の整数なら4行ヘッダ、そうでなければ2行ヘッダとみなす。
def _split(line):
    for sep in (",", "\t", ";"):
        line = line.replace(sep, " ")
    return line.split()


def _num(tok):
    try:
        v = float(tok)
    except (TypeError, ValueError):
        return None
    return v if math.isfinite(v) else None


def _status_int(tok):
    v = _num(tok)
    if v is None or v != int(v) or v < 0 or v > 9999:
        return None
    return int(v)


def parse_csv(text, fmt="auto"):
    """CSV テキストを読む。fmt は "auto"／"4"（4行ヘッダ）／"2"（2行ヘッダ）。"""
    if text.startswith("﻿"):
        text = text[1:]
    lines = [ln.strip() for ln in text.splitlines() if ln.strip()]
    if not lines:
        raise ValueError("空のファイルです")
    fs = FS
    elec = arr = None
    start = 0
    kind = "ヘッダなし（256 Hz とみなす）"
    first = _split(lines[0])
    if first and first[0].lower() == "ecg":
        # 「ECG,256」の1行ヘッダにも対応
        if len(first) > 1 and _num(first[1]) is not None:
            fs = _num(first[1])
            start = 1
        else:
            fs = _num(_split(lines[1])[0]) if len(lines) > 1 else None
            start = 2
        if not fs or fs <= 0:
            raise ValueError("2行目のサンプル周波数が読めません")
        rest = [(_split(ln) or [""])[0] for ln in lines[start:start + 2]]
        st = [_status_int(t) for t in rest]
        if fmt == "4" or (fmt == "auto" and len(st) == 2 and None not in st):
            if len(st) < 2 or None in st:
                raise ValueError("3行目（電極状態）・4行目（不整脈）が読めません")
            elec, arr = st
            start += 2
            kind = "ECG / %d Hz・4行ヘッダ" % fs
        else:
            kind = "ECG / %d Hz・2行ヘッダ" % fs
    samples = []
    for ln in lines[start:]:
        v = _num((_split(ln) or [""])[0])
        if v is not None:
            samples.append(int(round(v)))
    if len(samples) < FS:
        raise ValueError("サンプルが少なすぎます（%d 点）" % len(samples))
    return {"samples": samples, "fs": fs, "elec": elec, "arr": arr, "hdr": kind}


def build_csv(samples, fs, elec, arr):
    """4行ヘッダ形式の CSV テキストを作る。"""
    head = ["ECG", str(int(fs)), str(int(elec)), str(int(arr))]
    return "\n".join(head + [str(int(v)) for v in samples]) + "\n"


# ===== 解析 =====
def resample(samples, fs):
    if fs == FS:
        return list(samples)
    m = int(len(samples) * FS / fs)
    o = []
    for i in range(m):
        p = i * fs / FS
        a = int(p)
        b = min(a + 1, len(samples) - 1)
        f = p - a
        o.append(_js_round(samples[a] * (1 - f) + samples[b] * f))
    return o


def analyze(d):
    """parse_csv の結果を解析し、表示に必要な値をまとめて返す。"""
    samples = resample(d["samples"], d.get("fs") or FS)
    lead_iv = merge_iv(detect_lead_off(samples) + list(d.get("lead_off") or []))

    def in_lead(t):
        return any(a - 0.5 <= t <= b + 0.5 for a, b in lead_iv)
    beats = detect(samples, d.get("median") or 0)
    # 電極はがれの区間とその直後の最初のR-Rは不整脈判定から除外（保留）
    after = False
    for b in beats:
        b["lead"] = False
        t = b["idx"] / FS
        if in_lead(t):
            b["lead"] = True
            after = True
            continue
        if after:
            b["lead"] = True
            after = False
    judged = [b for b in beats if b["judged"] and b["rr"] > 0 and not b["lead"]]
    abn = [b for b in judged if b["flag"] != NONE]
    rrs = [b["rr"] for b in beats if b["rr"] > 0 and not b["lead"]]
    hr = _js_round(60.0 * FS / (sum(rrs) / float(len(rrs)))) if rrs else None
    sec = len(samples) / float(FS)
    lead_now = any(b >= sec - 0.6 for a, b in lead_iv)
    return {"samples": samples, "beats": beats, "lead_iv": lead_iv, "judged": judged,
            "abn": abn, "hr": hr, "sec": sec, "lead_now": lead_now}


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
BLE_VERSION = "2026-10-25 r16"     # 起動時に表示（どのファイルが動いているか確かめるため）
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
    _trace("objc %s" % sel)
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
#   1つ処理するごとに ecg_ble_trace_<sender|receiver>.txt へ書いてすぐ保存する。
#   Pythonista が落ちてもファイルは残るので、最後の行が「落ちる直前の処理」になる。
#   保存先はこのスクリプトと同じフォルダー（「ファイル」アプリで見つけやすい）。前回の記録は _prev.txt に残す。
#   ただし「ファイル」アプリから開いたスクリプトは、そのフォルダーに書き込めない（PermissionError）ので、
#   そのときは Pythonista 自身の Documents に置く。
_OLD_TRACE_DIR = os.path.expanduser("~/Documents")   # r10〜r14 の保存先


def _trace_dir():
    try:
        dirs = [os.path.dirname(os.path.abspath(__file__))]
    except NameError:
        dirs = []
    for d in dirs + [_OLD_TRACE_DIR]:
        try:
            probe = os.path.join(d, ".ecg_ble_write_test")
            with open(probe, "w"):
                pass
            os.remove(probe)
            return d
        except OSError:
            continue
    return _OLD_TRACE_DIR


TRACE_DIR = _trace_dir()
_TRACE = {"path": None}


def _trace_last(role, n=25):
    """前回の動作記録の最後の n 行（前回落ちた場所を画面で見るため）。新しい方の記録を使う。"""
    found = []
    for d in (TRACE_DIR, _OLD_TRACE_DIR):
        path = os.path.join(d, "ecg_ble_trace_%s.txt" % role)
        try:
            found.append((os.path.getmtime(path), path))
        except OSError:
            pass
    if not found:
        return []
    try:
        with open(max(found)[1], encoding="utf-8") as f:
            return f.read().splitlines()[-n:]
    except Exception:
        return []


def show_previous_trace(role, log):
    """前回の動作記録を画面に出す。BLE を始める前に呼ぶ（始めてすぐ落ちても見えるように）。"""
    last = _trace_last(role)
    if last:
        log("―― 前回の動作記録（最後の %d 行。前回落ちた場合は最後の行が落ちる直前の処理）――" % len(last))
        for ln in last:
            log("  前回 " + ln)
        log("―― ここまで ――")


def _trace_start(role):
    _TRACE["path"] = os.path.join(TRACE_DIR, "ecg_ble_trace_%s.txt" % role)
    try:
        if os.path.exists(_TRACE["path"]):
            os.replace(_TRACE["path"], os.path.join(TRACE_DIR, "ecg_ble_trace_%s_prev.txt" % role))
    except OSError:
        pass
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

import types as _types
core = _types.SimpleNamespace(**{k: v for k, v in dict(globals()).items() if not k.startswith("__")})
# ==================== 共通処理ここまで ====================

try:
    import dialogs
except ImportError:
    dialogs = None

DOC_TYPES = ["public.comma-separated-values-text", "public.plain-text", "public.text", "public.data"]


def in_thread(fn):
    """Pythonista の in_background の代わり。main() が wait_modal() でスクリプトを止めている間も
    動くよう、ボタンの処理（ファイル選択・送信など）を別スレッドで実行する。"""
    import functools
    import threading

    @functools.wraps(fn)
    def run(*args, **kwargs):
        threading.Thread(target=fn, args=args, kwargs=kwargs, daemon=True).start()
    return run


def prepare(name, raw, fmt="auto"):
    """元ファイルを読み、送信する4行ヘッダ CSV の材料を返す。"""
    d = core.parse_csv(raw.decode("utf-8-sig", "replace"), fmt)
    r = core.analyze(d)
    calc_elec = len(r["lead_iv"])      # 電極はがれ区間の数（0＝正常）
    calc_arr = len(r["abn"])           # 早期拍・休止の数（0＝なし）
    return {"name": name, "d": d, "r": r,
            "elec": d["elec"] if d["elec"] is not None else calc_elec,
            "arr": d["arr"] if d["arr"] is not None else calc_arr,
            "calc": (calc_elec, calc_arr)}


class Sender(ui.View):
    def __init__(self):
        self.background_color = "#f4f5f7"
        self.src = None
        self.title_l = ui.Label(text="ECG CSV 送信（iPad → iPhone）", font=("<system-bold>", 20))
        self.file_b = self._btn("① CSVを選ぶ（ダウンロード）", self.pick, True)
        self.file_l = ui.Label(text="ファイル未選択", font=("<system>", 14), text_color="#5d6772",
                               number_of_lines=0)
        self.elec_l = ui.Label(text="3行目 電極状態（0＝正常）", font=("<system>", 14))
        self.elec_t = ui.TextField(text="", keyboard_type=ui.KEYBOARD_NUMBER_PAD, alignment=ui.ALIGN_CENTER)
        self.arr_l = ui.Label(text="4行目 不整脈の件数（0＝なし）", font=("<system>", 14))
        self.arr_t = ui.TextField(text="", keyboard_type=ui.KEYBOARD_NUMBER_PAD, alignment=ui.ALIGN_CENTER)
        self.ble_l = ui.Label(text="送信先：iPhone（BLE で自動接続）", font=("<system>", 14), text_color="#5d6772")
        self.send_b = self._btn("② 送信（BLE）", self.send, True)
        self.log_v = ui.TextView(editable=False, font=("Menlo", 12), background_color="#ffffff",
                                 border_width=1, border_color="#d9dde2", corner_radius=8)
        for v in (self.title_l, self.file_b, self.file_l, self.elec_l, self.elec_t, self.arr_l, self.arr_t,
                  self.ble_l, self.send_b, self.log_v):
            self.add_subview(v)
        self.close_b = self._btn("閉じる", lambda s: self.close())
        self.add_subview(self.close_b)
        self.log("iPhone で ecg_viewer_iphone.py を起動しておいてください（BLE で自動接続します）")
        self.ble = core.BleSender(self.log, self.on_done)
        self.closed = None
        core.show_previous_trace("sender", self.log)
        self.log("3 秒後に BLE を開始します（記録の保存先: %s）" % core.TRACE_DIR)
        ui.delay(self.start_ble, 3)   # BLE はメインスレッドで開始する。前回の記録が画面に出てから始める

    def start_ble(self):
        try:
            self.ble.start()
        except Exception as e:
            self.log("BLE を開始できません: %s" % e)

    def _btn(self, t, act, pri=False):
        b = ui.Button(title=t, action=act, font=("<system-bold>", 15), corner_radius=9, border_width=1,
                      border_color="#1f2d3d" if pri else "#d9dde2",
                      background_color="#1f2d3d" if pri else "#ffffff",
                      tint_color="#ffffff" if pri else "#1b1f24")
        return b

    def layout(self):
        w = self.width
        m = 24
        cw = min(w - 2 * m, 640)
        x = (w - cw) / 2.0
        y = 40
        self.title_l.frame = (x, y, cw - 90, 32)
        self.close_b.frame = (x + cw - 80, y, 80, 32)
        y += 48
        self.file_b.frame = (x, y, cw, 44)
        y += 52
        self.file_l.frame = (x, y, cw, 58)
        y += 64
        for lab, tf in ((self.elec_l, self.elec_t), (self.arr_l, self.arr_t)):
            lab.frame = (x, y, cw - 110, 36)
            tf.frame = (x + cw - 100, y, 100, 36)
            y += 44
        self.ble_l.frame = (x, y, cw, 36)
        y += 44
        self.send_b.frame = (x, y, cw, 48)
        y += 60
        self.log_v.frame = (x, y, cw, max(80, self.height - y - 30))

    def log(self, m):
        def go():
            self.log_v.text = (self.log_v.text + "\n" if self.log_v.text else "") + m
        ui.delay(go, 0)

    @in_thread
    def pick(self, sender=None):
        if dialogs is None:
            return
        path = dialogs.pick_document(types=DOC_TYPES)
        if path:
            import functools
            ui.delay(functools.partial(self.load, path), 0)   # 読み込みはメインスレッドで

    def load(self, path, name=None):
        name = name or os.path.basename(path)
        try:
            with open(path, "rb") as f:
                self.src = prepare(name, f.read())
        except Exception as e:
            self.src = None
            self.log("読み込みに失敗（%s）: %s" % (name, e))
            return
        s = self.src
        d, r = s["d"], s["r"]
        self.file_l.text = "%s\n%s・%.1f 秒 / %d 点・解析：電極はがれ %d 区間、不整脈 %d 件" % (
            name, d["hdr"], r["sec"], len(r["samples"]), s["calc"][0], s["calc"][1])
        self.elec_t.text = str(s["elec"])
        self.arr_t.text = str(s["arr"])
        self.log("選択: %s（%s）" % (name, d["hdr"]))

    def send(self, sender=None):
        # ボタンの action はメインスレッドで動く。BLE の操作をするので別スレッドにしない
        if not self.src:
            self.log("先に CSV を選んでください")
            return
        try:
            elec, arr = int(self.elec_t.text or 0), int(self.arr_t.text or 0)
            if elec < 0 or arr < 0:
                raise ValueError
        except ValueError:
            self.log("3行目・4行目は 0 以上の整数で入力してください")
            return
        d = self.src["d"]
        body = core.build_csv(d["samples"], d["fs"], elec, arr).encode("utf-8")
        self.log("送信準備… %s（%d バイト）" % (self.src["name"], len(body)))
        try:
            self.ble.send(self.src["name"], body, "4")
        except Exception as e:
            self.log("送信に失敗: %s" % e)
            return
        self.send_b.enabled = False
        self.send_b.alpha = 0.5

    def on_done(self, ok, msg):
        core._trace("ui: on_done %s" % ok)
        if ok:
            self.log("送信しました。iPhone 側の保存名: %s" % msg)
        else:
            self.log("送信に失敗: %s" % msg)

        def go():
            core._trace("ui: enable button")
            self.send_b.enabled = True
            self.send_b.alpha = 1.0
        ui.delay(go, 0)

    def will_close(self):
        self.ble.stop()
        if self.closed is not None:
            self.closed.set()


def main():
    import threading
    v = Sender()
    v.closed = threading.Event()
    v.present("fullscreen", hide_title_bar=True)
    # 画面を閉じるまでスクリプトを終わらせない（終わると Pythonista が変数を片付け、BLE の通知で
    # NameError になる）。待つ間は Python のコードを一切動かさない Event.wait() にする。
    # wait_modal() などで待つ間に Python が動くと、メインスレッドの objc_util と同時に動いて
    # ArgumentError やクラッシュの原因になる。
    v.closed.wait()
    return v


if __name__ == "__main__":
    main()

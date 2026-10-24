# -*- coding: utf-8 -*-
"""ECG R-R ビューア（iPhone：Pythonista 3 版）

ecg_viewer.html（改修版）と同じ画面・動作を Pythonista 3 だけで実現する（仕様書 6.2）。
  縦向き : データ検出状態（灰色「データ未検出」→ 緑色「データ検出」）と横向きの案内
  横向き : 波形・電極状態・異常検知（6.2.2）

データの受け取り方
  1) iPad の送信プログラム（ecg_sender_ipad.py）から BLE で受信（自動で「データ検出」になる）
     iPad をサービス UUID で探して自動で接続する（Wi-Fi・IP アドレスの設定は不要）
     受信した CSV は Pythonista 3 の Downloads フォルダ（ファイルApp：このiPhone内 > Pythonista 3 > Downloads）に保存
  2) 「ファイル」ボタン → ファイルApp から選ぶ（AirDrop で受け取った「ダウンロード」フォルダ内の CSV など）

CSV 形式：1行目 ECG／2行目 サンプル周波数（256）／3行目 電極状態（0＝正常）／4行目 不整脈の件数（0＝なし）／5行目以降 心電データ
このファイル1つで動く（共通処理 ecg_core.py を埋め込み済み。build.py で生成）。
"""
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
    if text.startswith("\ufeff"):
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
#   ※ iPad（送信）は CoreBluetooth の操作を専用の作業スレッドで行い、iOS からの通知（メインスレッド）は
#     値を取り出して作業スレッドへ渡すだけにする（ble_diag_ipad.py で、iPhone が接続しても落ちなかった形）。
#     iPhone（受信）は CoreBluetooth の操作をメインスレッドで行う（デリゲート・ui.delay の予約）。
import queue
import threading
import zlib

SERVICE_UUID = "8F1D0001-2C5A-4E7B-9A63-5E0C6A1B2D01"
DATA_UUID = "8F1D0002-2C5A-4E7B-9A63-5E0C6A1B2D01"     # iPad → iPhone（Notify）
ACK_UUID = "8F1D0003-2C5A-4E7B-9A63-5E0C6A1B2D01"      # iPhone → iPad（Write）
BLE_NAME = "ECG-TX"
BLE_VERSION = "2026-10-25 r17"     # 起動時に表示（どのファイルが動いているか確かめるため）
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


def _fault_path(role):
    return os.path.join(TRACE_DIR, "ecg_ble_fault_%s.txt" % role)


def show_previous_trace(role, log):
    """前回の動作記録を画面に出す。BLE を始める前に呼ぶ（始めてすぐ落ちても見えるように）。"""
    last = _trace_last(role)
    if last:
        log("―― 前回の動作記録（最後の %d 行。前回落ちた場合は最後の行が落ちる直前の処理）――" % len(last))
        for ln in last:
            log("  前回 " + ln)
        log("―― ここまで ――")
    try:
        with open(_fault_path(role), encoding="utf-8", errors="replace") as f:
            fault = [ln for ln in f.read().splitlines() if ln.strip()]
    except OSError:
        fault = []
    if fault:
        log("―― 前回落ちた時の Python の位置（このスクリーンショットも送ってください）――")
        for ln in fault[:40]:
            log("  " + ln)
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
    # iOS の中で落ちた時に、その時動いていた Python の場所（全スレッド）をファイルに残す
    try:
        import faulthandler
        if _TRACE.get("fault"):
            faulthandler.disable()
            _TRACE["fault"].close()
        _TRACE["fault"] = open(_fault_path(role), "w")
        faulthandler.enable(_TRACE["fault"], all_threads=True)
    except Exception:
        _TRACE["fault"] = None


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
    _dispatch("sender", "_cb_state", pm)


def peripheralManager_didAddService_error_(_self, _cmd, pm, svc, err):
    _dispatch("sender", "_cb_added", err)


def peripheralManagerDidStartAdvertising_error_(_self, _cmd, pm, err):
    _dispatch("sender", "_cb_advertising", err)


def peripheralManager_central_didSubscribeToCharacteristic_(_self, _cmd, pm, central, ch):
    _dispatch("sender", "_cb_subscribed", central, ch)


def peripheralManager_central_didUnsubscribeFromCharacteristic_(_self, _cmd, pm, central, ch):
    _dispatch("sender", "_cb_unsubscribed")


def peripheralManagerIsReadyToUpdateSubscribers_(_self, _cmd, pm):
    _dispatch("sender", "_cb_ready")


def peripheralManager_didReceiveWriteRequests_(_self, _cmd, pm, reqs):
    _dispatch("sender", "_cb_writes", reqs)


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
    on_log・on_done は作業スレッドから呼ばれる（画面の部品は直接さわらないこと）。

    CoreBluetooth の操作は専用の作業スレッドで行う。iOS からの通知（メインスレッド）は、必要な値を
    取り出して作業スレッドへ渡すだけにする。iPad の診断（ble_diag_ipad.py）はこの形で、iPhone が
    接続しても落ちなかった。r16 まではメインスレッドで全部行い、iPhone の接続時に iPad が落ちた。
    threaded=False にすると、作業スレッドを使わずその場で処理する（PC での確認用）。
    """

    def __init__(self, on_log=None, on_done=None, threaded=True):
        self._set_log(on_log)
        self.on_done = on_done or (lambda ok, msg: None)
        self.pm = None
        self.delegate = None
        self.ready = False
        self.data_char = None
        self._ack_char = None    # サービスと特性は iOS が使い続けるので、閉じるまで手放さない
        self._svc = None
        self.job = None
        self._deadline = None    # この時刻までに終わらなければ失敗にする
        self._central = None     # 接続して通知を待っている iPhone
        self._pct = 0
        self._threaded = threaded
        self._events = queue.Queue()
        self._thread = None

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
        self._deadline = None
        if self.pm:
            _send(self.pm, "stopAdvertising", "v")
            _trace("finish: stopAdvertising done")
        self.on_log("送信を終了しました（%s）。次の「送信」を待っています" % ("成功" if ok else "失敗"))
        self.on_done(ok, msg)
        _trace("finish: on_done done")

    # --- 作業スレッド
    def start(self):
        """BLE を始める（画面から呼ぶ）。"""
        _trace_start("sender")
        self.on_log("BLE 送信を開始します（版 %s）" % BLE_VERSION)
        _ACTIVE["sender"] = self
        if self._threaded:
            self._thread = threading.Thread(target=self._run, name="ECGBleSender")
            self._thread.daemon = True
            self._thread.start()
        else:
            self._post("open")

    def _run(self):
        _trace("worker: start")
        self._try(self._handle, ("open",))
        while True:
            try:
                ev = self._events.get(timeout=0.5)
            except queue.Empty:
                ev = ("tick",)
            self._try(self._handle, ev)
            if ev[0] == "close":
                break
        _trace("worker: end")

    def _post(self, *ev):
        """作業スレッドに処理を頼む。"""
        if self._threaded:
            self._events.put(ev)
        else:
            self._try(self._handle, ev)

    def _handle(self, ev):
        if ev[0] != "tick":
            _trace("work %s" % ev[0])
        getattr(self, "_do_" + ev[0])(*ev[1:])

    def stop(self):
        """画面を閉じる時に呼ぶ。作業スレッドで後片付けして、終わるのを少し待つ。"""
        self.job = None
        self._post("close")
        if self._thread is not None:
            self._thread.join(3)
            self._thread = None
        if _ACTIVE.get("sender") is self:
            del _ACTIVE["sender"]

    def send(self, name, data, fmt="4", timeout=SEND_TIMEOUT):
        """画面の「送信」から呼ぶ。実際の送信は作業スレッドで行う。"""
        if self.job:
            raise RuntimeError("送信中です。終わるまでお待ちください")
        if not 0 < len(data) <= MAX_SIZE:
            raise ValueError("サイズが不正です（%d バイト）" % len(data))
        self.job = {"name": name, "data": data, "fmt": fmt, "timeout": timeout}
        self._post("send")

    # --- iOS からの通知（メインスレッド）。値を取り出して作業スレッドへ渡すだけにする
    def _cb_state(self, pm):
        self._post("state", _send(pm, "state", "q"))

    def _cb_added(self, err):
        self._post("added", (_err_text(err) or "エラー") if err else None)

    def _cb_advertising(self, err):
        self._post("advertising", (_err_text(err) or "エラー") if err else None)

    def _cb_subscribed(self, central, ch):
        if _uuid_of(ch) == DATA_UUID:
            self._post("subscribed", _retain(central))   # 作業スレッドが受け取るまで手放さない

    def _cb_unsubscribed(self):
        self._post("unsubscribed")

    def _cb_ready(self):
        self._post("ready")

    def _cb_writes(self, reqs):
        """書き込み（iPhone からの応答）は、ここで中身を読んで iOS に返事をする（要求は通知の間だけ有効）。"""
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
            self._post("ack", t)

    # --- 作業スレッドで行う処理
    def _do_open(self):
        self.delegate = _delegate("ECGBleSender", _SENDER_METHODS, ["CBPeripheralManagerDelegate"])
        self.pm = _new("CBPeripheralManager", "initWithDelegate:queue:", "@@@", _ptr_of(self.delegate), None)

    def _do_close(self):
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

    def _do_send(self):
        if not self.job:
            return
        self._deadline = time.time() + self.job["timeout"]
        if not self.pm or not self.ready:
            self.on_log("Bluetooth の準備を待っています…")
        else:
            self._kick()

    def _kick(self):
        if self._central:   # iPhone がもう接続して待っている
            self._start_job(self._central)
        else:
            self._advertise()

    def _do_tick(self):
        job = self.job
        if job is None or self._deadline is None or time.time() < self._deadline:
            return
        sec = job["timeout"]
        if "packets" in job:
            self.finish(False, "iPhone から受信の確認が届きませんでした（%d 秒）" % sec)
        else:
            self.finish(False, "iPhone が見つかりませんでした（%d 秒）。iPhone で ecg_viewer_iphone.py が"
                               "起動しているか確認してください" % sec)

    def _start_job(self, central):
        if not self.job or "packets" in self.job:
            return
        try:
            self.begin(int(_send(central, "maximumUpdateValueLength", "Q")))
        except Exception as e:
            self.finish(False, str(e))
            return
        self.pump(self._notify)

    def _do_state(self, st):
        if not self.pm:
            return
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
        _send(self.pm, "removeAllServices", "v")
        _send(self.pm, "addService:", "v@", svc)
        # 登録したサービス・特性は手放さない（r12 では登録直後に手放したため、iPhone が中身を
        # 確かめに来た時に解放済みになり、サービスが見えないうえ iPad が落ちた）
        self.data_char, self._ack_char, self._svc = data_char, ack_char, svc
        for p in old:   # 前に登録していたものは removeAllServices で外したので手放してよい
            _release(p)

    def _do_added(self, err):
        if err:
            self.on_log("BLE サービスの登録に失敗: %s" % err)
            return
        self.ready = True
        self.on_log("BLE の準備ができました")
        if self.job:
            self._kick()

    def _advertise(self):
        _send(self.pm, "stopAdvertising", "v")
        adv = _nsdict([(_nsstr("kCBAdvDataLocalName"), _nsstr(BLE_NAME)),
                       (_nsstr("kCBAdvDataServiceUUIDs"), _nsarray([_uuid(SERVICE_UUID)]))])
        _send(self.pm, "startAdvertising:", "v@", adv)

    def _do_advertising(self, err):
        if err:
            if self.job:
                self.finish(False, "アドバタイズを開始できません: %s" % err)
            return
        self.on_log("iPhone の接続を待っています…（BLE）")

    def _do_subscribed(self, central):
        _release(self._central)
        self._central = central   # _cb_subscribed で retain 済み
        if not self.pm:
            return
        if self.job:
            self._start_job(central)
        else:
            self.on_log("iPhone が接続しました。「送信」を押すと送ります")

    def _do_unsubscribed(self):
        _release(self._central)
        self._central = None
        if self.job and "packets" in self.job:
            self.finish(False, "送信の途中で iPhone との接続が切れました")

    def _do_ready(self):
        self.pump(self._notify)

    def _do_ack(self, text):
        self.ack(text)

    def _notify(self, pkt):
        return bool(_send(self.pm, "updateValue:forCharacteristic:onSubscribedCentrals:", "B@@@",
                          _nsdata(pkt), self.data_char, None))


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
except ImportError:  # PC でのシミュレーション時
    dialogs = None
try:
    import console
except ImportError:
    console = None

FS = core.FS
SAVE_DIR = os.environ.get("ECG_SAVE_DIR") or os.path.join(os.path.expanduser("~/Documents"), "Downloads")
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

LIGHT = dict(bg="#f4f5f7", panel="#ffffff", ink="#1b1f24", sub="#5d6772", line="#d9dde2",
             grid="#f0d4d4", grid2="#e3b8b8", wave="#14202b", ok="#1a8a4a", early="#d9730d",
             pause="#2563c9", warn="#c62828", lead="#8a96a3", btn="#1f2d3d", btnink="#ffffff",
             sel="#dfe8f8", chip="#f4f5f7")
DARK = dict(bg="#0d1014", panel="#161b21", ink="#e8ecf0", sub="#98a3ae", line="#2a323b",
            grid="#2a1c1e", grid2="#432a2d", wave="#d9f2e3", ok="#4cd488", early="#ffa24d",
            pause="#6fa8ff", warn="#ff6b6b", lead="#7d8c9b", btn="#e8ecf0", btnink="#0d1014",
            sel="#1d2a3d", chip="#0d1014")


def theme():
    try:
        return DARK if ui.get_ui_style() == "dark" else LIGHT
    except Exception:
        return LIGHT


C = theme()
F = "<system>"
FB = "<system-bold>"


def rgba(hexcol, a):
    h = hexcol.lstrip("#")
    return (int(h[0:2], 16) / 255.0, int(h[2:4], 16) / 255.0, int(h[4:6], 16) / 255.0, a)


def text(s, x, y, w, h, size=12, color=None, bold=False, align=ui.ALIGN_LEFT):
    ui.draw_string(s, rect=(x, y, w, h), font=(FB if bold else F, size),
                   color=color or C["ink"], alignment=align, line_break_mode=ui.LB_TRUNCATE_TAIL)


def rrect(x, y, w, h, r, fill=None, stroke=None, lw=1.0, dash=None):
    p = ui.Path.rounded_rect(x, y, w, h, r)
    if fill:
        ui.set_color(fill)
        p.fill()
    if stroke:
        ui.set_color(stroke)
        p.line_width = lw
        if dash:
            p.set_line_dash(dash)
        p.stroke()


def dot(cx, cy, r, color):
    ui.set_color(color)
    ui.Path.oval(cx - r, cy - r, 2 * r, 2 * r).fill()


def line(x0, y0, x1, y1, color, lw=1.0, dash=None):
    p = ui.Path()
    p.move_to(x0, y0)
    p.line_to(x1, y1)
    p.line_width = lw
    if dash:
        p.set_line_dash(dash)
    ui.set_color(color)
    p.stroke()


def make_button(title, action, primary=False, size=14):
    b = ui.Button(title=title)
    b.action = action
    b.font = (FB, size)
    b.corner_radius = 9
    b.border_width = 1
    b.border_color = C["btn"] if primary else C["line"]
    b.background_color = C["btn"] if primary else C["panel"]
    b.tint_color = C["btnink"] if primary else C["ink"]
    return b


def beat_color(b):
    if b["lead"]:
        return C["lead"]
    if not b["judged"]:
        return C["sub"]
    return C["early"] if b["flag"] == core.EARLY else C["pause"] if b["flag"] == core.PAUSE else C["ok"]


# ---------------------------------------------------------------- 縦向きの部品
class StatusCard(ui.View):
    """「データ未検出」（灰）／「データ検出」（緑）の枠。"""

    def __init__(self, app):
        self.app = app
        self.background_color = "clear"

    def draw(self):
        a, w, h = self.app, self.width, self.height
        on = a.cur is not None
        col = C["ok"] if on else C["lead"]
        rrect(1, 1, w - 2, h - 2, 14, fill=C["panel"], stroke=col if on else C["line"], lw=2)
        big = "データ検出" if on else "データ未検出"
        text(big, 30, 18, w - 60, 32, 24, C["ok"] if on else C["ink"], True, ui.ALIGN_CENTER)
        tw = ui.measure_string(big, font=(FB, 24))[0]
        dot(w / 2 - tw / 2 - 14, 34, 7, col)
        for i, s in enumerate(a.info_lines()[:3]):
            text(s, 12, 58 + i * 18, w - 24, 18, 13, C["sub"], align=ui.ALIGN_CENTER)


class Chip(ui.View):
    def __init__(self, caption):
        self.caption = caption
        self.value = "–"
        self.color = None
        self.background_color = "clear"

    def set(self, value, color=None):
        self.value, self.color = value, color
        self.set_needs_display()

    def draw(self):
        w, h = self.width, self.height
        rrect(0.5, 0.5, w - 1, h - 1, 10, fill=C["panel"], stroke=C["line"])
        text(self.value, 4, 10, w - 8, 22, 16, self.color or C["ink"], True, ui.ALIGN_CENTER)
        text(self.caption, 4, 36, w - 8, 16, 11, C["sub"], align=ui.ALIGN_CENTER)


class RotateBox(ui.View):
    def __init__(self, app):
        self.app = app
        self.background_color = "clear"

    def draw(self):
        w, h = self.width, self.height
        on = self.app.cur is not None
        if on:
            rrect(0, 0, w, h, 14, fill=C["btn"])
        else:
            rrect(1, 1, w - 2, h - 2, 14, fill=C["panel"], stroke=C["line"], dash=[4, 3])
        col = C["btnink"] if on else C["sub"]
        text("⟳", 0, 16, w, 44, 36, col, True, ui.ALIGN_CENTER)
        msg = "iPhoneを横向きにしてください" if on else "データを読み込むと表示できます"
        text(msg, 8, 66, w - 16, 26, 17, col, True, ui.ALIGN_CENTER)


# ---------------------------------------------------------------- 横向きの部品
class Badge(ui.View):
    def __init__(self):
        self.label = "–"
        self.color = C["lead"]
        self.background_color = "clear"

    def set(self, color, label):
        self.color, self.label = color, label
        self.width = ui.measure_string(label, font=(FB, 11.5))[0] + 30
        self.set_needs_display()

    def draw(self):
        w, h = self.width, self.height
        rrect(0.5, 0.5, w - 1, h - 1, h / 2 - 0.5, fill=C["panel"], stroke=self.color, lw=1.2)
        dot(12, h / 2, 4.5, self.color)
        text(self.label, 21, h / 2 - 8, w - 24, 16, 11.5, C["ink"], True)


class PlotView(ui.View):
    """心電図用紙の格子・波形・R波の印と R-R（ms）・電極はがれの網掛け。"""

    def __init__(self, app):
        self.app = app
        self.background_color = "clear"

    def draw(self):
        a, w, h = self.app, self.width, self.height
        rrect(0.5, 0.5, w - 1, h - 1, 10, fill=C["panel"], stroke=C["line"])
        cur = a.cur
        if cur is None:
            text("データ未検出：受信を待つか「ファイル」「ダミー」を押してください", 10, h / 2 - 10, w - 20, 20, 14,
                 C["sub"], align=ui.ALIGN_CENTER)
            return
        s, beats = cur["samples"], cur["beats"]
        n, sec = len(s), cur["sec"]
        L, R, T, B = 34.0, 8.0, 26.0, 24.0
        pw, ph = w - L - R, h - T - B
        mn, mx = min(s), max(s)
        pad = (mx - mn) * 0.12 or 1
        mn -= pad
        mx += pad

        def X(i):
            return L + pw * i / (n - 1)

        def Y(v):
            return T + ph * (1 - (v - mn) / (mx - mn))
        # 格子（細線0.2秒・太線1秒）
        k = 0
        while k * 0.2 <= sec + 1e-6:
            x = L + pw * (k * 0.2) / sec
            line(x, T, x, T + ph, C["grid2"] if k % 5 == 0 else C["grid"], 1 if k % 5 == 0 else 0.6)
            k += 1
        import math
        step_y = 10 ** math.floor(math.log10((mx - mn) / 5))
        v = math.ceil(mn / step_y) * step_y
        while v < mx:
            y = Y(v)
            line(L, y, L + pw, y, C["grid"], 0.6)
            v += step_y
        # 電極はがれ区間
        for a0, b0 in cur["lead_iv"]:
            x0, x1 = L + pw * a0 / sec, L + pw * min(b0, sec) / sec
            ui.set_color(rgba(C["lead"], 0.22))
            ui.Path.rect(x0, T, x1 - x0, ph).fill()
            text("電極はがれ", x0, T + ph / 2 - 8, x1 - x0, 16, 11, C["warn"], True, ui.ALIGN_CENTER)
        # 時間軸
        tick = 2 if sec > 12 else 1
        t = 0
        while t <= sec + 1e-6:
            text("%gs" % t, L + pw * t / sec - 20, h - 18, 40, 14, 11, C["sub"], align=ui.ALIGN_CENTER)
            t += tick
        # 波形（画面幅にあわせて間引く）
        step = max(1, int(n / (pw * 2)))
        p = ui.Path()
        p.move_to(X(0), Y(s[0]))
        for i in range(step, n, step):
            p.line_to(X(i), Y(s[i]))
        p.line_width = 1.2
        p.line_join_style = ui.LINE_JOIN_ROUND
        ui.set_color(C["wave"])
        p.stroke()
        # R波の印と R-R
        for i, b in enumerate(beats):
            col = beat_color(b)
            lo, hi = max(0, b["idx"] - 14), min(n - 1, b["idx"] + 2)
            best = max(range(lo, hi + 1), key=lambda q: (s[q], -q))
            x, y = X(best), Y(s[best])
            sel = i == a.selected
            dot(x, y - 7, 6 if sel else 4, col)
            if b["rr"] and not b["lead"]:
                text(str(core._js_round(b["rr"] * 1000.0 / FS)), x - 20, max(T - 16, y - 26), 40, 14, 11, col, True,
                     ui.ALIGN_CENTER)
            if sel:
                line(x, T, x, T + ph, col, 1.5, [3, 3])
        # 凡例・メッセージ
        lx = 8
        rrect(6, 5, 262, 18, 6, fill=rgba(C["panel"][:7], 0.85))
        for name, col in (("通常", C["ok"]), ("早期拍", C["early"]), ("休止", C["pause"]), ("電極はがれ", C["lead"])):
            dot(lx + 6, 14, 4, col)
            tw = ui.measure_string(name, font=(F, 11))[0]
            text(name, lx + 13, 7, tw + 4, 14, 11, C["sub"])
            lx += tw + 24
        if a.msg:
            text(a.msg, 8, 30, w - 16, 16, 12, C["warn"], align=ui.ALIGN_CENTER)


class SidePanel(ui.View):
    """判定・拍数/心拍/異常判定・異常検知一覧・拍の一覧ヘッダ（一覧本体は BeatList）。"""

    VH = 92  # 判定エリアの高さ

    def __init__(self, app):
        self.app = app
        self.background_color = "clear"

    def alerts(self):
        cur = self.app.cur
        al = []
        if cur is None:
            return al
        for a0, b0 in cur["lead_iv"]:
            al.append((a0, "電極はがれ", C["warn"], "%.1f–%.1f s" % (a0, b0)))
        for b in cur["abn"]:
            al.append((b["idx"] / FS, "不整脈疑い（%s）" % core.FLAGNAME[b["flag"]],
                       C["early"] if b["flag"] == core.EARLY else C["pause"], "%.1f s" % (b["idx"] / FS)))
        al.sort(key=lambda q: q[0])
        return al

    def alert_h(self):
        return 26 + max(1, min(4, len(self.alerts()))) * 18

    def draw(self):
        a, w, h = self.app, self.width, self.height
        rrect(0.5, 0.5, w - 1, h - 1, 10, fill=C["panel"], stroke=C["line"])
        v = a.verdict()
        text(v[0], 10, 8, w - 20, 20, 15, v[1], True)
        text(v[2], 10, 29, w - 20, 16, 12, C["sub"])
        cw = (w - 20 - 8) / 3.0
        for i, (val, cap) in enumerate(a.side_chips()):
            x = 10 + i * (cw + 4)
            rrect(x, 49, cw, 36, 6, fill=C["chip"])
            text(val, x, 51, cw, 18, 15, C["ink"], True, ui.ALIGN_CENTER)
            text(cap, x, 69, cw, 13, 10, C["sub"], align=ui.ALIGN_CENTER)
        y = self.VH
        line(0, y, w, y, C["line"])
        text("異常検知", 10, y + 5, w - 20, 14, 11, C["sub"], True)
        al = self.alerts()
        yy = y + 22
        if a.cur is None:
            text("—", 10, yy, w - 20, 16, 12, C["sub"])
        elif not al:
            text("異常なし", 10, yy, w - 20, 16, 12, C["ok"], True)
        for i, (_, name, col, tm) in enumerate(al[:4]):
            text(name, 10, yy + i * 18, w - 80, 16, 12, col, True)
            text(tm, w - 80, yy + i * 18, 70, 16, 12, C["ink"], align=ui.ALIGN_RIGHT)
        if len(al) > 4:
            text("ほか %d 件" % (len(al) - 4), w - 80, y + 5, 70, 14, 11, C["sub"], align=ui.ALIGN_RIGHT)
        y += self.alert_h()
        line(0, y, w, y, C["line"])
        text("時刻 s", 8, y + 5, 60, 14, 12, C["sub"], True)
        text("R-R ms", 64, y + 5, 60, 14, 12, C["sub"], True, ui.ALIGN_RIGHT)
        text("判定", w - 98, y + 5, 90, 14, 12, C["sub"], True, ui.ALIGN_RIGHT)
        line(0, y + 23, w, y + 23, C["line"])


class BeatList(ui.View):
    """拍ごとの時刻・R-R・判定。行をタップすると波形上に点線で位置を示す。"""
    RH = 22

    def __init__(self, app):
        self.app = app
        self.background_color = "clear"

    def draw(self):
        a, w = self.app, self.width
        if a.cur is None:
            return
        for i, b in enumerate(a.cur["beats"]):
            y = i * self.RH
            if i == a.selected:
                ui.set_color(C["sel"])
                ui.Path.rect(0, y, w, self.RH).fill()
            rr = str(core._js_round(b["rr"] * 1000.0 / FS)) if b["rr"] else "–"
            if b["lead"]:
                j, col = "保留（電極）", C["lead"]
            elif b["judged"]:
                j = core.FLAGNAME[b["flag"]]
                col = C["early"] if b["flag"] == core.EARLY else C["pause"] if b["flag"] == core.PAUSE else C["ink"]
            else:
                j, col = "（履歴不足）", C["ink"]
            text("%.2f" % (b["idx"] / float(FS)), 8, y + 3, 60, 16, 12)
            text(rr, 64, y + 3, 60, 16, 12, align=ui.ALIGN_RIGHT)
            text(j, w - 98, y + 3, 90, 16, 12, col, col != C["ink"], ui.ALIGN_RIGHT)
            line(0, y + self.RH - 0.5, w, y + self.RH - 0.5, C["line"], 0.5)

    def touch_began(self, touch):
        if self.app.cur is None:
            return
        i = int(touch.location[1] // self.RH)
        if 0 <= i < len(self.app.cur["beats"]):
            self.app.select(i)


# ---------------------------------------------------------------- 本体
class ECGViewer(ui.View):
    def __init__(self):
        self.background_color = C["bg"]
        self.cur = None          # 解析結果（ecg_core.analyze）
        self.meta = None         # name, hdr, elec, arr, path
        self.selected = -1
        self.msg = ""
        self.log = "起動しました"
        self.log_hist = []
        self.landscape = None
        self.receiver = None

        # 縦向き
        self.p = ui.View(background_color=C["bg"])
        self.p_title = ui.Label(text="ECG R-R ビューア", font=(FB, 19), text_color=C["ink"])
        self.p_status = StatusCard(self)
        self.p_chips = [Chip("電極状態"), Chip("不整脈検知"), Chip("心拍/分")]
        self.p_btns = [make_button("ファイル", self.pick_file, True), make_button("ダミー", self.demo),
                       make_button("ダミー（電極はがれ）", self.demo_lead)]
        self.p_btns2 = [make_button("共有", self.share, size=13), make_button("クリア", self.clear, size=13)]
        self.p_rot = RotateBox(self)
        self.p_recv = ui.Label(font=(F, 12), text_color=C["sub"], number_of_lines=0)
        for v in [self.p_title, self.p_status] + self.p_chips + self.p_btns + self.p_btns2 + [self.p_rot, self.p_recv]:
            self.p.add_subview(v)

        # 横向き
        self.l = ui.View(background_color=C["bg"])
        self.l_title = ui.Label(text="ECG R-R ビューア", font=(FB, 14), text_color=C["ink"])
        self.b_data, self.b_elec = Badge(), Badge()
        self.l_src = ui.Label(font=(F, 12), text_color=C["sub"])
        self.l_btns = [make_button("ダミー", self.demo, size=12.5), make_button("はがれ", self.demo_lead, size=12.5),
                       make_button("ファイル", self.pick_file, True, 12.5)]
        self.plot = PlotView(self)
        self.side = SidePanel(self)
        self.scroll = ui.ScrollView(background_color="clear", shows_horizontal_scroll_indicator=False)
        self.blist = BeatList(self)
        self.scroll.add_subview(self.blist)
        for v in [self.l_title, self.b_data, self.b_elec, self.l_src] + self.l_btns + [self.plot, self.side,
                                                                                      self.scroll]:
            self.l.add_subview(v)

        self.add_subview(self.p)
        self.add_subview(self.l)
        self.close_btn = make_button("×", self.close_app, size=16)
        self.add_subview(self.close_btn)
        self.render()

    # ---------- 受信
    def start_receiver(self):
        core.show_previous_trace("receiver", self.on_log)
        self.receiver = core.BleReceiver(SAVE_DIR, self.on_received, self.on_log)
        ui.delay(self.start_ble, 3)   # 前回の記録が画面に出てから始める

    def start_ble(self):
        try:
            self.receiver.start()
            self.on_log("BLE を準備しています…")
        except Exception as e:
            self.on_log("受信を開始できません: %s" % e)

    def on_log(self, m):
        self.log_hist = (self.log_hist + [m])[-4:]   # 直近4件を画面に出す（接続の進み具合が分かるように）
        self.log = "\n".join(self.log_hist)
        ui.delay(self.render_log, 0)

    def on_received(self, path, fmt):
        def go():
            self.load_path(path, fmt)
            if console:
                try:
                    console.hud_alert("データを受信しました", "success", 1.2)
                except Exception:
                    pass
        ui.delay(go, 0)

    def will_close(self):
        if self.receiver:
            self.receiver.stop()
        if getattr(self, "closed", None) is not None:
            self.closed.set()

    def close_app(self, sender=None):
        self.will_close()
        self.close()

    # ---------- 読み込み
    def load_path(self, path, fmt="auto"):
        try:
            with open(path, "rb") as f:
                raw = f.read()
            d = core.parse_csv(raw.decode("utf-8-sig", "replace"), fmt)
            self.set_data(d, os.path.basename(path), path)
        except Exception as e:
            self.msg = "読み込みに失敗: %s" % e
            self.log = "読み込みに失敗（%s）: %s" % (os.path.basename(path), e)
            self.render()

    def set_data(self, d, name, path=None):
        self.cur = core.analyze(d)
        self.meta = {"name": name, "hdr": d.get("hdr", ""), "elec": d.get("elec"), "arr": d.get("arr"),
                     "path": path, "fs": d.get("fs", FS)}
        self.selected = -1
        self.msg = ""
        if path:
            self.log = "表示中: %s" % path.replace(os.path.expanduser("~"), "~")
        self.render()

    @in_thread
    def pick_file(self, sender=None):
        if dialogs is None:
            return
        path = dialogs.pick_document(types=DOC_TYPES)
        if path:
            ui.delay(lambda: self.load_path(path), 0)

    def demo(self, sender=None):
        self.set_data({"samples": core.demo_ecg(20), "fs": FS, "hdr": "ダミー"}, "ダミー（20秒）")

    def demo_lead(self, sender=None):
        self.set_data({"samples": core.demo_ecg(30, [[20, 24]]), "fs": FS, "hdr": "ダミー"},
                      "ダミー・電極はがれ（30秒）")

    def clear(self, sender=None):
        self.cur = self.meta = None
        self.selected = -1
        self.msg = ""
        self.render()

    @in_thread
    def share(self, sender=None):
        """6.3 iPhoneからPCへの転送：共有シート（AirDrop・ファイルAppへ保存 など）。"""
        if not self.meta or console is None:
            return
        path = self.meta.get("path")
        if not path:  # ダミーは CSV にしてから共有
            m = self.meta
            path = os.path.join(SAVE_DIR, "ECG_DUMMY.csv")
            elec = m["elec"] if m["elec"] is not None else len(self.cur["lead_iv"])
            arr = m["arr"] if m["arr"] is not None else len(self.cur["abn"])
            with open(path, "w") as f:
                f.write(core.build_csv(self.cur["samples"], FS, elec, arr))
        console.open_in(path)

    def select(self, i):
        self.selected = i
        self.plot.set_needs_display()
        self.blist.set_needs_display()

    # ---------- 表示内容
    def elec_state(self):
        """電極状態：CSV 3行目（0＝正常）を優先し、波形からの検出（6.2.3）も反映する。"""
        m, cur = self.meta, self.cur
        if cur is None:
            return "–", None, "電極 –"
        code = m.get("elec")
        if code:  # 3行目が 0 以外
            return "異常", C["warn"], "電極 異常（%d）" % code
        if cur["lead_iv"]:
            s = "はがれ" if cur["lead_now"] else "はがれ有"
            return s, C["warn"], "電極" + s
        return "正常", C["ok"], "電極 正常"

    def arr_state(self):
        """不整脈検知：CSV 4行目（0＝なし／それ以外＝件数）を優先。無ければ解析結果。"""
        m, cur = self.meta, self.cur
        if cur is None:
            return "–", None
        n = m.get("arr")
        if n is None:
            if not cur["judged"]:
                return "判定不可", C["sub"]
            n = len(cur["abn"])
        return ("%d 件" % n, C["early"]) if n else ("なし", C["ok"])

    def info_lines(self):
        if self.cur is None:
            return ["iPad からの受信を待っています（BLE）",
                    "または「ファイル」から受信したCSVを選んでください"]
        m, c = self.meta, self.cur
        out = [m["name"], "%.1f 秒 / %d 点　（%s）" % (c["sec"], len(c["samples"]), m["hdr"])]
        if m.get("elec") is not None:
            out.append("ヘッダ：電極状態 %d ／ 不整脈 %d" % (m["elec"], m["arr"]))
        return out

    def verdict(self):
        c = self.cur
        if c is None:
            return "—", C["ink"], ""
        if not c["judged"]:
            return "判定不可", C["warn"], "R-R履歴が8拍分たまる前にデータが終わりました"
        if c["abn"]:
            return "不整脈の疑い", C["early"], "%d 拍で、直前の中央値からの外れ" % len(c["abn"])
        return "規則的", C["ok"], "判定した%d拍はすべて通常" % len(c["judged"])

    def side_chips(self):
        c = self.cur
        if c is None:
            return [("–", "拍数"), ("–", "心拍/分"), ("–", "異常判定")]
        return [(str(len(c["beats"])), "拍数"), (str(c["hr"]) if c["hr"] else "–", "心拍/分"),
                (str(len(c["abn"])) if c["judged"] else "–", "異常判定")]

    def render_log(self):
        self.p_recv.text = self.log

    def render(self):
        c = self.cur
        e = self.elec_state()
        self.p_chips[0].set(e[0], e[1])
        self.p_chips[1].set(*self.arr_state())
        self.p_chips[2].set(str(c["hr"]) if c and c["hr"] else "–")
        for b in self.p_btns2:
            b.enabled = c is not None
            b.alpha = 1.0 if c is not None else 0.4
        if c is None:
            self.b_data.set(C["lead"], "未検出")
            self.b_elec.set(C["lead"], "電極 –")
            self.l_src.text = "データ未検出"
        else:
            self.b_data.set(C["ok"], "データ検出")
            self.b_elec.set(e[1], e[2])
            m = self.meta
            self.l_src.text = "%s　%s　%.1f 秒 / %d 点" % (m["name"], m["hdr"], c["sec"], len(c["samples"]))
            if not c["judged"]:
                self.msg = self.msg or "短い波形では判定できません。前後を長く送ってください。"
        self.render_log()
        self.layout()
        for v in (self.p_status, self.p_rot, self.plot, self.side, self.blist):
            v.set_needs_display()

    # ---------- レイアウト（縦／横の切り替え）
    def layout(self):
        w, h = self.width, self.height
        land = w > h
        self.landscape = land
        self.p.hidden = land
        self.l.hidden = not land
        self.p.frame = self.l.frame = (0, 0, w, h)
        if land:
            self.layout_land(w, h)
            self.close_btn.frame = (w - 44 - 34, 8, 34, 30)
        else:
            self.layout_port(w, h)
            self.close_btn.frame = (w - 18 - 40, 52, 40, 34)

    def layout_port(self, w, h):
        m, top = 18, 52
        cw = w - 2 * m
        self.p_title.frame = (m, top, cw - 50, 34)
        self.p_status.frame = (m, top + 44, cw, 124)
        y = top + 44 + 124 + 12
        gw = (cw - 16) / 3.0
        for i, ch in enumerate(self.p_chips):
            ch.frame = (m + i * (gw + 8), y, gw, 58)
        y += 58 + 12
        widths = [ui.measure_string(b.title, font=(FB, 14))[0] + 28 for b in self.p_btns]
        extra = (cw - 16 - sum(widths)) / 3.0
        x = m
        for b, bw in zip(self.p_btns, widths):
            b.frame = (x, y, bw + extra, 40)
            x += bw + extra + 8
        y += 40 + 8
        for i, b in enumerate(self.p_btns2):
            b.frame = (m + i * (cw / 2.0 + 4), y, cw / 2.0 - 4, 34)
        y += 34 + 14
        self.p_rot.frame = (m, y, cw, 104)
        y += 104 + 12
        self.p_recv.frame = (m, y, cw, 72)

    def layout_land(self, w, h):
        sl, st, sb = 44, 6, 14   # セーフエリア（ノッチ・ホームバー）分の余白
        cw = w - 2 * sl
        x = sl
        self.l_title.frame = (x, st, 122, 34)
        x += 126
        self.b_data.frame = (x, st + 6, self.b_data.width, 22)
        x += self.b_data.width + 6
        self.b_elec.frame = (x, st + 6, self.b_elec.width, 22)
        x += self.b_elec.width + 8
        right = sl + cw - 40
        bws = [ui.measure_string(b.title, font=(FB, 12.5))[0] + 20 for b in self.l_btns]
        bx = right - sum(bws) - 6 * len(bws)
        for b, bw in zip(self.l_btns, bws):
            b.frame = (bx, st + 3, bw, 28)
            bx += bw + 6
        self.l_src.frame = (x, st, max(10, right - sum(bws) - 6 * len(bws) - x - 6), 34)
        top = st + 40
        mh = h - top - sb
        sw = 220
        self.plot.frame = (sl, top, cw - sw - 8, mh)
        self.side.frame = (sl + cw - sw, top, sw, mh)
        list_top = SidePanel.VH + self.side.alert_h() + 24
        self.scroll.frame = (self.side.x + 1, top + list_top, sw - 2, max(0, mh - list_top - 6))
        n = len(self.cur["beats"]) if self.cur else 0
        self.blist.frame = (0, 0, sw - 2, max(n * BeatList.RH, 1))
        self.scroll.content_size = (sw - 2, n * BeatList.RH)


def main():
    import threading
    v = ECGViewer()
    v.closed = threading.Event()
    v.present("fullscreen", hide_title_bar=True, orientations=["portrait", "landscape"])
    ui.delay(v.start_receiver, 0)   # BLE はメインスレッドで開始する
    # 画面を閉じるまでスクリプトを終わらせない（終わると Pythonista が変数を片付け、BLE の通知で
    # NameError になる）。待つ間は Python のコードを一切動かさない Event.wait() にする。
    # wait_modal() などで待つ間に Python が動くと、メインスレッドの objc_util と同時に動いて
    # ArgumentError やクラッシュの原因になる。
    v.closed.wait()
    return v


if __name__ == "__main__":
    main()

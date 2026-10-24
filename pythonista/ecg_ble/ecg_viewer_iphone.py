# -*- coding: utf-8 -*-
"""ECG R-R ビューア（iPhone：Pythonista 3 版）

ecg_viewer.html（改修版）と同じ画面・動作を Pythonista 3 だけで実現する（仕様書 6.2）。
  縦向き : データ検出状態（灰色「データ未検出」→ 緑色「データ検出」）と横向きの案内
  横向き : 波形・電極状態・異常検知（6.2.2）

データの受け取り方
  1) iPad の送信プログラム（ecg_sender_ipad.py）から Wi-Fi で受信（自動で「データ検出」になる）
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
import socket
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


# ===== iPad → iPhone 送受信（同じ Wi-Fi 内の TCP） =====
#   送信： b"ECGF1\n" + JSON（name, size, fmt）+ b"\n" + CSV本体
#   応答： b"OK <保存したファイル名>\n" または b"NG <理由>\n"
TCP_PORT = 50505
DISC_PORT = 50506          # 送信側が受信側を探す UDP（ブロードキャスト。使えない環境では IP を手入力）
MAGIC = b"ECGF1\n"
DISC_Q = b"ECGVIEW?"
DISC_A = b"ECGVIEW1"
MAX_SIZE = 20 * 1024 * 1024


def local_ip():
    for probe in ("10.255.255.255", "192.168.255.255", "8.8.8.8"):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            s.connect((probe, 1))
            ip = s.getsockname()[0]
            if ip and not ip.startswith("0."):
                return ip
        except OSError:
            pass
        finally:
            s.close()
    return "127.0.0.1"


def safe_name(name):
    base = os.path.basename(name or "").strip() or "received.csv"
    base = "".join(c for c in base if c not in '\\/:*?"<>|\0')
    if not base.lower().endswith((".csv", ".txt")):
        base += ".csv"
    return base


def _recv_line(conn, limit=4096):
    buf = b""
    while not buf.endswith(b"\n"):
        c = conn.recv(1)
        if not c:
            raise ValueError("接続が切れました")
        buf += c
        if len(buf) > limit:
            raise ValueError("ヘッダが長すぎます")
    return buf


def _recv_exact(conn, n):
    parts, got = [], 0
    while got < n:
        c = conn.recv(min(65536, n - got))
        if not c:
            raise ValueError("受信途中で切断されました（%d/%d バイト）" % (got, n))
        parts.append(c)
        got += len(c)
    return b"".join(parts)


def send_file(host, name, data, fmt="4", port=TCP_PORT, timeout=10.0):
    """CSV（bytes）を受信側へ送る。受信側が保存したファイル名を返す。"""
    meta = json.dumps({"name": name, "size": len(data), "fmt": fmt}, ensure_ascii=False)
    with socket.create_connection((host, port), timeout=timeout) as s:
        s.sendall(MAGIC + meta.encode("utf-8") + b"\n" + data)
        ans = _recv_line(s).decode("utf-8", "replace").strip()
    if not ans.startswith("OK"):
        raise IOError("受信側でエラー: " + ans)
    return ans[3:].strip()


def discover(timeout=1.5):
    """同じネットワークの受信側（iPhone）を UDP ブロードキャストで探す。[(ip, 名前)] を返す。"""
    found = {}
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        s.settimeout(0.3)
        for dst in ("255.255.255.255", "127.0.0.1"):
            try:
                s.sendto(DISC_Q, (dst, DISC_PORT))
            except OSError:
                pass
        end = time.time() + timeout
        while time.time() < end:
            try:
                data, addr = s.recvfrom(512)
            except socket.timeout:
                continue
            except OSError:
                break
            if data.startswith(DISC_A):
                found[addr[0]] = data[len(DISC_A):].decode("utf-8", "replace").strip()
    finally:
        s.close()
    return sorted(found.items())


class Receiver(object):
    """受信サーバ（iPhone側）。受け取った CSV を save_dir に保存し on_file(path, fmt) を呼ぶ。"""

    def __init__(self, save_dir, on_file, on_log=None, name="iPhone", port=TCP_PORT):
        self.save_dir = save_dir
        self.on_file = on_file
        self.on_log = on_log or (lambda m: None)
        self.name = name
        self.port = port
        self.running = False
        self._socks = []

    def start(self):
        import threading
        if not os.path.isdir(self.save_dir):
            os.makedirs(self.save_dir)
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind(("", self.port))
        srv.listen(2)
        srv.settimeout(0.5)
        self._socks.append(srv)
        self.running = True
        threading.Thread(target=self._serve, args=(srv,), daemon=True).start()
        try:
            u = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            u.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            u.bind(("", DISC_PORT))
            u.settimeout(0.5)
            self._socks.append(u)
            threading.Thread(target=self._answer, args=(u,), daemon=True).start()
        except OSError as e:
            self.on_log("自動検出は使えません（%s）。送信側でIPを入力してください" % e)

    def stop(self):
        self.running = False
        for s in self._socks:
            try:
                s.close()
            except OSError:
                pass
        self._socks = []

    def _answer(self, u):
        while self.running:
            try:
                data, addr = u.recvfrom(512)
            except socket.timeout:
                continue
            except OSError:
                break
            if data.startswith(DISC_Q):
                try:
                    u.sendto(DISC_A + b" " + self.name.encode("utf-8"), addr)
                except OSError:
                    pass

    def _serve(self, srv):
        while self.running:
            try:
                conn, addr = srv.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            try:
                conn.settimeout(15)
                path, fmt = self._handle(conn, addr)
                conn.sendall(("OK %s\n" % os.path.basename(path)).encode("utf-8"))
            except Exception as e:  # 受信エラーは送信側へ返し、画面にも出す
                self.on_log("受信エラー（%s）: %s" % (addr[0], e))
                try:
                    conn.sendall(("NG %s\n" % e).encode("utf-8"))
                except OSError:
                    pass
                continue
            finally:
                conn.close()
            self.on_file(path, fmt)

    def _handle(self, conn, addr):
        if _recv_line(conn, 16) != MAGIC:
            raise ValueError("送信プログラムからの接続ではありません")
        meta = json.loads(_recv_line(conn).decode("utf-8"))
        size = int(meta.get("size", -1))
        if not 0 < size <= MAX_SIZE:
            raise ValueError("サイズが不正です（%s）" % size)
        data = _recv_exact(conn, size)
        path = os.path.join(self.save_dir, safe_name(meta.get("name")))
        with open(path, "wb") as f:
            f.write(data)
        self.on_log("受信 %s（%d バイト、%s）" % (os.path.basename(path), size, addr[0]))
        return path, str(meta.get("fmt", "auto"))

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
        self.landscape = None
        self.receiver = None
        self.ip = core.local_ip()

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
        try:
            self.receiver = core.Receiver(SAVE_DIR, self.on_received, self.on_log,
                                          name="iPhone ECGビューア")
            self.receiver.start()
            self.on_log("受信待機中  %s : %d" % (self.ip, core.TCP_PORT))
        except OSError as e:
            self.on_log("受信を開始できません: %s" % e)

    def on_log(self, m):
        self.log = m
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

    @ui.in_background
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

    @ui.in_background
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
            return ["受信を待っています（%s : %d）" % (self.ip, core.TCP_PORT),
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
        self.p_recv.frame = (m, y, cw, 54)

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
    v = ECGViewer()
    v.start_receiver()
    v.present("fullscreen", hide_title_bar=True, orientations=["portrait", "landscape"])
    return v


if __name__ == "__main__":
    main()

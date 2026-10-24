# -*- coding: utf-8 -*-
"""ECG CSV 送信プログラム（iPad：Pythonista 3）

iPad の「ダウンロード」フォルダにある CSV（例：ECGtsHDdata1.csv）を選び、
同じ Wi-Fi にいる iPhone（ecg_viewer_iphone.py を実行中）へ送る。

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
except ImportError:
    dialogs = None

DOC_TYPES = ["public.comma-separated-values-text", "public.plain-text", "public.text", "public.data"]
CONF = os.path.join(os.path.expanduser("~/Documents"), ".ecg_sender.json")


def load_conf():
    try:
        with open(CONF) as f:
            return json.load(f)
    except Exception:
        return {}


def save_conf(d):
    try:
        with open(CONF, "w") as f:
            json.dump(d, f)
    except Exception:
        pass


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
        self.conf = load_conf()
        self.src = None
        self.title_l = ui.Label(text="ECG CSV 送信（iPad → iPhone）", font=("<system-bold>", 20))
        self.file_b = self._btn("① CSVを選ぶ（ダウンロード）", self.pick, True)
        self.file_l = ui.Label(text="ファイル未選択", font=("<system>", 14), text_color="#5d6772",
                               number_of_lines=0)
        self.elec_l = ui.Label(text="3行目 電極状態（0＝正常）", font=("<system>", 14))
        self.elec_t = ui.TextField(text="", keyboard_type=ui.KEYBOARD_NUMBER_PAD, alignment=ui.ALIGN_CENTER)
        self.arr_l = ui.Label(text="4行目 不整脈の件数（0＝なし）", font=("<system>", 14))
        self.arr_t = ui.TextField(text="", keyboard_type=ui.KEYBOARD_NUMBER_PAD, alignment=ui.ALIGN_CENTER)
        self.ip_l = ui.Label(text="送信先 iPhone の IP", font=("<system>", 14))
        self.ip_t = ui.TextField(text=self.conf.get("ip", ""), placeholder="例 192.168.1.23",
                                 keyboard_type=ui.KEYBOARD_DECIMAL_PAD)
        self.find_b = self._btn("② iPhoneを探す", self.find)
        self.send_b = self._btn("③ 送信", self.send, True)
        self.log_v = ui.TextView(editable=False, font=("Menlo", 12), background_color="#ffffff",
                                 border_width=1, border_color="#d9dde2", corner_radius=8)
        for v in (self.title_l, self.file_b, self.file_l, self.elec_l, self.elec_t, self.arr_l, self.arr_t,
                  self.ip_l, self.ip_t, self.find_b, self.send_b, self.log_v):
            self.add_subview(v)
        self.close_b = self._btn("閉じる", lambda s: self.close())
        self.add_subview(self.close_b)
        self.log("iPhone で ecg_viewer_iphone.py を起動し、画面の IP を確認してください")
        self.log("この iPad の IP: %s" % core.local_ip())

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
        self.ip_l.frame = (x, y, 150, 36)
        self.ip_t.frame = (x + 150, y, cw - 150 - 150, 36)
        self.find_b.frame = (x + cw - 140, y, 140, 36)
        y += 48
        self.send_b.frame = (x, y, cw, 48)
        y += 60
        self.log_v.frame = (x, y, cw, max(80, self.height - y - 30))

    def log(self, m):
        def go():
            self.log_v.text = (self.log_v.text + "\n" if self.log_v.text else "") + m
        ui.delay(go, 0)

    @ui.in_background
    def pick(self, sender=None):
        if dialogs is None:
            return
        path = dialogs.pick_document(types=DOC_TYPES)
        if path:
            self.load(path)

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

    @ui.in_background
    def find(self, sender=None):
        self.log("iPhone を探しています…")
        found = [f for f in core.discover() if f[0] != "127.0.0.1" or not self.ip_t.text]
        if not found:
            self.log("見つかりません。iPhone の画面に出ている IP を入力してください")
            return
        ip = found[0][0]
        if len(found) > 1 and dialogs:
            ip = dialogs.list_dialog("送信先", ["%s  %s" % f for f in found]) or ""
            ip = ip.split()[0] if ip else ""
        if ip:
            self.ip_t.text = ip
            self.log("見つかりました: %s" % ip)

    @ui.in_background
    def send(self, sender=None):
        if not self.src:
            self.log("先に CSV を選んでください")
            return
        ip = self.ip_t.text.strip()
        if not ip:
            self.log("送信先の IP を入力してください（または「iPhoneを探す」）")
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
        self.log("送信中… %s → %s（%d バイト）" % (self.src["name"], ip, len(body)))
        try:
            saved = core.send_file(ip, self.src["name"], body, "4")
        except Exception as e:
            self.log("送信に失敗: %s" % e)
            return
        self.conf["ip"] = ip
        save_conf(self.conf)
        self.log("送信しました。iPhone 側の保存名: %s" % saved)


def main():
    v = Sender()
    v.present("fullscreen", hide_title_bar=True)
    return v


if __name__ == "__main__":
    main()

# -*- coding: utf-8 -*-
"""iPad 送信プログラムの起動診断（Pythonista 3）

ecg_sender_ipad.py が画面を出す前に落ちるため、起動の手順を1段ずつ行って、どこで落ちるかを調べる。
ecg_sender_ipad.py と同じフォルダーに置いて実行する（送信はしない。最後に画面を閉じる）。
各段の結果は ble_diag_sender_log.txt に書いてすぐ保存する。Pythonista が落ちたら、もう一度
このスクリプトを実行すると、最初に「前回どこまで進んだか」をコンソールに表示する。
"""
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))


def _log_path():
    for d in (HERE, os.path.expanduser("~/Documents")):
        path = os.path.join(d, "ble_diag_sender_log.txt")
        try:
            with open(path, "a", encoding="utf-8"):
                pass
            return path
        except OSError:
            continue
    return None


LOG = _log_path()


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
    print("記録ファイル: %s" % LOG)
    try:
        with open(LOG, encoding="utf-8") as f:
            lines = f.read().splitlines()
    except (OSError, TypeError):
        lines = []
    if not lines:
        print("（前回の記録はありません）")
        return
    print("========== 前回の記録（最後の行が、落ちる直前に行っていた段）==========")
    for ln in lines[-30:]:
        print("  " + ln)
    print("========== ここまで ==========\n")


def main():
    show_previous()
    if LOG:
        with open(LOG, "w", encoding="utf-8") as f:
            f.write("送信プログラム起動診断の記録\n")
    write("起動診断を始めます（Python %s）" % sys.version.split()[0])
    keep = {}

    def s1():
        if not os.path.exists(os.path.join(HERE, "ecg_sender_ipad.py")):
            raise RuntimeError("同じフォルダーに ecg_sender_ipad.py がありません（名前に番号が付いていないか確認）")
        sys.path.insert(0, HERE)
        sys.modules.pop("ecg_sender_ipad", None)
        import ecg_sender_ipad
        keep["mod"] = ecg_sender_ipad
        write("    版 = %s ／ 記録の保存先 = %s" % (ecg_sender_ipad.core.BLE_VERSION, ecg_sender_ipad.core.TRACE_DIR))

    def s2():
        keep["view"] = keep["mod"].Sender()

    def s3():
        keep["view"].present("fullscreen", hide_title_bar=True)
        time.sleep(1.5)

    def s4():
        time.sleep(5)   # 送信プログラムは画面を出してから 3 秒後に BLE を始める
        ble = keep["view"].ble
        write("    BLE: PeripheralManager = %s ／ 準備完了 = %s" % (bool(ble.pm), ble.ready))

    def s5():
        keep["view"].close()
        time.sleep(1)

    steps = [
        ("ecg_sender_ipad.py を読み込む（画面はまだ出さない）", s1),
        ("送信画面を作る（Sender()）", s2),
        ("送信画面を表示する", s3),
        ("BLE の開始を待つ（3 秒後に始まる）", s4),
        ("画面を閉じる", s5),
    ]
    for i, (title, fn) in enumerate(steps, 1):
        write("[%d] %s … 開始" % (i, title))
        try:
            fn()
        except Exception as e:
            import traceback
            write("[%d] %s … エラー: %s: %s" % (i, title, type(e).__name__, e))
            write("    " + traceback.format_exc().replace("\n", " | "))
            break
        write("[%d] %s … OK" % (i, title))
    else:
        write("すべての段が OK でした。このスクリーンショットを送ってください。")


if __name__ == "__main__":
    main()

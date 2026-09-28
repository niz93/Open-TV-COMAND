#!/usr/bin/env python3

import json
import os
import time
import termios
import unidecode

JSON_FILE = os.path.expanduser("~/.config/LIVI/mediaData.json")
SERIAL_PORT = "/dev/ttyACM0"

BAUDRATE = 115200

def to_ascii(text: str) -> str:
    """Транскодирует любые Unicode-символы в похожие ASCII-символы."""
    if not text:
        return ""
    return unidecode.unidecode(text)

def configure_serial(fd):
    attrs = termios.tcgetattr(fd)

    # raw-ish mode
    attrs[0] = 0
    attrs[1] = 0
    attrs[2] = termios.CS8 | termios.CLOCAL | termios.CREAD
    attrs[3] = 0

    speed = {
        9600: termios.B9600,
        19200: termios.B19200,
        38400: termios.B38400,
        57600: termios.B57600,
        115200: termios.B115200,
    }[BAUDRATE]

    attrs[4] = speed   # input speed
    attrs[5] = speed   # output speed

    attrs[6][termios.VMIN] = 0
    attrs[6][termios.VTIME] = 0

    termios.tcsetattr(fd, termios.TCSANOW, attrs)

def read_song_name():
    try:
        with open(JSON_FILE, "r", encoding="utf-8") as f:
            data = json.load(f)

        return data["payload"]["media"]["MediaSongName"]

    except (FileNotFoundError, json.JSONDecodeError, KeyError):

        return None

def main():
    print(f"Watching: {JSON_FILE}")
    print(f"Serial:   {SERIAL_PORT} @ {BAUDRATE}")

    fd = os.open(
        SERIAL_PORT,
        os.O_RDWR | os.O_NOCTTY
    )

    configure_serial(fd)

    previous_song = None

    try:
        while True:
            song = read_song_name()

            if song is not None and song != previous_song:
                ascii_song = to_ascii(song)

                print(f"{song!r} -> {ascii_song!r}")

                message = ascii_song.encode("ascii")
                os.write(fd, message)

                previous_song = song

            time.sleep(0.2)

    finally:
        os.close(fd)

if __name__ == "__main__":
    main()

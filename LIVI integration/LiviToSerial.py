#!/usr/bin/env python3
import json
import os
import time
import termios
import unidecode

JSON_FILE = os.path.expanduser("~/.config/LIVI/state/media.json")
SERIAL_PORT = "/dev/ttyACM0"
BAUDRATE = 115200

# SCROLL SETTINGS
NUMBER_OF_CHARACTERS = 8              # Number of characters on the screen (threshold)
SCROLL_SPEED = 0.2    # Shear rate (in seconds)
PAUSE_START = 2.0   # Pause at the beginning of the track display (in seconds)

def to_ascii(text: str) -> str:
    return unidecode.unidecode(text) if text else ""

def configure_serial(fd):
    # Correct configuration of the termios structure for the raw mode
    attrs = termios.tcgetattr(fd)

    # attrs[0] = iflag, attrs[1] = oflag, attrs[2] = cflag, attrs[3] = lflag
    attrs[0] = 0  # Disabling input processing
    attrs[1] = 0  # Disabling outnput processing
    attrs[2] = termios.CS8 | termios.CLOCAL | termios.CREAD
    attrs[3] = 0  # Disabling canonical mode and echo

    # Настройка скорости
    speed = {9600: termios.B9600, 115200: termios.B115200}[BAUDRATE]
    attrs[4] = speed  # ispeed
    attrs[5] = speed  # ospeed

    # Non-blocking reading
    attrs[6][termios.VMIN] = 0
    attrs[6][termios.VTIME] = 0

    termios.tcsetattr(fd, termios.TCSANOW, attrs)

def read_song_name():
    try:
        with open(JSON_FILE, "r", encoding="utf-8") as f:
            song = json.load(f)["payload"]["title"]
            if not song or song.strip() == "-":
                return "NO PLAY"
            return song
    except (FileNotFoundError, json.JSONDecodeError, KeyError):
        return "NO PLAY"

def main():
    print(f"Watching: {JSON_FILE}")
    print(f"Serial: {SERIAL_PORT} @ {BAUDRATE}")

    try:
        fd = os.open(SERIAL_PORT, os.O_RDWR | os.O_NOCTTY)
    except OSError as e:
        print(f"Port opening error {SERIAL_PORT}: {e}")
        return

    configure_serial(fd)

    previous_song = None
    scroll_text = ""
    max_steps = 0
    current_step = 0

    last_action_time = 0
    state = "IDLE"

    try:
        while True:
            current_time = time.time()
            song = read_song_name()

            # 1. Detecting a track or status change
            if song != previous_song:
                ascii_song = to_ascii(song)
                print(f"{song!r} -> {ascii_song!r}")
                previous_song = song
                current_step = 0

                if len(ascii_song) > NUMBER_OF_CHARACTERS:
                    # Creating a looped string with spaces delimiters
                    scroll_text = ascii_song + "        " + ascii_song
                    max_steps = len(ascii_song) + 8
                    state = "PAUSE"
                    last_action_time = current_time + PAUSE_START

                    # Sending the first N characters for the initial pauseы
                    os.write(fd, scroll_text[:NUMBER_OF_CHARACTERS].encode("ascii"))
                else:
                    scroll_text = ascii_song
                    max_steps = 0
                    state = "IDLE"
                    os.write(fd, scroll_text.encode("ascii"))

            # 2. Processing the animation of a long track
            elif state == "PAUSE":
                if current_time >= last_action_time:
                    state = "SCROLLING"
                    last_action_time = current_time

            elif state == "SCROLLING":
                if current_time - last_action_time >= SCROLL_SPEED:
                    current_step += 1

                    frame = scroll_text[current_step : current_step + NUMBER_OF_CHARACTERS]
                    os.write(fd, frame.encode("ascii"))

                    last_action_time = current_time

                    if current_step >= max_steps:
                        state = "FINISHED"



            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\nThe script was stopped by the user.")
    finally:
        os.close(fd)

if __name__ == "__main__":
    main()

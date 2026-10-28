#!/usr/bin/env python3
import json
import os
import time
import termios
import unidecode

JSON_FILE = os.path.expanduser("~/.config/LIVI/state/media.json")
OUTPUT_JSON_FILE = os.path.expanduser("~/.config/LIVI/state/cardata.json")
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
    attrs[1] = 0  # Disabling output processing
    attrs[2] = termios.CS8 | termios.CLOCAL | termios.CREAD
    attrs[3] = 0  # Disabling canonical mode and echo

    # Baudrate speed setup
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
    print(f"Writing incoming data to: {OUTPUT_JSON_FILE}")
    print(f"Serial: {SERIAL_PORT} @ {BAUDRATE}")

    # Create directory for cardata.json if it does not exist yet
    os.makedirs(os.path.dirname(OUTPUT_JSON_FILE), exist_ok=True)

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
    
    input_buffer = b""

    try:
        while True:
            current_time = time.time()
            
            # --- Reading incoming data from SERIAL_PORT ---
            try:
                incoming_bytes = os.read(fd, 256)
                if incoming_bytes:
                    input_buffer += incoming_bytes
                    
                    # Split buffer by newline character
                    while b"\n" in input_buffer or b"\r" in input_buffer:
                        idx_n = input_buffer.find(b"\n")
                        idx_r = input_buffer.find(b"\r")
                        
                        if idx_n != -1 and idx_r != -1:
                            idx = min(idx_n, idx_r)
                        else:
                            idx = idx_n if idx_n != -1 else idx_r
                        
                        line_bytes = input_buffer[:idx]
                        input_buffer = input_buffer[idx + 1:]
                        
                        raw_text = line_bytes.decode("ascii", errors="ignore")
                        clean_text = raw_text.strip()
                        
                        if clean_text:
                            # FIX: Check that the string starts with CAN data to prevent loopback echo
                            if clean_text.startswith("RX ID"):
                                print(f"Received from Serial: {clean_text!r}")
                                
                                with open(OUTPUT_JSON_FILE, "w", encoding="utf-8") as f:
                                    json.dump({"serial_data": clean_text, "timestamp": current_time}, f, ensure_ascii=False, indent=4)
                            else:
                                # Process raw text echo strings if they contain a valid packet inside
                                if "RX ID" in clean_text:
                                    actual_data = clean_text[clean_text.find("RX ID"):]
                                    print(f"Cleaned & Received: {actual_data!r}")
                                    
                                    with open(OUTPUT_JSON_FILE, "w", encoding="utf-8") as f:
                                        json.dump({"serial_data": actual_data, "timestamp": current_time}, f, ensure_ascii=False, indent=4)
            except OSError:
                pass

            # --- Display scrolling text animation logic ---
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

                    # Sending the first N characters for the initial pause
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

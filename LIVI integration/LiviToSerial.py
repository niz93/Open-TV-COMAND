#!/usr/bin/env python3
import json
import os
import time
import termios
import unidecode
import re
import urllib.request

# Global configuration paths
JSON_FILE = os.path.expanduser("~/.config/LIVI/state/media.json")
OUTPUT_JSON_FILE = os.path.expanduser("~/.config/LIVI/state/cardata.json")
SERIAL_PORT = "/dev/ttyACM0"

# LIVI Telemetry REST API Endpoint
LIVI_TELEMETRY_URL = "http://localhost:4000/telemetry"

# Text scrolling animation parameters
NUMBER_OF_CHARACTERS = 8      # Visible display window width
SCROLL_SPEED = 0.5            # Delay between text shifts (in seconds)
PAUSE_START = 2.0             # Initial freeze time before scrolling starts

def to_ascii(text: str) -> str:
    """Converts localized UTF-8 text to clean ASCII for physical hardware displays."""
    return unidecode.unidecode(text) if text else ""

def configure_serial(fd):
    """Configures the termios structure for CDC ACM USB raw serial communication."""
    attrs = termios.tcgetattr(fd)

    attrs[0] = 0  # iflag: Disable all input processing
    attrs[1] = 0  # oflag: Disable all output processing
    attrs[2] = termios.CS8 | termios.CLOCAL | termios.CREAD  # cflag: 8-bit, local line, enable receiver
    attrs[3] = 0  # lflag: Disable canonical mode, echo, and signaling

    # Non-blocking read configuration
    attrs[6][termios.VMIN] = 0
    attrs[6][termios.VTIME] = 0

    termios.tcsetattr(fd, termios.TCSANOW, attrs)

def read_song_name():
    """Reads current track info from LIVI core media state file."""
    try:
        with open(JSON_FILE, "r", encoding="utf-8") as f:
            song = json.load(f)["payload"]["title"]
            if not song or song.strip() == "-":
                return "NO PLAY"
            return song
    except (FileNotFoundError, json.JSONDecodeError, KeyError):
        return "NO PLAY"

def send_to_livi(payload: dict):
    """Dispatches extracted vehicle metrics to LIVI API over low-latency HTTP POST."""
    try:
        data = json.dumps(payload).encode("utf-8")
        req = urllib.request.Request(
            LIVI_TELEMETRY_URL, 
            data=data, 
            headers={"Content-Type": "application/json"},
            method="POST"
        )
        # Short timeout keeps serial read execution safe and blocking-free
        with urllib.request.urlopen(req, timeout=0.2):
            pass
    except Exception as e:
        # Prevent runtime crashes if the LIVI backend service is rebooting
        print(f"LIVI Telemetry send error: {e}")

def parse_and_process_can(clean_text: str):
    """
    Parses raw STM32 ACM string frames and routes them to specific decoders.
    """
    
    # -------------------------------------------------------------------------
    # 1. Decode ID 0x0448 (COMAND to NAVI settings)
    # -------------------------------------------------------------------------
    if "0448" in clean_text or "448" in clean_text:
        # Match hexadecimal byte sequence following data markers
        data_match = re.search(r'(?:DATA|Data):\s*([0-9a-fA-F ]+)', clean_text)
        if data_match:
            hex_values = data_match.group(1).split()
        else:
            # Fallback: extract only the bytes that follow the 'HEX' keyword
            hex_part = clean_text.split("HEX")[-1]
            hex_values = re.findall(r'\b[0-9a-fA-F]{2}\b', hex_part)

        # Validate structured payload size requirement (XX E4 00 01/02 requires 4 bytes)
        if len(hex_values) >= 4:
            try:
                # Safely slice to prevent IndexError on shorter frames
                rx_data = [int(b, 16) for b in hex_values[:min(8, len(hex_values))]]
                
                # Check for COMAND verification signature: rx_data[1] == 0xE4 and rx_data[2] == 0x00
                if rx_data[1] == 0xE4 and rx_data[2] == 0x00:
                    print(f" -> Matches COMAND NAVI Settings! Data: {[hex(x) for x in rx_data]}")
                    
                    livi_payload = {}
                    
                    # rx_data[3]: Reverse Gear state mapping
                    revers_gear = rx_data[3]
                    if revers_gear != 0xFF:
                        if revers_gear == 0x01:
                            livi_payload["reverse"] = False
                            livi_payload["gear"] = "D"
                        elif revers_gear == 0x02:
                            livi_payload["reverse"] = True
                            livi_payload["gear"] = "R"
                            
                    if livi_payload:
                        print(f" -> Sending to LIVI: {livi_payload}")
                        send_to_livi(livi_payload)
                        
            except (ValueError, IndexError) as e:
                print(f"Error processing CAN bytes for ID 484: {e}")

    # -------------------------------------------------------------------------
    # 2. Decode ID 0x0264 (COMAND to TV settings)
    # -------------------------------------------------------------------------
    if "0464" in clean_text or "464" in clean_text:
        # Match hexadecimal byte sequence following data markers
        data_match = re.search(r'(?:DATA|Data):\s*([0-9a-fA-F ]+)', clean_text)
        if data_match:
            hex_values = data_match.group(1).split()
        else:
            # Fallback: extract only the bytes that follow the 'HEX' keyword
            hex_part = clean_text.split("HEX")[-1]
            hex_values = re.findall(r'\b[0-9a-fA-F]{2}\b', hex_part)

        # Validate structured payload size requirement (Full TV payload requires 8 bytes)
        if len(hex_values) >= 8:
            try:
                rx_data = [int(b, 16) for b in hex_values[:8]]
                
                # Check for COMAND verification signature: rx_data[1] == 0x24
                if rx_data[1] == 0x24:
                    print(f" -> Matches COMAND TV Settings! Data: {[hex(x) for x in rx_data]}")
                    
                    livi_payload = {}
                    
                    # rx_data[3]: Interior illumination brightness mapping (Dimmer)
                    light_interior = rx_data[3]
                    if light_interior != 0xFF:
                        if light_interior == 0x00:
                            livi_payload["dimmerPct"] = 0.0
                        elif 0x0C <= light_interior <= 0x64:
                            # 0x64 (Hex) matches exactly 100% (Dec)
                            livi_payload["dimmerPct"] = float(light_interior)

                    # rx_data[4]: Photocell Light Sensor mapping (Day/Night layout trigger)
                    light_sensor = rx_data[4]
                    if light_sensor != 0xFF:
                        if light_sensor == 0x00:
                            livi_payload["nightMode"] = False
                            livi_payload["darkMode"] = False
                            livi_payload["clusterNightMode"] = False
                        elif light_sensor == 0x40:
                            livi_payload["nightMode"] = True
                            livi_payload["darkMode"] = True
                            livi_payload["clusterNightMode"] = True
                            
                    if livi_payload:
                        print(f" -> Sending to LIVI: {livi_payload}")
                        send_to_livi(livi_payload)
                        
            except (ValueError, IndexError) as e:
                print(f"Error processing CAN bytes for ID 464: {e}")


def main():
    print(f"Watching: {JSON_FILE}")
    print(f"Writing incoming data to: {OUTPUT_JSON_FILE}")
    print(f"Serial: {SERIAL_PORT} (CDC ACM Raw Mode)")

    # Ensure volatile storage context paths exist inside runtime RAM file systems
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
            
            # --- Stream incoming transmission frames from STM32 hardware controller ---
            try:
                incoming_bytes = os.read(fd, 256)
                if incoming_bytes:
                    input_buffer += incoming_bytes
                    
                    # Split byte sequence boundaries precisely to retain processing alignment
                    while b"\n" in input_buffer or b"\r" in input_buffer:
                        idx_n = input_buffer.find(b"\n")
                        idx_r = input_buffer.find(b"\r")
                        
                        idx = min(idx_n, idx_r) if idx_n != -1 and idx_r != -1 else (idx_n if idx_n != -1 else idx_r)
                        line_bytes = input_buffer[:idx]
                        
                        # Compensate cleanly for varied trailing CR/LF configurations
                        skip = 1
                        if idx + 1 < len(input_buffer):
                            next_chars = input_buffer[idx:idx+2]
                            if next_chars in (b"\r\n", b"\n\r"):
                                skip = 2
                                
                        input_buffer = input_buffer[idx + skip:]
                        clean_text = line_bytes.decode("ascii", errors="ignore").strip()
                        
                        # Process verified messages
                        if clean_text.startswith("RX ID"):
                            print(f"Received from Serial: {clean_text!r}")
                            
                            parse_and_process_can(clean_text)
                            
                            # Cache current state directly to local configuration JSON files
                            with open(OUTPUT_JSON_FILE, "w", encoding="utf-8") as f:
                                json.dump(
                                    {"serial_data": clean_text, "timestamp": current_time}, 
                                    f, 
                                    ensure_ascii=False, 
                                    indent=4
                                )
            except OSError:
                pass

            # --- Hardware cluster text streaming interface workflow engine ---
            song = read_song_name()

            # Track alternation or initialization execution routine
            if song != previous_song:
                ascii_song = to_ascii(song)
                print(f"{song!r} -> {ascii_song!r}")
                previous_song = song
                current_step = 0

                if len(ascii_song) > NUMBER_OF_CHARACTERS:
                    scroll_text = ascii_song + "        " + ascii_song
                    max_steps = len(ascii_song) + 8
                    state = "PAUSE"
                    last_action_time = current_time + PAUSE_START
                    os.write(fd, scroll_text[:NUMBER_OF_CHARACTERS].encode("ascii"))
                else:
                    scroll_text = ascii_song
                    max_steps = 0
                    state = "IDLE"
                    os.write(fd, scroll_text.encode("ascii"))

            # Handle display timeout animations
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

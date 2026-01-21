import paho.mqtt.client as mqtt
import sqlite3
import json
import time

# --- CẤU HÌNH ---
MQTT_BROKER = "localhost"   # Máy tính chạy Docker
MQTT_PORT = 1883
MQTT_TOPIC = "lab4/telemetry" # Phải khớp với TOPIC_TELEMETRY trong code Gateway
DB_FILE = "vaccine_data.db"

# --- DATABASE ---
def init_db():
    conn = sqlite3.connect(DB_FILE)
    c = conn.cursor()
    # Tạo bảng lưu trữ [cite: 30]
    c.execute('''CREATE TABLE IF NOT EXISTS telemetry
                 (id INTEGER PRIMARY KEY AUTOINCREMENT,
                  seq INTEGER,
                  ts_node INTEGER,
                  temp REAL,
                  alarm INTEGER,
                  backup INTEGER,
                  received_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP)''')
    conn.commit()
    conn.close()
    print(f"Database {DB_FILE} sẵn sàng!")

def save_batch_to_db(samples):
    if not samples: return
    try:
        conn = sqlite3.connect(DB_FILE)
        c = conn.cursor()
        count = 0
        # Xử lý Batch: Tách từng mẫu trong gói ra để lưu [cite: 31, 32]
        for s in samples:
            c.execute("INSERT INTO telemetry (seq, ts_node, temp, alarm, backup) VALUES (?, ?, ?, ?, ?)",
                      (s['seq'], s['ts_ms'], s['temp'], s['alarm'], s['backup']))
            count += 1
            print(f"   -> [DB] Lưu mẫu Seq={s['seq']} | Temp={s['temp']}")
        
        conn.commit()
        conn.close()
        print(f" >> Đã lưu thành công {count} mẫu vào Database.")
    except Exception as e:
        print(f"Lỗi Database: {e}")

# --- MQTT HANDLER ---
def on_connect(client, userdata, flags, rc):
    print(f"Đã kết nối Broker (RC={rc})")
    client.subscribe(MQTT_TOPIC)
    print(f"Đang lắng nghe topic: {MQTT_TOPIC}")

def on_message(client, userdata, msg):
    try:
        payload = msg.payload.decode()
        # Gateway gửi lên: {"samples": [ ... ]}
        data = json.loads(payload)
        
        if "samples" in data:
            print(f"\nNhận gói Batch chứa {len(data['samples'])} mẫu:")
            save_batch_to_db(data["samples"])
        else:
            print("Nhận dữ liệu lạ (không có key 'samples')")
            
    except Exception as e:
        print(f"Lỗi xử lý dữ liệu: {e}")

# --- MAIN ---
if __name__ == "__main__":
    init_db()
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message
    
    print("Đang kết nối MQTT...")
    try:
        client.connect(MQTT_BROKER, MQTT_PORT, 60)
        client.loop_forever()
    except Exception as e:
        print(f"LỖI: Không thể kết nối MQTT. Hãy kiểm tra Docker! ({e})")
-----------------------Mappping-----------------



import serial
import matplotlib.pyplot as plt
import math

# ================= CẤU HÌNH =================
COM_PORT = 'COM12'
BAUD_RATE = 9600
SCALE = 0.05       # Hệ số chuyển đổi thời gian (ms) sang khoảng cách vẽ (pixels)
# ============================================

try:
    ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=0.1)
    print(f"Đã kết nối thành công tới {COM_PORT}")
except Exception as e:
    print(f"Lỗi kết nối tới {COM_PORT}: {e}")
    exit()

# Cấu hình Matplotlib Interactive Mode (Vẽ thời gian thực)
plt.ion()
fig, ax = plt.subplots(figsize=(8, 8))
line, = ax.plot([], [], 'b-', linewidth=3, label="Quỹ đạo xe")
ax.plot(0, 0, 'ro', markersize=8, label="Điểm xuất phát")
ax.set_aspect('equal', adjustable='datalim')
ax.set_title("Bản đồ Xe Dò Line (Mọi sa hình)", fontsize=14, fontweight='bold')
ax.grid(True, linestyle='--', alpha=0.6)
ax.legend()

# Khởi tạo toạ độ và hướng
x_data = [0.0]
y_data = [0.0]
current_angle = 90.0 # Bắt đầu hướng mũi xe lên trên (trục Y)

print("Đang chờ dữ liệu từ xe...")

try:
    while True:
        if ser.in_waiting > 0:
            raw_data = ser.readline().decode('utf-8').strip()
            if not raw_data:
                continue

            # 1. NHẬN LỆNH ĐI THẲNG -> VẼ LIỀN
            if raw_data.startswith("F"):
                parts = raw_data.split(",")
                if len(parts) == 2:
                    try:
                        time_ms = int(parts[1])
                        dist = time_ms * SCALE
                        if dist > 0:
                            print(f"Đi thẳng: {time_ms} ms -> {dist:.2f} px")
                            rad = math.radians(current_angle)
                            new_x = x_data[-1] + dist * math.cos(rad)
                            new_y = y_data[-1] + dist * math.sin(rad)
                            x_data.append(new_x)
                            y_data.append(new_y)
                    except ValueError:
                        pass

            # 2. NHẬN LỆNH RẼ TRÁI -> ĐỔI GÓC
            elif raw_data == "L":
                print(">> Xe rẽ TRÁI 90°")
                current_angle += 90

            # 3. NHẬN LỆNH RẼ PHẢI -> ĐỔI GÓC
            elif raw_data == "R":
                print(">> Xe rẽ PHẢI 90°")
                current_angle -= 90

            # 4. NHẬN LỆNH VỀ ĐÍCH
            elif raw_data == "S":
                print("\n==== XE ĐÃ VỀ ĐÍCH - HOÀN THÀNH BẢN ĐỒ ====")
                ax.plot(x_data[-1], y_data[-1], 'go', markersize=10, label="Điểm về đích")
                ax.legend()
                break

            # Cập nhật đồ thị hiển thị ngay lập tức
            line.set_xdata(x_data)
            line.set_ydata(y_data)
            ax.relim()
            ax.autoscale_view()
            fig.canvas.draw()
            fig.canvas.flush_events()

except KeyboardInterrupt:
    print("\nĐã dừng bằng phím.")
finally:
    ser.close()
    plt.ioff()
    plt.show() # Giữ cửa sổ bản đồ không bị tắt khi code kết thúc
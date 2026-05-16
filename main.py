import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from pathlib import Path

# ============================================================
# OUTPUT KLASÖRÜ
# ============================================================

OUTPUT_DIR = Path("outputs")
OUTPUT_DIR.mkdir(exist_ok=True)

np.random.seed(42)

# ============================================================
# ORTAM PARAMETRELERİ
# ============================================================

MAP_WIDTH = 20
MAP_HEIGHT = 15

start = np.array([1.2, 1.2])
goal = np.array([18.5, 13.2])

obstacles = [
    (3.0, 2.0, 2.5, 0.8, "Raf"),
    (7.0, 2.0, 2.0, 1.4, "Makine"),
    (12.0, 2.0, 3.0, 1.0, "Raf"),
    (16.5, 2.0, 1.5, 1.0, "Palet"),
    (2.0, 5.0, 3.0, 1.0, "Raf"),
    (6.5, 5.0, 2.0, 2.0, "Makine"),
    (11.0, 5.0, 2.2, 1.8, "Makine"),
    (15.0, 5.2, 2.5, 0.8, "Raf"),
    (3.0, 9.0, 2.5, 1.0, "Raf"),
    (8.0, 9.0, 2.0, 2.0, "Makine"),
    (13.0, 9.0, 2.5, 1.0, "Raf"),
    (17.0, 8.5, 1.0, 1.0, "Kolon"),
    (5.5, 12.0, 1.0, 1.0, "Kolon"),
    (9.0, 12.0, 2.0, 1.4, "Makine"),
    (14.0, 12.0, 1.0, 1.0, "Kolon"),
]

waypoints = [
    (1.2, 1.2),
    (1.2, 4.0),
    (5.8, 4.0),
    (5.8, 7.8),
    (10.0, 7.8),
    (10.0, 11.4),
    (16.0, 11.4),
    (18.5, 13.2),
]


# ============================================================
# YARDIMCI FONKSİYONLAR
# ============================================================

def normalize_angle(angle):
    return np.arctan2(np.sin(angle), np.cos(angle))


def point_inside_obstacle(px, py):
    for ox, oy, ow, oh, _ in obstacles:
        if ox <= px <= ox + ow and oy <= py <= oy + oh:
            return True
    return False


def moving_average(data, window_size=15):
    if len(data) < window_size:
        return data

    kernel = np.ones(window_size) / window_size
    return np.convolve(data, kernel, mode="same")


# ============================================================
# ROBOT HAREKET SİMÜLASYONU
# ============================================================

def simulate_robot_motion():
    dt = 0.1
    max_steps = 3000

    x = start[0]
    y = start[1]
    theta = 0.0

    v_max = 0.6
    w_max = 1.5

    k_v = 0.8
    k_w = 2.0

    true_path = []
    dr_path = []
    kf_path = []

    theta_list = []
    time_list = []

    error_list = []
    dr_error_list = []

    x_dr = x
    y_dr = y
    theta_dr = theta

    x_kf = np.array([x, y, theta])
    P = np.eye(3) * 0.1

    encoder_noise_std = 0.08
    imu_noise_std = 0.05

    Q = np.diag([0.02, 0.02, 0.01])
    R = np.diag([0.10, 0.10, 0.04])

    target_index = 1

    for step in range(max_steps):
        target = np.array(waypoints[target_index])

        dx = target[0] - x
        dy = target[1] - y

        distance = np.sqrt(dx ** 2 + dy ** 2)
        desired_theta = np.arctan2(dy, dx)
        angle_error = normalize_angle(desired_theta - theta)

        if distance < 0.15:
            target_index += 1

            if target_index >= len(waypoints):
                break

            continue

        v = np.clip(k_v * distance, 0.0, v_max)
        w = np.clip(k_w * angle_error, -w_max, w_max)

        if abs(angle_error) > 0.6:
            v = 0.15

        # Gerçek non-holonomic robot hareketi
        x = x + v * np.cos(theta) * dt
        y = y + v * np.sin(theta) * dt
        theta = normalize_angle(theta + w * dt)

        # Enkoder ve IMU gürültüsü
        v_enc = v + np.random.normal(0, encoder_noise_std)
        w_imu = w + np.random.normal(0, imu_noise_std)

        # Dead reckoning
        x_dr = x_dr + v_enc * np.cos(theta_dr) * dt
        y_dr = y_dr + v_enc * np.sin(theta_dr) * dt
        theta_dr = normalize_angle(theta_dr + w_imu * dt)

        # Kalman prediction
        x_pred = np.array([
            x_kf[0] + v_enc * np.cos(x_kf[2]) * dt,
            x_kf[1] + v_enc * np.sin(x_kf[2]) * dt,
            normalize_angle(x_kf[2] + w_imu * dt),
        ])

        F = np.array([
            [1, 0, -v_enc * np.sin(x_kf[2]) * dt],
            [0, 1,  v_enc * np.cos(x_kf[2]) * dt],
            [0, 0, 1],
        ])

        P_pred = F @ P @ F.T + Q

        # LiDAR + IMU ölçümü gibi modellenmiş gürültülü ölçüm
        z = np.array([
            x + np.random.normal(0, 0.10),
            y + np.random.normal(0, 0.10),
            theta + np.random.normal(0, 0.04),
        ])

        H = np.eye(3)

        y_res = z - H @ x_pred
        y_res[2] = normalize_angle(y_res[2])

        S = H @ P_pred @ H.T + R
        K = P_pred @ H.T @ np.linalg.inv(S)

        x_kf = x_pred + K @ y_res
        x_kf[2] = normalize_angle(x_kf[2])

        P = (np.eye(3) - K @ H) @ P_pred

        true_path.append((x, y))
        dr_path.append((x_dr, y_dr))
        kf_path.append((x_kf[0], x_kf[1]))

        theta_list.append(theta)
        time_list.append(step * dt)

        kf_error = np.sqrt((x - x_kf[0]) ** 2 + (y - x_kf[1]) ** 2)
        dr_error = np.sqrt((x - x_dr) ** 2 + (y - y_dr) ** 2)

        error_list.append(kf_error)
        dr_error_list.append(dr_error)

    return (
        np.array(true_path),
        np.array(dr_path),
        np.array(kf_path),
        np.array(theta_list),
        np.array(time_list),
        np.array(error_list),
        np.array(dr_error_list),
    )


# ============================================================
# LiDAR SİMÜLASYONU VE İŞLEME
# ============================================================

def simulate_lidar(robot_pose, max_range=5.0, angle_range=np.pi, num_beams=90):
    rx, ry, rtheta = robot_pose

    lidar_points = []
    angles = np.linspace(-angle_range / 2, angle_range / 2, num_beams)

    for angle in angles:
        beam_angle = rtheta + angle
        hit = False

        for r in np.arange(0, max_range, 0.05):
            px = rx + r * np.cos(beam_angle)
            py = ry + r * np.sin(beam_angle)

            if px < 0 or px > MAP_WIDTH or py < 0 or py > MAP_HEIGHT:
                lidar_points.append((
                    px + np.random.normal(0, 0.06),
                    py + np.random.normal(0, 0.06)
                ))
                hit = True
                break

            if point_inside_obstacle(px, py):
                lidar_points.append((
                    px + np.random.normal(0, 0.06),
                    py + np.random.normal(0, 0.06)
                ))
                hit = True
                break

        if not hit:
            px = rx + max_range * np.cos(beam_angle)
            py = ry + max_range * np.sin(beam_angle)

            lidar_points.append((
                px + np.random.normal(0, 0.06),
                py + np.random.normal(0, 0.06)
            ))

    return np.array(lidar_points)


def filter_lidar_points(lidar_points, robot_pose):
    rx, ry, _ = robot_pose

    filtered_points = []

    distances = np.sqrt(
        (lidar_points[:, 0] - rx) ** 2 +
        (lidar_points[:, 1] - ry) ** 2
    )

    for i in range(1, len(lidar_points) - 1):
        d_prev = distances[i - 1]
        d_curr = distances[i]
        d_next = distances[i + 1]

        # Mesafe eşikleme ile ani sıçramalar temizlenir
        if abs(d_curr - d_prev) < 0.55 and abs(d_curr - d_next) < 0.55:
            filtered_points.append(lidar_points[i])

    return np.array(filtered_points)


def cluster_lidar_points(filtered_points, distance_threshold=0.45):
    if len(filtered_points) == 0:
        return []

    clusters = []
    current_cluster = [filtered_points[0]]

    for i in range(1, len(filtered_points)):
        distance = np.linalg.norm(filtered_points[i] - filtered_points[i - 1])

        if distance < distance_threshold:
            current_cluster.append(filtered_points[i])
        else:
            clusters.append(np.array(current_cluster))
            current_cluster = [filtered_points[i]]

    clusters.append(np.array(current_cluster))

    return clusters


# ============================================================
# GÖRSEL TABAN
# ============================================================

def draw_factory_base(ax):
    ax.set_xlim(0, MAP_WIDTH)
    ax.set_ylim(0, MAP_HEIGHT)
    ax.set_aspect("equal", adjustable="box")

    floor = patches.Rectangle(
        (0, 0),
        MAP_WIDTH,
        MAP_HEIGHT,
        facecolor="#f2f2f2",
        edgecolor="black",
        linewidth=2
    )
    ax.add_patch(floor)

    # Depo alanı
    storage_area = patches.Rectangle(
        (0.4, 0.4),
        4.0,
        3.2,
        facecolor="#d9ead3",
        edgecolor="#38761d",
        linewidth=2,
        linestyle="--",
        alpha=0.8
    )
    ax.add_patch(storage_area)

    ax.text(
        2.4,
        2.8,
        "DEPO",
        ha="center",
        va="center",
        fontsize=11,
        fontweight="bold"
    )

    # Teslimat alanı
    delivery_area = patches.Rectangle(
        (16.5, 12.0),
        3.0,
        2.3,
        facecolor="#f4cccc",
        edgecolor="#cc0000",
        linewidth=2,
        linestyle="--",
        alpha=0.8
    )
    ax.add_patch(delivery_area)

    ax.text(
        18.0,
        12.5,
        "TESLİMAT",
        ha="center",
        va="center",
        fontsize=10,
        fontweight="bold"
    )

    # Koridor çizgileri
    corridor_lines = [
        ((1, 4), (18.5, 4)),
        ((1, 7.8), (18.5, 7.8)),
        ((1, 11.4), (18.5, 11.4)),
        ((5.8, 1), (5.8, 14)),
        ((10, 1), (10, 14)),
        ((16, 1), (16, 14)),
    ]

    for (x1, y1), (x2, y2) in corridor_lines:
        ax.plot(
            [x1, x2],
            [y1, y2],
            color="#d9a300",
            linestyle="--",
            linewidth=1.4,
            alpha=0.7
        )

    colors = {
        "Raf": "#8d6e63",
        "Makine": "#7f8c8d",
        "Kolon": "#555555",
        "Palet": "#c27c0e"
    }

    for ox, oy, ow, oh, otype in obstacles:
        rect = patches.Rectangle(
            (ox, oy),
            ow,
            oh,
            facecolor=colors.get(otype, "gray"),
            edgecolor="black",
            linewidth=1.5,
            alpha=0.9
        )

        ax.add_patch(rect)

        ax.text(
            ox + ow / 2,
            oy + oh / 2,
            otype,
            ha="center",
            va="center",
            fontsize=8,
            color="white",
            fontweight="bold"
        )

    ax.set_xlabel("X konumu (m)")
    ax.set_ylabel("Y konumu (m)")


def save_and_show(fig, filename):
    fig.savefig(
        OUTPUT_DIR / filename,
        dpi=150,
        bbox_inches="tight"
    )
    plt.show()


# ============================================================
# GRAFİKLER
# ============================================================

def plot_environment():
    fig, ax = plt.subplots(figsize=(12, 8))

    draw_factory_base(ax)

    ax.scatter(start[0], start[1], s=200, color="green", label="Başlangıç")
    ax.text(start[0] + 0.25, start[1], "S", fontsize=12, fontweight="bold", color="green")

    ax.scatter(goal[0], goal[1], s=250, marker="*", color="red", label="Hedef")
    ax.text(goal[0] + 0.25, goal[1], "G", fontsize=12, fontweight="bold", color="red")

    ax.set_title("2B Fabrika Ortam Haritası", fontsize=14, fontweight="bold")
    ax.grid(True, alpha=0.35)
    ax.legend(loc="lower right")

    plt.tight_layout()
    save_and_show(fig, "Figure_ortam_haritasi.png")

    print("✓ Ortam haritası kaydedildi.")


def plot_robot_motion(true_path):
    fig, ax = plt.subplots(figsize=(12, 8))

    draw_factory_base(ax)

    wp = np.array(waypoints)

    ax.plot(wp[:, 0], wp[:, 1], linestyle="--", linewidth=2.2, color="blue", label="Planlanan Yol")
    ax.plot(true_path[:, 0], true_path[:, 1], linewidth=2.7, color="red", label="Gerçek Yol")

    ax.scatter(start[0], start[1], s=200, color="green", label="Başlangıç")
    ax.text(start[0] + 0.25, start[1], "S", fontsize=12, fontweight="bold", color="green")

    ax.scatter(goal[0], goal[1], s=250, marker="*", color="red", label="Hedef")
    ax.text(goal[0] + 0.25, goal[1], "G", fontsize=12, fontweight="bold", color="red")

    ax.set_title("Planlanan Yol ve Gerçek Robot Yolu", fontsize=14, fontweight="bold")
    ax.grid(True, alpha=0.35)
    ax.legend(bbox_to_anchor=(1.03, 1), loc="upper left")

    plt.tight_layout()
    save_and_show(fig, "Figure_robot_yolu.png")

    print("✓ Robot yolu grafiği kaydedildi.")


def plot_theta(time_list, theta_list):
    fig, ax = plt.subplots(figsize=(11, 4))

    ax.plot(time_list, theta_list, linewidth=2, color="#2980b9")

    ax.set_title("Robot Yön Açısı Değişimi", fontsize=13, fontweight="bold")
    ax.set_xlabel("Zaman (s)")
    ax.set_ylabel("Theta (rad)")
    ax.grid(True, alpha=0.35)

    plt.tight_layout()
    save_and_show(fig, "Figure_theta.png")

    print("✓ Theta grafiği kaydedildi.")


def plot_lidar_scan(robot_pose, lidar_points):
    fig, ax = plt.subplots(figsize=(12, 8))

    draw_factory_base(ax)

    rx, ry, rtheta = robot_pose

    ax.scatter(rx, ry, s=180, color="blue", label="Robot")

    for point in lidar_points:
        ax.plot(
            [rx, point[0]],
            [ry, point[1]],
            color="cyan",
            linewidth=0.2,
            alpha=0.15
        )

    ax.scatter(
        lidar_points[:, 0],
        lidar_points[:, 1],
        s=10,
        color="red",
        label="LiDAR Noktaları"
    )

    ax.arrow(
        rx,
        ry,
        0.8 * np.cos(rtheta),
        0.8 * np.sin(rtheta),
        head_width=0.2,
        color="blue"
    )

    ax.set_title("2B LiDAR Tarama Görselleştirmesi", fontsize=14, fontweight="bold")
    ax.grid(True, alpha=0.35)
    ax.legend(loc="lower right")

    plt.tight_layout()
    save_and_show(fig, "Figure_lidar_tarama.png")

    print("✓ LiDAR tarama grafiği kaydedildi.")


def plot_raw_filtered_lidar(robot_pose, raw_points, filtered_points):
    fig, ax = plt.subplots(figsize=(12, 8))

    draw_factory_base(ax)

    rx, ry, rtheta = robot_pose

    ax.scatter(rx, ry, s=180, color="blue", label="Robot")

    ax.scatter(
        raw_points[:, 0],
        raw_points[:, 1],
        s=12,
        color="red",
        alpha=0.45,
        label="Ham LiDAR Verisi"
    )

    ax.scatter(
        filtered_points[:, 0],
        filtered_points[:, 1],
        s=18,
        color="green",
        label="Filtrelenmiş LiDAR Verisi"
    )

    ax.arrow(
        rx,
        ry,
        0.8 * np.cos(rtheta),
        0.8 * np.sin(rtheta),
        head_width=0.2,
        color="blue"
    )

    ax.set_title("Ham ve Filtrelenmiş LiDAR Verisi", fontsize=14, fontweight="bold")
    ax.grid(True, alpha=0.35)
    ax.legend(loc="lower right")

    plt.tight_layout()
    save_and_show(fig, "Figure_lidar_ham_filtreli.png")

    print("✓ Ham/filtrelenmiş LiDAR grafiği kaydedildi.")


def plot_lidar_clusters(robot_pose, clusters):
    fig, ax = plt.subplots(figsize=(12, 8))

    draw_factory_base(ax)

    rx, ry, rtheta = robot_pose

    ax.scatter(rx, ry, s=180, color="blue", label="Robot")

    for i, cluster in enumerate(clusters, start=1):
        ax.scatter(
            cluster[:, 0],
            cluster[:, 1],
            s=18,
            label=f"Küme {i}"
        )

        center_x = np.mean(cluster[:, 0])
        center_y = np.mean(cluster[:, 1])

        ax.text(
            center_x,
            center_y,
            f"C{i}",
            fontsize=9,
            fontweight="bold"
        )

    ax.arrow(
        rx,
        ry,
        0.8 * np.cos(rtheta),
        0.8 * np.sin(rtheta),
        head_width=0.2,
        color="blue"
    )

    ax.set_title("LiDAR Engel Kümeleme Sonuçları", fontsize=14, fontweight="bold")
    ax.grid(True, alpha=0.35)
    ax.legend(loc="lower right")

    plt.tight_layout()
    save_and_show(fig, "Figure_lidar_kumeleme.png")

    print("✓ LiDAR kümeleme grafiği kaydedildi.")


def plot_localization_results(true_path, dr_path, kf_path):
    fig, ax = plt.subplots(figsize=(12, 8))

    draw_factory_base(ax)

    ax.plot(true_path[:, 0], true_path[:, 1], color="red", linewidth=2.7, label="Gerçek Yol")
    ax.plot(dr_path[:, 0], dr_path[:, 1], color="orange", linestyle="--", linewidth=2, label="Dead Reckoning")
    ax.plot(kf_path[:, 0], kf_path[:, 1], color="blue", linestyle="-.", linewidth=2, label="Kalman Filtreli Tahmin")

    ax.scatter(start[0], start[1], s=200, color="green", label="Başlangıç")
    ax.scatter(goal[0], goal[1], s=250, marker="*", color="red", label="Hedef")

    ax.set_title("Lokalizasyon Sonuçları: Gerçek Yol, Dead Reckoning ve Kalman Filtresi", fontsize=13, fontweight="bold")
    ax.grid(True, alpha=0.35)
    ax.legend(bbox_to_anchor=(1.03, 1), loc="upper left")

    plt.tight_layout()
    save_and_show(fig, "Figure_lokalizasyon_2B.png")

    print("✓ Lokalizasyon grafiği kaydedildi.")


def plot_error_analysis(time_list, error_list, dr_error_list):
    rmse_kf = np.sqrt(np.mean(error_list ** 2))
    mae_kf = np.mean(np.abs(error_list))

    rmse_dr = np.sqrt(np.mean(dr_error_list ** 2))
    mae_dr = np.mean(np.abs(dr_error_list))

    smooth_kf = moving_average(error_list, 15)
    smooth_dr = moving_average(dr_error_list, 15)

    fig, ax = plt.subplots(figsize=(12, 5))

    ax.plot(
        time_list,
        smooth_dr,
        linewidth=2,
        linestyle="--",
        color="orange",
        label=f"Dead Reckoning | RMSE={rmse_dr:.3f} m, MAE={mae_dr:.3f} m"
    )

    ax.plot(
        time_list,
        smooth_kf,
        linewidth=2.2,
        color="blue",
        label=f"Kalman | RMSE={rmse_kf:.3f} m, MAE={mae_kf:.3f} m"
    )

    ax.set_title("Zaman Boyunca Konum Hatası", fontsize=13, fontweight="bold")
    ax.set_xlabel("Zaman (s)")
    ax.set_ylabel("Konum hatası (m)")
    ax.grid(True, alpha=0.35)
    ax.legend(loc="upper right")

    plt.tight_layout()
    save_and_show(fig, "Figure_hata_analizi.png")

    print("✓ Hata analizi kaydedildi.")
    print(f"Kalman RMSE: {rmse_kf:.3f} m")
    print(f"Kalman MAE : {mae_kf:.3f} m")
    print(f"Dead Reckoning RMSE: {rmse_dr:.3f} m")
    print(f"Dead Reckoning MAE : {mae_dr:.3f} m")


# ============================================================
# ANA PROGRAM
# ============================================================

if __name__ == "__main__":
    print("=" * 60)
    print(" LiDAR Tabanlı Otonom Navigasyon Simülasyonu")
    print("=" * 60)

    print("\n[1/3] Robot hareketi simüle ediliyor...")

    (
        true_path,
        dr_path,
        kf_path,
        theta_list,
        time_list,
        error_list,
        dr_error_list
    ) = simulate_robot_motion()

    print(f"Toplam adım: {len(true_path)}")

    sample_index = min(250, len(true_path) - 1)

    robot_pose = (
        true_path[sample_index][0],
        true_path[sample_index][1],
        theta_list[sample_index]
    )

    lidar_points = simulate_lidar(robot_pose)
    filtered_lidar_points = filter_lidar_points(lidar_points, robot_pose)
    lidar_clusters = cluster_lidar_points(filtered_lidar_points)

    print("\n[2/3] Grafikler oluşturuluyor...")

    plot_environment()
    plot_robot_motion(true_path)
    plot_theta(time_list, theta_list)

    plot_lidar_scan(robot_pose, lidar_points)
    plot_raw_filtered_lidar(robot_pose, lidar_points, filtered_lidar_points)
    plot_lidar_clusters(robot_pose, lidar_clusters)

    plot_localization_results(true_path, dr_path, kf_path)
    plot_error_analysis(time_list, error_list, dr_error_list)

    print("\n" + "=" * 60)
    print(" Simülasyon tamamlandı.")
    print(f" Grafikler '{OUTPUT_DIR}' klasörüne kaydedildi.")
    print("=" * 60)
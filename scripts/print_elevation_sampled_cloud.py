#!/usr/bin/env python3

import argparse
import csv
import struct
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2


class ElevationSampledCloudPrinter(Node):
    def __init__(
        self,
        topic: str,
        once: bool,
        rows: int,
        cols: int,
        show_grid: bool,
        save_csv: bool,
        csv_path: str,
    ) -> None:
        super().__init__("elevation_sampled_cloud_printer")
        self._once = once
        self._rows = rows
        self._cols = cols
        self._show_grid = show_grid
        self._save_csv = save_csv
        self._csv_path = Path(csv_path)
        self._subscription = self.create_subscription(
            PointCloud2,
            topic,
            self._callback,
            10,
        )
        self.get_logger().info(f"Subscribed to {topic}")

    def _callback(self, msg: PointCloud2) -> None:
        x_offset = None
        y_offset = None
        z_offset = None
        for field in msg.fields:
            if field.name == "x":
                x_offset = field.offset
            elif field.name == "y":
                y_offset = field.offset
            elif field.name == "z":
                z_offset = field.offset

        if x_offset is None or y_offset is None or z_offset is None:
            self.get_logger().error("PointCloud2 does not contain x/y/z fields.")
            return

        points = []
        for index in range(msg.width):
            base = index * msg.point_step
            x = struct.unpack_from("<f", msg.data, base + x_offset)[0]
            y = struct.unpack_from("<f", msg.data, base + y_offset)[0]
            z = struct.unpack_from("<f", msg.data, base + z_offset)[0]
            points.append((index, x, y, z))

        print("=" * 80)
        print(
            f"stamp={msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d} "
            f"frame_id={msg.header.frame_id} width={msg.width}"
        )

        if not points:
            print("Point cloud is empty.")
        else:
            for index, x, y, z in points:
                print(f"[{index:03d}] x={x:+.4f} y={y:+.4f} z={z:+.4f}")

            if self._show_grid:
                self._print_grid(points)

            if self._save_csv:
                self._write_csv(msg, points)

        if self._once:
            self.get_logger().info("Received one message, shutting down.")
            rclpy.shutdown()

    def _print_grid(self, points) -> None:
        print("-" * 80)
        print(f"Grid view: rows={self._rows}, cols={self._cols}")

        expected = self._rows * self._cols
        if expected != len(points):
            print(
                f"Grid size mismatch: rows*cols={expected}, point_count={len(points)}. "
                "Still printing using sequential fill."
            )

        for row in range(self._cols):
            print(f"col={row:02d}")
            for col in range(self._rows):
                linear_index = row * self._rows + col
                if linear_index >= len(points):
                    break
                index, x, y, z = points[linear_index]
                print(
                    f"  [{index:03d}] "
                    f"row_in_col={col:02d} x={x:+.4f} y={y:+.4f} z={z:+.4f}"
                )

    def _write_csv(self, msg: PointCloud2, points) -> None:
        self._csv_path.parent.mkdir(parents=True, exist_ok=True)
        with self._csv_path.open("w", newline="", encoding="utf-8") as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow(
                [
                    "index",
                    "frame_id",
                    "stamp_sec",
                    "stamp_nanosec",
                    "grid_col",
                    "row_in_col",
                    "x",
                    "y",
                    "z",
                ]
            )
            for linear_index, x, y, z in points:
                grid_col = linear_index // self._rows if self._rows > 0 else 0
                row_in_col = linear_index % self._rows if self._rows > 0 else linear_index
                writer.writerow(
                    [
                        linear_index,
                        msg.header.frame_id,
                        msg.header.stamp.sec,
                        msg.header.stamp.nanosec,
                        grid_col,
                        row_in_col,
                        f"{x:.6f}",
                        f"{y:.6f}",
                        f"{z:.6f}",
                    ]
                )
        self.get_logger().info(f"Saved CSV to {self._csv_path}")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Subscribe to sampled elevation cloud and print ordered xyz values."
    )
    parser.add_argument(
        "--topic",
        default="/elevation_sampled_cloud",
        help="PointCloud2 topic to subscribe to.",
    )
    parser.add_argument(
        "--once",
        action="store_true",
        help="Exit after printing one message.",
    )
    parser.add_argument(
        "--rows",
        type=int,
        default=26,
        help="Number of samples along each column (default: 26).",
    )
    parser.add_argument(
        "--cols",
        type=int,
        default=16,
        help="Number of columns (default: 16).",
    )
    parser.add_argument(
        "--show-grid",
        action="store_true",
        help="Also print the cloud as a 2D ordered grid view.",
    )
    parser.add_argument(
        "--save-csv",
        action="store_true",
        help="Save the parsed ordered points to a CSV file.",
    )
    parser.add_argument(
        "--csv-path",
        default="sampled_cloud.csv",
        help="CSV output path used when --save-csv is enabled.",
    )
    args = parser.parse_args()

    rclpy.init()
    node = ElevationSampledCloudPrinter(
        args.topic,
        args.once,
        max(1, args.rows),
        max(1, args.cols),
        args.show_grid,
        args.save_csv,
        args.csv_path,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

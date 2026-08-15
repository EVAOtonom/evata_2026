#!/usr/bin/env python3

import struct
from pathlib import Path

import numpy as np


# ============================================================
# Configuration
# ============================================================

PCD_FILE = Path("map.pcd")

# Output files
OUTPUT_HIGH_INTENSITY = Path("high_intensity_points.pcd")
OUTPUT_GROUND_CANDIDATES = Path("ground_high_intensity.pcd")

# Intensity threshold.
# Start with percentile-based threshold instead of guessing a fixed value.
INTENSITY_PERCENTILE = 27.5

# Ground filtering band.
# IMPORTANT:
# These values are only initial test values.
# We will adjust them after seeing your actual map coordinates.
USE_GROUND_FILTER = True

GROUND_Z_MIN = -5.50
GROUND_Z_MAX = 0.20


# ============================================================
# PCD reader
# ============================================================

def read_binary_pcd(path: Path):
    """
    Read a binary PCD containing:

        x y z intensity

    All fields are float32.
    """

    with path.open("rb") as f:
        header_lines = []

        while True:
            line = f.readline()

            if not line:
                raise RuntimeError("PCD header ended unexpectedly.")

            decoded = line.decode("ascii", errors="ignore").strip()
            header_lines.append(decoded)

            if decoded.startswith("DATA"):
                break

        header = {}

        for line in header_lines:
            parts = line.split(maxsplit=1)

            if len(parts) == 2:
                header[parts[0].upper()] = parts[1]

        if header.get("DATA", "").lower() != "binary":
            raise RuntimeError(
                "This script currently expects DATA binary."
            )

        fields = header["FIELDS"].split()
        sizes = [int(x) for x in header["SIZE"].split()]
        types = header["TYPE"].split()
        counts = [int(x) for x in header["COUNT"].split()]

        width = int(header["WIDTH"])
        height = int(header.get("HEIGHT", "1"))
        points = int(header["POINTS"])

        print("\n=== PCD HEADER ===")
        print(f"File   : {path}")
        print(f"Fields : {fields}")
        print(f"Points : {points}")
        print(f"Width  : {width}")
        print(f"Height : {height}")
        print(f"Data   : {header['DATA']}")
        print("==================\n")

        expected_fields = ["x", "y", "z", "intensity"]

        if fields != expected_fields:
            raise RuntimeError(
                f"Expected fields {expected_fields}, got {fields}"
            )

        if sizes != [4, 4, 4, 4]:
            raise RuntimeError(
                f"Expected SIZE 4 4 4 4, got {sizes}"
            )

        if types != ["F", "F", "F", "F"]:
            raise RuntimeError(
                f"Expected TYPE F F F F, got {types}"
            )

        if counts != [1, 1, 1, 1]:
            raise RuntimeError(
                f"Expected COUNT 1 1 1 1, got {counts}"
            )

        data = f.read()

    expected_bytes = points * 4 * 4

    if len(data) < expected_bytes:
        raise RuntimeError(
            f"Not enough binary data. "
            f"Expected {expected_bytes} bytes, got {len(data)}."
        )

    # Little-endian float32:
    dtype = np.dtype([
        ("x", "<f4"),
        ("y", "<f4"),
        ("z", "<f4"),
        ("intensity", "<f4"),
    ])

    cloud = np.frombuffer(
        data[:expected_bytes],
        dtype=dtype,
        count=points,
    )

    return cloud


# ============================================================
# PCD writer
# ============================================================

def write_binary_pcd(path: Path, cloud):
    """
    Write x y z intensity as binary PCD.
    """

    points = len(cloud)

    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z intensity\n"
        "SIZE 4 4 4 4\n"
        "TYPE F F F F\n"
        "COUNT 1 1 1 1\n"
        f"WIDTH {points}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {points}\n"
        "DATA binary\n"
    )

    with path.open("wb") as f:
        f.write(header.encode("ascii"))
        f.write(cloud.tobytes())

    print(f"Saved: {path}")
    print(f"Points: {points}")


# ============================================================
# Statistics
# ============================================================

def print_statistics(intensity):
    finite = intensity[np.isfinite(intensity)]

    if finite.size == 0:
        raise RuntimeError("No finite intensity values found.")

    percentiles = [0, 1, 5, 10, 25, 50, 75, 90, 95, 97, 98, 99, 99.5, 100]

    print("\n=== INTENSITY STATISTICS ===")

    print(f"Count : {finite.size}")
    print(f"Min   : {np.min(finite):.6f}")
    print(f"Max   : {np.max(finite):.6f}")
    print(f"Mean  : {np.mean(finite):.6f}")
    print(f"Median: {np.median(finite):.6f}")

    print("\nPercentiles:")
    for p in percentiles:
        value = np.percentile(finite, p)
        print(f"P{p:<5}: {value:.6f}")

    print("============================\n")


# ============================================================
# Main
# ============================================================

def main():
    if not PCD_FILE.exists():
        raise FileNotFoundError(
            f"PCD file not found: {PCD_FILE.resolve()}"
        )

    cloud = read_binary_pcd(PCD_FILE)

    x = cloud["x"]
    y = cloud["y"]
    z = cloud["z"]
    intensity = cloud["intensity"]

    print_statistics(intensity)

    finite_mask = (
        np.isfinite(x)
        & np.isfinite(y)
        & np.isfinite(z)
        & np.isfinite(intensity)
    )

    finite_intensity = intensity[finite_mask]

    threshold = np.percentile(
        finite_intensity,
        INTENSITY_PERCENTILE,
    )

    print(
        f"Using P{INTENSITY_PERCENTILE:.1f} "
        f"intensity threshold = {threshold:.6f}"
    )

    high_mask = finite_mask & (intensity >= threshold)

    print(
        f"High-intensity points: "
        f"{np.count_nonzero(high_mask):,} / {len(cloud):,}"
    )

    print(
        f"Percentage: "
        f"{100.0 * np.count_nonzero(high_mask) / len(cloud):.3f}%"
    )

    # --------------------------------------------------------
    # High-intensity points
    # --------------------------------------------------------

    high_cloud = cloud[high_mask].copy()

    write_binary_pcd(
        OUTPUT_HIGH_INTENSITY,
        high_cloud,
    )

    # --------------------------------------------------------
    # Optional ground filter
    # --------------------------------------------------------

    if USE_GROUND_FILTER:
        ground_mask = (
            high_mask
            & (z >= GROUND_Z_MIN)
            & (z <= GROUND_Z_MAX)
        )

        ground_cloud = cloud[ground_mask].copy()

        print(
            "\nGround + high-intensity points: "
            f"{len(ground_cloud):,}"
        )

        write_binary_pcd(
            OUTPUT_GROUND_CANDIDATES,
            ground_cloud,
        )

    # --------------------------------------------------------
    # Simple XY bounding box
    # --------------------------------------------------------

    print("\n=== MAP BOUNDS ===")
    print(f"X: {np.nanmin(x):.3f} -> {np.nanmax(x):.3f}")
    print(f"Y: {np.nanmin(y):.3f} -> {np.nanmax(y):.3f}")
    print(f"Z: {np.nanmin(z):.3f} -> {np.nanmax(z):.3f}")
    print("==================\n")

    print("Done.")


if __name__ == "__main__":
    main()

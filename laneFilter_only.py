#!/usr/bin/env python3

from pathlib import Path

import numpy as np

try:
    from scipy.ndimage import (
        median_filter,
        binary_opening,
        binary_closing,
        label,
    )
except ImportError:
    raise SystemExit(
        "\nSciPy is required.\n"
        "Install it with:\n\n"
        "    pip3 install scipy\n"
    )


# ============================================================
# CONFIGURATION
# ============================================================

PCD_FILE = Path("map.pcd")

# ------------------------------------------------------------
# Output files
# ------------------------------------------------------------

OUTPUT_GROUND = Path("ground_only.pcd")
OUTPUT_HIGH_INTENSITY = Path("ground_high_intensity.pcd")
OUTPUT_CONTRAST = Path("lane_candidates.pcd")


# ============================================================
# 1. Z FILTER
# ============================================================

# Keep only points inside this Z range.
#
# IMPORTANT:
# You requested:
#
#     minimum Z = -5.0
#
# Therefore:
#
#     -5.0 <= Z <= +0.30
#
# Points below -5.0 m are removed.
#
GROUND_Z_MIN = -50.0
GROUND_Z_MAX = 0.30


# ============================================================
# 2. XY GRID
# ============================================================

# Resolution of the XY intensity image.

GRID_RESOLUTION = 0.05  # meters


# ============================================================
# 3. ABSOLUTE INTENSITY FILTER
# ============================================================

# Keep points above this percentile of the intensity
# values of the filtered Z region.
#
# Example:
#
# 70 -> highest ~30%
# 80 -> highest ~20%
# 90 -> highest ~10%
#
# Keep this at 70 initially.

ABSOLUTE_INTENSITY_PERCENTILE = 50.0


# ============================================================
# 4. LOCAL CONTRAST FILTER
# ============================================================

# Local neighborhood size in grid cells.
#
# At 0.05 m resolution:
#
# 21 cells = approximately 1.05 m

LOCAL_MEDIAN_SIZE = 21


# Minimum intensity difference relative to local median.

MIN_LOCAL_CONTRAST = 10.0


# ============================================================
# 5. MORPHOLOGY
# ============================================================

MORPHOLOGY_SIZE = 1


# Minimum connected component size.

MIN_CLUSTER_CELLS = 8


# ============================================================
# PCD READER
# ============================================================

def read_binary_pcd(path: Path):
    """
    Read binary PCD with:

        x y z intensity

    All fields are float32.
    """

    if not path.exists():
        raise FileNotFoundError(
            f"PCD file not found:\n{path.resolve()}"
        )

    with path.open("rb") as f:

        header_lines = []

        while True:

            line = f.readline()

            if not line:
                raise RuntimeError(
                    "PCD header ended unexpectedly."
                )

            decoded = line.decode(
                "ascii",
                errors="ignore"
            ).strip()

            header_lines.append(decoded)

            if decoded.upper().startswith("DATA"):
                break

        header = {}

        for line in header_lines:

            parts = line.split(maxsplit=1)

            if len(parts) == 2:
                header[
                    parts[0].upper()
                ] = parts[1]

        if header.get("DATA", "").lower() != "binary":
            raise RuntimeError(
                "This script expects DATA binary."
            )

        fields = header["FIELDS"].split()

        sizes = [
            int(v)
            for v in header["SIZE"].split()
        ]

        types = header["TYPE"].split()

        counts = [
            int(v)
            for v in header["COUNT"].split()
        ]

        points = int(header["POINTS"])

        print()
        print("=" * 60)
        print("PCD HEADER")
        print("=" * 60)

        print("File   :", path)
        print("Fields :", fields)
        print("SIZE   :", sizes)
        print("TYPE   :", types)
        print("COUNT  :", counts)
        print("Points :", points)
        print("DATA   :", header["DATA"])

        print("=" * 60)

        expected_fields = [
            "x",
            "y",
            "z",
            "intensity"
        ]

        if fields != expected_fields:
            raise RuntimeError(
                f"\nExpected fields:\n"
                f"{expected_fields}\n\n"
                f"Found:\n"
                f"{fields}\n"
            )

        if sizes != [4, 4, 4, 4]:
            raise RuntimeError(
                f"Unexpected SIZE: {sizes}"
            )

        if types != [
            "F",
            "F",
            "F",
            "F"
        ]:
            raise RuntimeError(
                f"Unexpected TYPE: {types}"
            )

        data = f.read()

    dtype = np.dtype([
        ("x", "<f4"),
        ("y", "<f4"),
        ("z", "<f4"),
        ("intensity", "<f4"),
    ])

    bytes_per_point = 16

    expected_bytes = (
        points * bytes_per_point
    )

    if len(data) < expected_bytes:
        raise RuntimeError(
            f"Binary data is shorter than expected.\n"
            f"Expected: {expected_bytes}\n"
            f"Received: {len(data)}"
        )

    cloud = np.frombuffer(
        data[:expected_bytes],
        dtype=dtype,
        count=points
    ).copy()

    return cloud


# ============================================================
# PCD WRITER
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
        f.write(
            header.encode("ascii")
        )

        f.write(
            cloud.tobytes()
        )

    print()
    print(
        f"Saved: {path.resolve()}"
    )

    print(
        f"Points: {points:,}"
    )


# ============================================================
# STATISTICS
# ============================================================

def print_statistics(values, title):

    values = values[
        np.isfinite(values)
    ]

    if len(values) == 0:

        print(
            f"\n{title}: no valid values."
        )

        return

    percentiles = [
        0,
        10,
        25,
        50,
        70,
        75,
        80,
        90,
        95,
        97,
        99,
        100
    ]

    print()
    print("=" * 60)
    print(title)
    print("=" * 60)

    print(
        f"Count  : {len(values):,}"
    )

    print(
        f"Min    : {np.min(values):.4f}"
    )

    print(
        f"Max    : {np.max(values):.4f}"
    )

    print(
        f"Mean   : {np.mean(values):.4f}"
    )

    print(
        f"Median : {np.median(values):.4f}"
    )

    print()
    print("Percentiles:")

    for p in percentiles:

        value = np.percentile(
            values,
            p
        )

        print(
            f"P{p:<3} = {value:.4f}"
        )

    print("=" * 60)


# ============================================================
# BUILD XY GRID
# ============================================================

def create_intensity_grid(
    ground_cloud,
    resolution
):

    x = ground_cloud["x"]
    y = ground_cloud["y"]
    intensity = ground_cloud["intensity"]

    min_x = np.min(x)
    max_x = np.max(x)

    min_y = np.min(y)
    max_y = np.max(y)

    width = int(
        np.ceil(
            (max_x - min_x)
            / resolution
        )
    ) + 1

    height = int(
        np.ceil(
            (max_y - min_y)
            / resolution
        )
    ) + 1

    print()
    print("=" * 60)
    print("XY GRID")
    print("=" * 60)

    print(
        f"Resolution : {resolution:.3f} m"
    )

    print(
        f"X range    : "
        f"{min_x:.3f} -> {max_x:.3f}"
    )

    print(
        f"Y range    : "
        f"{min_y:.3f} -> {max_y:.3f}"
    )

    print(
        f"Grid width : {width}"
    )

    print(
        f"Grid height: {height}"
    )

    print(
        f"Grid cells : {width * height:,}"
    )

    # --------------------------------------------------------
    # Convert points into grid indexes
    # --------------------------------------------------------

    ix = (
        (x - min_x)
        / resolution
    ).astype(np.int32)

    iy = (
        (y - min_y)
        / resolution
    ).astype(np.int32)

    valid = (
        (ix >= 0)
        & (ix < width)
        & (iy >= 0)
        & (iy < height)
    )

    ix = ix[valid]
    iy = iy[valid]
    intensity = intensity[valid]

    # --------------------------------------------------------
    # Initialize grid
    # --------------------------------------------------------

    intensity_grid = np.full(
        (height, width),
        np.nan,
        dtype=np.float32
    )

    # Flatten indexes

    flat_index = (
        iy.astype(np.int64) * width
        + ix.astype(np.int64)
    )

    order = np.argsort(
        flat_index
    )

    flat_sorted = flat_index[
        order
    ]

    intensity_sorted = intensity[
        order
    ]

    unique_cells, first_indices = np.unique(
        flat_sorted,
        return_index=True
    )

    max_values = np.empty(
        len(unique_cells),
        dtype=np.float32
    )

    start_positions = list(
        first_indices
    )

    start_positions.append(
        len(flat_sorted)
    )

    # --------------------------------------------------------
    # Maximum intensity per grid cell
    # --------------------------------------------------------

    for i in range(
        len(unique_cells)
    ):

        start = (
            start_positions[i]
        )

        end = (
            start_positions[i + 1]
        )

        max_values[i] = np.max(
            intensity_sorted[
                start:end
            ]
        )

    gy = (
        unique_cells // width
    ).astype(np.int32)

    gx = (
        unique_cells % width
    ).astype(np.int32)

    intensity_grid[
        gy,
        gx
    ] = max_values

    occupied = np.isfinite(
        intensity_grid
    )

    print(
        f"Occupied cells: "
        f"{np.count_nonzero(occupied):,}"
    )

    print("=" * 60)

    return (
        intensity_grid,
        min_x,
        min_y
    )


# ============================================================
# LOCAL CONTRAST
# ============================================================

def calculate_local_contrast(
    intensity_grid,
    median_size
):

    print()
    print("=" * 60)
    print("LOCAL CONTRAST")
    print("=" * 60)

    print(
        f"Median window: "
        f"{median_size} x {median_size}"
    )

    occupied = np.isfinite(
        intensity_grid
    )

    filled = np.nan_to_num(
        intensity_grid,
        nan=0.0
    )

    local_median = median_filter(
        filled,
        size=median_size,
        mode="nearest"
    )

    contrast = (
        intensity_grid
        - local_median
    )

    contrast[
        ~occupied
    ] = np.nan

    finite_contrast = contrast[
        np.isfinite(contrast)
    ]

    if len(finite_contrast) > 0:

        print(
            f"Contrast min  : "
            f"{np.min(finite_contrast):.3f}"
        )

        print(
            f"Contrast max  : "
            f"{np.max(finite_contrast):.3f}"
        )

        print(
            f"Contrast mean : "
            f"{np.mean(finite_contrast):.3f}"
        )

        print(
            f"Contrast P95  : "
            f"{np.percentile(finite_contrast, 95):.3f}"
        )

    print("=" * 60)

    return contrast


# ============================================================
# BUILD CANDIDATE MASK
# ============================================================

def create_candidate_mask(
    intensity_grid,
    contrast,
    intensity_threshold,
    minimum_contrast
):

    occupied = np.isfinite(
        intensity_grid
    )

    mask = (
        occupied
        & (
            intensity_grid
            >= intensity_threshold
        )
        & (
            contrast
            >= minimum_contrast
        )
    )

    return mask


# ============================================================
# MORPHOLOGICAL CLEANUP
# ============================================================

def clean_mask(mask):

    if MORPHOLOGY_SIZE <= 0:
        return mask

    size = (
        2 * MORPHOLOGY_SIZE
        + 1
    )

    structure = np.ones(
        (size, size),
        dtype=bool
    )

    print()
    print("=" * 60)
    print("MORPHOLOGY")
    print("=" * 60)

    print(
        f"Kernel: {size} x {size}"
    )

    # Opening

    cleaned = binary_opening(
        mask,
        structure=structure
    )

    # Closing

    cleaned = binary_closing(
        cleaned,
        structure=structure
    )

    print(
        f"Before: "
        f"{np.count_nonzero(mask):,} cells"
    )

    print(
        f"After : "
        f"{np.count_nonzero(cleaned):,} cells"
    )

    print("=" * 60)

    return cleaned


# ============================================================
# REMOVE SMALL CLUSTERS
# ============================================================

def remove_small_clusters(mask):

    print()
    print("=" * 60)
    print("CONNECTED COMPONENT FILTER")
    print("=" * 60)

    labels, num_labels = label(
        mask
    )

    print(
        f"Detected clusters: "
        f"{num_labels:,}"
    )

    if num_labels == 0:
        return mask

    component_sizes = np.bincount(
        labels.ravel()
    )

    keep_mask = np.zeros(
        mask.shape,
        dtype=bool
    )

    kept_clusters = 0

    for component_id in range(
        1,
        num_labels + 1
    ):

        size = (
            component_sizes[
                component_id
            ]
        )

        if size >= MIN_CLUSTER_CELLS:

            keep_mask[
                labels == component_id
            ] = True

            kept_clusters += 1

    print(
        f"Kept clusters: "
        f"{kept_clusters:,}"
    )

    print(
        f"Minimum size : "
        f"{MIN_CLUSTER_CELLS} cells"
    )

    print("=" * 60)

    return keep_mask


# ============================================================
# GRID -> ORIGINAL POINTS
# ============================================================

def mask_original_points(
    cloud,
    candidate_grid,
    min_x,
    min_y,
    resolution
):

    x = cloud["x"]
    y = cloud["y"]

    height, width = (
        candidate_grid.shape
    )

    ix = (
        (x - min_x)
        / resolution
    ).astype(np.int32)

    iy = (
        (y - min_y)
        / resolution
    ).astype(np.int32)

    valid = (
        (ix >= 0)
        & (ix < width)
        & (iy >= 0)
        & (iy < height)
    )

    result_mask = np.zeros(
        len(cloud),
        dtype=bool
    )

    valid_indices = np.where(
        valid
    )[0]

    result_mask[
        valid_indices
    ] = candidate_grid[
        iy[valid],
        ix[valid]
    ]

    return result_mask


# ============================================================
# MAIN
# ============================================================

def main():

    # --------------------------------------------------------
    # LOAD PCD
    # --------------------------------------------------------

    cloud = read_binary_pcd(
        PCD_FILE
    )

    print()
    print("=" * 60)
    print("LOADED POINT CLOUD")
    print("=" * 60)

    print(
        f"Total points: "
        f"{len(cloud):,}"
    )

    print("=" * 60)

    # --------------------------------------------------------
    # Fields
    # --------------------------------------------------------

    x = cloud["x"]
    y = cloud["y"]
    z = cloud["z"]
    intensity = cloud["intensity"]

    # --------------------------------------------------------
    # Finite points
    # --------------------------------------------------------

    finite_mask = (
        np.isfinite(x)
        & np.isfinite(y)
        & np.isfinite(z)
        & np.isfinite(intensity)
    )

    # --------------------------------------------------------
    # Map bounds
    # --------------------------------------------------------

    valid_cloud = cloud[
        finite_mask
    ]

    print()
    print("=" * 60)
    print("MAP BOUNDS")
    print("=" * 60)

    print(
        f"X: "
        f"{np.min(valid_cloud['x']):.3f}"
        f" -> "
        f"{np.max(valid_cloud['x']):.3f}"
    )

    print(
        f"Y: "
        f"{np.min(valid_cloud['y']):.3f}"
        f" -> "
        f"{np.max(valid_cloud['y']):.3f}"
    )

    print(
        f"Z: "
        f"{np.min(valid_cloud['z']):.3f}"
        f" -> "
        f"{np.max(valid_cloud['z']):.3f}"
    )

    print("=" * 60)

    # ========================================================
    # STEP 1
    # Z FILTER
    # ========================================================

    ground_mask = (
        finite_mask
        & (z >= GROUND_Z_MIN)
        & (z <= GROUND_Z_MAX)
    )

    ground_cloud = cloud[
        ground_mask
    ].copy()

    print()
    print("=" * 60)
    print("Z FILTER")
    print("=" * 60)

    print(
        f"Keeping:"
    )

    print(
        f"  {GROUND_Z_MIN:.3f}"
        f" <= Z <= "
        f"{GROUND_Z_MAX:.3f}"
    )

    print(
        f"Original points : "
        f"{len(cloud):,}"
    )

    print(
        f"Filtered points : "
        f"{len(ground_cloud):,}"
    )

    print(
        f"Percentage      : "
        f"{100.0 * len(ground_cloud) / len(cloud):.3f}%"
    )

    print("=" * 60)

    if len(ground_cloud) == 0:

        raise RuntimeError(
            "\nNo points remain after Z filtering.\n"
            "Check GROUND_Z_MIN and "
            "GROUND_Z_MAX."
        )

    # --------------------------------------------------------
    # SAVE Z FILTER RESULT
    # --------------------------------------------------------

    write_binary_pcd(
        OUTPUT_GROUND,
        ground_cloud
    )

    # ========================================================
    # STEP 2
    # INTENSITY STATISTICS
    # ========================================================

    print_statistics(
        ground_cloud["intensity"],
        "FILTERED REGION INTENSITY"
    )

    # ========================================================
    # STEP 3
    # ABSOLUTE INTENSITY
    # ========================================================

    intensity_threshold = np.percentile(
        ground_cloud["intensity"],
        ABSOLUTE_INTENSITY_PERCENTILE
    )

    print()
    print("=" * 60)
    print("ABSOLUTE INTENSITY FILTER")
    print("=" * 60)

    print(
        f"Percentile : "
        f"P{ABSOLUTE_INTENSITY_PERCENTILE}"
    )

    print(
        f"Threshold  : "
        f"{intensity_threshold:.6f}"
    )

    print("=" * 60)

    high_mask_points = (
        ground_cloud["intensity"]
        >= intensity_threshold
    )

    high_intensity_cloud = (
        ground_cloud[
            high_mask_points
        ].copy()
    )

    print(
        f"\nHigh-intensity points: "
        f"{len(high_intensity_cloud):,}"
    )

    write_binary_pcd(
        OUTPUT_HIGH_INTENSITY,
        high_intensity_cloud
    )

    # ========================================================
    # STEP 4
    # XY GRID
    # ========================================================

    (
        intensity_grid,
        min_x,
        min_y
    ) = create_intensity_grid(
        ground_cloud,
        GRID_RESOLUTION
    )

    # ========================================================
    # STEP 5
    # LOCAL CONTRAST
    # ========================================================

    contrast = calculate_local_contrast(
        intensity_grid,
        LOCAL_MEDIAN_SIZE
    )

    # ========================================================
    # STEP 6
    # CANDIDATE MASK
    # ========================================================

    candidate_mask = create_candidate_mask(
        intensity_grid,
        contrast,
        intensity_threshold,
        MIN_LOCAL_CONTRAST
    )

    print()
    print("=" * 60)
    print("RAW CANDIDATES")
    print("=" * 60)

    print(
        f"Candidate cells: "
        f"{np.count_nonzero(candidate_mask):,}"
    )

    print("=" * 60)

    # ========================================================
    # STEP 7
    # MORPHOLOGICAL CLEANING
    # ========================================================

    candidate_mask = clean_mask(
        candidate_mask
    )

    # ========================================================
    # STEP 8
    # CLUSTER FILTER
    # ========================================================

    candidate_mask = (
        remove_small_clusters(
            candidate_mask
        )
    )

    # ========================================================
    # STEP 9
    # GRID -> ORIGINAL POINT CLOUD
    # ========================================================

    original_candidate_mask = (
        mask_original_points(
            ground_cloud,
            candidate_mask,
            min_x,
            min_y,
            GRID_RESOLUTION
        )
    )

    lane_candidates = (
        ground_cloud[
            original_candidate_mask
        ].copy()
    )

    # ========================================================
    # STEP 10
    # SAVE FINAL CANDIDATES
    # ========================================================

    print()
    print("=" * 60)
    print("FINAL LANE CANDIDATES")
    print("=" * 60)

    print(
        f"Candidate points: "
        f"{len(lane_candidates):,}"
    )

    if len(ground_cloud) > 0:

        print(
            f"Percentage of filtered points: "
            f"{100.0 * len(lane_candidates) / len(ground_cloud):.3f}%"
        )

    print("=" * 60)

    write_binary_pcd(
        OUTPUT_CONTRAST,
        lane_candidates
    )

    # ========================================================
    # SUMMARY
    # ========================================================

    print()
    print("=" * 60)
    print("COMPLETE")
    print("=" * 60)

    print(
        f"Input:"
    )

    print(
        f"  {PCD_FILE.resolve()}"
    )

    print()

    print(
        f"Z filtered:"
    )

    print(
        f"  {OUTPUT_GROUND.resolve()}"
    )

    print()

    print(
        f"High intensity:"
    )

    print(
        f"  {OUTPUT_HIGH_INTENSITY.resolve()}"
    )

    print()

    print(
        f"Lane candidates:"
    )

    print(
        f"  {OUTPUT_CONTRAST.resolve()}"
    )

    print("=" * 60)


# ============================================================
# ENTRY POINT
# ============================================================

if __name__ == "__main__":
    main()
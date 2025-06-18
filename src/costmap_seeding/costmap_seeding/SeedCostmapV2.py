import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_srvs.srv import Trigger  # A simple service for triggering operations

import requests
import os
from bs4 import BeautifulSoup
import rasterio
from rasterio.transform import Affine  # Explicitly import Affine
import folium
import zipfile
import xml.etree.ElementTree as ET
import shutil
import yaml  # For YAML file handling

# Updated imports for coordinate transformations
from rasterio.crs import CRS
from pyproj import Transformer  # Import Transformer from pyproj

import matplotlib.pyplot as plt
import numpy as np

# Import for 3D plotting
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm  # Colormap module
import scipy.ndimage  # Import for image processing operations (blur, zoom)
import math  # For math.ceil

# --- Utility Functions ---


def download_file_from_url(url, destination_folder):
    """
    Downloads a single file from a given URL to a specified folder.
    This version does not include a progress bar, only basic status messages.

    Args:
        url (str): The URL of the file to download.
        destination_folder (str): The local folder where the file will be saved.
                                  It will be created if it doesn't exist.

    Returns:
        str: The full path to the downloaded file, or None if download fails.
    """
    try:
        # Ensure the destination folder exists
        if not os.path.exists(destination_folder):
            os.makedirs(destination_folder)

        # Extract filename from the URL
        filename = os.path.join(destination_folder, url.split("/")[-1])

        if os.path.exists(filename):
            print(f"File '{filename}' already exists. Skipping download.")
            return filename

        print(f"Downloading: {url}")
        response = requests.get(url, stream=True)
        response.raise_for_status()  # Raise an exception for bad status codes (4xx or 5xx)

        with open(filename, "wb") as f:
            for chunk in response.iter_content(chunk_size=1024):
                if chunk:  # filter out keep-alive new chunks
                    f.write(chunk)

        print(f"Successfully downloaded '{filename}'")
        return filename

    except requests.exceptions.RequestException as e:
        print(f"Error downloading {url}: {e}")
        return None
    except Exception as e:
        print(f"An unexpected error occurred during download of {filename}: {e}")
        return None


def list_dtm_urls_from_kml(
    base_directory_url,
    kml_filename="INDEX_utm12_AB_Drumheller.kmz",
    local_temp_path="temp_kml_data",
):
    """
    Downloads and parses a KMZ file from the base directory to extract DTM .tif URLs.

    Args:
        base_directory_url (str): The base URL containing the KMZ file.
        kml_filename (str): The name of the KMZ file.
        local_temp_path (str): Temporary folder to store the KMZ and its extracted content.

    Returns:
        list: A list of absolute URLs to DTM .tif files.
    """
    dtm_urls = []
    kml_url = requests.compat.urljoin(base_directory_url, kml_filename)

    # Ensure temporary folder exists
    if not os.path.exists(local_temp_path):
        os.makedirs(local_temp_path)

    # 1. Download the KMZ file
    kmz_local_path = download_file_from_url(kml_url, local_temp_path)
    if not kmz_local_path:
        print(f"Failed to download KMZ file from {kml_url}")
        return []

    # 2. Extract KML from KMZ
    kml_extracted_path = None
    try:
        with zipfile.ZipFile(kmz_local_path, "r") as zip_ref:
            # KMZ files usually contain a 'doc.kml' or similar at the root
            kml_files = [f for f in zip_ref.namelist() if f.lower().endswith(".kml")]
            if not kml_files:
                print(f"No KML file found inside {kmz_local_path}")
                return []

            # Assuming the main KML file is the first one found (often 'doc.kml')
            kml_file_in_zip = kml_files[0]
            zip_ref.extract(kml_file_in_zip, local_temp_path)
            kml_extracted_path = os.path.join(local_temp_path, kml_file_in_zip)
            print(f"Extracted KML to: {kml_extracted_path}")
    except zipfile.BadZipFile:
        print(f"Error: {kmz_local_path} is not a valid KMZ file.")
        return []
    except Exception as e:
        print(f"Error extracting KML from KMZ: {e}")
        return []

    # 3. Parse the KML file to find DTM links
    if kml_extracted_path:
        try:
            tree = ET.parse(kml_extracted_path)
            root = tree.getroot()

            # KML uses XML namespaces. Define the default KML namespace.
            ns = {"kml": "http://www.opengis.net/kml/2.2"}

            # Iterate through all Placemark elements
            for placemark in root.findall(".//kml:Placemark", ns):
                description_element = placemark.find("kml:description", ns)
                if description_element is not None and description_element.text:
                    # The description contains CDATA with HTML links. Use BeautifulSoup to parse it.
                    description_html = description_element.text
                    soup = BeautifulSoup(description_html, "html.parser")

                    # Find the <a> tag whose text is "Digital Terrain Model" or its French equivalent
                    for a_tag in soup.find_all("a", href=True):
                        link_text = a_tag.get_text(strip=True)
                        if (
                            "Digital Terrain Model" in link_text
                            or "Modèle numérique de terrain" in link_text
                        ):
                            dtm_url = a_tag["href"]
                            if dtm_url:
                                dtm_urls.append(dtm_url)
                            break  # Found the DTM link for this placemark, move to the next placemark
        except ET.ParseError as e:
            print(f"Error parsing KML file {kml_extracted_path}: {e}")
        except Exception as e:
            print(f"An unexpected error occurred while parsing KML: {e}")

    return dtm_urls


def download_dtm_data_from_kml_index(
    base_data_directory_url, local_temp_kml_path, output_download_folder
):
    """
    Downloads all available DTM GeoTIFF files based on a KML index.

    Args:
        base_data_directory_url (str): The base URL containing the KMZ index file.
        local_temp_kml_path (str): Temporary folder to store the KMZ and its extracted KML content.
        output_download_folder (str): The local folder to save all downloaded GeoTIFFs.

    Returns:
        list: A list of file paths to the successfully downloaded DTM GeoTIFFs.
    """
    all_dtm_tif_urls = list_dtm_urls_from_kml(
        base_data_directory_url, local_temp_path=local_temp_kml_path
    )

    if not all_dtm_tif_urls:
        print("No DTM .tif URLs found from KMZ. Cannot download any data.")
        return []

    print(
        f"Found {len(all_dtm_tif_urls)} DTM .tif URLs. Starting download of all files."
    )

    downloaded_filepaths = []
    all_dtm_tif_urls.sort()  # Ensure consistent download order
    for tif_url in all_dtm_tif_urls:
        downloaded_path = download_file_from_url(tif_url, output_download_folder)
        if downloaded_path:
            downloaded_filepaths.append(downloaded_path)

    # Clean up temporary KML folder
    if not local_temp_kml_path:  # Check if local_temp_kml_path is not empty or None
        if os.path.exists(local_temp_kml_path):
            try:
                shutil.rmtree(local_temp_kml_path)
                print(f"Cleaned up temporary folder: {local_temp_kml_path}")
            except OSError as e:
                print(f"Error removing temporary folder {local_temp_kml_path}: {e}")

    return downloaded_filepaths


def get_elevation_grid_around_point(
    target_lat, target_lon, grid_size_km, downloaded_filepaths
):
    """
    Loads 1-meter resolution elevation data for a square grid centered on a lat/lon point,
    grabbing from the necessary local GeoTIFF files. Handles multiple intersecting tiles.

    Args:
        target_lat (float): Latitude of the center of the desired grid.
        target_lon (float): Longitude of the center of the desired grid.
        grid_size_km (float): The side length of the square grid in kilometers (e.g., 2 for 2km x 2km).
        downloaded_filepaths (list): A list of local paths to the downloaded DTM GeoTIFF files.

    Returns:
        tuple: (elevation_array, grid_transform, grid_crs, actual_center_lat, actual_center_lon, resolution_m)
               Returns (None, None, None, None, None, None) if data cannot be extracted.
    """
    grid_size_m = int(grid_size_km * 1000)
    resolution_m = 1  # DTM data is 1m resolution

    # Define the target CRS (UTM Zone 12N for Drumheller)
    target_crs = CRS.from_epsg(2956)  # NAD83 / UTM zone 12N

    # Create a transformer from WGS84 (EPSG:4326) to the target UTM CRS
    # always_xy=True ensures output is (x, y) = (lon, lat)
    utm_transformer = Transformer.from_crs("EPSG:4326", target_crs, always_xy=True)
    target_x, target_y = utm_transformer.transform(target_lon, target_lat)

    # Define the target grid's bounds in UTM
    utm_min_x = target_x - grid_size_m / 2
    utm_max_x = target_x + grid_size_m / 2
    utm_min_y = target_y - grid_size_m / 2
    utm_max_y = target_y + grid_size_m / 2

    # Prepare an empty array for the output grid
    output_shape = (grid_size_m, grid_size_m)
    output_elevation_array = np.full(
        output_shape, np.nan, dtype=np.float32
    )  # Initialize with NaN

    # Calculate the transform for the output array
    output_utm_top_left_x = utm_min_x
    output_utm_top_left_y = utm_max_y  # Y increases upwards, so top is max_y

    output_transform = Affine(
        resolution_m, 0, output_utm_top_left_x, 0, -resolution_m, output_utm_top_left_y
    )  # -resolution_m because Y decreases as row increases

    found_data = False

    for fp in downloaded_filepaths:
        try:
            with rasterio.open(fp) as src:
                # Check if the current tile's CRS matches the target CRS
                if src.crs == target_crs:
                    # Calculate window (slice) of the source dataset that overlaps with the target bounds
                    intersection_window = src.window(
                        utm_min_x, utm_min_y, utm_max_x, utm_max_y
                    )

                    # Ensure there is an actual overlap
                    if intersection_window.width > 0 and intersection_window.height > 0:
                        # Read data from the intersecting part of the source tile
                        source_data_chunk = src.read(1, window=intersection_window)

                        # Determine the pixel coordinates in the *output_elevation_array* where this chunk should be pasted
                        chunk_top_left_utm_x, chunk_top_left_utm_y = src.transform * (
                            intersection_window.col_off,
                            intersection_window.row_off,
                        )

                        dest_col_start = int(
                            round(
                                (chunk_top_left_utm_x - output_utm_top_left_x)
                                / resolution_m
                            )
                        )
                        dest_row_start = int(
                            round(
                                (output_utm_top_left_y - chunk_top_left_utm_y)
                                / resolution_m
                            )
                        )

                        paste_cols = source_data_chunk.shape[1]
                        paste_rows = source_data_chunk.shape[0]

                        # Adjust paste dimensions if it goes beyond the output array boundaries
                        paste_cols = min(paste_cols, output_shape[1] - dest_col_start)
                        paste_rows = min(paste_rows, output_shape[0] - dest_row_start)

                        if paste_cols > 0 and paste_rows > 0:
                            # Slice source_data_chunk to ensure it fits adjusted paste_rows/cols
                            output_elevation_array[
                                dest_row_start : dest_row_start + paste_rows,
                                dest_col_start : dest_col_start + paste_cols,
                            ] = source_data_chunk[0:paste_rows, 0:paste_cols]
                            found_data = True
                else:
                    print(
                        f"Skipping tile {os.path.basename(fp)} due to CRS mismatch. Expected {target_crs}, got {src.crs}."
                    )

        except rasterio.errors.RasterioIOError as e:
            print(
                f"Warning: Could not read part of GeoTIFF file {fp} for extraction: {e}"
            )
        except Exception as e:
            print(f"An unexpected error occurred while extracting data from {fp}: {e}")

    if not found_data:
        print("No overlapping DTM data found for the specified location.")
        return None, None, None, None, None, None

    # Calculate actual center lat/lon of the *extracted* grid
    extracted_min_x, extracted_max_y = output_transform * (0, 0)
    extracted_max_x, extracted_min_y = output_transform * (
        output_shape[1],
        output_shape[0],
    )

    extracted_center_x = (extracted_min_x + extracted_max_x) / 2
    extracted_center_y = (extracted_min_y + extracted_max_y) / 2

    # Convert actual center to Lat/Lon using the new Transformer API
    wgs84_transformer = Transformer.from_crs(target_crs, "EPSG:4326", always_xy=True)
    actual_center_lon, actual_center_lat = wgs84_transformer.transform(
        extracted_center_x, extracted_center_y
    )

    print(
        f"Extracted a {grid_size_m}x{grid_size_m}m grid centered near ({target_lat:.4f}, {target_lon:.4f})"
    )
    print(
        f"Actual extracted center (Lat/Lon): ({actual_center_lat:.4f}, {actual_center_lon:.4f})"
    )

    return (
        output_elevation_array,
        output_transform,
        target_crs,
        actual_center_lat,
        actual_center_lon,
        resolution_m,
    )


def visualize_extracted_grid(
    elevation_array,
    output_image_name="extracted_elevation_grid.png",
    output_html_name="extracted_elevation_grid_display.html",
):
    """
    Visualizes an extracted 1-meter resolution elevation grid as a grayscale image.

    Args:
        elevation_array (np.ndarray): The 2D NumPy array of elevation data.
        output_image_name (str): The name for the output PNG image.
        output_html_name (str): The name for the output HTML file.
    """
    if elevation_array is None or elevation_array.size == 0:
        print("No elevation array provided for visualization.")
        return

    # Handle no-data values before visualization
    valid_elevation_data = elevation_array[~np.isnan(elevation_array)]

    if valid_elevation_data.size == 0:
        print(f"No valid elevation data found in the extracted array.")
        return

    min_elevation = np.min(valid_elevation_data)
    max_elevation = np.max(valid_elevation_data)

    if min_elevation == max_elevation:
        print(
            f"Elevation range is flat (min={min_elevation}, max={max_elevation}). Cannot create gradient for visualization."
        )
        normalized_elevation = np.full(elevation_array.shape, 0.5)  # Mid-gray
    else:
        normalized_elevation = (elevation_array - min_elevation) / (
            max_elevation - min_elevation
        )

    plt.figure(figsize=(10, 10))
    plt.imshow(
        normalized_elevation, cmap="gray_r", origin="upper"
    )  # 'gray_r': white=min, black=max

    plt.colorbar(label="Normalized Elevation (White=Min, Black=Black)")
    plt.title(
        f"Extracted Elevation Grid ({elevation_array.shape[0]}x{elevation_array.shape[1]} pixels)\n(Min: {min_elevation:.2f}m, Max: {max_elevation:.2f}m)"
    )  # Corrected title for min/max display
    plt.xlabel("X (pixels)")
    plt.ylabel("Y (pixels)")
    plt.tight_layout()

    plt.savefig(output_image_name, bbox_inches="tight", dpi=300)
    plt.close()

    print(f"Extracted elevation grid image saved to: {output_image_name}")

    html_content = f"""
    <!DOCTYPE html>
    <html lang="en">
    <head>
        <meta charset="UTF-8">
        <meta name="viewport" content="width=device-width, initial-scale=1.0">
        <title>Extracted Elevation Grid Display (2D)</title>
        <style>
            body {{
                font-family: 'Inter', sans-serif;
                display: flex;
                flex-direction: column;
                align-items: center;
                justify-content: center;
                min-height: 100vh;
                margin: 0;
                background-color: #f0f0f0;
                padding: 20px;
                box-sizing: border-box;
            }}
            .container {{
                background-color: #ffffff;
                border-radius: 12px;
                box-shadow: 0 4px 12px rgba(0, 0, 0, 0.1);
                padding: 20px;
                text-align: center;
                max-width: 90%;
            }}
            h1 {{
                color: #333;
                margin-bottom: 20px;
            }}
            img {{
                max-width: 100%;
                height: auto;
                border-radius: 8px;
                box-shadow: 0 2px 8px rgba(0, 0, 0, 0.05);
            }}
            p {{
                color: #666;
                margin-top: 15px;
            }}
        </style>
        <link href="https://fonts.googleapis.com/css2?family=Inter:wght@400;600&display=swap" rel="stylesheet">
    </head>
    <body>
        <div class="container">
            <h1>Extracted Elevation Grid Visualization (2D)</h1>
            <img src="{output_image_name}" alt="Extracted Elevation Grid">
            <p>This image displays the 1-meter resolution elevation data for the 2km x 2km grid.</p>
            <p>White represents the minimum elevation, and black represents the maximum elevation.</p>
            <p>Minimum Elevation: {min_elevation:.2f} meters</p>
            <p>Maximum Elevation: {max_elevation:.2f} meters</p>
        </div>
    </body>
    </html>
    """
    with open(output_html_name, "w") as f:
        f.write(html_content)
    print(f"HTML display page saved to: {output_html_name}")


def visualize_3d_elevation(
    elevation_array,
    actual_center_lat,
    actual_center_lon,
    output_image_name="3d_elevation_plot.png",
    show_plot=False,
):
    """
    Visualizes an extracted 1-meter resolution elevation grid as a 3D surface plot.

    Args:
        elevation_array (np.ndarray): The 2D NumPy array of elevation data.
        actual_center_lat (float): Latitude of the actual center of the extracted grid.
        actual_center_lon (float): Longitude of the actual center of the extracted grid.
        output_image_name (str): The name for the output PNG image.
        show_plot (bool): If True, calls plt.show() to display the plot interactively. Defaults to False.
    """
    if elevation_array is None or elevation_array.size == 0:
        print("No elevation array provided for 3D visualization.")
        return

    valid_elevation_data = elevation_array[~np.isnan(elevation_array)]
    if valid_elevation_data.size == 0:
        print(f"No valid elevation data found in the extracted array for 3D plot.")
        return

    min_elevation = np.min(valid_elevation_data)
    max_elevation = np.max(valid_elevation_data)

    # Fill NaN values with the minimum elevation for visual continuity in the 3D plot
    elevation_plot_data = np.nan_to_num(elevation_array, nan=min_elevation)

    rows, cols = elevation_plot_data.shape
    x = np.arange(0, cols, 1)
    y = np.arange(0, rows, 1)
    X, Y = np.meshgrid(x, y)

    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection="3d")

    surf = ax.plot_surface(
        X, Y, elevation_plot_data, cmap=cm.terrain, linewidth=0, antialiased=True
    )

    fig.colorbar(surf, shrink=0.5, aspect=10, label="Elevation (meters)")

    ax.set_title(
        f"3D Elevation Plot around ({actual_center_lat:.4f}, {actual_center_lon:.4f})"
    )
    ax.set_xlabel("X (pixels)")
    ax.set_ylabel("Y (pixels)")
    ax.set_zlabel("Elevation (meters)")

    # Calculate the ranges for each axis
    x_range = X.max() - X.min()
    y_range = Y.max() - Y.min()
    z_range = elevation_plot_data.max() - elevation_plot_data.min()

    # Find the maximum range among the three axes
    max_range = np.max([x_range, y_range, z_range])

    # Set common limits for all axes to ensure equal scaling
    # Center each axis's limits around its mean
    x_center = (X.max() + X.min()) / 2
    y_center = (Y.max() + Y.min()) / 2
    z_center = (elevation_plot_data.max() + elevation_plot_data.min()) / 2

    ax.set_xlim([x_center - max_range / 2, x_center + max_range / 2])
    ax.set_ylim([y_center - max_range / 2, y_center + max_range / 2])
    ax.set_zlim([z_center - max_range / 2, z_center + max_range / 2])

    # Force equal aspect ratio
    ax.set_box_aspect([x_range, y_range, z_range])

    plt.tight_layout()
    plt.savefig(output_image_name, bbox_inches="tight", dpi=300)
    print(f"3D elevation plot saved to: {output_image_name}")

    if show_plot:
        plt.show()
    plt.close()


def calculate_traversability_cost(
    elevation_data_2d,
    original_resolution_m,
    target_resolution_m,
    original_transform,  # New parameter
    original_crs,  # New parameter
    slope_weight=0.5,
    roughness_weight=0.5,
    max_slope_deg=20.0,  # Maximum traversable slope in degrees
    max_roughness_m=0.5,  # Maximum traversable roughness in meters
    roughness_blur_sigma=1.0,  # Sigma for Gaussian blur for roughness calculation (in pixels)
):
    """
    Calculates a 2D traversability cost array (0.0 to 1.0) from elevation data.
    Processes large arrays in chunks to manage memory.
    Shrinks the array based on valid data before processing.

    Args:
        elevation_data_2d (np.ndarray): The input 2D NumPy array of elevation data (e.g., 1m resolution).
        original_resolution_m (float): The spatial resolution of the input elevation data in meters.
        target_resolution_m (float): The desired spatial resolution for the output cost map in meters.
        original_transform (rasterio.transform.Affine): The affine transform of the original elevation_data_2d.
        original_crs (rasterio.crs.CRS): The CRS of the original elevation_data_2d.
        slope_weight (float): Weight for the slope cost component (0.0 to 1.0).
        roughness_weight (float): Weight for the roughness cost component (0.0 to 1.0).
        max_slope_deg (float): Slope in degrees above which traversability becomes very high cost (1.0).
        max_roughness_m (float): Roughness in meters above which traversability becomes very high cost (1.0).
        roughness_blur_sigma (float): Standard deviation for the Gaussian filter used to smooth elevations
                                      for roughness calculation. In original pixel units.

    Returns:
        tuple: (np.ndarray, Affine, CRS)
               A 2D NumPy array representing the traversability cost (0.0 to 1.0),
               interpolated to the target resolution, along with the adjusted Affine transform
               and the original CRS. Returns (None, None, None) if input is invalid or
               contains no valid data.
    """
    if elevation_data_2d is None or elevation_data_2d.size == 0:
        print("Input elevation data is empty or invalid for cost calculation.")
        return None, None, None

    # 1. Find the bounding box of valid (non-NaN) data
    non_nan_rows = np.where(~np.all(np.isnan(elevation_data_2d), axis=1))[0]
    non_nan_cols = np.where(~np.all(np.isnan(elevation_data_2d), axis=0))[0]

    if non_nan_rows.size == 0 or non_nan_cols.size == 0:
        print(
            "Warning: Input elevation data contains only NaN values. No costmap will be generated with valid data."
        )
        # Create a 1x1 array of NaN, which write_elevation_costmap will handle as 255 (unknown)
        output_cost_array = np.full((1, 1), np.nan, dtype=np.float32)

        # Create a minimal transform for this 1x1 NaN array
        # Center the 1x1 at the original array's center for best representation if all unknown
        center_x_orig, center_y_orig = original_transform * (
            elevation_data_2d.shape[1] / 2,
            elevation_data_2d.shape[0] / 2,
        )

        # Calculate new top-left for 1x1 array that would place its center at original array's center
        new_top_left_x = center_x_orig - (original_resolution_m / 2)
        new_top_left_y = center_y_orig + (
            original_resolution_m / 2
        )  # Y increases upwards, so add to top-left

        adjusted_transform = Affine(
            original_resolution_m,
            0,
            new_top_left_x,
            0,
            -original_resolution_m,
            new_top_left_y,
        )

        return output_cost_array, adjusted_transform, original_crs

    min_row_crop = non_nan_rows[0]
    max_row_crop = non_nan_rows[-1]
    min_col_crop = non_nan_cols[0]
    max_col_crop = non_nan_cols[-1]

    # Crop the elevation data to the bounding box
    elevation_data_cropped = elevation_data_2d[
        min_row_crop : max_row_crop + 1, min_col_crop : max_col_crop + 1
    ]

    # Calculate the top-left coordinate of the cropped area in the original CRS (UTM)
    new_top_left_utm_x, new_top_left_utm_y = original_transform * (
        min_col_crop,
        min_row_crop,
    )

    # Create the adjusted transform for the cropped array
    adjusted_transform = Affine(
        original_resolution_m,
        0,
        new_top_left_utm_x,
        0,
        -original_resolution_m,
        new_top_left_utm_y,
    )

    rows, cols = elevation_data_cropped.shape
    print(
        f"Elevation data cropped to {rows}x{cols} from original. Proceeding with calculations on cropped data."
    )

    # Determine padding needed for kernel operations (gradient and gaussian blur)
    PAD_PIXELS = int(math.ceil(5 * roughness_blur_sigma))
    if PAD_PIXELS == 0:  # Ensure minimum padding for gradient
        PAD_PIXELS = 1

    # Define a maximum chunk size based on desired memory usage (approx. 0.5 GB for float32 data)
    # 0.5 GB = 0.5 * 1024 * 1024 * 1024 bytes
    # float32 = 4 bytes per element
    # Max elements = (0.5 * 1024^3) / 4 = 0.125 * 2^30 = 2^27 elements
    # Max dimension = sqrt(2^27) approx 11585
    MAX_CHUNK_DIM = int(
        11585 / 2.0
    )  # Max dimension for a chunk (e.g., 11585x11585 chunk)
    # Ensure chunk dimension is large enough to accommodate padding
    if MAX_CHUNK_DIM < 2 * PAD_PIXELS + 1:
        MAX_CHUNK_DIM = 2 * PAD_PIXELS + 1

    # Check if chunking is necessary
    if rows * cols > MAX_CHUNK_DIM * MAX_CHUNK_DIM:
        print(
            f"Cropped array size ({rows}x{cols}) exceeds max chunk size ({MAX_CHUNK_DIM}x{MAX_CHUNK_DIM}). Processing in chunks."
        )

        # Initialize output cost array at original resolution
        combined_cost_original_res = np.empty_like(
            elevation_data_cropped, dtype=np.float32
        )

        # Calculate step size for iteration, ensuring overlap for padding
        effective_chunk_dim = MAX_CHUNK_DIM - 2 * PAD_PIXELS
        if effective_chunk_dim <= 0:
            print(
                "Error: Effective chunk dimension is too small. Increase MAX_CHUNK_DIM or decrease roughness_blur_sigma."
            )
            return None, None, None

        for r_start in range(0, rows, effective_chunk_dim):
            for c_start in range(0, cols, effective_chunk_dim):
                # Define the actual slice for the output (without padding)
                r_end = min(r_start + effective_chunk_dim, rows)
                c_end = min(c_start + effective_chunk_dim, cols)

                # Define the padded slice for input (extending beyond actual slice)
                padded_r_start = max(0, r_start - PAD_PIXELS)
                padded_r_end = min(rows, r_end + PAD_PIXELS)
                padded_c_start = max(0, c_start - PAD_PIXELS)
                padded_c_end = min(cols, c_end + PAD_PIXELS)

                # Extract the padded chunk
                current_elevation_chunk = elevation_data_cropped[
                    padded_r_start:padded_r_end, padded_c_start:padded_c_end
                ]

                # If a chunk is entirely outside the valid area (e.g., due to min/max clipping causing small overlap)
                if current_elevation_chunk.size == 0:
                    continue

                # Calculate slope for the chunk
                dz_dy_chunk, dz_dx_chunk = np.gradient(
                    current_elevation_chunk, original_resolution_m
                )
                slope_radians_chunk = np.arctan(
                    np.sqrt(dz_dx_chunk**2 + dz_dy_chunk**2)
                )
                slope_degrees_chunk = np.degrees(slope_radians_chunk)
                slope_cost_chunk = np.clip(
                    slope_degrees_chunk / max_slope_deg, 0.0, 1.0
                )

                # Calculate roughness for the chunk
                smoothed_elevation_chunk = scipy.ndimage.gaussian_filter(
                    current_elevation_chunk,
                    sigma=roughness_blur_sigma,
                    mode="constant",
                    cval=np.nan,
                )
                roughness_chunk = np.abs(
                    current_elevation_chunk - smoothed_elevation_chunk
                )
                roughness_cost_chunk = np.clip(
                    roughness_chunk / max_roughness_m, 0.0, 1.0
                )

                # Combine costs for the chunk
                total_weight = slope_weight + roughness_weight
                if total_weight == 0:
                    chunk_combined_cost = np.zeros_like(slope_cost_chunk)
                else:
                    normalized_slope_weight = slope_weight / total_weight
                    normalized_roughness_weight = roughness_weight / total_weight
                    chunk_combined_cost = (
                        normalized_slope_weight * slope_cost_chunk
                    ) + (normalized_roughness_weight * roughness_cost_chunk)

                # Crop the chunk_combined_cost to remove padding and place into the main array
                crop_r_start = r_start - padded_r_start
                crop_c_start = c_start - padded_c_start
                crop_r_end = crop_r_start + (r_end - r_start)
                crop_c_end = crop_c_start + (c_end - c_start)

                combined_cost_original_res[r_start:r_end, c_start:c_end] = (
                    chunk_combined_cost[
                        crop_r_start:crop_r_end, crop_c_start:crop_c_end
                    ]
                )
        print(
            f"Cost map generated in chunks at original {original_resolution_m}m resolution."
        )
    else:  # No chunking needed, process the whole array directly
        print(
            f"Cropped array size ({rows}x{cols}) is within max chunk size. Processing as a whole."
        )
        # 1. Calculate Slope
        dz_dy, dz_dx = np.gradient(elevation_data_cropped, original_resolution_m)
        slope_radians = np.arctan(np.sqrt(dz_dx**2 + dz_dy**2))
        slope_degrees = np.degrees(slope_radians)
        slope_cost = np.clip(slope_degrees / max_slope_deg, 0.0, 1.0)
        print(
            f"Slope cost calculated. Min: {np.nanmin(slope_cost) if not np.all(np.isnan(slope_cost)) else 'NaN' :.2f}, Max: {np.nanmax(slope_cost) if not np.all(np.isnan(slope_cost)) else 'NaN' :.2f}"
        )

        # 2. Calculate Roughness
        smoothed_elevation = scipy.ndimage.gaussian_filter(
            elevation_data_cropped,
            sigma=roughness_blur_sigma,
            mode="constant",
            cval=np.nan,
        )
        roughness = np.abs(elevation_data_cropped - smoothed_elevation)
        roughness_cost = np.clip(roughness / max_roughness_m, 0.0, 1.0)
        print(
            f"Roughness cost calculated. Min: {np.nanmin(roughness_cost) if not np.all(np.isnan(roughness_cost)) else 'NaN' :.2f}, Max: {np.nanmax(roughness_cost) if not np.all(np.isnan(roughness_cost)) else 'NaN' :.2f}"
        )

        # 3. Combine the cost of slope with the cost of roughness
        total_weight = slope_weight + roughness_weight
        if total_weight == 0:
            print(
                "Warning: Both slope_weight and roughness_weight are zero. Returning zero cost map with NaNs retained."
            )
            combined_cost_original_res = np.zeros_like(
                slope_cost
            )  # Still need to apply NaN mask later
        else:
            normalized_slope_weight = slope_weight / total_weight
            normalized_roughness_weight = roughness_weight / total_weight
            combined_cost_original_res = (normalized_slope_weight * slope_cost) + (
                normalized_roughness_weight * roughness_cost
            )

    # 4. Limit the values to be within 0 to 1 (NaNs remain NaN)
    combined_cost_original_res = np.clip(combined_cost_original_res, 0.0, 1.0)
    print(
        f"Final combined cost (at original resolution). Min: {np.nanmin(combined_cost_original_res) if not np.all(np.isnan(combined_cost_original_res)) else 'NaN' :.2f}, Max: {np.nanmax(combined_cost_original_res) if not np.all(np.isnan(combined_cost_original_res)) else 'NaN' :.2f}"
    )

    # 5. Interpolate if target_resolution_m is different
    if original_resolution_m != target_resolution_m:
        zoom_factor = original_resolution_m / target_resolution_m
        output_cost_array = scipy.ndimage.zoom(
            combined_cost_original_res,
            zoom_factor,
            order=3,
            mode="constant",
            cval=np.nan,
        )
        print(
            f"Cost map interpolated from {original_resolution_m}m to {target_resolution_m}m resolution."
        )
        # If interpolated, the transform needs to be adjusted for the new pixel size
        # The top-left corner remains the same, but the resolution changes
        adjusted_transform = Affine(
            target_resolution_m,
            adjusted_transform.b,
            adjusted_transform.c,
            adjusted_transform.d,
            -target_resolution_m,
            adjusted_transform.f,
        )

    else:
        output_cost_array = combined_cost_original_res
        print(
            f"Cost map generated at original {original_resolution_m}m resolution (no interpolation needed)."
        )

    return output_cost_array, adjusted_transform, original_crs


def visualize_cost_map(
    cost_array,
    title="Traversability Cost Map",
    output_image_name="traversability_cost_map.png",
):
    """
    Visualizes a 2D traversability cost map and saves it as a PNG image.

    Args:
        cost_array (np.ndarray): The 2D NumPy array of cost values (0.0 to 1.0).
        title (str): Title for the plot.
        output_image_name (str): The name for the output PNG image.
    """
    if cost_array is None or cost_array.size == 0:
        print("No cost array provided for visualization.")
        return

    plt.figure(figsize=(10, 10))
    # Using a colormap that goes from green (low cost) to red (high cost)
    plt.imshow(cost_array, cmap="RdYlGn_r", origin="upper", vmin=0.0, vmax=1.0)

    plt.colorbar(label="Traversability Cost (0.0 = Low, 1.0 = High)")
    plt.title(f"{title} ({cost_array.shape[0]}x{cost_array.shape[1]} pixels)")
    plt.xlabel("X (pixels)")
    plt.ylabel("Y (pixels)")
    plt.tight_layout()

    plt.savefig(output_image_name, bbox_inches="tight", dpi=300)
    plt.close()

    print(f"Cost map image saved to: {output_image_name}")


def write_elevation_costmap(
    cost_array,
    costmap_origin_transform,
    costmap_crs,
    output_filepath_stem,
    resolution_m,
):
    """
    Writes the costmap as a grayscale PNG image and a YAML file with metadata.
    This function expects the cost_array to be already cropped to valid data extent.

    Args:
        cost_array (np.ndarray): The 2D NumPy array of cost values (0.0 to 1.0 or NaN for unknown).
        costmap_origin_transform (rasterio.transform.Affine): The affine transform of the *cropped* costmap's top-left.
        costmap_crs (rasterio.crs.CRS): The CRS of the costmap.
        output_filepath_stem (str): The base path and filename stem (e.g., '/path/to/costmaps/tile_X_Y').
                                    '.png' and '.yaml' extensions will be added automatically.
        resolution_m (float): The resolution of the costmap in meters (after any interpolation).
    """
    if cost_array is None or cost_array.size == 0:
        print("No cost array provided for writing.")
        return

    # Ensure output directory exists
    output_dir = os.path.dirname(output_filepath_stem)
    os.makedirs(output_dir, exist_ok=True)

    # Make a copy to avoid modifying the original array passed in
    cost_array_processed = np.copy(cost_array)

    # Set NaN values to a temporary high value that will map to 255
    cost_array_processed[np.isnan(cost_array_processed)] = (
        1.00001  # Slightly above 1.0 for mapping to 255
    )

    # Scale cost values (0.0-1.0) to 0-254. Values > 1.0 (our NaN placeholder) will map to 255.
    scaled_cost_array = (cost_array_processed * 254).astype(np.uint8)

    # Explicitly set the values that were NaN to 255
    scaled_cost_array[scaled_cost_array > 254] = 255

    image_filename = f"{output_filepath_stem}.png"
    plt.imsave(image_filename, scaled_cost_array, cmap="gray", vmin=0, vmax=255)
    print(
        f"Costmap image saved to: {image_filename} (dimensions: {scaled_cost_array.shape[0]}x{scaled_cost_array.shape[1]})"
    )

    # Prepare and save YAML metadata
    rows, cols = scaled_cost_array.shape  # Use shape of the actual saved image

    minx, maxy = costmap_origin_transform * (
        0,
        0,
    )  # Top-left (col, row) -> (x, y) of the *cropped* image
    maxx, miny = costmap_origin_transform * (
        cols,
        rows,
    )  # Bottom-right (col, row) -> (x, y) of the *cropped* image

    # Create a transformer from costmap_crs to WGS84 (lat/lon)
    wgs84_transformer = Transformer.from_crs(costmap_crs, "EPSG:4326", always_xy=True)

    # Convert corner bounds to Lat/Lon
    top_left_lon, top_left_lat = wgs84_transformer.transform(minx, maxy)
    top_right_lon, top_right_lat = wgs84_transformer.transform(maxx, maxy)
    bottom_right_lon, bottom_right_lat = wgs84_transformer.transform(maxx, miny)
    bottom_left_lon, bottom_left_lat = wgs84_transformer.transform(minx, miny)

    # Calculate center point in original CRS (UTM)
    center_x = (minx + maxx) / 2
    center_y = (miny + maxy) / 2
    center_lon, center_lat = wgs84_transformer.transform(center_x, center_y)

    metadata = {
        "filename": os.path.basename(image_filename),
        "resolution_m": resolution_m,
        "value_scale": {"min": 0, "max": 254},  # Updated max value
        "corners_lat_lon": {
            "top_left": {"lat": top_left_lat, "lon": top_left_lon},
            "top_right": {"lat": top_right_lat, "lon": top_right_lon},
            "bottom_right": {"lat": bottom_right_lat, "lon": bottom_right_lon},
            "bottom_left": {"lat": bottom_left_lat, "lon": bottom_left_lon},
        },
        "center_lat_lon": {"lat": center_lat, "lon": center_lon},
    }

    yaml_filename = f"{output_filepath_stem}.yaml"
    with open(yaml_filename, "w") as f:
        yaml.dump(metadata, f, default_flow_style=False)
    print(f"Costmap metadata saved to: {yaml_filename}")


class DTMProcessorNode(Node):
    def __init__(self):
        super().__init__("dtm_processor_node")
        self.get_logger().info("DTM Processor Node starting...")

        # Declare ROS2 parameter for the parent directory of datasets
        self.declare_parameter("dataset_parent_dir", "/usr/local/cprt")
        self.dataset_parent_dir = (
            self.get_parameter("dataset_parent_dir").get_parameter_value().string_value
        )

        # Check if the dataset_parent_dir exists and has usable permissions
        if not os.path.exists(self.dataset_parent_dir):
            self.get_logger().error(
                f"Base directory '{self.dataset_parent_dir}' does not exist. Please create it and ensure permissions."
            )
            # Depending on desired behavior, you might want to raise an exception or rclpy.shutdown() here
        elif not os.access(
            self.dataset_parent_dir, os.W_OK | os.X_OK
        ):  # Check for write and execute permissions
            self.get_logger().error(
                f"Base directory '{self.dataset_parent_dir}' does not have sufficient write/execute permissions. Please adjust permissions."
            )
            # Depending on desired behavior, you might want to raise an exception or rclpy.shutdown() here
        else:
            self.get_logger().info(
                f"Base directory '{self.dataset_parent_dir}' exists and has usable permissions."
            )

        # Set the full data base directory based on the parameter
        self.data_base_dir = os.path.join(
            self.dataset_parent_dir, "elevation_datasets", "CanElevation"
        )
        os.makedirs(
            self.data_base_dir, exist_ok=True
        )  # Ensure the full data base directory exists
        self.get_logger().info(f"Data base directory set to: {self.data_base_dir}")

        # Declare remaining ROS2 parameters
        self.declare_parameter("download_dtms", False)
        self.declare_parameter(
            "dataset_names", ["Drumheller_DTM"]
        )  # Default dataset name
        self.declare_parameter(
            "Drumheller_DTM_url",
            "https://ftp.maps.canada.ca/pub/elevation/dem_mne/highresolution_hauteresolution/dtm_mnt/1m/AB/Drumheller/",
        )
        # Add parameter for costmap resolution (used when generating new costmaps)
        self.declare_parameter(
            "costmap_target_resolution_m", 1.0
        )  # Use 1.0 resolution instead so no interpolating is needed

        # Retrieve remaining ROS2 parameters
        self.download_dtms = (
            self.get_parameter("download_dtms").get_parameter_value().bool_value
        )
        self.dataset_names = (
            self.get_parameter("dataset_names").get_parameter_value().string_array_value
        )
        self.costmap_target_resolution_m = (
            self.get_parameter("costmap_target_resolution_m")
            .get_parameter_value()
            .double_value
        )

        # Configure QoS for services
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            # durability=DurabilityPolicy.TRANSIENT_LOCAL # Essential for services
        )

        # Create services
        self.process_drumheller_srv = self.create_service(
            Trigger,
            "process_drumheller_dtm",
            self.process_drumheller_dtm_callback,
            qos_profile=qos_profile,
        )
        self.get_logger().info('Service "/process_drumheller_dtm" is ready.')

        self.generate_costmaps_srv = self.create_service(
            Trigger,
            "generate_elevation_costmaps",
            self.generate_elevation_costmaps_callback,
            qos_profile=qos_profile,
        )
        self.get_logger().info('Service "/generate_elevation_costmaps" is ready.')

        # If download_dtms parameter is true, initiate download on node startup
        if self.download_dtms:
            self.get_logger().info(
                'Parameter "download_dtms" is true. Initiating all DTM downloads.'
            )
            self.download_all_dtms()
        else:
            self.get_logger().info(
                'Parameter "download_dtms" is false. Skipping DTM downloads on startup.'
            )

    def download_all_dtms(self):
        """
        Downloads DTM files for all specified datasets and generates dtm_index.yaml files.
        The output path is structured under self.data_base_dir.
        """
        for dataset_name in self.dataset_names:
            self.get_logger().info(f"Processing dataset: {dataset_name}")
            try:
                # Retrieve URL for the current dataset
                dataset_url_param_name = f"{dataset_name}_url"
                if not self.has_parameter(dataset_url_param_name):
                    self.get_logger().error(
                        f"Missing parameter: {dataset_url_param_name}. Skipping dataset {dataset_name}."
                    )
                    continue

                base_data_directory_url = (
                    self.get_parameter(dataset_url_param_name)
                    .get_parameter_value()
                    .string_value
                )
                self.get_logger().info(
                    f"Using base URL for {dataset_name}: {base_data_directory_url}"
                )

                output_download_folder = os.path.join(self.data_base_dir, dataset_name)
                temp_kml_processing_folder = os.path.join(
                    output_download_folder, "temp_kml_data"
                )

                self.get_logger().info(
                    f"Downloading DTM files for {dataset_name} to {output_download_folder}..."
                )
                downloaded_filepaths = download_dtm_data_from_kml_index(
                    base_data_directory_url,
                    temp_kml_processing_folder,
                    output_download_folder,
                )

                if downloaded_filepaths:
                    self.get_logger().info(
                        f"Successfully downloaded {len(downloaded_filepaths)} DTM files for {dataset_name}."
                    )

                    # Generate dtm_index.yaml
                    dtm_index_path = os.path.join(
                        output_download_folder, "dtm_index.yaml"
                    )
                    dtm_info = {
                        "dataset_name": dataset_name,
                        "base_url": base_data_directory_url,
                        "dtm_files": [
                            os.path.basename(fp) for fp in downloaded_filepaths
                        ],
                    }
                    with open(dtm_index_path, "w") as f:
                        yaml.dump(dtm_info, f, default_flow_style=False)
                    self.get_logger().info(
                        f"Generated dtm_index.yaml for {dataset_name} at {dtm_index_path}"
                    )
                else:
                    self.get_logger().warning(
                        f"No DTM files downloaded for {dataset_name}."
                    )

            except Exception as e:
                self.get_logger().error(f"Error processing dataset {dataset_name}: {e}")

    def generate_elevation_costmaps_callback(self, request, response):
        """
        ROS2 Service callback to trigger the download of all DTMs and then
        generate traversability costmaps for each DTM tile, saving them as PNG and YAML.
        """
        self.get_logger().info("Received request to generate elevation costmaps.")

        try:
            # 1. Run download_all_dtms (this will also ensure directories exist)
            self.download_all_dtms()  # This will download only if not already present

            # 2. Iterate over all dataset names
            for dataset_name in self.dataset_names:
                self.get_logger().info(
                    f"Generating costmaps for dataset: {dataset_name}"
                )
                dataset_data_folder = os.path.join(self.data_base_dir, dataset_name)
                dtm_index_path = os.path.join(dataset_data_folder, "dtm_index.yaml")
                costmap_output_dir = os.path.join(dataset_data_folder, "costmaps")
                os.makedirs(
                    costmap_output_dir, exist_ok=True
                )  # Ensure costmaps directory exists

                if not os.path.exists(dtm_index_path):
                    self.get_logger().warning(
                        f"DTM index file not found for {dataset_name} at {dtm_index_path}. Skipping costmap generation for this dataset."
                    )
                    continue

                downloaded_filepaths = []
                try:
                    with open(dtm_index_path, "r") as f:
                        dtm_info = yaml.safe_load(f)
                        if "dtm_files" in dtm_info:
                            # Construct full paths from filenames listed in YAML
                            downloaded_filepaths = [
                                os.path.join(dataset_data_folder, f)
                                for f in dtm_info["dtm_files"]
                            ]
                        else:
                            self.get_logger().error(
                                f"dtm_files key not found in {dtm_index_path}. Skipping."
                            )
                            continue
                except Exception as e:
                    self.get_logger().error(
                        f"Error loading DTM index file for {dataset_name}: {e}. Skipping."
                    )
                    continue

                if not downloaded_filepaths:
                    self.get_logger().warning(
                        f"No DTM files listed in {dtm_index_path}. Skipping costmap generation for this dataset."
                    )
                    continue

                # 3. Iterate over all DTM files in the dataset
                for dtm_filepath in downloaded_filepaths:
                    if not os.path.exists(dtm_filepath):
                        self.get_logger().warning(
                            f"DTM file {dtm_filepath} not found. Skipping costmap generation for this tile."
                        )
                        continue

                    self.get_logger().info(
                        f"Processing DTM tile: {os.path.basename(dtm_filepath)}"
                    )
                    try:
                        with rasterio.open(dtm_filepath) as src:
                            elevation_array_tile = src.read(1)
                            original_resolution_m = src.res[
                                0
                            ]  # Assuming square pixels, res is (width, height)
                            src_transform = src.transform
                            src_crs = src.crs

                            # Transform all no_data values to NaN for processing
                            # Get the no_data value from the rasterio source if it exists
                            no_data_value = src.nodata
                            if no_data_value is not None:
                                elevation_array_tile = np.where(
                                    elevation_array_tile == no_data_value,
                                    np.nan,
                                    elevation_array_tile,
                                )

                                # Print number of NaNs for debugging
                                num_nans = np.sum(np.isnan(elevation_array_tile))
                                self.get_logger().info(
                                    f"Tile {os.path.basename(dtm_filepath)} contains {num_nans} NaN values (no_data)."
                                )

                            # Call calculate_traversability_cost
                            (
                                traversability_cost_map,
                                adjusted_transform,
                                adjusted_crs,
                            ) = calculate_traversability_cost(
                                elevation_array_tile,
                                original_resolution_m,
                                self.costmap_target_resolution_m,  # Use ROS parameter for target resolution
                                src_transform,  # Pass original transform
                                src_crs,  # Pass original CRS
                                slope_weight=0.7,  # Example weights, can be params too
                                roughness_weight=0.3,
                                max_slope_deg=25.0,
                                max_roughness_m=0.75,
                                roughness_blur_sigma=4.0,
                            )

                            if traversability_cost_map is not None:
                                tile_filename_stem = os.path.splitext(
                                    os.path.basename(dtm_filepath)
                                )[0]
                                output_filepath_stem = os.path.join(
                                    costmap_output_dir, tile_filename_stem
                                )

                                # Pass adjusted_transform and adjusted_crs to write_elevation_costmap
                                write_elevation_costmap(
                                    traversability_cost_map,
                                    adjusted_transform,
                                    adjusted_crs,
                                    output_filepath_stem,
                                    self.costmap_target_resolution_m,
                                )
                            else:
                                self.get_logger().warning(
                                    f"Failed to calculate traversability cost map for {dtm_filepath}. Possibly all data was NaN."
                                )
                    except rasterio.errors.RasterioIOError as e:
                        self.get_logger().error(
                            f"Error reading GeoTIFF file {dtm_filepath}: {e}. Skipping tile."
                        )
                    except Exception as e:
                        self.get_logger().error(
                            f"An unexpected error occurred while processing tile {dtm_filepath}: {e}. Skipping tile."
                        )

            response.success = True
            response.message = "All elevation costmaps generated successfully."
            self.get_logger().info(response.message)

        except Exception as e:
            response.success = False
            response.message = f"An error occurred during costmap generation: {e}"
            self.get_logger().error(response.message)

        return response

    def process_drumheller_dtm_callback(self, request, response):
        """
        ROS2 Service callback to trigger the extraction and visualization
        of Drumheller DTM data and traversability cost using already downloaded data.
        This callback relies on data being available via download_all_dtms or manual placement.
        """
        self.get_logger().info(
            "Received request to process Drumheller DTM data for visualization."
        )

        # --- Configuration for processing (fixed for Drumheller for this callback) ---
        dataset_name_to_process = (
            "Drumheller_DTM"  # Hardcoded for this specific service callback
        )
        # The output_data_folder for a specific dataset
        output_data_folder = os.path.join(self.data_base_dir, dataset_name_to_process)
        dtm_index_path = os.path.join(output_data_folder, "dtm_index.yaml")

        # Check if the dataset directory and index file exist
        if not os.path.exists(output_data_folder) or not os.path.exists(dtm_index_path):
            response.success = False
            response.message = f"DTM data for '{dataset_name_to_process}' not found at '{output_data_folder}' or '{dtm_index_path}'. Please ensure 'download_dtms' was true on startup or data was manually placed."
            self.get_logger().error(response.message)
            return response

        # Load file paths from dtm_index.yaml
        downloaded_filepaths = []
        try:
            with open(dtm_index_path, "r") as f:
                dtm_info = yaml.safe_load(f)
                if "dtm_files" in dtm_info:
                    # Construct full paths from filenames listed in YAML
                    downloaded_filepaths = [
                        os.path.join(output_data_folder, f)
                        for f in dtm_info["dtm_files"]
                    ]
                else:
                    raise ValueError("dtm_files key not found in dtm_index.yaml")
            self.get_logger().info(
                f"Loaded {len(downloaded_filepaths)} DTM file paths from {dtm_index_path}"
            )

        except Exception as e:
            response.success = False
            response.message = (
                f"Error loading DTM index file for {dataset_name_to_process}: {e}"
            )
            self.get_logger().error(response.message)
            return response

        if not downloaded_filepaths:
            response.success = False
            response.message = (
                f"No DTM file paths found in {dtm_index_path}. Cannot process."
            )
            self.get_logger().error(response.message)
            return response

        # Example Lat/Lon for Drumheller (approximate center)
        target_lat = 51.380792205117494
        target_lon = -112.53393186181891
        grid_size_km = 0.4  # 0.4km x 0.4km grid for faster testing

        try:
            # --- Step 2 & 3: Get and visualize a specific elevation grid ---
            self.get_logger().info(
                f"--- Step 2 & 3: Extracting and visualizing a {grid_size_km}km x {grid_size_km}km elevation grid ---"
            )

            (
                elevation_array,
                grid_transform,
                grid_crs,
                actual_center_lat,
                actual_center_lon,
                resolution_m,
            ) = get_elevation_grid_around_point(
                target_lat, target_lon, grid_size_km, downloaded_filepaths
            )

            if elevation_array is not None:
                self.get_logger().info(
                    f"Extracted elevation grid shape: {elevation_array.shape}, resolution: {resolution_m}m"
                )
                self.get_logger().info(
                    f"Grid center (Lat/Lon): ({actual_center_lat:.4f}, {actual_center_lon:.4f})"
                )

                # Create output directory for visualizations if it doesn't exist
                os.makedirs(
                    os.path.join(output_data_folder, "visualizations"), exist_ok=True
                )

                # Visualize the 2D elevation grid
                self.get_logger().info("Generating 2D elevation grid visualization...")
                visualize_extracted_grid(
                    elevation_array,
                    output_image_name=os.path.join(
                        output_data_folder,
                        "visualizations",
                        "extracted_elevation_grid.png",
                    ),
                    output_html_name=os.path.join(
                        output_data_folder,
                        "visualizations",
                        "extracted_elevation_grid_display.html",
                    ),
                )

                # Visualize the 3D elevation grid (save as PNG only)
                self.get_logger().info("Generating 3D elevation visualization...")
                visualize_3d_elevation(
                    elevation_array,
                    actual_center_lat,
                    actual_center_lon,
                    output_image_name=os.path.join(
                        output_data_folder, "visualizations", "3d_elevation_plot.png"
                    ),
                    show_plot=False,
                )

                # --- Step 4: Calculate and Visualize Traversability Cost Map ---
                self.get_logger().info(
                    "--- Step 4: Calculating and Visualizing Traversability Cost Map ---"
                )

                # Parameters for cost calculation - note: this callback processes only a subset,
                # for full dataset costmap generation use generate_elevation_costmaps_callback
                original_res = (
                    resolution_m  # Should be 1m from get_elevation_grid_around_point
                )
                target_cost_map_res = (
                    self.costmap_target_resolution_m
                )  # New resolution for cost map (e.g., 0.5 meters)

                # Adjust these weights and thresholds as needed for your application
                slope_w = 0.7
                roughness_w = 0.3
                max_slope = 25.0  # Max traversable slope in degrees
                max_roughness = (
                    0.75  # Max traversable roughness in meters (e.g., 0.75m difference)
                )
                roughness_blur_s = 4.0  # Sigma for blurring roughness (adjust based on scale of roughness you care about)

                traversability_cost_map, adjusted_transform, adjusted_crs = (
                    calculate_traversability_cost(
                        elevation_array,
                        original_res,
                        target_cost_map_res,
                        grid_transform,  # Pass the transform of the extracted grid
                        grid_crs,  # Pass the CRS of the extracted grid
                        slope_weight=slope_w,
                        roughness_weight=roughness_w,
                        max_slope_deg=max_slope,
                        max_roughness_m=max_roughness,
                        roughness_blur_sigma=roughness_blur_s,
                    )
                )

                if traversability_cost_map is not None:
                    self.get_logger().info(
                        f"Traversability cost map shape: {traversability_cost_map.shape}"
                    )
                    self.get_logger().info(
                        "Generating traversability cost map visualization..."
                    )
                    visualize_cost_map(
                        traversability_cost_map,
                        title="Traversability Cost Map",
                        output_image_name=os.path.join(
                            output_data_folder,
                            "visualizations",
                            "traversability_cost_map.png",
                        ),
                    )
                    # Pass the adjusted transform and CRS
                    write_elevation_costmap(
                        traversability_cost_map,
                        adjusted_transform,
                        adjusted_crs,
                        os.path.join(
                            output_data_folder, "costmaps", "drumheller_cropped_costmap"
                        ),  # Example output stem
                        target_cost_map_res,
                    )
                    response.success = True
                    response.message = "DTM processing completed successfully."
                    self.get_logger().info(response.message)
                else:
                    response.success = False
                    response.message = "Failed to calculate traversability cost map."
                    self.get_logger().error(response.message)

            else:
                response.success = False
                response.message = "Could not extract elevation grid for visualization and cost calculation."
                self.get_logger().error(response.message)

        except Exception as e:
            response.success = False
            response.message = f"An error occurred during DTM processing: {e}"
            self.get_logger().error(response.message)

        return response


def main(args=None):
    rclpy.init(args=args)
    node = DTMProcessorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

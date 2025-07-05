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
import traceback

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
    local_kml_path="kml",
):
    """
    Downloads and parses a KMZ file from the base directory to extract DTM .tif URLs.

    Args:
        base_directory_url (str): The base URL containing the KMZ file.
        kml_filename (str): The name of the KMZ file.
        local_kml_path (str): Folder to store the KMZ and its extracted content.

    Returns:
        list: A list of absolute URLs to DTM .tif files.
    """
    dtm_urls = []
    kml_url = requests.compat.urljoin(base_directory_url, kml_filename)

    # Ensure KML folder exists
    if not os.path.exists(local_kml_path):
        os.makedirs(local_kml_path)

    # 1. Download the KMZ file
    kmz_local_path = download_file_from_url(kml_url, local_kml_path)
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
            zip_ref.extract(kml_file_in_zip, local_kml_path)
            kml_extracted_path = os.path.join(local_kml_path, kml_file_in_zip)
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
    base_data_directory_url, local_kml_folder, output_dtm_folder
):
    """
    Downloads all available DTM GeoTIFF files based on a KML index.

    Args:
        base_directory_url (str): The base URL containing the KMZ index file.
        local_kml_folder (str): Folder to store the KMZ and its extracted KML content (will not be deleted).
        output_dtm_folder (str): The local folder to save all downloaded GeoTIFFs.

    Returns:
        list: A list of file paths to the successfully downloaded DTM GeoTIFFs.
    """
    all_dtm_tif_urls = list_dtm_urls_from_kml(
        base_data_directory_url, local_kml_path=local_kml_folder
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
        downloaded_path = download_file_from_url(tif_url, output_dtm_folder)
        if downloaded_path:
            downloaded_filepaths.append(downloaded_path)

    return downloaded_filepaths


def calculate_traversability_cost(
    elevation_data_2d,
    original_resolution_m,
    target_resolution_m,
    original_transform,  # New parameter
    original_crs,  # New parameter
    max_chunk_dimension,  # ROS2 parameter
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
        max_chunk_dimension (int): The maximum dimension (width or height) for a processing chunk.
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

    # Use the max_chunk_dimension from parameters
    # Ensure chunk dimension is large enough to accommodate padding
    if max_chunk_dimension < 2 * PAD_PIXELS + 1:
        # Fallback to a safe minimum if parameter is too small
        MAX_CHUNK_DIM_EFFECTIVE = 2 * PAD_PIXELS + 1
        print(
            f"Warning: max_chunk_dimension ({max_chunk_dimension}) is too small for padding ({PAD_PIXELS}). Using effective chunk dimension of {MAX_CHUNK_DIM_EFFECTIVE}."
        )
    else:
        MAX_CHUNK_DIM_EFFECTIVE = max_chunk_dimension

    # Check if chunking is necessary
    if rows * cols > MAX_CHUNK_DIM_EFFECTIVE * MAX_CHUNK_DIM_EFFECTIVE:
        print(
            f"Cropped array size ({rows}x{cols}) exceeds max chunk size ({MAX_CHUNK_DIM_EFFECTIVE}x{MAX_CHUNK_DIM_EFFECTIVE}). Processing in chunks."
        )

        # Initialize output cost array at original resolution
        combined_cost_original_res = np.empty_like(
            elevation_data_cropped, dtype=np.float32
        )

        # Calculate step size for iteration, ensuring overlap for padding
        effective_chunk_dim = MAX_CHUNK_DIM_EFFECTIVE - 2 * PAD_PIXELS
        if effective_chunk_dim <= 0:
            print(
                "Error: Effective chunk dimension is too small after accounting for padding. Increase max_chunk_dimension or decrease roughness_blur_sigma."
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

    # Define the value for unknown/NaN regions in the output image (255)
    UNKNOWN_VALUE_PNG = 255

    # Set NaN values to the UNKNOWN_VALUE_PNG
    cost_array_processed[np.isnan(cost_array_processed)] = (
        UNKNOWN_VALUE_PNG + 1
    )  # Temporarily set to > 255 for scaling

    # Scale cost values (0.0-1.0) to 0-254. Values set to UNKNOWN_VALUE_PNG+1 will map correctly to UNKNOWN_VALUE_PNG.
    scaled_cost_array = (cost_array_processed * 254).astype(np.uint8)

    # Explicitly set the values that were originally NaN (or our placeholder) to 255
    scaled_cost_array[scaled_cost_array > 254] = UNKNOWN_VALUE_PNG

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

    # --- Calculate Map Heading ---
    # The "up" direction of the map corresponds to the vector pointing from a pixel
    # to the pixel directly above it in the image (decreasing row index).
    # This vector in pixel space is (0, -1).
    # Applying the affine transform to this vector gives its direction in world coordinates:
    # dx = transform.a * 0 + transform.b * (-1) = -transform.b
    # dy = transform.d * 0 + transform.e * (-1) = -transform.e
    up_vector_x = -costmap_origin_transform.b
    up_vector_y = -costmap_origin_transform.e

    # Calculate the angle of this "up" vector counter-clockwise from the positive Easting (X) axis.
    # math.atan2 returns an angle in radians in the range (-pi, pi].
    angle_rad_ccw_from_easting = math.atan2(up_vector_y, up_vector_x)
    angle_deg_ccw_from_easting = math.degrees(angle_rad_ccw_from_easting)

    # Normalize the angle to be within [0, 360)
    map_north_direction_deg_ccw_from_easting = (
        angle_deg_ccw_from_easting % 360 + 360
    ) % 360

    # Calculate heading clockwise from North (0 degrees).
    # North is +Y, which is 90 degrees CCW from +X (Easting).
    # So, angle from North CCW = (angle from Easting CCW) - 90
    # heading clockwise from North = (360 - angle from North CCW) % 360
    angle_from_north_ccw = map_north_direction_deg_ccw_from_easting - 90
    map_heading_degrees_clockwise_from_north = (360 - angle_from_north_ccw) % 360

    metadata = {
        "filename": os.path.basename(image_filename),
        "resolution_m": resolution_m,
        "value_scale": {
            "min": 0,
            "max": 254,
            "unknown": UNKNOWN_VALUE_PNG,
        },  # Added unknown value
        "corners_lat_lon": {
            "top_left": {"lat": top_left_lat, "lon": top_left_lon},
            "top_right": {"lat": top_right_lat, "lon": top_right_lon},
            "bottom_right": {"lat": bottom_right_lat, "lon": bottom_right_lon},
            "bottom_left": {"lat": bottom_left_lat, "lon": bottom_left_lon},
        },
        "center_lat_lon": {"lat": center_lat, "lon": center_lon},
        # New metadata fields for map orientation/heading
        "map_orientation": {
            "map_north_direction_deg_ccw_from_easting": map_north_direction_deg_ccw_from_easting,  # Angle of the "up" vector
            "heading_deg_clockwise_from_north": map_heading_degrees_clockwise_from_north,
        },
    }

    yaml_filename = f"{output_filepath_stem}.yaml"
    with open(yaml_filename, "w") as f:
        yaml.dump(metadata, f, default_flow_style=False)
    print(f"Costmap metadata saved to: {yaml_filename}")


class CanElevationDownloader(Node):  # Renamed node
    def __init__(self):
        super().__init__("can_elevation_downloader_node")  # Renamed node
        self.get_logger().info("CanElevationDownloader Node starting...")

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
            "dataset_names", ["Drumheller"]
        )  # Changed to 'Drumheller'
        self.declare_parameter(
            "Drumheller_url",
            "https://ftp.maps.canada.ca/pub/elevation/dem_mne/highresolution_hauteresolution/dtm_mnt/1m/AB/Drumheller/",
        )  # Changed parameter name

        # Add parameters for calculate_traversability_cost
        self.declare_parameter("target_map_resolution_meters", 1.0)  # Renamed parameter
        self.declare_parameter(
            "max_chunk_dimension", 6000
        )  # New parameter for max chunk size
        self.declare_parameter("slope_weight", 0.7)
        self.declare_parameter("roughness_weight", 0.3)
        self.declare_parameter("max_slope_deg", 25.0)
        self.declare_parameter("max_roughness_m", 0.75)
        self.declare_parameter("roughness_blur_sigma", 4.0)

        # Retrieve remaining ROS2 parameters
        self.download_dtms = (
            self.get_parameter("download_dtms").get_parameter_value().bool_value
        )
        self.dataset_names = (
            self.get_parameter("dataset_names").get_parameter_value().string_array_value
        )

        # Retrieve new parameters for costmap calculation
        self.target_map_resolution_meters = (
            self.get_parameter("target_map_resolution_meters")
            .get_parameter_value()
            .double_value
        )
        self.max_chunk_dimension = (
            self.get_parameter("max_chunk_dimension")
            .get_parameter_value()
            .integer_value
        )
        self.slope_weight = (
            self.get_parameter("slope_weight").get_parameter_value().double_value
        )
        self.roughness_weight = (
            self.get_parameter("roughness_weight").get_parameter_value().double_value
        )
        self.max_slope_deg = (
            self.get_parameter("max_slope_deg").get_parameter_value().double_value
        )
        self.max_roughness_m = (
            self.get_parameter("max_roughness_m").get_parameter_value().double_value
        )
        self.roughness_blur_sigma = (
            self.get_parameter("roughness_blur_sigma")
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

                # Define specific folders for KML and DTM within the dataset folder
                dataset_base_folder = os.path.join(self.data_base_dir, dataset_name)
                kml_output_folder = os.path.join(
                    dataset_base_folder, "kml"
                )  # KML in dedicated 'kml' folder
                dtm_output_folder = os.path.join(
                    dataset_base_folder, "dtm"
                )  # DTMs in dedicated 'dtm' folder

                # Ensure these new folders exist
                os.makedirs(kml_output_folder, exist_ok=True)
                os.makedirs(dtm_output_folder, exist_ok=True)

                self.get_logger().info(
                    f"Downloading DTM files for {dataset_name} to {dtm_output_folder}..."
                )
                downloaded_filepaths = download_dtm_data_from_kml_index(
                    base_data_directory_url,
                    kml_output_folder,  # Pass the dedicated KML folder
                    dtm_output_folder,  # Pass the dedicated DTM folder
                )

                if downloaded_filepaths:
                    self.get_logger().info(
                        f"Successfully downloaded {len(downloaded_filepaths)} DTM files for {dataset_name}."
                    )

                    # Generate dtm_index.yaml within the dataset_base_folder
                    dtm_index_path = os.path.join(dataset_base_folder, "dtm_index.yaml")
                    dtm_info = {
                        "dataset_name": dataset_name,
                        "base_url": base_data_directory_url,
                        "dtm_files": [
                            os.path.relpath(fp, dtm_output_folder)
                            for fp in downloaded_filepaths
                        ],  # Store relative paths
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

                dataset_base_folder = os.path.join(self.data_base_dir, dataset_name)
                dtm_input_folder = os.path.join(
                    dataset_base_folder, "dtm"
                )  # DTMs are now in 'dtm' subfolder
                dtm_index_path = os.path.join(
                    dataset_base_folder, "dtm_index.yaml"
                )  # Index is at dataset root
                costmap_output_dir = os.path.join(dataset_base_folder, "costmaps")
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
                            # Construct full paths using the dtm_input_folder
                            downloaded_filepaths = [
                                os.path.join(dtm_input_folder, f)
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

                            # Call calculate_traversability_cost with ROS2 parameters
                            (
                                traversability_cost_map,
                                adjusted_transform,
                                adjusted_crs,
                            ) = calculate_traversability_cost(
                                elevation_array_tile,
                                original_resolution_m,
                                self.target_map_resolution_meters,  # Use ROS parameter for target resolution
                                src_transform,  # Pass original transform
                                src_crs,  # Pass original CRS
                                self.max_chunk_dimension,  # Pass ROS parameter for max chunk dimension
                                slope_weight=self.slope_weight,
                                roughness_weight=self.roughness_weight,
                                max_slope_deg=self.max_slope_deg,
                                max_roughness_m=self.max_roughness_m,
                                roughness_blur_sigma=self.roughness_blur_sigma,
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
                                    self.target_map_resolution_meters,  # Use ROS parameter for resolution
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
                        traceback.print_exc()

            response.success = True
            response.message = "All elevation costmaps generated successfully."
            self.get_logger().info(response.message)

        except Exception as e:
            response.success = False
            response.message = f"An error occurred during costmap generation: {e}"
            self.get_logger().error(response.message)

        return response


def main(args=None):
    rclpy.init(args=args)
    node = CanElevationDownloader()  # Renamed node
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

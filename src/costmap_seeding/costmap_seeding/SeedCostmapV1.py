import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_srvs.srv import Trigger # A simple service for triggering operations

import requests
import os
from bs4 import BeautifulSoup 
import rasterio
import folium 
import zipfile
import xml.etree.ElementTree as ET 
import shutil 
import yaml # For YAML file handling
# Updated imports for coordinate transformations
from rasterio.crs import CRS
from rasterio.transform import Affine
from pyproj import Transformer # Import Transformer from pyproj

import matplotlib.pyplot as plt
import numpy as np
# Import for 3D plotting
from mpl_toolkits.mplot3d import Axes3D 
from matplotlib import cm # Colormap module
import scipy.ndimage # Import for image processing operations (blur, zoom)

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
        filename = os.path.join(destination_folder, url.split('/')[-1])

        if os.path.exists(filename):
            print(f"File '{filename}' already exists. Skipping download.")
            return filename

        print(f"Downloading: {url}")
        response = requests.get(url, stream=True)
        response.raise_for_status() # Raise an exception for bad status codes (4xx or 5xx)

        with open(filename, 'wb') as f:
            for chunk in response.iter_content(chunk_size=1024):
                if chunk: # filter out keep-alive new chunks
                    f.write(chunk)

        print(f"Successfully downloaded '{filename}'")
        return filename

    except requests.exceptions.RequestException as e:
        print(f"Error downloading {url}: {e}")
        return None
    except Exception as e:
        print(f"An unexpected error occurred during download of {filename}: {e}")
        return None

def list_dtm_urls_from_kml(base_directory_url, kml_filename="INDEX_utm12_AB_Drumheller.kmz", local_temp_path="temp_kml_data"):
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
        with zipfile.ZipFile(kmz_local_path, 'r') as zip_ref:
            # KMZ files usually contain a 'doc.kml' or similar at the root
            kml_files = [f for f in zip_ref.namelist() if f.lower().endswith('.kml')]
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
            ns = {'kml': 'http://www.opengis.net/kml/2.2'}
            
            # Iterate through all Placemark elements
            for placemark in root.findall('.//kml:Placemark', ns):
                description_element = placemark.find('kml:description', ns)
                if description_element is not None and description_element.text:
                    # The description contains CDATA with HTML links. Use BeautifulSoup to parse it.
                    description_html = description_element.text 
                    soup = BeautifulSoup(description_html, 'html.parser')
                    
                    # Find the <a> tag whose text is "Digital Terrain Model" or its French equivalent
                    for a_tag in soup.find_all('a', href=True):
                        link_text = a_tag.get_text(strip=True)
                        if "Digital Terrain Model" in link_text or "Modèle numérique de terrain" in link_text:
                            dtm_url = a_tag['href']
                            if dtm_url:
                                dtm_urls.append(dtm_url)
                            break # Found the DTM link for this placemark, move to the next placemark
        except ET.ParseError as e:
            print(f"Error parsing KML file {kml_extracted_path}: {e}")
        except Exception as e:
            print(f"An unexpected error occurred while parsing KML: {e}")
    
    return dtm_urls

def download_dtm_data_from_kml_index(base_data_directory_url, local_temp_kml_path, output_download_folder):
    """
    Downloads all available DTM GeoTIFF files based on a KML index.

    Args:
        base_data_directory_url (str): The base URL containing the KMZ index file.
        local_temp_kml_path (str): Temporary folder to store the KMZ and its extracted KML content.
        output_download_folder (str): The local folder to save all downloaded GeoTIFFs.

    Returns:
        list: A list of file paths to the successfully downloaded DTM GeoTIFFs.
    """
    all_dtm_tif_urls = list_dtm_urls_from_kml(base_data_directory_url, local_temp_path=local_temp_kml_path)

    if not all_dtm_tif_urls:
        print("No DTM .tif URLs found from KMZ. Cannot download any data.")
        return []

    print(f"Found {len(all_dtm_tif_urls)} DTM .tif URLs. Starting download of all files.")
    
    downloaded_filepaths = []
    all_dtm_tif_urls.sort() # Ensure consistent download order
    for tif_url in all_dtm_tif_urls:
        downloaded_path = download_file_from_url(tif_url, output_download_folder)
        if downloaded_path:
            downloaded_filepaths.append(downloaded_path)

    # Clean up temporary KML folder
    if os.path.exists(local_temp_kml_path):
        try:
            shutil.rmtree(local_temp_kml_path)
            print(f"Cleaned up temporary folder: {local_temp_kml_path}")
        except OSError as e:
            print(f"Error removing temporary folder {local_temp_kml_path}: {e}")

    return downloaded_filepaths

def get_elevation_grid_around_point(target_lat, target_lon, grid_size_km, downloaded_filepaths):
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
    resolution_m = 1 # DTM data is 1m resolution

    # Define the target CRS (UTM Zone 12N for Drumheller)
    target_crs = CRS.from_epsg(2956) # NAD83 / UTM zone 12N

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
    output_elevation_array = np.full(output_shape, np.nan, dtype=np.float32) # Initialize with NaN
    
    # Calculate the transform for the output array
    output_utm_top_left_x = utm_min_x
    output_utm_top_left_y = utm_max_y # Y increases upwards, so top is max_y

    output_transform = Affine(resolution_m, 0, output_utm_top_left_x,
                              0, -resolution_m, output_utm_top_left_y) # -resolution_m because Y decreases as row increases

    found_data = False

    for fp in downloaded_filepaths:
        try:
            with rasterio.open(fp) as src:
                # Check if the current tile's CRS matches the target CRS
                if src.crs == target_crs:
                    # Calculate window (slice) of the source dataset that overlaps with the target bounds
                    intersection_window = src.window(utm_min_x, utm_min_y, utm_max_x, utm_max_y)

                    # Ensure there is an actual overlap
                    if intersection_window.width > 0 and intersection_window.height > 0:
                        # Read data from the intersecting part of the source tile
                        source_data_chunk = src.read(1, window=intersection_window)

                        # Determine the pixel coordinates in the *output_elevation_array* where this chunk should be pasted
                        chunk_top_left_utm_x, chunk_top_left_utm_y = src.transform * (intersection_window.col_off, intersection_window.row_off)

                        dest_col_start = int(round((chunk_top_left_utm_x - output_utm_top_left_x) / resolution_m))
                        dest_row_start = int(round((output_utm_top_left_y - chunk_top_left_utm_y) / resolution_m))

                        paste_cols = source_data_chunk.shape[1]
                        paste_rows = source_data_chunk.shape[0]

                        # Adjust paste dimensions if it goes beyond the output array boundaries
                        paste_cols = min(paste_cols, output_shape[1] - dest_col_start)
                        paste_rows = min(paste_rows, output_shape[0] - dest_row_start)

                        if paste_cols > 0 and paste_rows > 0:
                            # Slice source_data_chunk to ensure it fits adjusted paste_rows/cols
                            output_elevation_array[dest_row_start : dest_row_start + paste_rows, 
                                                   dest_col_start : dest_col_start + paste_cols] = \
                                source_data_chunk[0:paste_rows, 0:paste_cols] 
                            found_data = True
                else:
                    print(f"Skipping tile {os.path.basename(fp)} due to CRS mismatch. Expected {target_crs}, got {src.crs}.")

        except rasterio.errors.RasterioIOError as e:
            print(f"Warning: Could not read part of GeoTIFF file {fp} for extraction: {e}")
        except Exception as e:
            print(f"An unexpected error occurred while extracting data from {fp}: {e}")

    if not found_data:
        print("No overlapping DTM data found for the specified location.")
        return None, None, None, None, None, None

    # Calculate actual center lat/lon of the *extracted* grid
    extracted_min_x, extracted_max_y = output_transform * (0, 0)
    extracted_max_x, extracted_min_y = output_transform * (output_shape[1], output_shape[0])

    extracted_center_x = (extracted_min_x + extracted_max_x) / 2
    extracted_center_y = (extracted_min_y + extracted_max_y) / 2

    # Convert actual center to Lat/Lon using the new Transformer API
    wgs84_transformer = Transformer.from_crs(target_crs, "EPSG:4326", always_xy=True)
    actual_center_lon, actual_center_lat = wgs84_transformer.transform(extracted_center_x, extracted_center_y)

    print(f"Extracted a {grid_size_m}x{grid_size_m}m grid centered near ({target_lat:.4f}, {target_lon:.4f})")
    print(f"Actual extracted center (Lat/Lon): ({actual_center_lat:.4f}, {actual_center_lon:.4f})")

    return output_elevation_array, output_transform, target_crs, actual_center_lat, actual_center_lon, resolution_m

def visualize_extracted_grid(elevation_array, output_image_name="extracted_elevation_grid.png", output_html_name="extracted_elevation_grid_display.html"):
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
        print(f"Elevation range is flat (min={min_elevation}, max={max_elevation}). Cannot create gradient for visualization.")
        normalized_elevation = np.full(elevation_array.shape, 0.5) # Mid-gray
    else:
        normalized_elevation = (elevation_array - min_elevation) / (max_elevation - min_elevation)

    plt.figure(figsize=(10, 10)) 
    plt.imshow(normalized_elevation, cmap='gray_r', origin='upper') # 'gray_r': white=min, black=max
    
    plt.colorbar(label='Normalized Elevation (White=Min, Black=Black)')
    plt.title(f'Extracted Elevation Grid ({elevation_array.shape[0]}x{elevation_array.shape[1]} pixels)\n(Min: {min_elevation:.2f}m, Max: {max_elevation:.2f}m)')
    plt.xlabel('X (pixels)')
    plt.ylabel('Y (pixels)')
    plt.tight_layout() 

    plt.savefig(output_image_name, bbox_inches='tight', dpi=300)
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
    with open(output_html_name, 'w') as f:
        f.write(html_content)
    print(f"HTML display page saved to: {output_html_name}")

def visualize_3d_elevation(elevation_array, actual_center_lat, actual_center_lon, output_image_name="3d_elevation_plot.png", show_plot=False):
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
    ax = fig.add_subplot(111, projection='3d')

    surf = ax.plot_surface(X, Y, elevation_plot_data, cmap=cm.terrain,
                           linewidth=0, antialiased=True)

    fig.colorbar(surf, shrink=0.5, aspect=10, label='Elevation (meters)')

    ax.set_title(f'3D Elevation Plot around ({actual_center_lat:.4f}, {actual_center_lon:.4f})')
    ax.set_xlabel('X (pixels)')
    ax.set_ylabel('Y (pixels)')
    ax.set_zlabel('Elevation (meters)')

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
    plt.savefig(output_image_name, bbox_inches='tight', dpi=300)
    print(f"3D elevation plot saved to: {output_image_name}")

    if show_plot:
        plt.show()
    plt.close()

def calculate_traversability_cost(
    elevation_data_2d, 
    original_resolution_m, 
    target_resolution_m,
    slope_weight=0.5, 
    roughness_weight=0.5,
    max_slope_deg=20.0, # Maximum traversable slope in degrees
    max_roughness_m=0.5, # Maximum traversable roughness in meters
    roughness_blur_sigma=1.0 # Sigma for Gaussian blur for roughness calculation (in pixels)
):
    """
    Calculates a 2D traversability cost array (0.0 to 1.0) from elevation data.

    Args:
        elevation_data_2d (np.ndarray): The input 2D NumPy array of elevation data (e.g., 1m resolution).
        original_resolution_m (float): The spatial resolution of the input elevation data in meters.
        target_resolution_m (float): The desired spatial resolution for the output cost map in meters.
        slope_weight (float): Weight for the slope cost component (0.0 to 1.0).
        roughness_weight (float): Weight for the roughness cost component (0.0 to 1.0).
        max_slope_deg (float): Slope in degrees above which traversability becomes very high cost (1.0).
        max_roughness_m (float): Roughness in meters above which traversability becomes very high cost (1.0).
        roughness_blur_sigma (float): Standard deviation for the Gaussian filter used to smooth elevations
                                      for roughness calculation. In original pixel units.

    Returns:
        np.ndarray: A 2D NumPy array representing the traversability cost (0.0 to 1.0),
                    interpolated to the target resolution. Returns None if input is invalid.
    """
    if elevation_data_2d is None or elevation_data_2d.size == 0:
        print("Input elevation data is empty or invalid for cost calculation.")
        return None

    # Make a copy to avoid modifying the original array
    elevation_data_clean = np.copy(elevation_data_2d)
    # Replace NaNs with the mean or a suitable value for calculations
    # For robust calculation, replace NaNs for gradient/blur
    nan_mask = np.isnan(elevation_data_clean)
    if np.any(nan_mask):
        valid_values = elevation_data_clean[~nan_mask]
        if valid_values.size > 0:
            elevation_data_clean[nan_mask] = np.mean(valid_values)
        else:
            # If all are NaNs, cannot proceed
            print("Elevation data contains only NaNs. Cannot calculate cost.")
            return np.full(elevation_data_clean.shape, 1.0, dtype=np.float32) # Return high cost

    # 1. Calculate Slope
    # Calculate gradients in x and y directions
    dz_dy, dz_dx = np.gradient(elevation_data_clean, original_resolution_m)
    
    # Calculate total slope (magnitude of gradient)
    slope_radians = np.arctan(np.sqrt(dz_dx**2 + dz_dy**2))
    slope_degrees = np.degrees(slope_radians)

    # Convert slope to cost (0 to 1)
    # Linear mapping: 0 cost at 0 slope, 1 cost at max_slope_deg
    slope_cost = np.clip(slope_degrees / max_slope_deg, 0.0, 1.0)
    print(f"Slope cost calculated. Min: {np.min(slope_cost):.2f}, Max: {np.max(slope_cost):.2f}")

    # 2. Calculate Roughness
    # Blur the elevations
    smoothed_elevation = scipy.ndimage.gaussian_filter(elevation_data_clean, sigma=roughness_blur_sigma)
    
    # Roughness as absolute difference
    roughness = np.abs(elevation_data_clean - smoothed_elevation)

    # Convert roughness to cost (0 to 1)
    roughness_cost = np.clip(roughness / max_roughness_m, 0.0, 1.0)
    print(f"Roughness cost calculated. Min: {np.min(roughness_cost):.2f}, Max: {np.max(roughness_cost):.2f}")

    # 3. Combine the cost of slope with the cost of roughness
    # Ensure weights sum to 1, or normalize them
    total_weight = slope_weight + roughness_weight
    if total_weight == 0:
        print("Warning: Both slope_weight and roughness_weight are zero. Returning zero cost map.")
        combined_cost = np.zeros_like(slope_cost)
    else:
        normalized_slope_weight = slope_weight / total_weight
        normalized_roughness_weight = roughness_weight / total_weight
        combined_cost = (normalized_slope_weight * slope_cost) + (normalized_roughness_weight * roughness_cost)

    # 4. Limit the values to be within 0 to 1
    combined_cost = np.clip(combined_cost, 0.0, 1.0)
    print(f"Combined cost calculated. Min: {np.min(combined_cost):.2f}, Max: {np.max(combined_cost):.2f}")

    # 5. Interpolate if target_resolution_m is different
    if original_resolution_m != target_resolution_m:
        zoom_factor = original_resolution_m / target_resolution_m
        output_cost_array = scipy.ndimage.zoom(combined_cost, zoom_factor, order=3) # order=3 for cubic interpolation
        print(f"Cost map interpolated from {original_resolution_m}m to {target_resolution_m}m resolution.")
    else:
        output_cost_array = combined_cost
        print(f"Cost map generated at original {original_resolution_m}m resolution (no interpolation needed).")

    return output_cost_array

def visualize_cost_map(cost_array, title="Traversability Cost Map", output_image_name="traversability_cost_map.png"):
    """
    Visualizes a 2D traversability cost map and saves it as a PNG image.
    HTML generation has been removed as per request.

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
    plt.imshow(cost_array, cmap='RdYlGn_r', origin='upper', vmin=0.0, vmax=1.0) 
    
    plt.colorbar(label='Traversability Cost (0.0 = Low, 1.0 = High)')
    plt.title(f'{title} ({cost_array.shape[0]}x{cost_array.shape[1]} pixels)')
    plt.xlabel('X (pixels)')
    plt.ylabel('Y (pixels)')
    plt.tight_layout() 

    plt.savefig(output_image_name, bbox_inches='tight', dpi=300)
    plt.close() 

    print(f"Cost map image saved to: {output_image_name}")

class DTMProcessorNode(Node):
    def __init__(self):
        super().__init__('dtm_processor_node')
        self.get_logger().info('DTM Processor Node starting...')

        # Define the base directory for CPRT data
        CPRT_ROOT_DIR = "/usr/local/cprt"
        
        # Check if /usr/local/cprt exists and has usable permissions
        if not os.path.exists(CPRT_ROOT_DIR):
            raise FileNotFoundError(f"Base directory '{CPRT_ROOT_DIR}' does not exist. Please create it and ensure permissions.")

        elif not os.access(CPRT_ROOT_DIR, os.W_OK | os.X_OK): # Check for write and execute permissions
            raise PermissionError(f"Base directory '{CPRT_ROOT_DIR}' does not have sufficient write/execute permissions. Please adjust permissions.")
        else:
            self.get_logger().info(f"Base directory '{CPRT_ROOT_DIR}' exists and has usable permissions.")

        # Set the data base directory to the specified path structure
        self.data_base_dir = os.path.join(CPRT_ROOT_DIR, 'elevation_datasets', 'CanElevation')
        os.makedirs(self.data_base_dir, exist_ok=True) # Ensure the full data base directory exists
        self.get_logger().info(f"Data base directory set to: {self.data_base_dir}")


        # Declare ROS2 parameters
        self.declare_parameter('download_dtms', False)
        self.declare_parameter('dataset_names', ['Drumheller_DTM']) # Default dataset name
        # Example for how dataset_url parameters will be declared/expected
        self.declare_parameter('Drumheller_DTM_url', "https://ftp.maps.canada.ca/pub/elevation/dem_mne/highresolution_hauteresolution/dtm_mnt/1m/AB/Drumheller/")


        # Retrieve ROS2 parameters
        self.download_dtms = self.get_parameter('download_dtms').get_parameter_value().bool_value
        self.dataset_names = self.get_parameter('dataset_names').get_parameter_value().string_array_value

        # Configure QoS for services
        # qos_profile = QoSProfile(
        #     reliability=ReliabilityPolicy.RELIABLE,
        #     history=HistoryPolicy.KEEP_LAST,
        #     depth=10,
        #     durability=DurabilityPolicy.TRANSIENT_LOCAL # Essential for services
        # )


        # If download_dtms parameter is true, initiate download on node startup
        if self.download_dtms:
            self.get_logger().info('Parameter "download_dtms" is true. Initiating all DTM downloads.')
            self.download_all_dtms()
        else:
            self.get_logger().info('Parameter "download_dtms" is false. Skipping DTM downloads on startup.')

        # Create a service for triggering the DTM processing
        self.srv = self.create_service(Trigger, 'process_drumheller_dtm', self.process_drumheller_dtm_callback) #, qos_profile=qos_profile)
        self.get_logger().info('Service "/process_drumheller_dtm" is ready.')

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
                    self.get_logger().error(f"Missing parameter: {dataset_url_param_name}. Skipping dataset {dataset_name}.")
                    continue
                
                base_data_directory_url = self.get_parameter(dataset_url_param_name).get_parameter_value().string_value
                self.get_logger().info(f"Using base URL for {dataset_name}: {base_data_directory_url}")

                output_download_folder = os.path.join(self.data_base_dir, dataset_name)
                temp_kml_processing_folder = os.path.join(output_download_folder, "temp_kml_data")

                self.get_logger().info(f"Downloading DTM files for {dataset_name} to {output_download_folder}...")
                downloaded_filepaths = download_dtm_data_from_kml_index(
                    base_data_directory_url,
                    temp_kml_processing_folder,
                    output_download_folder
                )

                if downloaded_filepaths:
                    self.get_logger().info(f"Successfully downloaded {len(downloaded_filepaths)} DTM files for {dataset_name}.")
                    
                    # Generate dtm_index.yaml
                    dtm_index_path = os.path.join(output_download_folder, "dtm_index.yaml")
                    dtm_info = {
                        'dataset_name': dataset_name,
                        'base_url': base_data_directory_url,
                        'dtm_files': [os.path.basename(fp) for fp in downloaded_filepaths]
                    }
                    with open(dtm_index_path, 'w') as f:
                        yaml.dump(dtm_info, f, default_flow_style=False)
                    self.get_logger().info(f"Generated dtm_index.yaml for {dataset_name} at {dtm_index_path}")
                else:
                    self.get_logger().warning(f"No DTM files downloaded for {dataset_name}.")

            except Exception as e:
                self.get_logger().error(f"Error processing dataset {dataset_name}: {e}")

    def process_drumheller_dtm_callback(self, request, response):
        """
        ROS2 Service callback to trigger the extraction and visualization
        of Drumheller DTM data and traversability cost using already downloaded data.
        """
        self.get_logger().info('Received request to process Drumheller DTM data.')
        
        # --- Configuration for processing (fixed for Drumheller for this callback) ---
        dataset_name_to_process = 'Drumheller_DTM' # Hardcoded for this specific service callback
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
            with open(dtm_index_path, 'r') as f:
                dtm_info = yaml.safe_load(f)
                if 'dtm_files' in dtm_info:
                    # Construct full paths from filenames listed in YAML
                    downloaded_filepaths = [os.path.join(output_data_folder, f) for f in dtm_info['dtm_files']]
                else:
                    raise ValueError("dtm_files key not found in dtm_index.yaml")
            self.get_logger().info(f"Loaded {len(downloaded_filepaths)} DTM file paths from {dtm_index_path}")

        except Exception as e:
            response.success = False
            response.message = f"Error loading DTM index file for {dataset_name_to_process}: {e}"
            self.get_logger().error(response.message)
            return response

        if not downloaded_filepaths:
            response.success = False
            response.message = f"No DTM file paths found in {dtm_index_path}. Cannot process."
            self.get_logger().error(response.message)
            return response


        # Example Lat/Lon for Drumheller (approximate center)
        target_lat = 51.380792205117494
        target_lon = -112.53393186181891
        grid_size_km = 0.4 # 0.4km x 0.4km grid for faster testing

        try:
            # --- Step 2 & 3: Get and visualize a specific elevation grid ---
            self.get_logger().info(f"--- Step 2 & 3: Extracting and visualizing a {grid_size_km}km x {grid_size_km}km elevation grid ---")
            
            elevation_array, grid_transform, grid_crs, actual_center_lat, actual_center_lon, resolution_m = \
                get_elevation_grid_around_point(target_lat, target_lon, grid_size_km, downloaded_filepaths)

            if elevation_array is not None:
                self.get_logger().info(f"Extracted elevation grid shape: {elevation_array.shape}, resolution: {resolution_m}m")
                self.get_logger().info(f"Grid center (Lat/Lon): ({actual_center_lat:.4f}, {actual_center_lon:.4f})")
                
                # Create output directory for visualizations if it doesn't exist
                os.makedirs(os.path.join(output_data_folder, "visualizations"), exist_ok=True)

                # Visualize the 2D elevation grid
                self.get_logger().info("Generating 2D elevation grid visualization...")
                visualize_extracted_grid(
                    elevation_array, 
                    output_image_name=os.path.join(output_data_folder, "visualizations", "extracted_elevation_grid.png"), 
                    output_html_name=os.path.join(output_data_folder, "visualizations", "extracted_elevation_grid_display.html")
                )

                # Visualize the 3D elevation grid (save as PNG only)
                self.get_logger().info("Generating 3D elevation visualization...")
                visualize_3d_elevation(
                    elevation_array, 
                    actual_center_lat, actual_center_lon, 
                    output_image_name=os.path.join(output_data_folder, "visualizations", "3d_elevation_plot.png"), 
                    show_plot=False
                )

                # --- Step 4: Calculate and Visualize Traversability Cost Map ---
                self.get_logger().info("--- Step 4: Calculating and Visualizing Traversability Cost Map ---")
                
                # Parameters for cost calculation
                original_res = resolution_m # Should be 1m from get_elevation_grid_around_point
                target_cost_map_res = 0.5 # New resolution for cost map (e.g., 0.5 meters)
                
                # Adjust these weights and thresholds as needed for your application
                slope_w = 0.7
                roughness_w = 0.3
                max_slope = 25.0 # Max traversable slope in degrees
                max_roughness = 0.75 # Max traversable roughness in meters (e.g., 0.75m difference)
                roughness_blur_s = 4.0 # Sigma for blurring roughness (adjust based on scale of roughness you care about)

                traversability_cost_map = calculate_traversability_cost(
                    elevation_array, 
                    original_res, 
                    target_cost_map_res,
                    slope_weight=slope_w,
                    roughness_weight=roughness_w,
                    max_slope_deg=max_slope,
                    max_roughness_m=max_roughness,
                    roughness_blur_sigma=roughness_blur_s
                )

                if traversability_cost_map is not None:
                    self.get_logger().info(f"Traversability cost map shape: {traversability_cost_map.shape}")
                    self.get_logger().info("Generating traversability cost map visualization...")
                    visualize_cost_map(
                        traversability_cost_map, 
                        title="Traversability Cost Map", 
                        output_image_name=os.path.join(output_data_folder, "visualizations", "traversability_cost_map.png")
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

if __name__ == '__main__':
    main()

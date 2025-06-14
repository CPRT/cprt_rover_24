import requests
import os
from tqdm import tqdm
from bs4 import BeautifulSoup # Used for parsing HTML within KML CDATA
import rasterio
import folium # Still imported, but not used in this specific visualization function
import zipfile
import xml.etree.ElementTree as ET # For parsing the KML XML structure
import shutil # For temporary folder cleanup
from rasterio.warp import transform_bounds # For coordinate transformation (though not strictly needed for grid display)

import matplotlib.pyplot as plt
import numpy as np

def download_file_from_url(url, destination_folder):
    """
    Downloads a single file from a given URL to a specified folder,
    with a progress bar.

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

        total_size = int(response.headers.get('content-length', 0))
        block_size = 1024  # 1 KB

        with open(filename, 'wb') as f:
            # Use tqdm for a progress bar
            with tqdm(total=total_size, unit='iB', unit_scale=True, desc=os.path.basename(filename)) as pbar:
                for chunk in response.iter_content(chunk_size=block_size):
                    if chunk: # filter out keep-alive new chunks
                        f.write(chunk)
                        pbar.update(len(chunk))

        print(f"Successfully downloaded '{filename}'")
        return filename

    except requests.exceptions.RequestException as e:
        print(f"Error downloading {url}: {e}")
        return None
    except Exception as e:
        print(f"An unexpected error occurred during download of {url}: {e}")
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

def visualize_elevation_grid(filepath, output_image_name="elevation_grid.png", output_html_name="elevation_grid_display.html"):
    """
    Loads a GeoTIFF DTM file, visualizes its elevation as a grayscale grid,
    and saves it as an image, then displays it in an HTML file.

    Args:
        filepath (str): The local path to the GeoTIFF DTM file.
        output_image_name (str): The name for the output PNG image.
        output_html_name (str): The name for the output HTML file.
    """
    if not os.path.exists(filepath):
        print(f"Error: GeoTIFF file not found at {filepath}")
        return

    try:
        # Open the GeoTIFF file
        with rasterio.open(filepath) as src:
            # Read the first band (elevation data)
            elevation_data = src.read(1)

            # Handle no-data values (if present). Set them to NaN or a low value.
            # Replace `src.nodata` with 0 or np.nan for visualization if present.
            if src.nodata is not None:
                elevation_data = np.where(elevation_data == src.nodata, np.nan, elevation_data)
                print(f"Nodata value {src.nodata} found and replaced with NaN.")

            # Filter out NaN values for min/max calculation, or handle them
            valid_elevation_data = elevation_data[~np.isnan(elevation_data)]

            if valid_elevation_data.size == 0:
                print(f"No valid elevation data found in {filepath} after handling nodata.")
                return

            # Determine min and max elevation values for normalization
            min_elevation = np.min(valid_elevation_data)
            max_elevation = np.max(valid_elevation_data)
            
            if min_elevation == max_elevation:
                print(f"Elevation range is flat (min={min_elevation}, max={max_elevation}). Cannot create gradient.")
                # Create a uniform gray image if elevation is flat
                normalized_elevation = np.full(elevation_data.shape, 0.5) # Mid-gray
            else:
                # Normalize elevation data to 0-1 range
                normalized_elevation = (elevation_data - min_elevation) / (max_elevation - min_elevation)

            # Create the plot
            plt.figure(figsize=(10, 10)) # Adjust figure size as needed
            
            # Use 'gray' colormap: 0 (min_elevation) will be black, 1 (max_elevation) will be white.
            # To reverse: min_elevation white, max_elevation black, use 'gray_r'.
            # Based on user request "minimum elevation is white and the maximum elevation is black"
            # we should use 'gray_r' (reversed grayscale).
            plt.imshow(normalized_elevation, cmap='gray_r', origin='upper') # origin='upper' for typical image display
            
            plt.colorbar(label='Normalized Elevation (White=Min, Black=Max)')
            plt.title(f'Elevation Grid for {os.path.basename(filepath)}\n(Min: {min_elevation:.2f}m, Max: {max_elevation:.2f}m)')
            plt.xlabel('X (pixels)')
            plt.ylabel('Y (pixels)')
            plt.tight_layout() # Adjust plot to prevent labels from overlapping

            # Save the image
            plt.savefig(output_image_name, bbox_inches='tight', dpi=300)
            plt.close() # Close the plot to free memory

            print(f"Elevation grid image saved to: {output_image_name}")

            # Create an HTML file to display the image
            html_content = f"""
            <!DOCTYPE html>
            <html lang="en">
            <head>
                <meta charset="UTF-8">
                <meta name="viewport" content="width=device-width, initial-scale=1.0">
                <title>Elevation Grid Display</title>
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
                    <h1>Elevation Grid Visualization</h1>
                    <img src="{output_image_name}" alt="Elevation Grid">
                    <p>This image displays the elevation data from the DTM file '{os.path.basename(filepath)}'.</p>
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

    except rasterio.errors.RasterioIOError as e:
        print(f"Error reading GeoTIFF file {filepath}: {e}. It might be corrupted or not a valid GeoTIFF.")
    except Exception as e:
        print(f"An unexpected error occurred during visualization: {e}")

if __name__ == "__main__":
    # The base directory URL containing the KMZ file
    base_data_directory_url = "https://ftp.maps.canada.ca/pub/elevation/dem_mne/highresolution_hauteresolution/dtm_mnt/1m/AB/Drumheller/"
    
    # Destination folder for downloaded GeoTIFF files
    output_download_folder = "CanElevation_Drumheller_DTM"
    # Temporary folder for KMZ extraction
    temp_kml_processing_folder = os.path.join(output_download_folder, "temp_kml_data")
    
    # We will visualize the first downloaded DTM file.
    # Set to 1 to ensure only one file is processed for this visualization type.
    num_files_to_download = 1 

    # 1. Extract DTM .tif URLs from the KMZ file
    all_dtm_tif_urls = list_dtm_urls_from_kml(base_data_directory_url, local_temp_path=temp_kml_processing_folder)
    
    if not all_dtm_tif_urls:
        print("No DTM .tif URLs found from KMZ or unable to process KMZ. Exiting.")
    else:
        print(f"Found {len(all_dtm_tif_urls)} DTM .tif URLs from KMZ. Attempting to download the first {num_files_to_download}.")
        
        # 2. Download the first DTM file
        downloaded_filepaths = []
        all_dtm_tif_urls.sort() # Ensure consistent download order
        for i, tif_url in enumerate(all_dtm_tif_urls):
            if i >= num_files_to_download:
                break
            downloaded_path = download_file_from_url(tif_url, output_download_folder)
            if downloaded_path:
                downloaded_filepaths.append(downloaded_path)
        
        # 3. Visualize the elevation grid for the first downloaded file
        if downloaded_filepaths:
            visualize_elevation_grid(downloaded_filepaths[0]) # Pass only the first file
        else:
            print("No DTM files were successfully downloaded for visualization. Check console for errors.")

    # Clean up temporary KML folder
    if os.path.exists(temp_kml_processing_folder):
        try:
            shutil.rmtree(temp_kml_processing_folder)
            print(f"Cleaned up temporary folder: {temp_kml_processing_folder}")
        except OSError as e:
            print(f"Error removing temporary folder {temp_kml_processing_folder}: {e}")

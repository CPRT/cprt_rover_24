import requests
import os
from tqdm import tqdm
from bs4 import BeautifulSoup # Used for parsing HTML within KML CDATA
import rasterio
import folium
import zipfile
import xml.etree.ElementTree as ET # For parsing the KML XML structure
import shutil # For temporary folder cleanup
from rasterio.warp import transform_bounds # Import specifically for coordinate transformation

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

def visualize_coverage(filepaths, output_map_name="can_elevation_map.html"):
    """
    Creates an interactive HTML map showing the geographic coverage of GeoTIFF files.

    Args:
        filepaths (list): A list of local paths to GeoTIFF files.
        output_map_name (str): The name of the HTML file to save the map to.
    """
    if not filepaths:
        print("No GeoTIFF files provided for visualization.")
        return

    # Initialize map - ensure centering uses transformed (lat/lon) coordinates
    m = None
    try:
        with rasterio.open(filepaths[0]) as src:
            src_bounds = src.bounds
            src_crs = src.crs

            # Transform bounds from source CRS to WGS84 (EPSG:4326) for Folium
            lon_min, lat_min, lon_max, lat_max = transform_bounds(
                src_crs, 'EPSG:4326',
                src_bounds.left, src_bounds.bottom, src_bounds.right, src_bounds.top
            )
            
            # Calculate center for map initialization using transformed coordinates
            center_lat = (lat_min + lat_max) / 2
            center_lon = (lon_min + lon_max) / 2
            
            m = folium.Map(location=[center_lat, center_lon], zoom_start=10)
            print(f"Map centered on first file (Lat/Lon): Lat {center_lat}, Lon {center_lon}")
    except Exception as e:
        print(f"Could not read bounds of first file for map centering: {e}. Using default center for Drumheller.")
        # Fallback to default for Drumheller area: approximate center
        m = folium.Map(location=[51.4, -112.7], zoom_start=9) 

    # Add each GeoTIFF's bounds to the map
    for fp in filepaths:
        try:
            with rasterio.open(fp) as src:
                # Get the bounding box in the source CRS
                src_bounds = src.bounds
                src_crs = src.crs

                # Transform bounds from source CRS to WGS84 (EPSG:4326) for Folium
                lon_min, lat_min, lon_max, lat_max = transform_bounds(
                    src_crs, 'EPSG:4326',
                    src_bounds.left, src_bounds.bottom, src_bounds.right, src_bounds.top
                )
                bounds_wgs84 = [(lat_min, lon_min), (lat_max, lon_max)]
                
                # Add a rectangle for the coverage area
                folium.Rectangle(
                    bounds=bounds_wgs84, # Use transformed bounds here
                    color='#0000FF', # Blue color
                    fill=True,
                    fill_color='#0000FF',
                    fill_opacity=0.2,
                    tooltip=os.path.basename(fp)
                ).add_to(m)
                print(f"Added bounds of {os.path.basename(fp)} to map.")
        except Exception as e:
            print(f"Could not read GeoTIFF {fp} for visualization: {e}. It might be corrupted or not a valid GeoTIFF.")

    # Save the map to an HTML file
    m.save(output_map_name)
    print(f"\nInteractive map saved to {output_map_name}")
    print(f"Open '{output_map_name}' in your web browser to view the coverage.")


if __name__ == "__main__":
    # The new base directory URL containing the KMZ file
    base_data_directory_url = "https://ftp.maps.canada.ca/pub/elevation/dem_mne/highresolution_hauteresolution/dtm_mnt/1m/AB/Drumheller/"
    
    # Destination folder for downloaded GeoTIFF files
    output_download_folder = "CanElevation_Drumheller_DTM"
    # Temporary folder for KMZ extraction
    temp_kml_processing_folder = os.path.join(output_download_folder, "temp_kml_data")
    
    # Number of .tif files to download from the list extracted from KML
    num_files_to_download = 20

    # 1. Extract DTM .tif URLs from the KMZ file
    all_dtm_tif_urls = list_dtm_urls_from_kml(base_data_directory_url, local_temp_path=temp_kml_processing_folder)
    
    if not all_dtm_tif_urls:
        print("No DTM .tif URLs found from KMZ or unable to process KMZ. Please ensure the URL is correct and contains the KMZ file. Exiting.")
    else:
        print(f"Found {len(all_dtm_tif_urls)} DTM .tif URLs from KMZ. Attempting to download the first {num_files_to_download}.")
        
        # 2. Download the first X DTM files
        downloaded_filepaths = []
        # Sort URLs to ensure consistent downloads if order isn't guaranteed by KML parsing
        all_dtm_tif_urls.sort() 
        for i, tif_url in enumerate(all_dtm_tif_urls):
            if i >= num_files_to_download:
                break
            downloaded_path = download_file_from_url(tif_url, output_download_folder)
            if downloaded_path:
                downloaded_filepaths.append(downloaded_path)
        
        # 3. Visualize the coverage
        if downloaded_filepaths:
            visualize_coverage(downloaded_filepaths)
        else:
            print("No DTM files were successfully downloaded for visualization. Check console for errors.")

    # Clean up temporary KML folder
    if os.path.exists(temp_kml_processing_folder):
        try:
            shutil.rmtree(temp_kml_processing_folder)
            print(f"Cleaned up temporary folder: {temp_kml_processing_folder}")
        except OSError as e:
            print(f"Error removing temporary folder {temp_kml_processing_folder}: {e}")


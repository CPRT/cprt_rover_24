import os
import zipfile

# try to import gdown, if it fails, pip install it, if it still fails to import, raise an error
try:
    import gdown
except ImportError:
    try:
        import pip
        pip.main(['install', 'gdown'])
        import gdown
    except Exception as e:
        raise ImportError("gdown is not installed and could not be installed automatically. "
                          "Please install it manually using 'pip install gdown'.")

def convert_google_drive_link(shared_link):
    """
    Converts a Google Drive shared link to a direct download link.

    Args:
        shared_link (str): The Google Drive shared link.  It should be in the
                         format 'https://drive.google.com/file/d/FILE_ID/view?usp=sharing'
                         or a similar format.

    Returns:
        str: The direct download link, or None if the link is invalid.
    """
    if "drive.google.com" not in shared_link:
        return None  # Not a Google Drive link

    # Extract the file ID.  This is the tricky part, as the URL can have
    # slightly different formats.
    file_id = None
    parts = shared_link.split("/")
    for i, part in enumerate(parts):
        if part == "d":
            file_id = parts[i+1]
            break
    if not file_id:
        return None

    # Handle different possible query parameters.
    if "view" in shared_link or "edit" in shared_link or "open" in shared_link:
      direct_link = f"https://drive.google.com/uc?id={file_id}"
      return direct_link
    else:
      direct_link = f"https://drive.google.com/uc?id={file_id}"
      return direct_link


def download_will_urc_keyboard_model(run_by_ros2: bool = True):
    """Downloads Will's URC keyboard model from Google Drive.
    Saves it into a location for the keyboard_detector.py to use.
    """
    gdrive_shared_link = "https://drive.google.com/file/d/16Sl4oJvDsmucR-SEOE_6SekNSnPdk9se/view?usp=sharing"
    
    download_url = convert_google_drive_link(gdrive_shared_link)
    if download_url is None:
        raise ValueError("Invalid Google Drive link provided.")

    output = "will_urc_keyboard_model_gdrive.zip"

    # Get the package directory
    if run_by_ros2:
        from ament_index_python.packages import get_package_share_directory
        package_dir = get_package_share_directory('tf_keyboard')
    else:
        package_dir = "../" # Must run script from the directory where the script is located

    # Define the destination path
    destination = os.path.join(package_dir, output)

    # Check if the file already exists
    if os.path.exists(os.path.join(package_dir, "model")):
        print("Will's Keyboard AI Model already exists. Skipping download.")
        return
    
    # Download the file
    gdown.download(download_url, destination)

    # Unzip the downloaded file
    with zipfile.ZipFile(destination, 'r') as zip_ref:
        zip_ref.extractall(package_dir)

    # Remove the zip file after extraction
    os.remove(destination)

    print("Will's Keyboard AI Model downloaded and extracted successfully.")


if __name__ == "__main__":
    download_will_urc_keyboard_model(False)
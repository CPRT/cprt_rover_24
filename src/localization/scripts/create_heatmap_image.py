import cv2
import numpy as np
import os


def create_and_save_heatmap(
    image_path, output_directory=".", lower_threshold=0, upper_threshold=None
):
    """
    Opens an image, converts it to a heatmap based on specified thresholds, and saves it.

    Args:
        image_path (str): Path to the input image.
        output_directory (str): Directory where the heatmap image will be saved.
                                Defaults to the current directory.
        lower_threshold (int): Lower bound for pixel values to be included in the heatmap.
                                Defaults to 0.
        upper_threshold (int, optional): Upper bound for pixel values to be included in the heatmap.
                                         If None, it defaults to the max value of the image's data type
                                         (255 for uint8, 65535 for uint16).
    """
    img = cv2.imread(image_path, cv2.IMREAD_UNCHANGED)

    if img is None:
        print(f"Error: Could not load image from {image_path}")
        return

    # Convert to grayscale if it's a color image
    if img.ndim == 3:
        print(
            "Warning: Loaded a color image. Converting to grayscale for heatmap generation."
        )
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Determine upper_threshold based on image data type if not provided
    if upper_threshold is None:
        if img.dtype == np.uint8:
            upper_threshold = 255
        elif img.dtype == np.uint16:
            upper_threshold = 65535
        else:
            print(
                f"Warning: Unexpected image data type: {img.dtype}. Defaulting upper threshold to 65535."
            )
            upper_threshold = 65535

    # Ensure output directory exists
    os.makedirs(output_directory, exist_ok=True)

    # Initialize processed_image as a 3-channel (BGR) image, all black
    processed_image = np.zeros((img.shape[0], img.shape[1], 3), dtype=np.uint8)

    # Create a mask for pixels within the threshold
    mask = (img >= lower_threshold) & (img <= upper_threshold)

    if np.any(mask):
        values_in_range = img[mask]

        min_val = np.min(values_in_range)
        max_val = np.max(values_in_range)

        if max_val == min_val:
            # If all values in range are the same, make them a mid-color
            colormap_input_values = np.full_like(values_in_range, 127, dtype=np.uint8)
        else:
            # Normalize to 0-255 range for applyColorMap
            colormap_input_values = (
                (values_in_range - min_val) / (max_val - min_val) * 255
            ).astype(np.uint8)

        # Apply JET colormap
        heatmap_colors = cv2.applyColorMap(colormap_input_values, cv2.COLORMAP_JET)

        # Reshape heatmap_colors from (N, 1, 3) to (N, 3) for direct assignment
        heatmap_colors = heatmap_colors.reshape(-1, 3)

        # Place the heatmap colors onto the processed_image at the masked locations
        processed_image[mask] = heatmap_colors
    else:
        print(
            "No pixels found within the specified threshold range. Outputting a black image."
        )

    # Construct the output filename
    base_name = os.path.basename(image_path)
    name_without_ext = os.path.splitext(base_name)[0]
    output_filename = f"{name_without_ext}_heatmap.png"
    output_path = os.path.join(output_directory, output_filename)

    # Save the heatmap image
    cv2.imwrite(output_path, processed_image)
    print(f"Heatmap saved to: {output_path}")


if __name__ == "__main__":
    # Example Usage:
    # Replace "path/to/your/image.png" with the actual path to your image
    # You can also specify an output directory and custom thresholds.

    # Prompt user for image path
    while True:
        try:
            print("\nEnter the path to your image (e.g., my_image.png):")
            path_input = input("Image path: ").strip()

            if not path_input:
                print("No image path provided. Exiting.")
                exit()

            if os.path.exists(path_input):
                break
            else:
                print(f"File not found: {path_input}. Please try again.")

        except EOFError:
            print("\nExiting.")
            exit()
        except KeyboardInterrupt:
            print("\nExiting.")
            exit()

    # Call the function to create and save the heatmap
    create_and_save_heatmap(
        path_input,
        output_directory="./output_heatmaps",
        lower_threshold=0,
        upper_threshold=15000,
    )
    # You can also use default thresholds:
    # create_and_save_heatmap(path_input, output_directory="./output_heatmaps")

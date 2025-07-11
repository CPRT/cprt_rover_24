import cv2
import numpy as np
import os

# --- Global Variables ---
original_image = None
display_image = None
lower_threshold = 0
upper_threshold = 65535 # Max value for mono16
zoom_level = 1.0
min_zoom_level = 1.0 # Minimum zoom level
view_x, view_y = 0, 0 # Top-left corner of the current view
window_name = "LiDAR Masking Tool"
image_path = None # To store the path of the loaded image

# --- Trackbar Callback Functions ---
def on_lower_threshold_change(val):
    global lower_threshold
    lower_threshold = val
    update_display()

def on_upper_threshold_change(val):
    global upper_threshold
    upper_threshold = val
    update_display()

# --- Mouse Callback for Zooming and Panning ---
is_panning = False
start_x, start_y = 0, 0

def mouse_callback(event, x, y, flags, param):
    global view_x, view_y, is_panning, start_x, start_y, zoom_level, original_image, min_zoom_level

    if original_image is None:
        return

    # Convert mouse coordinates to original image coordinates
    # img_x = int(x / zoom_level + view_x) # This line isn't directly used for pan/zoom logic below but useful for debugging
    # img_y = int(y / zoom_level + view_y) # Same here

    if event == cv2.EVENT_LBUTTONDOWN:
        is_panning = True
        start_x, start_y = x, y
    elif event == cv2.EVENT_LBUTTONUP:
        is_panning = False
    elif event == cv2.EVENT_MOUSEMOVE and is_panning:
        dx = x - start_x
        dy = y - start_y
        view_x -= int(dx / zoom_level)
        view_y -= int(dy / zoom_level)
        start_x, start_y = x, y
        # Clamp view_x, view_y to stay within image bounds - will be re-clamped in update_display anyway
        update_display()
    elif event == cv2.EVENT_MOUSEWHEEL:
        if flags > 0: # Scroll up (zoom in)
            new_zoom_level = min(8.0, zoom_level * 1.2) # Max zoom level 8x
        else: # Scroll down (zoom out)
            new_zoom_level = max(0.1, zoom_level / 1.2) # Min zoom level 1x (original size)
        
        # Adjust view_x, view_y to zoom towards the mouse pointer
        if new_zoom_level != zoom_level:
            # Current visible window dimensions
            current_win_width = display_image.shape[1] if display_image is not None else original_image.shape[1]
            current_win_height = display_image.shape[0] if display_image is not None else original_image.shape[0]

            # Mouse position relative to the currently displayed window (0-1)
            mouse_rel_x = x / current_win_width
            mouse_rel_y = y / current_win_height

            # Mouse position in original image coordinates (before new zoom)
            old_img_x = view_x + mouse_rel_x * (current_win_width / zoom_level)
            old_img_y = view_y + mouse_rel_y * (current_win_height / zoom_level)

            zoom_level = new_zoom_level

            # Calculate new view_x, view_y to keep the mouse point fixed in the zoomed image
            view_x = int(old_img_x - mouse_rel_x * (current_win_width / zoom_level))
            view_y = int(old_img_y - mouse_rel_y * (current_win_height / zoom_level))
            
            # Clamp view_x, view_y to stay within image bounds - will be re-clamped in update_display anyway
            update_display()

# --- Image Processing and Display ---
def update_display():
    global display_image, original_image, lower_threshold, upper_threshold, zoom_level, view_x, view_y

    if original_image is None:
        return

    # 1. Apply thresholds and create heatmap
    # Initialize processed_image as a 3-channel (BGR) image, all black
    processed_image = np.zeros((original_image.shape[0], original_image.shape[1], 3), dtype=np.uint8)

    # Create a mask for pixels within the threshold
    mask = (original_image >= lower_threshold) & (original_image <= upper_threshold)

    # Check if there are any values within the threshold
    if np.any(mask): # Use np.any to check if mask contains any True values
        # Extract values within the mask from the original_image
        values_in_range = original_image[mask]

        # Normalize values within the range (0 to 1) for colormap
        min_val = np.min(values_in_range)
        max_val = np.max(values_in_range)

        if max_val == min_val:
            # If all values in range are the same, make them a mid-color (e.g., green in JET)
            colormap_input_values = np.full_like(values_in_range, 127, dtype=np.uint8) # A single mid-gray value for colormap
        else:
            # Normalize to 0-255 range for applyColorMap
            # Ensure the values are not entirely NaN or Inf (though unlikely with range data)
            if np.isnan(min_val) or np.isinf(min_val) or np.isnan(max_val) or np.isinf(max_val):
                colormap_input_values = np.zeros_like(values_in_range, dtype=np.uint8)
                print("Warning: Min/Max values for heatmap are NaN/Inf. Outputting black for heatmap.")
            else:
                colormap_input_values = ((values_in_range - min_val) / (max_val - min_val) * 255).astype(np.uint8)
        
        # Apply JET colormap to the normalized values
        heatmap_colors = cv2.applyColorMap(colormap_input_values, cv2.COLORMAP_JET)
        
        # Reshape heatmap_colors from (N, 1, 3) to (N, 3) for direct assignment
        # This resolves the broadcasting error.
        heatmap_colors = heatmap_colors.reshape(-1, 3)

        # Place the heatmap colors onto the processed_image at the masked locations
        processed_image[mask] = heatmap_colors
    
    # Everything outside the threshold stays black
    # This is already handled by initializing processed_image as zeros.

    # 2. Apply zoom and pan
    # Calculate the visible region based on zoom and pan
    current_width_in_orig = int(original_image.shape[1] / zoom_level)
    current_height_in_orig = int(original_image.shape[0] / zoom_level)

    # Ensure dimensions are at least 1 to avoid issues with very high zoom
    current_width_in_orig = max(1, current_width_in_orig)
    current_height_in_orig = max(1, current_height_in_orig)

    # Ensure view_x and view_y are within bounds for the cropping
    if current_width_in_orig < original_image.shape[1]:
        view_x = max(0, min(view_x, original_image.shape[1] - current_width_in_orig))
    else:
        view_x = 0

    if current_height_in_orig < original_image.shape[0]:
        view_y = max(0, min(view_y, original_image.shape[0] - current_height_in_orig))
    else:
        view_y = 0

    
    cropped_image = processed_image[view_y : view_y + current_height_in_orig, view_x : view_x + current_width_in_orig]
    
    # Resize to fit the window's original size
    # We want the window to always be the same size as the original image was, but showing the zoomed/panned view
    display_image = cv2.resize(cropped_image, (original_image.shape[1], original_image.shape[0]), interpolation=cv2.INTER_NEAREST) # Use NEAREST for pixel art/crispness

    cv2.imshow(window_name, display_image)

def load_image(path):
    global original_image, lower_threshold, upper_threshold, image_path, zoom_level, view_x, view_y

    img = cv2.imread(path, cv2.IMREAD_UNCHANGED) # Load image as is (e.g., mono16)

    if img is None:
        print(f"Error: Could not load image from {path}")
        return False

    if img.ndim == 3: # If it's a color image, convert to grayscale first
        print("Warning: Loaded a color image. Converting to grayscale.")
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    if img.dtype == np.uint8:
        print("Loaded 8-bit image. Max threshold set to 255.")
        lower_threshold = 0
        upper_threshold = 255
        cv2.setTrackbarMax("Upper Threshold", window_name, 255)
    elif img.dtype == np.uint16:
        print("Loaded 16-bit image. Max threshold set to 65535.")
        lower_threshold = 0
        upper_threshold = 65535
        cv2.setTrackbarMax("Upper Threshold", window_name, 65535)
    else:
        print(f"Warning: Unexpected image data type: {img.dtype}. Treating as 16-bit. Max threshold set to 65535.")
        lower_threshold = 0
        upper_threshold = 65535
        cv2.setTrackbarMax("Upper Threshold", window_name, 65535)
    
    original_image = img
    image_path = path # Store the path
    zoom_level = 1.0 # Reset zoom
    view_x, view_y = 0, 0 # Reset pan
    
    # Set trackbar positions based on loaded image type
    cv2.setTrackbarPos("Lower Threshold", window_name, lower_threshold)
    cv2.setTrackbarPos("Upper Threshold", window_name, upper_threshold)

    update_display()
    return True

# --- Main Program ---
if __name__ == "__main__":
    print("LiDAR Masking Tool")
    print("--------------------")
    print("Instructions:")
    print("  - Use 'Lower Threshold' and 'Upper Threshold' sliders to define the range of pixel values.")
    print("  - Pixels outside this range will be black.")
    print("  - Pixels within this range will be displayed with a JET heatmap based on their value.")
    print("  - Left-click and drag to pan around the image.")
    print("  - Scroll mouse wheel up/down to zoom in/out.")
    print("  - Press 'q' or 'ESC' to quit.")
    print("  - Press 'r' to reset zoom and pan.")
    print("  - Press 's' to save the current masked image as a PNG.")

    # Prompt user for image path
    while True:
        try:
            # Suggest a common default path for Ouster images
            default_pic_dir = os.path.expanduser("~/Pictures")
            suggested_path = os.path.join(default_pic_dir, "ouster_range_image_*.png")
            print(f"\nEnter the path to your LiDAR range image (e.g., {suggested_path}):")
            path_input = input("Image path: ").strip()

            if not path_input:
                print("No image path provided. Exiting.")
                exit()
            
            if os.path.exists(path_input):
                break
            else:
                print(f"File not found: {path_input}. Please try again.")

        except EOFError: # Handles Ctrl+D on Linux/macOS
            print("\nExiting.")
            exit()
        except KeyboardInterrupt: # Handles Ctrl+C
            print("\nExiting.")
            exit()


    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.setMouseCallback(window_name, mouse_callback)

    # Initialize trackbars. Max value set temporarily to 65535, will adjust based on image type
    cv2.createTrackbar("Lower Threshold", window_name, lower_threshold, 65535, on_lower_threshold_change)
    cv2.createTrackbar("Upper Threshold", window_name, upper_threshold, 65535, on_upper_threshold_change)

    if not load_image(path_input):
        exit()

    while True:
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q') or key == 27: # 'q' or ESC key to exit
            break
        elif key == ord('r'): # 'r' to reset zoom and pan
            zoom_level = 1.0
            view_x, view_y = 0, 0
            update_display()
            print("Zoom and pan reset.")
        elif key == ord('s'): # 's' to save the current masked image
            if original_image is not None:
                # Determine save directory based on loaded image's directory, or default to Pictures
                if image_path:
                    save_dir = os.path.dirname(image_path)
                    base_name = os.path.basename(image_path)
                    name_without_ext = os.path.splitext(base_name)[0]
                else:
                    save_dir = os.path.expanduser("~/Pictures")
                    name_without_ext = "custom_ouster_range_image" # Fallback name

                # Create the binary mask: white (255) for within threshold, black (0) for outside
                binary_mask = np.zeros_like(original_image, dtype=np.uint8)
                mask_within_threshold = (original_image >= lower_threshold) & (original_image <= upper_threshold)
                binary_mask[mask_within_threshold] = 255 # White for within threshold

                mask_save_path = os.path.join(save_dir, f"{name_without_ext}_mask.png")
                cv2.imwrite(mask_save_path, binary_mask)
                print(f"Binary mask saved to: {mask_save_path}")

                # Optionally, save the current heatmap view as well
                if display_image is not None: # Ensure display_image has been generated
                    heatmap_save_path = os.path.join(save_dir, f"{name_without_ext}_heatmap_view.png")
                    cv2.imwrite(heatmap_save_path, display_image)
                    print(f"Current heatmap view saved to: {heatmap_save_path}")
                else:
                    print("No heatmap view to save (display_image is empty).")

            else:
                print("No image loaded to save.")


    cv2.destroyAllWindows()
    print("Exiting LiDAR Masking Tool.")
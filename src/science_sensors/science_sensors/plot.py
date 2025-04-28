import numpy as np
import matplotlib.pyplot as plt

# Load the CSV data.
# Adjust the filename if necessary.
data = np.loadtxt("spectrum_data.csv", delimiter=",")

# Split the data into columns.
wavelength = data[:, 0]
intensity = data[:, 1]

# Create a new figure.
plt.figure(figsize=(10, 6))

# Plot wavelength vs intensity.
plt.plot(wavelength, intensity, marker='o', linestyle='-', label="Spectrum")

# Add title and labels.
plt.title("StellarNet Spectrum")
plt.xlabel("Wavelength")
plt.ylabel("Intensity")

# Add a grid and legend.
plt.grid(True)
plt.legend()

# Display the plot.
plt.show()

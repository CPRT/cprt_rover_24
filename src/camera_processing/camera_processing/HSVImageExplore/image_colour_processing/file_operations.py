from typing import Tuple, Callable, List
from numpy import ndarray
import os, cv2

class MultipleImages:
    def __init__(self):
        self._images: List[ndarray] = []
        self._index_accessed: int = -1

    def as_list(self) -> List[ndarray]:
        return list(self._images)
    
    def add_from_directory(self, directory: str):
        filenames = os.listdir(directory)
        filenames.sort()

        for filename in filenames:
            img = cv2.imread(os.path.join(directory, filename))
            if img is not None:
                self._images.append(img)

    def add_image(self, image: ndarray):
        if image is not None:
            self._images.append(image)

    def get_next_image(self) -> ndarray:
        print(f"get_next_images index: {self._index_accessed}, length: {len(self._images)}")
        if len(self._images) == 0:
            raise Exception("Cannot get next image from an empty list of images")
        
        self._index_accessed = min(self._index_accessed + 1, len(self._images) - 1)
        print(f"get_next_images index: {self._index_accessed}, length: {len(self._images)}")
        return self._images[self._index_accessed]
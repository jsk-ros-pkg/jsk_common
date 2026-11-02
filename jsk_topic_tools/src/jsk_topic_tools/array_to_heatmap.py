"""Shared ROS 1/ROS 2 scalar-matrix decoding and heatmap rendering."""

import copy

import cv2
import numpy as np
from sensor_msgs.msg import Image

from jsk_ros1_ros2_compat.rospy_rclpy_compat import message_type_name


# cv2.LINE_AA (OpenCV >= 3) is cv2.CV_AA (16) in OpenCV 2.4 (indigo)
LINE_AA = getattr(cv2, 'LINE_AA', 16)


def tobytes(array):
    """ndarray.tobytes(), or tostring() on numpy < 1.9 (indigo), which has
    no tobytes() (newer numpy has no tostring())."""
    return array.tobytes() if hasattr(array, 'tobytes') else array.tostring()


ARRAY_TYPES = {'std_msgs/' + name + 'MultiArray' for name in (
    'Float32', 'Float64', 'Int8', 'Int16', 'Int32', 'Int64',
    'UInt8', 'UInt16', 'UInt32', 'UInt64')}
IMAGE_DTYPES = {
    'mono8': 'u1', 'mono16': 'u2', '8UC1': 'u1', '8SC1': 'i1',
    '16UC1': 'u2', '16SC1': 'i2', '32SC1': 'i4',
    '32FC1': 'f4', '64FC1': 'f8',
}


def message_to_matrix(msg, rows=0, cols=0):
    """Decode a scalar Image or row-major numeric MultiArray, including padding."""
    type_name = message_type_name(msg)
    if type_name == 'sensor_msgs/Image':
        if msg.encoding not in IMAGE_DTYPES:
            raise ValueError('Expected a single-channel numeric Image encoding')
        dtype = np.dtype(IMAGE_DTYPES[msg.encoding]).newbyteorder(
            '>' if msg.is_bigendian else '<')
        if msg.height <= 0 or msg.width <= 0:
            raise ValueError('Image dimensions must be positive')
        if msg.step < msg.width * dtype.itemsize:
            raise ValueError('Image step is shorter than a row')
        data = bytes(bytearray(msg.data))
        if len(data) < msg.height * msg.step:
            raise ValueError('Image data is shorter than height * step')
        return np.ndarray((msg.height, msg.width), dtype=dtype,
                          buffer=data, strides=(msg.step, dtype.itemsize))
    if type_name not in ARRAY_TYPES:
        raise ValueError('Unsupported input type: ' + type_name)
    dims = msg.layout.dim
    if len(dims) == 2:
        rows, cols = dims[0].size, dims[1].size
        stride = dims[1].stride
    elif not dims:
        stride = cols
    else:
        raise ValueError('MultiArray must have two dimensions or an empty layout')
    if rows <= 0 or cols <= 0 or stride < cols:
        raise ValueError('Invalid matrix shape/stride; empty layout needs rows and cols')
    offset = msg.layout.data_offset
    if offset + (rows - 1) * stride + cols > len(msg.data):
        raise ValueError('MultiArray data is shorter than its layout')
    return np.asarray([msg.data[offset + r * stride:offset + r * stride + cols]
                       for r in range(rows)])


class HeatmapRenderer(object):
    """Render matrix cells as colored squares with optional numeric values.

    Missing color bounds use each frame's finite minimum/maximum. Nonfinite
    values are gray. Fixed bounds make colors comparable across frames.
    """

    def __init__(self, cell_size=64, show_values=True, precision=2,
                 colormap='jet', min_value=None, max_value=None):
        if int(cell_size) != cell_size or cell_size < 16:
            raise ValueError('cell_size must be an integer >= 16')
        if int(precision) != precision or not 0 <= precision <= 10:
            raise ValueError('precision must be an integer in [0, 10]')
        self.cell_size = int(cell_size)
        self.show_values = show_values
        self.precision = int(precision)
        self.colormap = getattr(cv2, 'COLORMAP_' + colormap.upper(), None)
        if self.colormap is None:
            raise ValueError('Unknown OpenCV colormap: ' + colormap)
        for bound in (min_value, max_value):
            if bound is not None and not np.isfinite(bound):
                raise ValueError('Color bounds must be finite')
        if min_value is not None and max_value is not None and min_value >= max_value:
            raise ValueError('min_value must be less than max_value')
        self.min_value = min_value
        self.max_value = max_value

    def render(self, matrix):
        values = np.asarray(matrix, dtype=np.float64)
        if values.ndim != 2 or not values.size:
            raise ValueError('Expected a nonempty 2D matrix')
        finite = np.isfinite(values)
        lo = self.min_value
        hi = self.max_value
        if lo is None:
            lo = float(values[finite].min()) if finite.any() else 0.0
        if hi is None:
            hi = float(values[finite].max()) if finite.any() else lo
        normalized = np.zeros(values.shape, dtype=np.float64)
        if hi > lo:
            # Scale first to avoid overflow when the endpoints have opposite signs.
            scale = max(abs(lo), abs(hi), 1.0)
            normalized[finite] = ((np.clip(values[finite], lo, hi) / scale - lo / scale)
                                  / (hi / scale - lo / scale))
        elif self.max_value is not None and self.min_value is None:
            # With only an upper bound, a frame entirely above it saturates high.
            normalized[finite] = 1.0
        colors = cv2.applyColorMap(np.uint8(np.clip(normalized, 0, 1) * 255),
                                  self.colormap)
        colors[~finite] = (128, 128, 128)
        size = self.cell_size
        image = np.repeat(np.repeat(colors, size, axis=0), size, axis=1)
        for row in range(values.shape[0]):
            for col in range(values.shape[1]):
                x, y = col * size, row * size
                cv2.rectangle(image, (x, y), (x + size - 1, y + size - 1),
                              (50, 50, 50), 1)
                if not self.show_values:
                    continue
                label = format(values[row, col], '.' + str(self.precision) + 'f')
                # Keep very large numbers readable inside a cell.
                if len(label) > 10:
                    label = format(values[row, col], '.2e')
                font = cv2.FONT_HERSHEY_SIMPLEX
                (w, h), baseline = cv2.getTextSize(label, font, 1.0, 1)
                font_scale = min(0.7, float(size - 8) / max(w, 1),
                                 float(size - 8) / max(h + baseline, 1))
                (w, h), _ = cv2.getTextSize(label, font, font_scale, 1)
                b, g, r = colors[row, col]
                ink = (0, 0, 0) if .114*b + .587*g + .299*r > 140 else (255, 255, 255)
                cv2.putText(image, label, (x + (size - w)//2, y + (size + h)//2),
                            font, font_scale, ink, 1, LINE_AA)
        return image

    def to_image(self, msg, stamp=None, frame_id='', rows=0, cols=0):
        image = self.render(message_to_matrix(msg, rows, cols))
        output = Image()
        if hasattr(msg, 'header'):
            output.header = copy.deepcopy(msg.header)
        else:
            if stamp is not None:
                output.header.stamp = stamp
            output.header.frame_id = frame_id
        output.height, output.width = image.shape[:2]
        output.encoding = 'bgr8'
        output.is_bigendian = 0
        output.step = output.width * 3
        output.data = tobytes(image)
        return output

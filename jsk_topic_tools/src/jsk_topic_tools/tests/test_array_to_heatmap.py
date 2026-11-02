import unittest

import numpy as np
from jsk_ros1_ros2_compat.rosbag import stamp_from_nanoseconds
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, MultiArrayDimension

from jsk_topic_tools.array_to_heatmap import HeatmapRenderer, message_to_matrix, tobytes


class TestHeatmap(unittest.TestCase):
    def test_array_padding(self):
        msg = Float64MultiArray(data=[99., 1., 2., 99., 3., 4.])
        msg.layout.data_offset = 1
        msg.layout.dim = [MultiArrayDimension(size=2, stride=6),
                          MultiArrayDimension(size=2, stride=3)]
        np.testing.assert_array_equal(message_to_matrix(msg), [[1, 2], [3, 4]])
        msg.layout.dim = []
        msg.data = [99., 1., 2., 3., 4.]
        np.testing.assert_array_equal(message_to_matrix(msg, 2, 2), [[1, 2], [3, 4]])
        with self.assertRaises(ValueError):
            message_to_matrix(msg)

    def test_image_byte_order_padding_and_header(self):
        msg = Image(height=2, width=2, encoding='16UC1', is_bigendian=1,
                    step=6, data=tobytes(np.array([[1, 256, 99], [512, 1024, 99]],
                                                  dtype='>u2')))
        msg.header.stamp = stamp_from_nanoseconds(12000000034)
        msg.header.frame_id = 'sensor'
        np.testing.assert_array_equal(message_to_matrix(msg), [[1, 256], [512, 1024]])
        out = HeatmapRenderer(cell_size=32).to_image(msg)
        self.assertEqual(out.header, msg.header)
        self.assertEqual((out.width, out.height, out.step), (64, 64, 192))
        self.assertEqual(len(out.data), out.height * out.step)
        self.assertEqual(out.encoding, 'bgr8')

    def test_colors_labels_and_nonfinite(self):
        renderer = HeatmapRenderer(cell_size=64, show_values=False,
                                   min_value=0, max_value=10)
        matrix = [[-1, 0, 10, 11, float('nan')]]
        plain = renderer.render(matrix)
        np.testing.assert_array_equal(plain[32, 32], plain[32, 96])
        np.testing.assert_array_equal(plain[32, 160], plain[32, 224])
        np.testing.assert_array_equal(plain[32, 288], [128, 128, 128])
        renderer.show_values = True
        self.assertTrue(np.any(renderer.render(matrix) != plain))
        self.assertEqual(renderer.render([[1, 1]]).shape, (64, 128, 3))
        self.assertEqual(renderer.render([[float('inf')]]).shape, (64, 64, 3))

    def test_upper_bound_saturation(self):
        fixed = HeatmapRenderer(show_values=False, min_value=0, max_value=1)
        upper_only = HeatmapRenderer(show_values=False, max_value=1)
        np.testing.assert_array_equal(upper_only.render([[2]])[32, 32],
                                      fixed.render([[1]])[32, 32])

    def test_bad_inputs(self):
        with self.assertRaises(ValueError):
            HeatmapRenderer(min_value=1, max_value=0)
        with self.assertRaises(ValueError):
            HeatmapRenderer(cell_size=4)
        with self.assertRaises(ValueError):
            message_to_matrix(Image(height=1, width=1, encoding='rgb8',
                                    step=3, data=b'123'))


if __name__ == '__main__':
    unittest.main()

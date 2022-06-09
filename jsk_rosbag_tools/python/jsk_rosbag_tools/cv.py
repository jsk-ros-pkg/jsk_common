import struct

import cv2
import cv_bridge
import numpy as np
import sensor_msgs.msg


_bridge = cv_bridge.CvBridge()


def img_to_msg(img, encoding="bgr8",
               compress=True):
    """Convert numpy image to ROS message.

    """
    msg = _bridge.cv2_to_imgmsg(img, encoding=encoding)
    if compress is True:
        msg = compress_img_msg(msg)
    return msg


def msg_to_img(msg):
    if msg.encoding in ['bgra8', 'bgr8',
                        'rgba8', 'rgb8']:
        return msg_to_bgr(msg)
    elif msg.encoding in ['mono8']:
        return msg_to_mono(msg)
    else:
        return msg_to_np_depth(msg)


def compressed_format(msg):
    if ';' not in msg.format:
        fmt = ''
        if msg.format not in ['png', 'jpeg', 'rvl']:
            raise RuntimeError(
                'Unsupported or invalid compresssion format {}'
                'Please report this error'
                'https://github.com/jsk-ros-pkg/jsk_common/issues/new'
                .format(msg.format))
        if msg.format in ['rvl']:
            compr_type = 'compressedDepth'
        else:
            # could not determine compressed format.
            compr_type = ''
    else:
        fmt, compr_type = msg.format.split(';')
    # remove white space
    fmt = fmt.strip()
    compr_type = compr_type.strip()
    return fmt, compr_type


def decompresse_imgmsg(msg):
    if ';' not in msg.format:
        fmt = ''
        if msg.format not in ['png', 'jpeg', 'rvl']:
            raise RuntimeError(
                'Unsupported or invalid compresssion format {}'
                'Please report this error'
                'https://github.com/jsk-ros-pkg/jsk_common/issues/new'
                .format(msg.format))
        if msg.format in ['rvl']:
            compr_type = 'compressedDepth'
        else:
            try:
                return msg_to_bgr(msg, compressed=True)
            except Exception:
                pass
            try:
                return msg_to_np_depth(msg, compressed=True)
            except Exception:
                raise RuntimeError(
                    'Unsupported or invalid compresssion format {}'
                    'Please report this error'
                    'https://github.com/jsk-ros-pkg/jsk_common/issues/new'
                    .format(msg.format))
    else:
        fmt, compr_type = compressed_format(msg)
    if compr_type == 'compressedDepth':
        return msg_to_np_depth(msg, compressed=True)
    else:
        return msg_to_bgr(msg, compressed=True)


def msg_to_bgr(msg, compressed=False):
    if compressed:
        np_arr = np.fromstring(
            msg.data, np.uint8)
        img = cv2.imdecode(
            np_arr, cv2.IMREAD_COLOR)
        if msg.format == 'rgb8' or msg.format == 'rgba8':
            bgr_img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        else:
            bgr_img = img
    else:
        img = _bridge.imgmsg_to_cv2(
            msg,
            desired_encoding=msg.encoding)
        if msg.encoding == 'rgb8' or msg.encoding == 'rgba8':
            bgr_img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        else:
            bgr_img = img
    return bgr_img


def msg_to_mono(msg, compressed=False):
    if compressed:
        np_arr = np.fromstring(msg.data, np.uint8)
        img = cv2.imdecode(
            np_arr, cv2.IMREAD_GRAYSCALE)
    else:
        img = _bridge.imgmsg_to_cv2(
            msg,
            desired_encoding=msg.encoding)
    return img


def msg_to_np_depth(msg, compressed=False, rescale=True):
    if compressed:
        # 'msg' as type CompressedImage
        depth_fmt, compr_type = msg.format.split(';')
        # remove white space
        depth_fmt = depth_fmt.strip()
        compr_type = compr_type.strip()
        if compr_type != "compressedDepth":
            raise Exception("Compression type is not 'compressedDepth'."
                            "You probably subscribed to the wrong topic. "
                            "get compresstion_type: {}".format(compr_type))

        # remove header from raw data
        depth_header_size = 12
        raw_data = msg.data[depth_header_size:]

        depth_img_raw = cv2.imdecode(np.fromstring(raw_data, np.uint8),
                                     cv2.IMREAD_UNCHANGED)
        if depth_img_raw is None:
            # probably wrong header size
            raise Exception("Could not decode compressed depth image."
                            "You may need to change 'depth_header_size'!")

        if depth_fmt == "16UC1":
            if rescale:
                depth_img = depth_img_raw / 1000.0
            else:
                depth_img = depth_img_raw
        elif depth_fmt == "32FC1":
            raw_header = msg.data[:depth_header_size]
            # header: int, float, float
            [compfmt, depthQuantA, depthQuantB] = struct.unpack('iff',
                                                                raw_header)
            if rescale:
                depth_img_scaled = depthQuantA / (
                    depth_img_raw.astype(np.float32) - depthQuantB)
            else:
                depth_img_scaled = depth_img_raw
            # filter max values
            depth_img_scaled[depth_img_raw == 0] = 0

            # depth_img_scaled provides distance in meters as f32
            # for storing it as png, we need to convert
            # it to 16UC1 again (depth in mm)
            depth_img = depth_img_scaled
        else:
            raise Exception(
                "Decoding of '" + depth_fmt + "' is not implemented!")
    else:
        depth_img = _bridge.imgmsg_to_cv2(
            msg,
            desired_encoding='passthrough')
        if msg.encoding == '16UC1':
            if rescale:
                depth_img = np.asarray(depth_img, dtype=np.float32)
                depth_img /= 1000.0  # convert metric: mm -> m
        elif msg.encoding == '32FC1':
            pass
        else:
            print('Unsupported depth encoding: %s'
                  % msg.encoding)
    return depth_img


def compress_img_msg(msg):
    compressed_msg = sensor_msgs.msg.CompressedImage(
        header=msg.header)
    compressed_msg.format = msg.encoding + '; jpeg compressed bgr8'

    img = msg_to_bgr(msg, compressed=False)
    compressed_msg.data = np.array(cv2.imencode(
        '.jpg', img)[1]).tostring()
    return compressed_msg


def compress_depth_msg(msg, depth_quantization=100, depth_max=None):
    compressed_msg = sensor_msgs.msg.CompressedImage(
        header=msg.header)
    compressed_msg.format = '{}; compressedDepth'.format(
        msg.encoding)
    if msg.encoding == '32FC1':
        depth = msg_to_np_depth(
            msg, compressed=False, rescale=True)
        if depth_max is None:
            depth_max = depth.max() + 1.0
        depth_quant_a = depth_quantization * (depth_quantization + 1.0)
        depth_quant_b = 1.0 - depth_quant_a / depth_max
        inv_depth_img = np.zeros_like(depth, dtype=np.uint16)
        target_pixel = np.logical_and(depth_max > depth, depth > 0)
        inv_depth_img[target_pixel] = depth_quant_a / \
            depth[target_pixel] + depth_quant_b

        compressed_msg.data = struct.pack(
            'iff', 0, depth_quant_a, depth_quant_b)
        compressed_msg.data += np.array(
            cv2.imencode('.png', inv_depth_img)[1]).tostring()
    elif msg.encoding == '16UC1':
        depth = msg_to_np_depth(
            msg, compressed=False, rescale=False).copy()
        if depth_max is None:
            depth_max = depth.max()
        depth_max_ushort = depth_max * 1000.0
        depth[depth > depth_max_ushort] = 0

        compressed_msg.data = " " * 12
        compressed_msg.data += np.array(
            cv2.imencode('.png', depth)[1]).tostring()
    else:
        raise NotImplementedError(
            'Unsupported compressed depth image format {}'
            .format(msg.encoding))

    return compressed_msg

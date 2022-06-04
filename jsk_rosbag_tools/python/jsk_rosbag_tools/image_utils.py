import cv2
import PIL


def pil_to_cv2_interpolation(interpolation):
    if isinstance(interpolation, str):
        interpolation = interpolation.lower()
        if interpolation == 'nearest':
            cv_interpolation = cv2.INTER_NEAREST
        elif interpolation == 'bilinear':
            cv_interpolation = cv2.INTER_LINEAR
        elif interpolation == 'bicubic':
            cv_interpolation = cv2.INTER_CUBIC
        elif interpolation == 'lanczos':
            cv_interpolation = cv2.INTER_LANCZOS4
        else:
            raise ValueError(
                'Not valid Interpolation. '
                'Valid interpolation methods are '
                'nearest, bilinear, bicubic and lanczos.')
    else:
        if interpolation == PIL.Image.NEAREST:
            cv_interpolation = cv2.INTER_NEAREST
        elif interpolation == PIL.Image.BILINEAR:
            cv_interpolation = cv2.INTER_LINEAR
        elif interpolation == PIL.Image.BICUBIC:
            cv_interpolation = cv2.INTER_CUBIC
        elif interpolation == PIL.Image.LANCZOS:
            cv_interpolation = cv2.INTER_LANCZOS4
        else:
            raise ValueError(
                'Not valid Interpolation. '
                'Valid interpolation methods are '
                'PIL.Image.NEAREST, PIL.Image.BILINEAR, '
                'PIL.Image.BICUBIC and PIL.Image.LANCZOS.')
    return cv_interpolation


def resize_keeping_aspect_ratio(img, width=None, height=None,
                                interpolation='bilinear'):
    if (width and height) or (width is None and height is None):
        raise ValueError('Only width or height should be specified.')
    if width == img.shape[1] and height == img.shape[0]:
        return img
    if width:
        height = width * img.shape[0] / img.shape[1]
    else:
        width = height * img.shape[1] / img.shape[0]
    height = int(height)
    width = int(width)
    cv_interpolation = pil_to_cv2_interpolation(interpolation)
    return cv2.resize(img, (width, height),
                      interpolation=cv_interpolation)

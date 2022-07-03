import rosbag
from tqdm import tqdm

from jsk_rosbag_tools.cv import compress_depth_msg
from jsk_rosbag_tools.cv import compress_img_msg
from jsk_rosbag_tools.info import get_topic_dict


def compress_bag_imgs(input_bagfilepath, output_bagfilepath,
                      compressed_topics=None,
                      show_progress_bar=True):
    compressed_topics = compressed_topics or []
    input_bag = rosbag.Bag(input_bagfilepath)

    topic_dict = get_topic_dict(input_bagfilepath)
    with rosbag.Bag(output_bagfilepath, 'w') as outbag:
        if show_progress_bar:
            progress = tqdm(total=input_bag.get_message_count())
        for topic, msg, t in input_bag:
            if show_progress_bar:
                progress.update(1)
                # update the progress with a post fix
                progress.set_postfix(time=t)
            if topic_dict[topic]['type'] == 'sensor_msgs/Image':
                if msg.encoding in ['bgra8', 'bgr8',
                                    'rgba8', 'rgb8']:
                    msg = compress_img_msg(msg)
                    topic += '/compressed'
                elif msg.encoding in ['16UC1', '32FC1']:
                    msg = compress_depth_msg(msg)
                    if topic.lstrip('/') in compressed_topics:
                        topic += '/compressed'
                    else:
                        topic += '/compressedDepth'
                else:
                    pass
            outbag.write(topic, msg, t)

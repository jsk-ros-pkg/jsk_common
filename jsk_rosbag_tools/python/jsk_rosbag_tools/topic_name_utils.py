def topic_name_to_file_name(topic_name):
    if topic_name[0] == '/':
        replaced_topic_name = topic_name[1:]
    else:
        replaced_topic_name = topic_name
    replaced_topic_name = replaced_topic_name.replace('/', '--slash--')
    return replaced_topic_name

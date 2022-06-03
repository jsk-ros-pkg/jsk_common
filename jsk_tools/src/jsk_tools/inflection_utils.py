import re


def camel_to_snake(word):
    """Convert Camel case to snake case.

    Examples
    --------
    >>> camel_to_snake('clusterPointIndices')
    'cluster_point_indices'
    """
    word = re.sub(r"([A-Z]+)([A-Z][a-z])", r'\1_\2', word)
    word = re.sub(r"([a-z\d])([A-Z])", r'\1_\2', word)
    word = word.replace("-", "_")
    return word.lower()

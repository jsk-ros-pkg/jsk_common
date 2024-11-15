from dynamic_reconfigure.client import Client


class AudibleWarningReconfigureClient(object):

    def __init__(self, client_name='audible_warning'):
        super(AudibleWarningReconfigureClient).__init__()
        self.client_name = client_name
        self.client = Client(client_name, timeout=3)

    def reconfigure(self, **kwargs):
        return self.client.update_configuration(kwargs)

    def enable(self):
        return self.client.update_configuration({'enable': True})

    def disable(self):
        return self.client.update_configuration({'enable': False})

import logging

import gmqtt

logger = logging.getLogger(__name__)


class Client:
    def __init__(self, client: gmqtt.Client, name: str, handlers={}):
        self.client = client
        self.name = name
        self.handlers = handlers.copy()
        client.on_message = self.on_message
        client.on_connect = self.on_connect
        client.on_disconnect = self.on_disconnect

    def register(self, topic: str, handler):
        # TODO: QoS
        self.handlers[topic] = handler

    def on_connect(self, client, flags, rc, properties):
        logger.info("Connection to %s open", self.name)
        for topic in self.handlers:
            client.subscribe(topic)

    def on_disconnect(self, client, flags):
        logger.warning("Connection to %s closed", self.name)

    async def on_message(self, client, topic, payload, qos, properties):
        logger.debug("Message %s %s", topic, payload.decode())
        handler = self.handlers.get(topic)
        if not handler:
            logger.error("Ignoring topic %r", topic)
            return
        # TODO: check if handler is async
        # TODO: figure out error handling/logging in gmqtt
        await handler(self, topic, payload)
        return 0

    async def connect(self, host: str, port: int):
        try:
            await self.client.connect(host, port, raise_exc=False)
        except Exception as e:
            await self.client.reconnect()

    def publish(self, topic: str, payload, qos=0):
        if self.client.is_connected:
            self.client.publish(topic, payload, qos)

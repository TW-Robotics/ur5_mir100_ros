import websocket
import thread

import json
import traceback
import time

import string
import random


class RosbridgeSetup():
    def __init__(self, host, port, name='mir_9', timeout=0.01):
        self.callbacks = {}
        self.service_callbacks = {}
        self.resp = None
        self.timeout = timeout
        self.connection = RosbridgeWSConnection(host, port)
        self.connection.registerCallback(self.onMessageReceived)
        self.name = name

    def publish(self, topic, obj):
        pub = { "op": "publish", "topic": topic, "msg": obj }
        #print obj
        self.send(pub)

    def subscribe(self, topic, callback, throttle_rate=-1):
        if self.addCallback(topic, callback):
            sub = { "op": "subscribe", "topic": topic }
            if throttle_rate > 0:
                sub['throttle_rate'] = throttle_rate

            self.send(sub)

    def unhook(self, callback):
        keys_for_deletion = []
        for key, values in self.callbacks.iteritems():
            for value in values:
                if callback == value:
                    print "Found!"
                    values.remove(value)
                    if len(values) == 0:
                        keys_for_deletion.append(key)

        for key in keys_for_deletion:
            self.unsubscribe(key)
            self.callbacks.pop(key)

    def unsubscribe(self, topic):
        unsub = { "op": "unsubscribe", "topic": topic }
        self.send(unsub)

    def callService(self, serviceName, callback = None, msg = None):
        id = self.generate_id()
        call = { "op": "call_service", "id": id, "service": serviceName}
        if msg != None:
            call['args'] = msg

        if callback == None:
            self.resp = None
            def internalCB(msg):
                self.resp = msg
                return None

            self.addServiceCallback(id, internalCB)
            self.send(call)

            while self.resp == None:
                time.sleep(0.01)

            return self.resp

        self.addServiceCallback(id, callback)
        self.send(call)
        return None


    def send(self, obj):
        try:
            self.connection.sendString(json.dumps(obj))
            print obj
        except:
            traceback.print_exc()
            raise

    def generate_id(self, chars = 16):
        return ''.join(random.SystemRandom().choice(string.ascii_letters + string.digits) for _ in range(chars))

    def addServiceCallback(self, id, callback):
        self.service_callbacks[id] = callback

    def addCallback(self, topic, callback):
        if (self.callbacks.has_key(topic)):
            self.callbacks[topic].append(callback)
            return False

        self.callbacks[topic] = [ callback ]
        return True

    def onMessageReceived(self, message):
        try:
            # Load the string into a JSON object
            obj = json.loads(message)
            # print "Received: ", obj

            if obj.has_key('op'):
                option = obj['op']
                if option == "publish": # A message from a topic we have subscribed to..
                    topic = obj["topic"]
                    msg = obj["msg"]
                    if self.callbacks.has_key(topic):
                        for callback in self.callbacks[topic]:
                            try:
                                callback(msg)
                            except:
                                print "exception on callback", callback, "from", topic
                                traceback.print_exc()
                                raise
                elif option == "service_response":
                    if obj.has_key("id"):
                        id = obj["id"]
                        values = obj["values"]
                        if self.service_callbacks.has_key(id):
                            try:
                                #print 'id:', id, 'func:', self.service_callbacks[id]
                                self.service_callbacks[id](values)
                            except:
                                print "exception on callback ID:", id
                                traceback.print_exc()
                                raise
                    else:
                        print "Missing ID!"
                else:
                    print "Recieved unknown option - it was: ", option
            else:
                print "No OP key!"
        except:
            print "exception in onMessageReceived"
            print "message", message
            traceback.print_exc()
            raise


class RosbridgeWSConnection():
    def __init__(self, host, port):
        self.ws = websocket.WebSocketApp(("ws://%s:%d/" % (host, port)), on_message = self.on_message, on_error = self.on_error, on_close = self.on_close)
        self.ws.on_open = self.handle_connect
        self.run_thread = thread.start_new_thread(self.run, ())
        self.connected = False
        self.callbacks = []
        self.preconnectionBuffer = []

    def handle_connect(self, ws):
        print "### OPEN ###"
        self.connected=True
        for msg in self.preconnectionBuffer:
            self.sendString(msg)

        self.preconnectionBuffer = []

    def sendString(self, message):
        if (self.connected):
            self.ws.send(message)
        else:
            self.preconnectionBuffer.append(message)

    def on_error(self, ws, error):
        print "Error: %s" % error

    def on_close(self, ws):
        print "### closed ###"

    def run(self, *args):
        self.ws.run_forever()

    def on_message(self, ws, message):
        # Call the handlers
        for callback in self.callbacks:
            callback(message)

    def registerCallback(self, callback):
        self.callbacks.append(callback)

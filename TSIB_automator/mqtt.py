import json
import re
import paho.mqtt.client as mqtt
import mqtt_connect


class Mqtt(object):
    main_topic = 'rarach/timelab/'
    PTOPIC_CONNECT = 'rarach/timelab/connected'

    actor = {}

    def __init__(self):
        self.client = mqtt.Client(client_id="", clean_session=True, userdata="default", protocol=mqtt.MQTTv311, transport="tcp")
        self.client.on_message=self.on_message
        self.client.on_connect = self.on_connect
        self.client.on_subscribe = self.on_subscribe
        print("connecting to broker")
        self.connected = mqtt_connect.connect(self.client)
        if not self.connected:
            print("Unable to connect to MQTT broker server. Quiting.")
            return None

        self.client.loop_start()

    def registerActor(self, topic, fnc):
        self.actor[topic] = fnc
        # subscribe to topic
        if self.client and self.connected:
            self.client.subscribe(topic, 2)


    def on_connect(self, client, userdata, flags, rc):
        if rc==0:
            print("Connected to MQTT broker.")
            # print("Subscribing to topics\n")
            # print(STOPIC_SET)
            # (result, mid) = self.client.subscribe(STOPIC_SET + "/#", qos=2)
            # if (result != 0):
            #     print("Error subscribing to topics.")
            #     return
            self.publish(self.PTOPIC_CONNECT, {"value": "CONNECTED"})
        else:
            print("Failed to connect to MQTT broker. Return Code " + str(rc))

    def on_subscribe(self, client, userdata, mid, granted_qos):
        print("Subscribed to " + "" + " messages sucessfull. Mid is " + str(mid) + ", granted QOS is " + str(granted_qos))

    def on_message(self, client, userdata, message):
        try:
            payload = message.payload.decode("utf-8")
            print("message received  ", payload,\
                "topic",message.topic,"retained ",message.retain)
            if message.retain==1:
                print("This is a retained message")
            self.log(message.topic, payload)
            js = json.loads(payload)
            self.decode_command(message.topic, js)
            self.client.loop()
        except:
            print("Exception in mqtt on_message")

    def _decode_command(self, topic, message):
        for actor_topic in self.actor:
            re_topic = "^" + actor_topic.replace("+", "[^/]+").replace("#", ".+[/.+]*") + "$"
            if re.search(re_topic, topic):
                print("Notifying actor with topic " + actor_topic)
                self.arctor[actor_topic].fnc(message)                
        #if topic.startswith(STOPIC_SET) and "required_temperature" in topic:
            #value = message["value"]
        print("Topic {0} not implemented".format(topic))

    def publish(self, topic, msg, qos=0, retain=False):
        try:
            if self.client and self.connected:
                payload = json.dumps(msg)
                if type(msg) == float:
                    payload = msg
                self.client.publish(topic, payload, qos=qos, retain=retain)
                #print("Published " + topic + " " + str(msg))
                #print("Published " + topic)
        except:
            print("Unable to publish MQTT events.")
            if not self.connected:
                self.connected = mqtt_connect.connect(self.client)
                print("Client reconnected")
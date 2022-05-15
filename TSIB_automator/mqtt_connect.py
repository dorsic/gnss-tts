import paho.mqtt.client as mqtt

broker_address="broker.hivemq.com" 

def connect(client):
    try:
        client.connect(broker_address)
        return True
    except:
        return False

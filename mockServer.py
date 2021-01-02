#!/usr/bin/env python3
"""
Debug server
"""

from flask import Flask, request, send_from_directory, g
import json
import os

app = Flask(__name__, static_url_path='', static_folder='data/')

voltageValue = 116
update_mills = 900000

global mqtt_server
global mqtt_port
global mqtt_user
global client_id

mqtt_server = "192.168.1.123"
mqtt_port = 1883
mqtt_user = "mqttUser"
client_id = "POWER"
mqtt_password = ""

@app.route('/bootstrap.min.css')
def serve_static():
    """
    Magic BS to ship out a zipped static file
    """
    root_dir = os.path.dirname(os.getcwd())
    response = send_from_directory(os.path.join(root_dir, 'CurrentSensor/data'), "bootstrap.min.css.gz", mimetype="text/css; charset=UTF-8")
    response.headers['Content-Encoding'] = 'gzip'
    response.headers['Content-Type'] = 'text/css; charset=UTF-8'
    response.headers['Vary'] = 'Accept-Encoding'
    return response

@app.route('/webserial')
def ws():
    return "Only available on esp"

@app.route('/values')
def values():
    return """{"ApparentPower":0.080034,"WattsSum":61.37917,"MillsSum":53855,"kwh":0.0002, "voltage": %d, "update_s":%d }""" % (voltageValue, update_mills/1000), 200, {'ContentType':'text/json'} 

@app.route('/mqtt', methods=['POST','GET'])
def mqtt():
    global mqtt_server
    global mqtt_port
    global mqtt_user
    global client_id
    if request.method == 'POST':
        content = request.json
        print(content)
        return json.dumps({'success':True}), 200, {'ContentType':'text/json'}
    else:
        return json.dumps({"mqtt_server":mqtt_server, "mqtt_port":mqtt_port, "mqtt_user":mqtt_user, "client_id":client_id })


@app.route('/update', methods=['POST'])
def update_s():
    global update_mills
    global voltageValue

    if "voltage" in request.args:
        voltageValue = int(request.args.get("voltage"))

    if "update_s" in request.args:
        update_mills = int(request.args.get("update_s")) * 1000

    print(voltageValue, update_mills)
    return json.dumps({'success':True}), 200, {'ContentType':'text/json'} 


@app.route('/')
def root():
    return app.send_static_file('index.htm')


@app.route('/status')
def status():
    return json.dumps({"status":"Connected","mqttConn":True,"mqttState":"Connected","SSID":"MY_AP","IP":"192.168.1.162","APIP":"(IP unset)"}), 200, {'ContentType':'text/json'}


@app.route('/connect', methods=['POST'])
def connect():
    content = request.json
    print("Connecting to wifi:", content)
    return json.dumps({'success':True}), 200, {'ContentType':'text/json'}


scannedOnce = False
@app.route('/scan')
def scan():
    global scannedOnce

    if not scannedOnce:
        scannedOnce = True
        return json.dumps({"error":"again"}), 200, {'ContentType':'text/json'}
    
    print("wifi scan")
    return (json.dumps({
            "list":
                [
                    {"ssid":"tasmota-0372","rssi":-75,"enc":7},
                    {"ssid":"ESP-3C9E16","rssi":-44,"enc":7},
                    {"ssid":"MyWifi","rssi":-33,"enc":8},
                ]
            }),
        200,
        {'ContentType':'text/json'})


if __name__ == '__main__':
    app.run(debug=True)

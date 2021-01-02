function post_data(url,data) {
    fetch(url, {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json'
        },
        body: JSON.stringify(data)
    })
    .then((result) => {
        console.log('Success:', result);
    })
    .catch((error) => {
        console.error('Error:', error);
    });
}

function create_model() {
    window.model = { 
        "ApparentPower": 0, 
        "WattsSum": 0, 
        "MillsSum": 0, 
        "kwh": 0, 
        "voltage":0, 
        "voltage_new":'', 
        "show_voltage": false, 
        "update_s":0, 
        "show_update_s": false, 
        "update_s_new":''};
   return window.model;
}

function save_voltage(voltage_new) {
    post_data('./update?voltage='+voltage_new, "");
}

function save_update_s(update_s) {
    post_data('./update?update_s='+update_s, "");
}

function save_mqtt(mqtt_server, mqtt_port, client_id, mqtt_user, mqtt_password) {
    const data = { "mqtt_server": mqtt_server, "mqtt_port":mqtt_port, "client_id":client_id, "mqtt_user":mqtt_user, "mqtt_password":mqtt_password };
    post_data('./mqtt', data);
}

function connect(ssid, password){
    const data = { "ssid": ssid, "password":password};
    post_data('./connect', data);
}

function scan() {
    fetch('/scan')
        .then(response => response.json())
        .then(data => {
            console.log(data);
            
            if("error" in data && data.error === "again") {
                setTimeout(() => { scan(); }, 2000);
            } else {
                this.APs = data.list;
                let event = new CustomEvent("items-load", {
                    detail: {
                        items: this.APs
                    }
                });
                window.dispatchEvent(event);
            }
        }
    );
}
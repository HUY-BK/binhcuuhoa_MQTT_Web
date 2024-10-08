const brokerUrl='wss://broker.hivemq.com:8884/mqtt';
const clientId = 'clientId-bu1cEeNFt2'+ Math.floor(Math.random() * 10000);
client = new Paho.MQTT.Client(brokerUrl, clientId);

client.connect({
    onSuccess: onConnect,
    onFailure: function (message) {
        console.log("Connection failed: " + message.errorMessage);
    }
});

function onConnect() {
    console.log("Connected to MQTT broker");
    client.subscribe("devID/bientro"); // Subscribe to potentiometer topic
    client.subscribe("devID/cambien"); // Subscribe to BNO055 topic
}

client.onMessageArrived = function (message) {
    const topic = message.destinationName;
    const payload = message.payloadString; // lay noi dung ma khong dinh dang


    if (topic === "devID/bientro") {
        document.getElementById("potentiometerData").innerText = payload;
    } else if (topic === "devID/cambien") {
        const dulieu= JSON.parse(payload);
        document.getElementById("tx").innerText = dulieu.tx.toFixed(2);
        document.getElementById("ty").innerText = dulieu.ty.toFixed(2);
        document.getElementById("tz").innerText = dulieu.tz.toFixed(2);
        document.getElementById("gx").innerText = dulieu.gx.toFixed(2);
        document.getElementById("gy").innerText = dulieu.gy.toFixed(2);
        document.getElementById("gz").innerText = dulieu.gz.toFixed(2);
    }
};
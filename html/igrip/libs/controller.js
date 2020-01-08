//SETUP CONTAINERS
var widgets = {};
var nameToUUID = {};

var allLoaded = false;

//# of widgets loaded
var loadedArr = 0
var connector;

var lastPacketsSent = {};

function loadComplete(){
    loadedArr++;

    if(loadedArr == 6){
        setupWidgets();
        setupConnection();
    }
}

function setupConnection(){
    connector = new Connector();
}

function setupWidgets(){
    widget1Frame.contentWindow.exportRoot.setWidgetName("Position");
    widgets[widget1Frame.contentWindow.exportRoot.UUID] = widget1Frame.contentWindow.exportRoot;
    nameToUUID["joint0_0"] = widget1Frame.contentWindow.exportRoot.UUID;

    widget2Frame.contentWindow.exportRoot.setWidgetName("Velocity");
    widget2Frame.contentWindow.exportRoot.setColor("#00b354");
    widgets[widget2Frame.contentWindow.exportRoot.UUID] = widget2Frame.contentWindow.exportRoot;
    nameToUUID["joint0_1"] = widget2Frame.contentWindow.exportRoot.UUID;

    widget3Frame.contentWindow.exportRoot.setWidgetName("Stiffness");
    widget3Frame.contentWindow.exportRoot.setColor("#bb34ff");
    widgets[widget3Frame.contentWindow.exportRoot.UUID] = widget3Frame.contentWindow.exportRoot;
    nameToUUID["joint0_2"] = widget3Frame.contentWindow.exportRoot.UUID;

    widget4Frame.contentWindow.exportRoot.setWidgetName("Position");
    widgets[widget4Frame.contentWindow.exportRoot.UUID] = widget4Frame.contentWindow.exportRoot;
    nameToUUID["joint1_0"] = widget4Frame.contentWindow.exportRoot.UUID;

    widget5Frame.contentWindow.exportRoot.setWidgetName("Velocity");
    widget5Frame.contentWindow.exportRoot.setColor("#00b354");
    widgets[widget5Frame.contentWindow.exportRoot.UUID] = widget5Frame.contentWindow.exportRoot;
    nameToUUID["joint1_1"] = widget5Frame.contentWindow.exportRoot.UUID;

    widget6Frame.contentWindow.exportRoot.setWidgetName("Stiffness");
    widget6Frame.contentWindow.exportRoot.setColor("#bb34ff");
    widgets[widget6Frame.contentWindow.exportRoot.UUID] = widget6Frame.contentWindow.exportRoot;
    nameToUUID["joint1_2"] = widget6Frame.contentWindow.exportRoot.UUID;
}

//SETUP WIDGETS
var widget1Frame = document.getElementById("dof1_position");
widget1Frame.addEventListener("load", function() {
    setTimeout(function(){ 
        //notifyLoad();
    }, 1000);
});

var widget2Frame = document.getElementById("dof1_velocity");
widget2Frame.addEventListener("load", function() {
    setTimeout(function(){ 
        //notifyLoad()
    }, 1000);
});

var widget3Frame = document.getElementById("dof1_stiffness");
widget3Frame.addEventListener("load", function() {
    setTimeout(function(){ 
        //notifyLoad();
    }, 1000);
});

var widget4Frame = document.getElementById("dof2_position");
widget4Frame.addEventListener("load", function() {
    setTimeout(function(){ 
        //notifyLoad();
    }, 1000);
});

var widget5Frame = document.getElementById("dof2_velocity");
widget5Frame.addEventListener("load", function() {
    setTimeout(function(){ 
        //notifyLoad()
    }, 1000);
});

var widget6Frame = document.getElementById("dof2_stiffness");
widget6Frame.addEventListener("load", function() {
    setTimeout(function(){ 
        //notifyLoad();
    }, 1000);
});

//SETUP CONTROLLER HELPERS
function uuidToName(uuid){
    for(var key in nameToUUID){
        if(nameToUUID[key] == uuid){
            return key;
        }
    }

    return "";
}


function setValueCallback(val, uuid){
    if(connector){
        if(lastPacketsSent[uuid] != val){
            lastPacketsSent[uuid] = val;
            connector.sendValuePacket(uuidToName(uuid), val)
        }
    }
}

function setTrueValue(name, val){
    widgets[nameToUUID[name]].setTrueValue(val);
}

function setBounds(name, lowerBound, upperBound){
    console.log("setting bounds of: " + name + " to: " + lowerBound + " " + upperBound)
    widgets[nameToUUID[name]].setLowerBound(lowerBound);
    widgets[nameToUUID[name]].setUpperBound(upperBound);
}

class Connector{
    constructor(host, port){
        this.ws = new WebSocket("ws://127.0.0.1:5679/");
        this.ws.onmessage = this.onMessage;
        this.ws.onopen = this.onOpen;
        this.ws.onclose = this.onClose;
        this.ws.onerror = this.onError;

        //this.onOpen();
    }
    //SETUP WEBSOCKET CLIENT

    onMessage(event) {
        console.log("recv: " + event.data);
        
        //handle set true value
        if(event.data.indexOf('%')){
            var tokens = event.data.split('%');

            //feedback value update
            if(tokens[0] == "fv"){
                if(nameToUUID[tokens[1]] && tokens[2]){
                    console.log("Setting feedback (true) value: " + tokens[1] + " " + tokens[2]);
                    setTrueValue(tokens[1], parseFloat(tokens[2]));
                }
            }else if(tokens[0] == "svb"){ //set value bound
                if(nameToUUID[tokens[1]] && tokens[2] && tokens[3]){
                    console.log("Setting bounds for " + tokens[1] + ": " + tokens[2] + ", " + tokens[3]);
                    setBounds(tokens[1], parseFloat(tokens[2]), parseFloat(tokens[3]));
                }
            }else{
                console.log("Unhandled packet received: " + tokens[0]);
            }
        }
    }

    onOpen(){
        console.log('Connection established!');
        document.getElementById('statusBox').innerHTML = ""
        document.getElementById('topStatus').innerHTML = "Status: Online"
    }

    onClose(){
        console.log('Connection lost!');
        document.getElementById('statusBox').innerHTML = "Connection lost."
        document.getElementById('topStatus').innerHTML = "Status: <font color='#ff0000'>Offline</font>"
    }

    onError(){
        console.log('Connection error!')
        document.getElementById('statusBox').innerHTML = "Connection error :("
        document.getElementById('topStatus').innerHTML = "Status: <font color='#ff0000'>Offline</font>"
    }

    sendValuePacket(src, val){
        var pkt = ['uv', src, val].join('%');
        this.ws.send(pkt);
    }
}
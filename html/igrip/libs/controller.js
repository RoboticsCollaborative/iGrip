//SETUP CONTAINERS
var widgets = {};
var nameToUUID = {};

//SETUP WIDGETS
var widget1Frame = document.getElementById("dof1_position");
widget1Frame.addEventListener("load", function() {
    setTimeout(function(){ 
        widget1Frame.contentWindow.exportRoot.setWidgetName("Position");

        widgets[widget1Frame.contentWindow.exportRoot.UUID] = widget1Frame.contentWindow.exportRoot;
        nameToUUID["widget1"] = widget1Frame.contentWindow.exportRoot.UUID;
    }, 1000);
});

var widget2Frame = document.getElementById("dof1_velocity");
widget2Frame.addEventListener("load", function() {
    setTimeout(function(){ 
        widget2Frame.contentWindow.exportRoot.setWidgetName("Velocity");
        widget2Frame.contentWindow.exportRoot.setColor("#00b354");
        widgets[widget2Frame.contentWindow.exportRoot.UUID] = widget2Frame.contentWindow.exportRoot;
        nameToUUID["widget2"] = widget2Frame.contentWindow.exportRoot.UUID;
    }, 1000);
});

var widget3Frame = document.getElementById("dof1_stiffness");
widget3Frame.addEventListener("load", function() {
    setTimeout(function(){ 
        widget3Frame.contentWindow.exportRoot.setWidgetName("Stiffness");
        widget3Frame.contentWindow.exportRoot.setColor("#bb34ff");
        widgets[widget3Frame.contentWindow.exportRoot.UUID] = widget3Frame.contentWindow.exportRoot;
        nameToUUID["widget3"] = widget3Frame.contentWindow.exportRoot.UUID;
    }, 1000);
});

//SETUP CONTROLLER HELPERS
function uuidToName(uuid){
    for(var key in nameToUUID){
        if(nameToUUID[key] == uuid){
            return key;
        }
    }

    return "N/A";
}

function setValueCallback(val, uuid){
    //console.log("Callback from " + uuidToName(uuid) + " - setValue: " + val);
    sendValuePacket(uuidToName(uuid), val)
}

function setTrueValue(name, val){
    widgets[nameToUUID[name]].setTrueValue(val);
}

//SETUP WEBSOCKET CLIENT
var ws = new WebSocket("ws://127.0.0.1:5679/");

ws.onmessage = function (event) {
    console.log("recv: " + event.data);
    
    //handle set true value
    if(event.data.indexOf('%')){
        var tokens = event.data.split('%');

        //feedback value update
        if(tokens[0] == "fv"){
            if(nameToUUID[tokens[1]] && tokens[2]){
                console.log("Setting feedback (true) value: " + tokens[1] + " " + tokens[2]);
                setTrueValue(tokens[1], tokens[2]);
            }
        }else{
            console.log("Unhandled packet received: " + tokens[0]);
        }
    }
};

ws.onopen = function(){
    console.log('Connection established!');
    document.getElementById('statusBox').innerHTML = ""
    document.getElementById('topStatus').innerHTML = "Status: Online"
}

ws.onclose = function(){
    console.log('Connection lost!');
    document.getElementById('statusBox').innerHTML = "Connection lost."
    document.getElementById('topStatus').innerHTML = "Status: <font color='#ff0000'>Offline</font>"
}

ws.onerror = function(){
    console.log('Connection error!')
    document.getElementById('statusBox').innerHTML = "Connection error :("
    document.getElementById('topStatus').innerHTML = "Status: <font color='#ff0000'>Offline</font>"
}

function sendValuePacket(src, val){
    var pkt = ['uv', src, val].join('%');
    ws.send(pkt);
}
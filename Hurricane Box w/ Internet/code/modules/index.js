var $ = require( "jquery" );
var express = require('express');
var app = express();
var path = require('path');
var fs = require('fs');
const readline = require('readline');
const Stream = require('stream');
const bodyParser = require('body-parser');
const router = express.Router();

app.use(bodyParser.urlencoded({ extended: false }));
app.use(bodyParser.json());


var lastMessage = "";
var led_status = 0;
var lastNMessages = [];
var N =25;
var lastTime = -1;

// viewed at http://localhost:8080
app.get('/', function(req, res) {
  res.sendFile(path.join(__dirname + '/index.html'));
});



app.post('/status', (req,res) => {
  led_status = req.body.led_status;
  res.end('yes');
});

app.get('/status', function(req,res) {
  res.send([led_status]);
})


// request data at http://localhost:8080/data or just "/data"
app.get('/data', function(req, res) {
  res.send(lastNMessages);  // Send array of data back to requestor
});


// request data at http://localhost:8080/data or just "/data"
app.get('/data/last', function(req, res) {
  res.send(lastMessage.split(','))
});


app.listen(4000);



// Required module
var dgram = require('dgram');

// Port and IP
var PORT = 1234;
var HOST = '192.168.1.111';

// Create socket
var server = dgram.createSocket('udp4');



// Create server
server.on('listening', function () {
    var address = server.address();
    console.log('UDP Server listening on ' + address.address + ":" + address.port);
});

// On connection, print out received message
server.on('message', function (message, remote) {
    lastMessage = message.toString();
    time = lastMessage.split(',');
    time = parseInt(time[0]);
    if (time > lastTime)
    {
      lastNMessages.push(lastMessage);
      lastTime = time;
      if (lastNMessages.length > N)
      {
        lastNMessages.shift();
      }
    }
    else
    {
      lastNMessages = [];
      lastNMessages.push(lastMessage);
      lastTime = time;
    }
    console.log(remote.address + ':' + remote.port +' - ' + message);

    server.send(led_status.toString(),remote.port,remote.address,function(error){
      if(error){
        console.log('MEH!');
      }
      else {
        console.log('Sent: ' + led_status.toString());
      }
    });

});

// Bind server to port and IP
server.bind(PORT, HOST);
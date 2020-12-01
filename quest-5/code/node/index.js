var express = require('express');
var app = express();
var path = require('path');
const bodyParser = require('body-parser');

app.use(bodyParser.urlencoded({ extended: false }));
app.use(bodyParser.json());


var status = 0;

// viewed at http://localhost:8080
app.get('/', function(req, res) {
  res.sendFile(path.join(__dirname + '/index.html'));
});



app.post('/status', (req,res) => {
  status = req.body.status;
  res.end('yes');
});

app.get('/status', function(req,res) {
  res.send([status]);
})


app.listen(4000);



// Required module
var dgram = require('dgram');

// Port and IP
var PORT = 1234;
var HOST = '192.168.1.139';

// Create socket
var server = dgram.createSocket('udp4');



// Create server
server.on('listening', function () {
    var address = server.address();
    console.log('UDP Server listening on ' + address.address + ":" + address.port);
});

// On connection, print out received message
server.on('message', function (message, remote) {
    
    console.log(remote.address + ':' + remote.port +' - ' + message);

    server.send(status.toString(),remote.port,remote.address,function(error){
      if(error){
        console.log('MEH!');
      }
      else {
        console.log('Sent: ' + status.toString());
      }
    });

});

// Bind server to port and IP
server.bind(PORT, HOST);
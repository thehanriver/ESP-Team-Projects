import {led_status,toggleLED} from './toggle.js';

var express = require('express');
var app = express();
var path = require('path');
var fs = require('fs');
const readline = require('readline');
const Stream = require('stream');

var lastMessage = "";

// getLastLine function is written by Michael Hobbs
// and taken from https://stackoverflow.com/questions/40107433/read-last-line-of-a-large-file-with-nodejs/46349455
getLastLine = (fileName, minLength) => {
  let inStream = fs.createReadStream(fileName);
  let outStream = new Stream;
  return new Promise((resolve, reject)=> {
      let rl = readline.createInterface(inStream, outStream);

      let lastLine = '';
      rl.on('line', function (line) {
          if (line.length >= minLength) {
              lastLine = line;
          }
      });

      rl.on('error', reject)

      rl.on('close', function () {
          resolve(lastLine)
      });
  })
}

// viewed at http://localhost:8080
app.get('/', function(req, res) {
  res.sendFile(path.join(__dirname + '/index.html'));
});


app.get('/toggle.js', function(req, res) {
  res.sendFile(path.join(__dirname + '/toggle.js'));
});
// // request data at http://localhost:8080/data or just "/data"
// app.get('/data', function(req, res) {
//   var data = [];  // Array to hold all csv data
//   var last_row = "";
//   fs.createReadStream('../data/sensors.csv')  // path to csv
//   .pipe(csv())
//   .on('data', (row) => {
//     // add thing to check time on row to check if it is a new row. push to data only if it is new
//     if (row === last_row){
//       return;
//     } else {
//       // console.log(row);
//       data.push(row);  // Add row of data to array
//       last_row = row;
//     }
//   })
//   .on('end', () => {
//     res.send(data);  // Send array of data back to requestor
//   });
// });

// // request data at http://localhost:8080/data or just "/data"
// app.get('/data/last', function(req, res) {
//   data_last = []
//   last_lastLine = '';
//   const filename = '../data/sensors.csv';
//   getLastLine(filename, 1)
//     .then((lastLine)=> {
//       if (lastLine === last_lastLine){
//         return;
//       } else {
//         last_lastLine = lastLine;
//         data_last.push(lastLine.split(','))
//         res.send(data_last);
//       }
//     })
//     .catch((err)=> {
//         console.error(err);
//         res.send(err);
//     });
// });

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

// led_status = 0;

// function toggleLED() {
//     if (!led_status)
//     {
//       led_status=1;
//     }
//     else
//     {
//       led_status=0
//     }
// }


// Create server
server.on('listening', function () {
    var address = server.address();
    console.log('UDP Server listening on ' + address.address + ":" + address.port);
});

// On connection, print out received message
server.on('message', function (message, remote) {
    lastMessage = message.toString();
    console.log(remote.address + ':' + remote.port +' - ' + message);
    //num = (num+1)%2;
    // Send Ok acknowledgement
    server.send(led_status.toString(),remote.port,remote.address,function(error){
      if(error){
        console.log('MEH!');
      }
      else {
        console.log('Sent: Ok');
      }
    });

});

// Bind server to port and IP
server.bind(PORT, HOST);
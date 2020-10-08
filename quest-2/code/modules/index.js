var express = require('express');
var app = express();
var path = require('path');
var fs = require('fs');
var csv = require("csv-parse");
const readline = require('readline');
const Stream = require('stream');

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
// request data at http://localhost:8080/data or just "/data"
app.get('/data', function(req, res) {
  var data = [];  // Array to hold all csv data
  var last_row = "";
  fs.createReadStream('../data/sensors.csv')  // path to csv
  .pipe(csv())
  .on('data', (row) => {
    // add thing to check time on row to check if it is a new row. push to data only if it is new
    if (row === last_row){
      return;
    } else {
      // console.log(row);
      data.push(row);  // Add row of data to array
      last_row = row;
    }
  })
  .on('end', () => {
    res.send(data);  // Send array of data back to requestor
  });
});

// request data at http://localhost:8080/data or just "/data"
app.get('/data/last', function(req, res) {
  data_last = []
  last_lastLine = '';
  const filename = '../data/sensors.csv';
  getLastLine(filename, 1)
    .then((lastLine)=> {
      if (lastLine === last_lastLine){
        return;
      } else {
        last_lastLine = lastLine;
        data_last.push(lastLine.split(','))
        res.send(data_last);
      }
    })
    .catch((err)=> {
        console.error(err);
        res.send(err);
    });
});


app.listen(8080);


var $ = require( "jquery" );
var express = require('express');
var app = express();
var path = require('path');
var http = require('http').Server(app);


const bodyParser = require('body-parser');
//useUnifiedTopology: true,
var MongoClient = require('mongodb').MongoClient;
const uri = "mongodb+srv://thehanriver:1GBSt0rage%21@cluster0.zottf.mongodb.net/Election?retryWrites=true&w=majority";
// const uri = "mongodb+srv://mario:1GBSt0rage%21@vivcluster.h5rba.mongodb.net/testSmok?retryWrites=true&w=majority";
// useUnifiedTopology: true,
var client = new MongoClient(uri, { useUnifiedTopology: true,useNewUrlParser: true});

// Required module
var dgram = require('dgram');

// Port and IP
var PORT = 1111;
var HOST = '192.168.1.131';

// Create socket
var server = dgram.createSocket('udp4');

// Create server
server.on('listening', function () {
    var address = server.address();
    console.log('UDP Server listening on ' + address.address + ":" + address.port);
});

// On connection, print out received message
server.on('message', function (message, remote) {
    var myObj = [];
    var id;
    var password_x;
    var password_y;
    var password_z;
    var setbit;
    var buffer;
    var myObj;
    var result;
    const projection = {
      "id": id,
    }
    //console.log("Message: ",message);


    buffer = message.toString();
    buffer = buffer.split(':');
    setbit = parseInt(buffer[0]);
    id = parseInt(buffer[1]);
    password_x = parseInt(buffer[2]);
    password_y = parseInt(buffer[3]);
    password_z = parseInt(buffer[4]);
    myObj.push({id: id , x: password_x , y: password_y, z: password_z});

    switch (setbit) {
      case 1:

        MongoClient.connect(uri, function(err, client) {
          assert.equal(null, err);
          console.log("CONNECTED");
          var collection = client.db("Key").collection("Users");



          // collection.findOne(myObj, function(err, res) {
          //   if (err) throw err;
          //   console.log("works");
          //   // db.close();
          // });

          collection.inserOne(myObj, function(err, res) {
            if (err) throw err;
            console.log("works");
            // db.close();
          });
          client.close();
        });
      break;

      case 0:

        MongoClient.connect(uri, function(err, client) {
          assert.equal(null, err);
          console.log("CONNECTED");
          var collection = client.db("Key").collection("Users");
            result = itemsCollection.findOne(query, projection)
            if(!result)
              console.log("NOT FOUND");
            else {
              console.log("FOUND");
            }
      //     collection.findOne(myObj, function(err, res) {
      //       if (err) throw err;
      //       console.log("works");
      //       // db.close();
      //     });
      //     client.close();
      //   });
      // break;

    }

    console.log(remote.address + ':' + remote.port +' - ' + message);
    server.send("vote ",remote.port,remote.address,function(error){
      if(error){
        console.log('Error: could not reply');
      }
      else {
        console.log('Sent: ');
      }
    });

});

// Bind server to port and IP
server.bind(PORT, HOST);
app.listen(4000);

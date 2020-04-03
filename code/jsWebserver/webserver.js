var http = require('http');
var url = require('url');
var fs = require('fs');
const SerialPort = require('serialport');
const Readline = SerialPort.parsers.Readline;
var dgram = require('dgram');

var HOST = '192.168.1.109';
var WEBPORT = 3000;
var UDPPORT = 8080;
var ESPHOST = 0;
var ESPPORT = 0;

// ======================== WEBSERVER

console.log('Initializing Webserver on port: ' + WEBPORT);

http.createServer(function (req, res) {
  var q = url.parse(req.url, true);
  var filename = "." + q.pathname;

  if (q.pathname == "/light") {
    LED()
    res.write('light recieved.'); //write a response to the client
    res.end(); //end the response
  }

  else{ 
    fs.readFile(filename, function(err, data) {
      if (err) {
        fs.readFile("404.html", function(err, data) {
          res.writeHead(200, {'Content-Type': 'text/html'});
          res.write(data);
          return res.end();
        });
      }
      else {
      res.writeHead(200, {'Content-Type': 'text/html'});
      res.write(data);
      return res.end();
  }
  });
}
}).listen(WEBPORT);

// end Webserver

// ======================== ESP ENDPOINT

// Create socket
var server = dgram.createSocket('udp4');

// Create server
server.on('listening', function () {
    var address = server.address();
    console.log('Initializing UDP Server on ' + address.address + ":" + address.port + "\n\n");
});

function tryParseJson (str) {
    try {
        JSON.parse(str);
    } catch (e) {
        return false;
    }
    return JSON.parse(str);
}

// On connection, print out received message
server.on('message', function (message, remote) {
    //console.log(remote.address + ':' + remote.port +' - ' + message);
    const sensorData = tryParseJson(message);

    if (message == "INIT") {
      ESPHOST = remote.address
      ESPPORT = remote.port
      console.log('Device connected and initialized @: ' + remote.address + ':' + remote.port);
    }
    else { 
      // append message to data file
      fs.appendFile('data/battery.csv', `\n${sensorData.voltage}`, function (err) {
            if (err) return console.log(err);
            console.log('- Data Recieved from ESP @ ' + remote.address + ':' + remote.port + ' containing ' + message);
      });
      fs.appendFile('data/thermistor.csv', `\n${sensorData.thermistor}`, function (err) {
            if (err) return console.log(err);
            console.log('- Data Recieved from ESP @ ' + remote.address + ':' + remote.port + ' containing ' + message);
      });
      fs.appendFile('data/steps.csv', `\n${sensorData.steps}`, function (err) {
            if (err) return console.log(err);
            console.log('- Data Recieved from ESP @ ' + remote.address + ':' + remote.port + ' containing ' + message);
      });
    }

});

function LED() {
  if (ESPPORT != 0 && ESPHOST != 0){
  server.send("Light",ESPPORT,ESPHOST,function(error){
      if(error){
        console.log('ERROR: Sending Light Signal to ESP.');
      }
      else{
        console.log('- Sending Light Signal to ESP: ' + ESPPORT + ':' + ESPHOST);
      }
    });
  }
  else {
    console.log('ERROR: esp not connected, cannot light LED');
  }
}

// Bind server to port and IP
server.bind(UDPPORT, HOST);

// end esp endpoint


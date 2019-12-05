'use strict';

const net = require('net')

var ip = '127.0.0.1'
var port = 5762 // MSP port on UART2

// via MSP:
const MSP = require('./node-msp') // https://github.com/cs8425/node-msp
var msp = new MSP()

var init = 0;

var RC_MAP = '';
var channelMSPIndexes = {
		A: 3,
		E: 2,
		R: 1,
		T: 0,
		'1': 4,
		'2': 5,
		'3': 6,
		'4': 7,
}


msp.on('frame', function(err, frame){
	if(err) return
//	console.log((new Date()).getTime(), 'frame', JSON.stringify(frame))

//	var obj = msp.parseFrame(frame)
//	console.log((new Date()).getTime(), 'data', obj)
})
msp.on('data', function(obj){
//	console.log((new Date()).getTime(), 'data', obj.code, obj)
	if(obj.code == msp.Codes.MSP_RX_MAP){
		RC_MAP = obj.RC_MAP;
		for(var i=0; i<RC_MAP.length; i++){
//			console.log(RC_MAP[i], i)
			channelMSPIndexes[RC_MAP[i]] = i
		}
		console.log('RC_MAP', RC_MAP)
		console.log('channelMSPIndexes', channelMSPIndexes)
		init = 1;
	}
})

var client = net.connect(port, ip, function(){
	console.log('connected to FC!', msp)

	msp.setSender(function(data){
		//console.log('_write', data)
		client.write(data)
	})

//	msp.pull_FC_info()

	msp.send_message(msp.Codes.MSP_RX_MAP, false, false);
})
client.on('error', function(err){
	console.log('FC err', err)
	js.close()
	init = 0
})
client.on('data', function(data){
	//console.log(data)
	msp.readbytes(data)
})
client.on('end', function(){
	console.log('disconnected from server')
})

////////////////////////
// from: https://gist.github.com/creationix/1695870
///////////////////////
var FS = require('fs');
var EventEmitter = require('events').EventEmitter;

// http://www.mjmwired.net/kernel/Documentation/input/joystick-api.txt
function parse(buffer) {
	var event = {
		time: buffer.readUInt32LE(0),
		number: buffer[7],
		value: buffer.readInt16LE(4)
	}
	if (buffer[6] & 0x80) event.init = true;
	if (buffer[6] & 0x01) event.type = "button";
	if (buffer[6] & 0x02) event.type = "axis";
	return event;
}

// Expose as a nice JavaScript API
function Joystick(id) {
	this.onOpen = this.onOpen.bind(this);
	this.onRead = this.onRead.bind(this);
	this.buffer = new Buffer(8);
	FS.open("/dev/input/js" + id, "r", this.onOpen);
}
Joystick.prototype = Object.create(EventEmitter.prototype, {
	constructor: {value: Joystick}
});

Joystick.prototype.onOpen = function (err, fd) {
	if (err) return this.emit("error", err);
	this.fd = fd;
	this.startRead();
};

Joystick.prototype.startRead = function () {
	FS.read(this.fd, this.buffer, 0, 8, null, this.onRead);
};

Joystick.prototype.onRead = function (err, bytesRead) {
	if (err) return this.emit("error", err);
	var event = parse(this.buffer);
	this.emit(event.type, event);
	if (this.fd) this.startRead();
};

Joystick.prototype.close = function (callback) {
	FS.close(this.fd, callback);
	this.fd = undefined;
};
///////////////////

// define joystick channel
var joy2RC = {
	0: 'A',
	1: 'E',
	2: '1',
	3: 'T',
	4: 'R',
	5: '2',
}
var joyBtn2RC = {
	0: '1',
	1: '4',
	2: '3',
	3: '3',
	4: '3',
	5: '3',
}
var channelValues = [1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500];
var js = new Joystick(0);
js.on('button', function(data){
	var ch = data.number
	var val = data.value
	val = 800 * val + 1100

	channelValues[channelMSPIndexes[joyBtn2RC[ch]]] = val;
});
js.on('axis', function(data){
	var ch = data.number
	var val = data.value
	val = (1000 * val / 65535) + 1500

	channelValues[channelMSPIndexes[joy2RC[ch]]] = val;
console.log(channelValues)
//	msp.send_message(msp.Codes.MSP_SET_RAW_RC, channelValues);
});

function sendMSP() {
	if(init){
		var buf = Buffer.alloc(channelValues.length*2)
		for(var i=0; i<channelValues.length; i++){
			buf.writeUInt16LE(channelValues[i] ,2*i)
		}
		msp.send_message(msp.Codes.MSP_SET_RAW_RC, buf)
	}
	var t = setTimeout(sendMSP, 20);
}
sendMSP()


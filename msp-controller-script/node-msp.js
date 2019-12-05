// copy & modify from https://github.com/betaflight/betaflight-configurator
'use strict';

var util = require('util');
var EventEmitter = require('events').EventEmitter;

var MSPCodes = require('./MSPCodes');

var MSP = function () {
	var self = this
	self.state = 0
	self.message_direction = 1	// 1, '>' from FC; 0, '<' to FC
	self.message_length_expected = 0
	self.message_length_received = 0

	self.message_checksum = 0
	self.messageIsJumboFrame = false
	self.crcError = false

	self.message_buffer = null
	self.message_buffer_uint8_view = null

	self.packet_error = 0
	self.unsupported = 0

	self.last_received_timestamp = null
	self.JUMBO_FRAME_SIZE_LIMIT = 255

	self.timeout = 1000


	self.sender = null
	self.sendFIFO = []
	self.sending = false
//	self.on('data', read)
};

util.inherits(MSP, EventEmitter);
module.exports = MSP;

MSP.prototype.Codes = MSPCodes

MSP.prototype.setSender = function(func){
	var self = this
	if(func){
		self.sender = func
	}else{
		self.sender = null
	}
}

MSP.prototype.sendRawFrame = function(rawFrame){
	var self = this
	self.sendFIFO.push(rawFrame)
	if(!self.sending){
//		process.nextTick(self.sendWorker, self)
		setImmediate(self.sendWorker, self)
	}
}

MSP.prototype.sendWorker = function(self){
//	var self = this
	if(self.sendFIFO.length){
		self.sender(self.sendFIFO.shift())
	}
	if(self.sendFIFO.length){
//		process.nextTick(self.sendWorker, self)
		setImmediate(self.sendWorker, self)
	}
}

MSP.prototype.readbytes = function (data){
	var self = this
	for (var i = 0; i < data.length; i++) {
		switch (self.state) {
			case 0: // sync char 1
				if (data[i] == 36) { // $
					self.state++;
				}
				break;
			case 1: // sync char 2
				if (data[i] == 77) { // M
					self.state++;
				} else { // restart and try again
					self.state = 0;
				}
				break;
			case 2: // direction (should be >)
				self.unsupported = 0;
				if (data[i] == 62) { // > from FC
					self.message_direction = 1;
				} else if (data[i] == 60) { // < to FC
					self.message_direction = 0;
				} else if (data[i] == 33) { // !
					// FC reports unsupported message error
					self.unsupported = 1;
				}

				self.state++;
				break;
			case 3:
				self.message_length_expected = data[i];
				if (self.message_length_expected === self.JUMBO_FRAME_SIZE_LIMIT) {
					self.messageIsJumboFrame = true;
				}

				self.message_checksum = data[i];

				self.state++;
				break;
			case 4:
				self.code = data[i];
				self.message_checksum ^= data[i];

				if (self.message_length_expected > 0) {
					// process payload
					if (self.messageIsJumboFrame) {
						self.state++;
					} else {
						self.state = self.state + 3;
					}
				} else {
					// no payload
					self.state += 5;
					self.message_buffer = new ArrayBuffer(0);
					self.message_buffer_uint8_view = new Uint8Array(self.message_buffer);
				}
				break;
			case 5:
				self.message_length_expected = data[i];

				self.message_checksum ^= data[i];

				self.state++;

				break;
			case 6:
				self.message_length_expected = self.message_length_expected  + 256 * data[i];

				self.message_checksum ^= data[i];

				self.state++;

				break;
			case 7:
				// setup arraybuffer
				self.message_buffer = new ArrayBuffer(self.message_length_expected);
				self.message_buffer_uint8_view = new Uint8Array(self.message_buffer);

				self.state++;
			case 8: // payload
				self.message_buffer_uint8_view[self.message_length_received] = data[i];
				self.message_checksum ^= data[i];
				self.message_length_received++;

				if (self.message_length_received >= self.message_length_expected) {
					self.state++;
				}
				break;
			case 9:
				var buf = null
				if (self.message_checksum == data[i]) {
					// message received, store buffer
					buf = Buffer.from(self.message_buffer, 0, self.message_length_expected)
				} else {
					console.log('code: ' + self.code + ' - crc failed');
					self.packet_error++;
					self.crcError = true;
				}
				// Reset variables
				self.message_length_received = 0;
				self.state = 0;
				self.messageIsJumboFrame = false;
				//self.notify();

				var frame = {
					crcError: self.crcError,
					dir: self.message_direction,
					code: self.code,
					data: buf
				}
				self.emit('frame', self.crcError, frame)

				// send raw frame if is a custom command
				if(self.code == MSPCodes.MSP_EXT_CMD){
					self.emit('extcmd', frame)
				}

				var obj = self.parseFrame(frame)
				self.emit('data', obj)

				self.crcError = false;
				break;

			default:
				console.log('Unknown state detected: ' + self.state);
		}
	}
}

MSP.prototype.create_message = function (code, data, dir) {
	var self = this
	var bufferOut, bufView;
	var dircode = 0

	if(dir){
		dircode = '>'.charCodeAt(0) // 62 '>' from FC
	}else{
		dircode = '<'.charCodeAt(0) // 60 '<' to FC
	}

	// always reserve 6 bytes for protocol overhead !
	if (data) {
		var size = data.length + 6,
		checksum = 0;

		bufferOut = new ArrayBuffer(size);
		bufView = new Uint8Array(bufferOut);

		bufView[0] = 36; // $
		bufView[1] = 77; // M
		bufView[2] = dircode; // < or >
		bufView[3] = data.length;
		bufView[4] = code;

		checksum = bufView[3] ^ bufView[4];

		for (var i = 0; i < data.length; i++) {
			bufView[i + 5] = data[i];

			checksum ^= bufView[i + 5];
		}

		bufView[5 + data.length] = checksum;
	} else {
		bufferOut = new ArrayBuffer(6);
		bufView = new Uint8Array(bufferOut);

		bufView[0] = 36; // $
		bufView[1] = 77; // M
		bufView[2] = dircode; // < or >
		bufView[3] = 0; // data length
		bufView[4] = code; // code
		bufView[5] = bufView[3] ^ bufView[4]; // checksum
	}

	return Buffer.from(bufferOut);
}

MSP.prototype.send_message = function (code, data, callback) {
	var self = this
	var buf = self.create_message(code, data)
	self.sendRawFrame(buf)
}

MSP.prototype.pull_FC_info = function (callback) {
	var self = this
	var pulllist = [
		MSPCodes.MSP_API_VERSION,
		MSPCodes.MSP_FC_VARIANT,
		MSPCodes.MSP_FC_VERSION,
		MSPCodes.MSP_BOARD_INFO,
		MSPCodes.MSP_BUILD_INFO,
		MSPCodes.MSP_NAME,
		MSPCodes.MSP_UID,
		MSPCodes.MSP_RX_MAP,
		MSPCodes.MSP_BOXNAMES,
		MSPCodes.MSP_BOXIDS,
//		MSPCodes.MSP_PIDNAMES,
//		MSPCodes.MSP_PID,
	]

	for(var i=0; i<pulllist.length; i++){
		self.send_message(pulllist[i], null)
	}
}

MSP.prototype.parseFrame = function (frame, dir){
	var self = this
	var dir = frame.dir
	var code = frame.code
	var data = frame.data
    var crcError = frame.crcError
	var obj = {}

	if(crcError){
		return obj
	}

	if(dir == 0){ // to FC
		return obj
	}

	obj.code = code
	switch (code) {
		case MSPCodes.MSP_STATUS:
			obj.cycleTime = data.readU16()
			obj.i2cError = data.readU16()
			obj.activeSensors = data.readU16()
			obj.mode = data.readU32()
			obj.profile = data.readU8()
			break
		case MSPCodes.MSP_STATUS_EX:
			obj.cycleTime = data.readU16()
			obj.i2cError = data.readU16()
			obj.activeSensors = data.readU16()
			obj.mode = data.readU32()
			obj.profile = data.readU8()
			obj.cpuload = data.readU16()
			obj.numProfiles = data.readU8()
			obj.rateProfile = data.readU8()
			break

		case MSPCodes.MSP_RAW_IMU:
			// 512 for mpu6050, 256 for mma
			// currently we are unable to differentiate between the sensor types, so we are goign with 512
			obj.accelerometer = [0,0,0]
			obj.accelerometer[0] = data.read16() / 512
			obj.accelerometer[1] = data.read16() / 512
			obj.accelerometer[2] = data.read16() / 512

			// properly scaled
			obj.gyroscope = [0,0,0]
			obj.gyroscope[0] = data.read16() * (4 / 16.4)
			obj.gyroscope[1] = data.read16() * (4 / 16.4)
			obj.gyroscope[2] = data.read16() * (4 / 16.4)

			// no clue about scaling factor
			obj.magnetometer = [0,0,0]
			obj.magnetometer[0] = data.read16() / 1090
			obj.magnetometer[1] = data.read16() / 1090
			obj.magnetometer[2] = data.read16() / 1090
			break

		case MSPCodes.MSP_SERVO:
			obj.servo = []
			var servoCount = data.byteLength / 2
			var servo = obj.servo
			for (var i = 0; i < servoCount; i++) {
				servo.push(data.readU16())
			}
			break

		case MSPCodes.MSP_MOTOR:
			obj.motor = []
			var motorCount = data.byteLength / 2
			var motor = obj.motor
			for (var i = 0; i < motorCount; i++) {
				motor.push(data.readU16())
			}
			break

		case MSPCodes.MSP_RC:
			obj.active_channels = data.byteLength / 2
			obj.channels = []
			var channels = obj.channels
			for (var i = 0; i < obj.active_channels; i++) {
				channels.push(data.readU16())
			}
			break

		case MSPCodes.MSP_RAW_GPS:
			obj.fix = data.readU8()
			obj.numSat = data.readU8()
			obj.lat = data.read32()
			obj.lon = data.read32()
			obj.alt = data.readU16()
			obj.speed = data.readU16()
			obj.ground_course = data.readU16()
			break

		case MSPCodes.MSP_COMP_GPS:
		break

		case MSPCodes.MSP_ATTITUDE:
			obj.x = data.read16() / 10.0 // x
			obj.y = data.read16() / 10.0 // y
			obj.z = data.read16() // z
			break
		case MSPCodes.MSP_ALTITUDE:
			obj.altitude = parseFloat((data.read32() / 100.0).toFixed(2)) // correct scale factor
			break
		case MSPCodes.MSP_DEBUG:
			obj.debug = []
			var debug = obj.debug
			for (var i = 0; i < 4; i++) debug.push(data.read16())
			break


// get once
		case MSPCodes.MSP_API_VERSION:
			obj.mspProtocolVersion = data.readU8()
			obj.apiVersion = data.readU8() + '.' + data.readU8() + '.0'
			break

		case MSPCodes.MSP_FC_VARIANT:
			var identifier = ''
			for (var i = 0; i < 4; i++) {
				identifier += String.fromCharCode(data.readU8())
			}
			obj.flightControllerIdentifier = identifier
			break

		case MSPCodes.MSP_FC_VERSION:
			obj.flightControllerVersion = data.readU8() + '.' + data.readU8() + '.' + data.readU8()
			break

		case MSPCodes.MSP_BUILD_INFO:
			var dateLength = 11
			var buff = [];
			for (var i = 0; i < dateLength; i++) {
				buff.push(data.readU8());
			}
			buff.push(32); // ascii space

			var timeLength = 8;
			for (var i = 0; i < timeLength; i++) {
				buff.push(data.readU8());
			}
			obj.buildInfo = String.fromCharCode.apply(null, buff);
			break;

		case MSPCodes.MSP_BOARD_INFO:
			var identifier = ''
			for (var i = 0; i < 4; i++) {
				identifier += String.fromCharCode(data.readU8());
			}
			obj.boardIdentifier = identifier
			obj.boardVersion = data.readU16()
			break

		case MSPCodes.MSP_NAME:
			obj.name = ''
			if(data.length !== 0){
				var char
				while ((char = data.readU8()) !== null) {
					obj.name += String.fromCharCode(char)
				}
			}
			break

		case MSPCodes.MSP_UID:
			obj.uid = data.readU32().toString(16)
			obj.uid += '-' + data.readU32().toString(16)
			obj.uid += '-' + data.readU32().toString(16)
			break

		case MSPCodes.MSP_RX_MAP:
			var RC_MAP = []; // empty the array as new data is coming in
			for (var i = 0; i < data.byteLength; i++) {
				RC_MAP.push(data.readU8())
			}

			var RC_MAP_Letters = ['A', 'E', 'R', 'T', '1', '2', '3', '4'];
			var strBuffer = [];
			for (var i = 0; i < RC_MAP.length; i++) {
				strBuffer[RC_MAP[i]] = RC_MAP_Letters[i];
			}
			obj.RC_MAP = strBuffer.join('')
			break

		case MSPCodes.MSP_BOXNAMES:
			obj.AUX_CONFIG = [] // empty the array as new data is coming in

			var buff = [];
			for (var i = 0; i < data.byteLength; i++) {
				var char = data.readU8()
				if (char == 0x3B) { // ; (delimeter char)
					obj.AUX_CONFIG.push(String.fromCharCode.apply(null, buff)) // convert bytes into ASCII and save as strings

					// empty buffer
					buff = []
				} else {
					buff.push(char)
				}
			}
			break

		case MSPCodes.MSP_BOXIDS:
			obj.AUX_CONFIG_IDS = [] // empty the array as new data is coming in

			for (var i = 0; i < data.byteLength; i++) {
				obj.AUX_CONFIG_IDS.push(data.readU8())
			}
			break

		case MSPCodes.MSP_PIDNAMES:
			obj.PID_names = [] // empty the array as new data is coming in

			var buff = []
			for (var i = 0; i < data.byteLength; i++) {
				var char = data.readU8()
				if (char == 0x3B) { // ; (delimeter char)
					obj.PID_names.push(String.fromCharCode.apply(null, buff)) // convert bytes into ASCII and save as strings

					// empty buffer
					buff = []
				} else {
					buff.push(char)
				}
			}
			break

// got ack only
		case MSPCodes.MSP_SET_RAW_RC:
			break
		case MSPCodes.MSP_SET_PID:
			console.log('PID settings saved')
			break
		case MSPCodes.MSP_ACC_CALIBRATION:
			console.log('Accel calibration executed')
			break
		case MSPCodes.MSP_MAG_CALIBRATION:
			console.log('Mag calibration executed')
			break
		case MSPCodes.MSP_SET_RX_MAP:
			console.log('RCMAP saved')
			break

// skip
		case MSPCodes.MSP_EXT_CMD:
		case MSPCodes.MSP_ANALOG:
			break

		default:
			console.log('code not found', code, data)
	}

	return obj
}

Buffer.prototype.__mspoffset = 0
Buffer.prototype.readU32 = function (){
	var u32 = this.readUInt32LE(this.__mspoffset)
	this.__mspoffset += 4
	return u32
}
Buffer.prototype.read32 = function (){
	var i32 = this.readInt32LE(this.__mspoffset)
	this.__mspoffset += 4
	return i32
}
Buffer.prototype.readU16 = function (){
	var u16 = this.readUInt16LE(this.__mspoffset)
	this.__mspoffset += 2
	return u16
}
Buffer.prototype.read16 = function (){
	var i16 = this.readInt16LE(this.__mspoffset)
	this.__mspoffset += 2
	return i16
}
Buffer.prototype.readU8 = function (){
	var u8 = this.readUInt8(this.__mspoffset)
	this.__mspoffset += 1
	return u8
}
Buffer.prototype.read8 = function (){
	var i8 = this.readInt8(this.__mspoffset)
	this.__mspoffset += 1
	return i8
}

function Decoder(bytes, port) {
  var decoded = {};
  
  if(port == 1) {
    decoded.mode="gps";
    decoded.latitude = ((bytes[0]<<16)>>>0) + ((bytes[1]<<8)>>>0) + bytes[2];
    decoded.latitude = (decoded.latitude / 16777215.0 * 180) - 90;
    decoded.longitude = ((bytes[3]<<16)>>>0) + ((bytes[4]<<8)>>>0) + bytes[5];
    decoded.longitude = (decoded.longitude / 16777215.0 * 360) - 180;
    var altValue = ((bytes[6]<<8)>>>0) + bytes[7];
    var sign = bytes[6] & (1 << 7);
    
    if(sign) {
      decoded.altitude = 0xFFFF0000 | altValue;
    } else {
      decoded.altitude = altValue;
    }
    
    decoded.hdop = bytes[8] / 10.0;
    decoded.google = decoded.latitude +","+decoded.longitude;
  }
  
  if(port==2) {
    decoded.mode="simple";
  }
  return decoded;
}

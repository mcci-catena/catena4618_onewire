//
// Module:  catena-message-port6-decoder-node-red.js
//
// Function:
//      This Node-RED decoding function decodes the record sent by the Catena 4618 
//      onewire sensro app
//
// License:
//      Copyright (C) 2021, MCCI Corporation.
//      See LICENSE in accompanying git repository.
//

// begin catena-message-port6-decoder-ttn.js
/*

Name:   catena-message-port6-decoder-ttn.js

Function:
    Decode MCCI port 0x06 messages for TTN console.  Differs from the generic decoder
    in that it supports an additional temperature field from the 4901 OneWire probe.

Copyright and License:
    See accompanying LICENSE file at https://github.com/mcci-catena/catena4618_onewire/

Author:
    Terry Moore, MCCI Corporation   September 2021

*/

// calculate dewpoint (degrees C) given temperature (C) and relative humidity (0..100)
// from http://andrew.rsmas.miami.edu/bmcnoldy/Humidity.html
// rearranged for efficiency and to deal sanely with very low (< 1%) RH
function dewpoint(t, rh) {
    var c1 = 243.04;
    var c2 = 17.625;
    var h = rh / 100;
    if (h <= 0.01)
        h = 0.01;
    else if (h > 1.0)
        h = 1.0;

    var lnh = Math.log(h);
    var tpc1 = t + c1;
    var txc2 = t * c2;
    var txc2_tpc1 = txc2 / tpc1;

    var tdew = c1 * (lnh + txc2_tpc1) / (c2 - lnh - txc2_tpc1);
    return tdew;
}

/*

Name:   CalculateHeatIndex()

Description:
        Calculate the NWS heat index given dry-bulb T and RH

Definition:
        function CalculateHeatIndex(t, rh) -> value or null

Description:
        T is a Farentheit temperature in [76,120]; rh is a
        relative humidity in [0,100]. The heat index is computed
        and returned; or an error is returned.

Returns:
        number => heat index in Farenheit.
        null => error.

References:
        https://github.com/mcci-catena/heat-index/
        https://www.wpc.ncep.noaa.gov/html/heatindex_equation.shtml

        Results was checked against the full chart at iweathernet.com:
        https://www.iweathernet.com/wxnetcms/wp-content/uploads/2015/07/heat-index-chart-relative-humidity-2.png

        The MCCI-Catena heat-index site has a test js script to generate CSV to
        match the chart, a spreadsheet that recreates the chart, and a
        spreadsheet that compares results.

*/

function CalculateHeatIndex(t, rh) {
    var tRounded = Math.floor(t + 0.5);

    // return null outside the specified range of input parameters
    if (tRounded < 76 || tRounded > 126)
        return null;
    if (rh < 0 || rh > 100)
        return null;

    // according to the NWS, we try this first, and use it if we can
    var tHeatEasy = 0.5 * (t + 61.0 + ((t - 68.0) * 1.2) + (rh * 0.094));

    // The NWS says we use tHeatEasy if (tHeatHeasy + t)/2 < 80.0
    // This is the same computation:
    if ((tHeatEasy + t) < 160.0)
            return tHeatEasy;

    // need to use the hard form, and possibly adjust.
    var t2 = t * t;         // t squared
    var rh2 = rh * rh;      // rh squared
    var tResult =
        -42.379 +
        (2.04901523 * t) +
        (10.14333127 * rh) +
        (-0.22475541 * t * rh) +
        (-0.00683783 * t2) +
        (-0.05481717 * rh2) +
        (0.00122874 * t2 * rh) +
        (0.00085282 * t * rh2) +
        (-0.00000199 * t2 * rh2);

    // these adjustments come from the NWA page, and are needed to
    // match the reference table.
    var tAdjust;
    if (rh < 13.0 && 80.0 <= t && t <= 112.0)
        tAdjust = -((13.0 - rh) / 4.0) * Math.sqrt((17.0 - Math.abs(t - 95.0)) / 17.0);
    else if (rh > 85.0 && 80.0 <= t && t <= 87.0)
        tAdjust = ((rh - 85.0) / 10.0) * ((87.0 - t) / 5.0);
    else
        tAdjust = 0;

    // apply the adjustment
    tResult += tAdjust;

    // finally, the reference tables have no data above 183 (rounded),
    // so filter out answers that we have no way to vouch for.
    if (tResult >= 183.5)
        return null;
    else
        return tResult;
}

function CalculateHeatIndexCelsius(t, rh) {
    var result = CalculateHeatIndex(t, rh);
    if (result !== null) {
        // convert to celsius.
        result = (result - 32) * 5 / 9;
    }
    return result;
}

function uflt162f(rawUflt16)
    {
    // rawUflt16 is the 2-byte number decoded from wherever;
    // it's in range 0..0xFFFF
    // bits 15..12 are the exponent
    // bits 11..0 are the the mantissa. Unlike IEEE format,
    // the msb is explicit; this means that numbers
    // might not be normalized, but makes coding for
    // underflow easier.
    // As with IEEE format, negative zero is possible, so
    // we special-case that in hopes that JavaScript will
    // also cooperate.
    //
    // The result is a number in the half-open interval [0, 1.0);
    //

    // throw away high bits for repeatability.
    rawUflt16 &= 0xFFFF;

    // extract the exponent
    var exp1 = (rawUflt16 >> 12) & 0xF;

    // extract the "mantissa" (the fractional part)
    var mant1 = (rawUflt16 & 0xFFF) / 4096.0;

    // convert back to a floating point number. We hope
    // that Math.pow(2, k) is handled efficiently by
    // the JS interpreter! If this is time critical code,
    // you can replace by a suitable shift and divide.
    var f_unscaled = mant1 * Math.pow(2, exp1 - 15);

    return f_unscaled;
    }

/*

Name:  Decoder()

Function:
    Decode an MCCI Catena port-2 message for The Things Network console.

Definition:
    function Decoder(bytes, port) -> object

Description:
    This function decodes the message given by the byte array `bytes[]`,
    and returns an object with values in engineering units.

Returns:
    Object, or null if the bytes could not be decoded.

*/

function Decoder(bytes, port) {
    // Decode an uplink message from a buffer
    // (array) of bytes to an object of fields.
    var decoded = {};

    if (! (port === 3))
        return null;

    // see catena-message-port3-format.md
    // i is used as the index into the message. Start with the flag byte.
    // note that there's no discriminator.
    // test vectors are also available there.
    var i = 0;
    // fetch the bitmap.
    var flags = bytes[i++];

    if (flags & 0x1) {
        // set Vraw to a uint16, and increment pointer
        var Vraw = (bytes[i] << 8) + bytes[i + 1];
        i += 2;
        // interpret uint16 as an int16 instead.
        if (Vraw & 0x8000)
            Vraw += -0x10000;
        // scale and save in result.
        decoded.Vbat = Vraw / 4096.0;
    }

    if (flags & 0x2) {
        var Vraw = (bytes[i] << 8) + bytes[i + 1];
        i += 2;
        if (Vraw & 0x8000)
            Vraw += -0x10000;
        decoded.VDD = Vraw / 4096.0;
    }

    if (flags & 0x4) {
        var iBoot = bytes[i];
        i += 1;
        decoded.boot = iBoot;
    }

    if (flags & 0x8) {
        // we have temp, pressure, RH
        var tRaw = (bytes[i] << 8) + bytes[i + 1];
        if (tRaw & 0x8000)
            tRaw = -0x10000 + tRaw;
        i += 2;
        var rhRaw = (bytes[i] << 8) + bytes[i + 1];
        i += 2;

        decoded.t = tRaw / 256;
        decoded.rh = rhRaw / 65535.0 * 100;
        decoded.tDew =  dewpoint(decoded.t, decoded.rh);
        decoded.tHeatIndexC = CalculateHeatIndexCelsius(decoded.t, decoded.rh);
    }

    if (flags & 0x10) {
        // we have light irradiance info
        var irradiance = {};
        decoded.irradiance = irradiance;

        var lightRaw = (bytes[i] << 8) + bytes[i + 1];
        i += 2;
        irradiance.White = uflt162f(lightRaw) * Math.pow(2, 24);
    }

    // Vbus from USB connector
    if (flags & 0x20) {
        var Vraw = (bytes[i] << 8) + bytes[i + 1];
        i += 2;
        if (Vraw & 0x8000)
            Vraw += -0x10000;
        decoded.Vbus = Vraw / 4096.0;
    }

    // Probe temperature
    if (flags & 0x40) {
        var Traw = (bytes[i] << 8) + bytes[i + 1];
        i += 2;
        if (Traw & 0x8000)
            Traw += -0x10000;
        decoded.Tprobe= Traw / 256.0;
    }

    // at this point, decoded has the real values.
    return decoded;
}

// TTN V3 decoder
function decodeUplink(tInput) {
    var decoded = Decoder(tInput.bytes, tInput.fPort);
    var result = {};
    result.data = decoded;
    return result;
}
// end catena-message-port6-decoder-ttn.js

/*

Node-RED function body.

Input:
    msg     the object to be decoded.

            msg.payload_raw is taken
            as the raw payload if present; otheriwse msg.payload
            is taken to be a raw payload.

            msg.port is taken to be the LoRaWAN port nubmer.


Returns:
    This function returns a message body. It's a mutation of the
    input msg; msg.payload is changed to the decoded data, and
    msg.local is set to additional application-specific information.

*/

var b;

if ("payload_raw" in msg) {
    // the console already decoded this
    b = msg.payload_raw;  // pick up data for convenience
    // msg.payload_fields still has the decoded data
}
else {
    // no console debug
    b = msg.payload;  // pick up data for conveneince
}

var result = Decoder(b, msg.port);

// now update msg with the new payload and new .local field
// the old msg.payload is overwritten.
msg.payload = result;
msg.local =
    {
        nodeType: "Catena 4618",
        platformType: "Catena 461x",
        radioType: "Murata",
        applicationName: "OneWire sensor"
    };

return msg;

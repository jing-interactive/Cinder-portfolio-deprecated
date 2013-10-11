var ctx;
var pixels;

var w = 212 * 2;
var h = 120 * 2;

var map = new Array(64 * 64 * 64);
var texmap = new Array(16 * 16 * 3 * 16);
var textures = {
    'grass-top': 1,
    'stone': 4,
    'brick': 5,
    'wood': 7,
    'leafy': 8,
    'water': 9
};

function init() {
    // There are 16 different textures.
    for ( var texture = 1; texture < 16; texture++) {
        
        // Pick a random brightness
        var br = 255 - ((Math.random() * 96) | 0); // (The "|0" ensures integer)
        
        // The map for the texture - 3 16x16 squares.
        // Square 1: y = [0,15] - Top of cube
        // Square 2: y = [16,31] - Sides of cube (only 2 are ever visible)
        // Square 3: y = [32,47] - Bottom of cube
        // 'y' basically denotes the row of the map.
        for ( var y = 0; y < 16 * 3; y++) {
            
            // Hence, 'x' denotes the column per row.
            for ( var x = 0; x < 16; x++) {
                var color = 0x966C4A; //dirt brown
                if (texture == textures['stone'])
                    color = 0x7F7F7F; // stone gray
                if (texture != textures['stone'] || ((Math.random() * 3) | 0) == 0) {
                    br = 255 - ((Math.random() * 96) | 0);
                }
                // If we are in the top (16 + 5 or 6) rows of the grass topped block
                if ((texture == textures['grass-top'] && y < (((x * x * 3 + x * 81) >> 2) & 3) + 18)) {
                    color = 0x6AAA40; // grass green
                } else if ((texture == textures['grass-top'] && y < (((x * x * 3 + x * 81) >> 2) & 3) + 19)) {
                    br = br * 2 / 3; // reduce brightness by a third
                }
                
                if (texture == textures['wood']) {
                    color = 0x675231; // woody brown
                    
                    // If we are in square 1 or 3
                    if (x > 0 && x < 15
                            && ((y > 0 && y < 15) || (y > 32 && y < 47))) {
                        color = 0xBC9862; // light wood
                        // Makes concentric squares
                        var xd = (x - 7);
                        var yd = ((y & 15) - 7);
                        if (xd < 0)
                            xd = 1 - xd;
                        if (yd < 0)
                            yd = 1 - yd;
                        if (yd > xd)
                            xd = yd;
                        // fiddles with br for the light/dark sq's.
                        br = 196 - ((Math.random() * 32) | 0) + xd % 3 * 32;
                    } else if (((Math.random() * 2) | 0) == 0) {
                        // Makes the sides with the woody brown
                        br = br * (150 - (x & 1) * 100) / 100;
                    }
                }

                if (texture == textures['brick']) {
                    color = 0xB53A15; // Brick red
                    // Every 8th column, but alternately offset by 4
                    // OR every 4th row
                    if ((x + (y >> 2) * 4) % 8 == 0 || y % 4 == 0) {
                        color = 0xBCAFA5; // Beige brick spacing
                    }
                }

                if (texture == textures['water']) {
                    color = 0x4040ff; // Watery blue
                }

                // If we're in the third square of the texture map,
                // Cut down brightness by half (bottom of cubes)
                var brr = br;
                if (y >= 32)
                    brr /= 2;

                if (texture == textures['leafy']) {
                    color = 0x50D937; // leaf green

                    // 50% chance of no leaf spots
                    if (((Math.random() * 2) | 0) == 0) {
                        color = 0;
                        brr = 255;
                    }
                }

                // Some color map and putting it into the huge texture map
                var col = (((color >> 16) & 0xff) * brr / 255) << 16
                        | (((color >> 8) & 0xff) * brr / 255) << 8
                        | (((color) & 0xff) * brr / 255);
                texmap[x + y * 16 + texture * 256 * 3] = col;
            } // End of column loop
        } // End of row loop
    } // End of 16 textures loop

    ctx = document.getElementById('game').getContext('2d');

    for ( var x = 0; x < 64; x++) {
        for ( var y = 0; y < 64; y++) {
            for ( var z = 0; z < 64; z++) {
                var i = z << 12 | y << 6 | x;
                var yd = (y - 32.5) * 0.4;
                var zd = (z - 32.5) * 0.4;
                map[i] = (Math.random() * 16) | 0;
                if (Math.random() > Math.sqrt(Math.sqrt(yd * yd + zd * zd)) - 0.8)
                    map[i] = 0;
            }
        }
    }

    pixels = ctx.createImageData(w, h);

    for ( var i = 0; i < w * h; i++) {
        pixels.data[i * 4 + 3] = 255;
    }

    // setInterval(clock, 1000 / 100);
    renderMinecraft();
    ctx.putImageData(pixels, 0, 0);
};

function clock() {
    renderMinecraft();
    ctx.putImageData(pixels, 0, 0);
};

var f = 0;
function renderMinecraft() {
    var xRot = Math.sin(Date.now() % 10000 / 10000 * Math.PI * 2) * 0.4
            + Math.PI / 2;
    var yRot = Math.cos(Date.now() % 10000 / 10000 * Math.PI * 2) * 0.4;
    var yCos = Math.cos(yRot);
    var ySin = Math.sin(yRot);
    var xCos = Math.cos(xRot);
    var xSin = Math.sin(xRot);

    var ox = 32.5 + Date.now() % 10000 / 10000 * 64;
    var oy = 32.5;
    var oz = 32.5;

    f++;
    for ( var x = 0; x < w; x++) {
        var ___xd = (x - w / 2) / h;
        for ( var y = 0; y < h; y++) {
            var __yd = (y - h / 2) / h;
            var __zd = 1;

            var ___zd = __zd * yCos + __yd * ySin;
            var _yd = __yd * yCos - __zd * ySin;

            var _xd = ___xd * xCos + ___zd * xSin;
            var _zd = ___zd * xCos - ___xd * xSin;

            var col = 0;
            var br = 255;
            var ddist = 0;

            var closest = 32;
            for ( var d = 0; d < 3; d++) {
                var dimLength = _xd;
                if (d == 1)
                    dimLength = _yd;
                if (d == 2)
                    dimLength = _zd;

                var ll = 1 / (dimLength < 0 ? -dimLength : dimLength);
                var xd = (_xd) * ll;
                var yd = (_yd) * ll;
                var zd = (_zd) * ll;

                var initial = ox - (ox | 0);
                if (d == 1)
                    initial = oy - (oy | 0);
                if (d == 2)
                    initial = oz - (oz | 0);
                if (dimLength > 0)
                    initial = 1 - initial;

                var dist = ll * initial;

                var xp = ox + xd * initial;
                var yp = oy + yd * initial;
                var zp = oz + zd * initial;

                if (dimLength < 0) {
                    if (d == 0)
                        xp--;
                    if (d == 1)
                        yp--;
                    if (d == 2)
                        zp--;
                }

                while (dist < closest) {
                    var tex = map[(zp & 63) << 12 | (yp & 63) << 6 | (xp & 63)];

                    if (tex > 0) {
                        var u = ((xp + zp) * 16) & 15;
                        var v = ((yp * 16) & 15) + 16;
                        if (d == 1) {
                            u = (xp * 16) & 15;
                            v = ((zp * 16) & 15);
                            if (yd < 0)
                                v += 32;
                        }

                        var cc = texmap[u + v * 16 + tex * 256 * 3];
                        if (cc > 0) {
                            col = cc;
                            ddist = 255 - ((dist / 32 * 255) | 0);
                            br = 255 * (255 - ((d + 2) % 3) * 50) / 255;
                            closest = dist;
                        }
                    }
                    xp += xd;
                    yp += yd;
                    zp += zd;
                    dist += ll;
                }
            }

            var r = ((col >> 16) & 0xff) * br * ddist / (255 * 255);
            var g = ((col >> 8) & 0xff) * br * ddist / (255 * 255);
            var b = ((col) & 0xff) * br * ddist / (255 * 255);// + (255 -

            pixels.data[(x + y * w) * 4 + 0] = r;
            pixels.data[(x + y * w) * 4 + 1] = g;
            pixels.data[(x + y * w) * 4 + 2] = b;
        }
    }
}

init();